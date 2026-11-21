//! Chunky-style background chunk pregeneration.
//!
//! A [`PregenTask`] drives every chunk in a selected area (square or circle
//! around a center) to the `Full` stage through the normal chunk system: it
//! holds a bounded window of `FULL_CHUNK_LEVEL` tickets and waits for each
//! chunk to be published, then releases the ticket so the scheduler unloads
//! and saves the chunk again. This is the same path a visiting player takes,
//! so a pregenerated chunk is indistinguishable from one a player walked
//! through.

use std::sync::{
    Arc, Mutex, Weak,
    atomic::{AtomicBool, AtomicUsize, Ordering},
};
use std::time::{Duration, Instant};

use pumpkin_util::math::vector2::Vector2;
use pumpkin_util::text::TextComponent;
use pumpkin_world::chunk_system::ChunkLoading;
use tokio::time::sleep;
use tracing::{info, warn};

use crate::entity::player::Player;
use crate::world::World;

/// Chunks held open (ticketed) at once. Larger windows feed the generation
/// pool a deeper queue but keep more partially generated chunks in memory.
const WINDOW: usize = 96;
/// Total area cap, so a mistyped radius cannot allocate gigabytes of positions.
const MAX_CHUNKS: usize = 2_000_000;
/// How often a waiting chunk re-checks for cancel, shutdown or completion
/// through another path. Also bounds how long a cancel takes to land.
const CHUNK_POLL: Duration = Duration::from_secs(1);
/// Give up on a chunk entirely after this long, leave a warning and move on.
const CHUNK_GIVE_UP: Duration = Duration::from_secs(120);

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PregenShape {
    Square,
    Circle,
}

impl PregenShape {
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Square => "square",
            Self::Circle => "circle",
        }
    }
}

/// The `/chunky` selection shared by the command: radius, center and shape are
/// configured first, then `start` runs with whatever is set.
#[derive(Clone, Default)]
pub struct Selection {
    pub radius: i32,
    pub center: Option<(f64, f64)>,
    pub shape: Option<PregenShape>,
    pub world: Option<String>,
}

static SELECTION: Mutex<Option<Selection>> = Mutex::new(None);
static TASKS: Mutex<Vec<Arc<PregenTask>>> = Mutex::new(Vec::new());

/// Identifies a world for reporting and the one-task-per-world check. The
/// vanilla layout keeps every dimension in the same `world` folder, so the
/// folder name alone cannot tell them apart.
fn world_id(world: &World) -> &str {
    world
        .dimension
        .minecraft_name
        .strip_prefix("minecraft:")
        .unwrap_or(world.dimension.minecraft_name)
}

pub fn with_selection<R>(f: impl FnOnce(&mut Selection) -> R) -> R {
    let mut guard = SELECTION
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    let mut selection = guard.clone().unwrap_or_default();
    let result = f(&mut selection);
    *guard = Some(selection);
    result
}

/// Snapshot of one task's state for `/chunky progress`.
pub struct PregenProgress {
    pub world_name: String,
    pub done: usize,
    pub total: usize,
    pub skipped: usize,
    pub paused: bool,
    pub elapsed: Duration,
}

pub struct PregenTask {
    world: Arc<World>,
    positions: Arc<[Vector2<i32>]>,
    started: Instant,
    reporter: Option<Weak<Player>>,
    /// Seconds between progress reports.
    quiet: AtomicUsize,
    paused: AtomicBool,
    cancelled: AtomicBool,
    active: AtomicUsize,
    done: AtomicUsize,
    skipped: AtomicUsize,
}

impl PregenTask {
    pub fn progress(&self) -> PregenProgress {
        PregenProgress {
            world_name: world_id(&self.world).to_string(),
            done: self.done.load(Ordering::Relaxed),
            total: self.positions.len(),
            skipped: self.skipped.load(Ordering::Relaxed),
            paused: self.paused.load(Ordering::Relaxed),
            elapsed: self.started.elapsed(),
        }
    }

    pub fn pause(&self) {
        self.paused.store(true, Ordering::Relaxed);
    }

    pub fn resume(&self) {
        self.paused.store(false, Ordering::Relaxed);
    }

    pub fn cancel(&self) {
        self.cancelled.store(true, Ordering::Relaxed);
    }

    pub fn set_quiet(&self, seconds: usize) {
        self.quiet.store(seconds.max(1), Ordering::Relaxed);
    }

    fn stopping(&self) -> bool {
        self.cancelled.load(Ordering::Relaxed)
            || self
                .world
                .level
                .shut_down_chunk_system
                .load(Ordering::Relaxed)
    }

    /// Waits until `pos` is a fully generated (published) chunk, holding a
    /// `FULL_CHUNK_LEVEL` ticket for it. Mirrors `Level::fetch_chunk`, but
    /// with timeouts so one bad chunk cannot stall the whole task.
    async fn pregen_chunk(&self, pos: Vector2<i32>) {
        let level = &self.world.level;
        if level.read_chunk_sync(&pos, |_| ()).is_some() {
            return;
        }

        let deadline = Instant::now() + CHUNK_GIVE_UP;
        let recv = level.chunk_listener.add_single_chunk_listener(pos);
        tokio::pin!(recv);
        let mut ticketed = false;
        loop {
            if self.stopping() {
                break;
            }
            if !ticketed {
                let mut loading = level
                    .chunk_loading
                    .lock()
                    .unwrap_or_else(std::sync::PoisonError::into_inner);
                loading.add_ticket(pos, ChunkLoading::FULL_CHUNK_LEVEL);
                loading.send_change();
                ticketed = true;
            }

            // Poll instead of waiting in one long block, so a cancel or a
            // shutdown lands within a second even while chunks are grinding.
            let resolved = tokio::select! {
                res = &mut recv => {
                    let _ = res;
                    true
                }
                () = tokio::time::sleep(CHUNK_POLL) => false,
            };
            if resolved {
                break;
            }
            // The chunk was published in the window before the listener
            // registered, or generation is still queued behind other work.
            if level.read_chunk_sync(&pos, |_| ()).is_some() {
                break;
            }
            if Instant::now() >= deadline {
                warn!(
                    "chunky: chunk ({}, {}) in '{}' did not finish, skipping",
                    pos.x,
                    pos.y,
                    world_id(&self.world)
                );
                self.skipped.fetch_add(1, Ordering::Relaxed);
                break;
            }
        }

        if ticketed {
            let mut loading = level
                .chunk_loading
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            loading.remove_ticket(pos, ChunkLoading::FULL_CHUNK_LEVEL);
            loading.send_change();
        }
    }

    async fn drive(self: Arc<Self>) {
        let total = self.positions.len();
        info!(
            "chunky: task started for '{}' ({} chunks)",
            world_id(&self.world),
            total
        );
        self.report();

        let mut index = 0usize;
        let mut next_report = Instant::now() + self.report_interval();
        while index < total {
            if self.stopping() {
                break;
            }
            if self.paused.load(Ordering::Relaxed) {
                self.tick_report(&mut next_report);
                sleep(Duration::from_millis(100)).await;
                continue;
            }

            while index < total && self.active.load(Ordering::Relaxed) < WINDOW {
                let pos = self.positions[index];
                index += 1;
                self.active.fetch_add(1, Ordering::Relaxed);
                let task = self.clone();
                tokio::spawn(async move {
                    task.pregen_chunk(pos).await;
                    task.active.fetch_sub(1, Ordering::Relaxed);
                    task.done.fetch_add(1, Ordering::Relaxed);
                });
            }

            self.tick_report(&mut next_report);
            sleep(Duration::from_millis(50)).await;
        }

        // Let the in-flight window drain; the sub-tasks are time-bounded and
        // release their own tickets, also when the task was cancelled.
        while self.active.load(Ordering::Relaxed) > 0 {
            self.tick_report(&mut next_report);
            sleep(Duration::from_millis(100)).await;
        }

        let cancelled = self.cancelled.load(Ordering::Relaxed);
        info!(
            "chunky: task {} for '{}' ({} chunks, {} skipped)",
            if cancelled { "cancelled" } else { "finished" },
            world_id(&self.world),
            self.done.load(Ordering::Relaxed),
            self.skipped.load(Ordering::Relaxed)
        );
        self.report();

        let mut tasks = TASKS
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        tasks.retain(|task| !Arc::ptr_eq(task, &self));
    }

    fn report_interval(&self) -> Duration {
        Duration::from_secs(self.quiet.load(Ordering::Relaxed).max(1) as u64)
    }

    fn tick_report(&self, next_report: &mut Instant) {
        let now = Instant::now();
        if now >= *next_report {
            self.report();
            *next_report = now + self.report_interval();
        }
    }

    fn report(&self) {
        let line = self.progress_line();
        info!("{line}");
        if let Some(player) = self.reporter.as_ref().and_then(Weak::upgrade) {
            player.send_system_message(&TextComponent::text(line));
        }
    }

    fn progress_line(&self) -> String {
        let done = self.done.load(Ordering::Relaxed);
        let total = self.positions.len();
        let skipped = self.skipped.load(Ordering::Relaxed);
        let elapsed = self.started.elapsed();
        let percent = (done as f64 / total.max(1) as f64) * 100.0;
        let cps = if elapsed.as_secs_f64() > 0.0 {
            done as f64 / elapsed.as_secs_f64()
        } else {
            0.0
        };
        let eta = if cps > 0.0 {
            format_duration(Duration::from_secs_f64((total - done) as f64 / cps))
        } else {
            "-".to_string()
        };
        let state = if self.cancelled.load(Ordering::Relaxed) {
            "cancelled"
        } else if self.paused.load(Ordering::Relaxed) {
            "paused"
        } else {
            "running"
        };
        let skipped_note = if skipped > 0 {
            format!(", skipped {skipped}")
        } else {
            String::new()
        };
        format!(
            "chunky: task {state} for '{}': {}/{} chunks ({percent:.1}%), elapsed {}, rate {cps:.1} cps, ETA {eta}{skipped_note}",
            world_id(&self.world),
            done,
            total,
            format_duration(elapsed),
        )
    }
}

fn format_duration(duration: Duration) -> String {
    let secs = duration.as_secs();
    if secs >= 3600 {
        format!(
            "{}h{:02}m{:02}s",
            secs / 3600,
            (secs % 3600) / 60,
            secs % 60
        )
    } else if secs >= 60 {
        format!("{}m{:02}s", secs / 60, secs % 60)
    } else {
        format!("{secs}s")
    }
}

/// Starts a pregeneration task for `world`. Fails if a task is already
/// running for that world or the selection covers too many chunks.
pub fn start(
    world: &Arc<World>,
    center_x: f64,
    center_z: f64,
    radius: i32,
    shape: PregenShape,
    reporter: Option<Weak<Player>>,
) -> Result<Arc<PregenTask>, String> {
    if radius <= 0 {
        return Err("Radius must be positive".to_string());
    }

    let min_cx = ((center_x - f64::from(radius)) / 16.0).floor() as i64;
    let max_cx = ((center_x + f64::from(radius)) / 16.0).floor() as i64;
    let min_cz = ((center_z - f64::from(radius)) / 16.0).floor() as i64;
    let max_cz = ((center_z + f64::from(radius)) / 16.0).floor() as i64;
    let bound = (max_cx - min_cx + 1) * (max_cz - min_cz + 1);
    if bound as usize > MAX_CHUNKS {
        return Err(format!(
            "Selected area covers about {bound} chunks (max {MAX_CHUNKS})"
        ));
    }

    let mut positions: Vec<Vector2<i32>> = Vec::with_capacity(bound as usize);
    for cx in min_cx..=max_cx {
        for cz in min_cz..=max_cz {
            let dx = (cx * 16 + 8) as f64 - center_x;
            let dz = (cz * 16 + 8) as f64 - center_z;
            let inside = match shape {
                PregenShape::Square => {
                    dx.abs() <= f64::from(radius) && dz.abs() <= f64::from(radius)
                }
                PregenShape::Circle => dx * dx + dz * dz <= f64::from(radius).powi(2),
            };
            if inside {
                positions.push(Vector2::new(cx as i32, cz as i32));
            }
        }
    }
    // Center first, then outward, so the useful area is done first.
    positions.sort_unstable_by(|a, b| {
        let da = (a.x as f64 * 16.0 + 8.0 - center_x).powi(2)
            + (a.y as f64 * 16.0 + 8.0 - center_z).powi(2);
        let db = (b.x as f64 * 16.0 + 8.0 - center_x).powi(2)
            + (b.y as f64 * 16.0 + 8.0 - center_z).powi(2);
        da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
    });

    let task = Arc::new(PregenTask {
        world: world.clone(),
        positions: Arc::from(positions),
        started: Instant::now(),
        reporter,
        quiet: AtomicUsize::new(1),
        paused: AtomicBool::new(false),
        cancelled: AtomicBool::new(false),
        active: AtomicUsize::new(0),
        done: AtomicUsize::new(0),
        skipped: AtomicUsize::new(0),
    });

    let mut tasks = TASKS
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    if tasks
        .iter()
        .any(|task| world_id(&task.world) == world_id(world))
    {
        return Err(format!(
            "Task already running for world '{}'",
            world_id(world)
        ));
    }
    tasks.push(task.clone());
    drop(tasks);

    let driver = task.clone();
    world.level.spawn_task(async move {
        driver.drive().await;
    });
    Ok(task)
}

pub fn pause_all() -> usize {
    let tasks = TASKS
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    for task in tasks.iter() {
        task.pause();
    }
    tasks.len()
}

pub fn resume_all() -> usize {
    let tasks = TASKS
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    for task in tasks.iter() {
        task.resume();
    }
    tasks.len()
}

pub fn cancel_all() -> usize {
    let tasks = TASKS
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    for task in tasks.iter() {
        task.cancel();
    }
    tasks.len()
}

#[must_use]
pub fn active_tasks() -> Vec<Arc<PregenTask>> {
    TASKS
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .clone()
}
