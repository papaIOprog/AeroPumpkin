//! Server-side sub-levels: plots of real chunks parked at remote coordinates,
//! placed in the world by a physics body.
//!
//! The model and the wire protocol follow Sable so that the real Sable client
//! mod renders Pumpkin's sub-levels: a sub-level's content is a set of ordinary
//! chunks at a "plot" coordinate, streamed with vanilla chunk packets, and only
//! the pose travels over custom payloads. Entities keep their parked
//! coordinates; Sable clients relocate anything inside a tracked plot.
//!
//! Clients without Sable get the lightweight `pumpkin:sublevel` debug channel
//! (a wireframe box) that the Pumpkin Sublevels mod renders.
//!
//! Everything here runs sequentially on the world tick thread; the physics
//! step never touches rayon (see `pumpkin-sublevel`).

pub mod sable;

use std::sync::Arc;
use std::time::{Duration, Instant};

use bytes::{BufMut, Bytes};
use pumpkin_nbt::compound::NbtCompound;
use pumpkin_protocol::codec::var_int::VarInt;
use pumpkin_protocol::java::client::play::{CBundleDelimiter, CCustomPayload};
use pumpkin_protocol::ser::NetworkWriteExt;
use pumpkin_protocol::{ClientPacket, MultiVersionJavaPacket};
use pumpkin_sublevel::BodySnapshot;
use pumpkin_util::math::vector2::Vector2;
use pumpkin_util::math::vector3::Vector3;
use rustc_hash::{FxHashMap, FxHashSet};
use tracing::{debug, info, warn};
use uuid::Uuid;

use super::World;
use super::block_entity_from_nbt;
use crate::block::entities::BlockEntity;
use crate::entity::EntityBase;
use crate::entity::player::Player;
use crate::net::java::JavaClient;
use crate::net::java::chunk_data::CChunkData;
use pumpkin_data::Block;
use pumpkin_data::entity::EntityType;
use pumpkin_util::math::position::BlockPos;
use pumpkin_world::chunk_system::ChunkLoading;
use pumpkin_world::world::BlockFlags;

use crate::entity::r#type::from_type;

/// Each plot is `1 << PLOT_LOG_SIZE` chunks on a side, as in Sable.
pub const PLOT_LOG_SIZE: i32 = 7;
/// The plotyard is `1 << PLOT_LOG_SIZE` plots on a side, starting here.
pub const PLOTYARD_ORIGIN: i32 = 10_000;
/// Sub-levels are networked to players within this many blocks, Sable's default.
pub const TRACKING_RANGE: f64 = 320.0;
/// Assembled regions stay narrow enough to sit around one plot centre and
/// small enough to copy in one pass.
const MAX_ASSEMBLE_SPAN: i32 = 128;
const MAX_ASSEMBLE_VOLUME: i64 = 65_536;
/// The pose origin sits at this y in plot space: (min + max build height) / 2.
pub const PLOT_ORIGIN_Y: i32 = 128;
/// How long a plot chunk may take to generate before the sub-level stays
/// unsynced.
const PLOT_CHUNK_TIMEOUT: Duration = Duration::from_secs(120);

/// Custom payload channel for the lightweight debug protocol.
pub const DEBUG_CHANNEL: &str = "pumpkin:sublevel";

const DEBUG_MSG_CREATE: u8 = 1;
const DEBUG_MSG_TRANSFORM: u8 = 2;
const DEBUG_MSG_REMOVE: u8 = 3;
const DEBUG_REFRESH_TICKS: u64 = 100;
const DEBUG_TRANSFORM_TICKS: u64 = 2;

/// Sable sends one snapshot bundle per server tick; this is the nominal gap.
const TICK_MS: i32 = 50;

struct SubLevel {
    id: u32,
    uuid: Uuid,
    /// Index within the plotyard, in plots.
    plot: Vector2<i32>,
    /// The parked chunk at the plot centre; riders spawn on it.
    chunk: Vector2<i32>,
    /// Every parked chunk holding content, for the full sync.
    chunks: Vec<Vector2<i32>>,
    /// The parked block the body sits on: the pose's rotation point.
    pivot: Vector3<f64>,
    /// Plot-local block bounds of the content, inclusive.
    bounds: [i32; 6],
    half_extents: [f32; 3],
    /// Set once the parked chunks hold the content and can be sent.
    ready: bool,
    last_pose: sable::Pose,
    networked_pose: sable::Pose,
    stop_sent: bool,
    tracking: FxHashSet<Uuid>,
}

impl SubLevel {
    fn plot_long(&self) -> i64 {
        sable::chunk_pos_to_long(self.plot.x, self.plot.y)
    }

    /// This tick's movement update: a snapshot entry while the pose keeps
    /// changing, one stop notice when it comes to rest, then nothing.
    fn movement_ticket(&mut self, bodies: &[BodySnapshot]) -> Option<Ticket> {
        let plot = self.plot_long();
        if self.last_pose.moved_beyond(&self.networked_pose) {
            self.networked_pose = self.last_pose;
            self.stop_sent = false;
            let body = bodies.iter().find(|body| body.id == self.id);
            Some(Ticket {
                plot,
                moving: true,
                entry: body.map(|body| sable::SnapshotEntry {
                    plot,
                    pose: self.networked_pose,
                    linear_velocity: body.linear_velocity,
                    angular_velocity: body.angular_velocity,
                }),
            })
        } else if !self.stop_sent {
            self.stop_sent = true;
            Some(Ticket {
                plot,
                moving: false,
                entry: None,
            })
        } else {
            None
        }
    }
}

/// One sub-level's movement in a player's per-tick update.
#[derive(Clone, Copy)]
struct Ticket {
    plot: i64,
    moving: bool,
    entry: Option<sable::SnapshotEntry>,
}

pub struct SubLevelManager {
    physics: pumpkin_sublevel::SubLevelSystem,
    levels: Vec<SubLevel>,
    /// Sable's interpolation tick, advanced once per server tick.
    tick: i32,
    last_send: Option<Instant>,
    /// Players still receiving per-tick snapshot info.
    updating: FxHashSet<Uuid>,
    debug_tick: u64,
}

impl Default for SubLevelManager {
    fn default() -> Self {
        Self::new()
    }
}

impl SubLevelManager {
    #[must_use]
    pub fn new() -> Self {
        Self {
            physics: pumpkin_sublevel::SubLevelSystem::new(),
            levels: Vec::new(),
            tick: 0,
            last_send: None,
            updating: FxHashSet::default(),
            debug_tick: 0,
        }
    }

    /// Allocates a plot and a body at `pose`, the plot origin's world
    /// position. The content chunk is prepared asynchronously by
    /// [`World::prepare_sublevel_chunk`]; until then the sub-level is not
    /// networked. Returns the new id and the parked chunk.
    pub fn spawn(&mut self, pose: Vector3<f64>, half_extents: [f32; 3]) -> (u32, Vector2<i32>) {
        let plot = self.first_free_plot();
        let corner = Vector2::new(
            (PLOTYARD_ORIGIN + plot.x) << PLOT_LOG_SIZE,
            (PLOTYARD_ORIGIN + plot.y) << PLOT_LOG_SIZE,
        );
        let center = Vector2::new(
            corner.x + (1 << (PLOT_LOG_SIZE - 1)),
            corner.y + (1 << (PLOT_LOG_SIZE - 1)),
        );

        let pose_f32 = pose.to_f32_lossy();
        let id = self
            .physics
            .spawn([pose_f32.x, pose_f32.y, pose_f32.z], half_extents);
        // The platform always fills its chunk's middle 16x16 with its top
        // face at the plot origin. Bounds are absolute parked-world block
        // coordinates, tight to the content and inclusive, as in Sable: the
        // client sizes its render sections and collision box from them.
        self.levels.push(SubLevel {
            id,
            uuid: Uuid::new_v4(),
            plot,
            chunk: center,
            chunks: vec![center],
            pivot: plot_center_block(center).to_f64(),
            bounds: [
                center.x * 16,
                PLOT_ORIGIN_Y - 1,
                center.y * 16,
                center.x * 16 + 15,
                PLOT_ORIGIN_Y - 1,
                center.y * 16 + 15,
            ],
            half_extents,
            ready: false,
            last_pose: sable::Pose::default(),
            networked_pose: sable::Pose::default(),
            stop_sent: false,
            tracking: FxHashSet::default(),
        });
        (id, center)
    }

    /// Allocates a plot for content copied out of a world region.
    /// `bounds` are the region's block extents relative to the plot centre
    /// block, the anchor the caller's pose sits on, so the copy renders
    /// exactly where the region was. Returns the new id; the parked chunks
    /// are filled by [`World::assemble_sublevel`], which tightens the bounds
    /// to the copied content before the sub-level is networked.
    pub fn assemble(
        &mut self,
        pose: Vector3<f64>,
        bounds: [i32; 6],
        pivot_offset: Vector3<f64>,
    ) -> u32 {
        let plot = self.first_free_plot();
        let corner = Vector2::new(
            (PLOTYARD_ORIGIN + plot.x) << PLOT_LOG_SIZE,
            (PLOTYARD_ORIGIN + plot.y) << PLOT_LOG_SIZE,
        );
        let center = Vector2::new(
            corner.x + (1 << (PLOT_LOG_SIZE - 1)),
            corner.y + (1 << (PLOT_LOG_SIZE - 1)),
        );

        let parked = parked_bounds(plot_center_block(center), &bounds);
        // The pose rotates about the parked point the body sits on.
        let pivot = plot_center_block(center).to_f64() + pivot_offset;
        let half_extents = [
            (bounds[3] - bounds[0] + 1) as f32 / 2.0,
            (bounds[4] - bounds[1] + 1) as f32 / 2.0,
            (bounds[5] - bounds[2] + 1) as f32 / 2.0,
        ];
        let pose_f32 = pose.to_f32_lossy();
        let id = self
            .physics
            .spawn([pose_f32.x, pose_f32.y, pose_f32.z], half_extents);
        self.levels.push(SubLevel {
            id,
            uuid: Uuid::new_v4(),
            plot,
            chunk: center,
            chunks: content_chunks(&parked),
            pivot,
            bounds: parked,
            half_extents,
            ready: false,
            last_pose: sable::Pose::default(),
            networked_pose: sable::Pose::default(),
            stop_sent: false,
            tracking: FxHashSet::default(),
        });
        id
    }

    /// Shrinks sub-level `id`'s bounds to the blocks actually parked in its
    /// plot and re-derives its content chunks, as Sable's per-chunk tight
    /// boxes do. Called once the copy finished, before the sub-level is
    /// networked.
    pub fn tighten_bounds(&mut self, id: u32, bounds: [i32; 6]) {
        if let Some(level) = self.levels.iter_mut().find(|level| level.id == id) {
            level.chunks = content_chunks(&bounds);
            level.bounds = bounds;
        }
    }

    fn first_free_plot(&self) -> Vector2<i32> {
        let side = 1 << PLOT_LOG_SIZE;
        (0..side)
            .flat_map(|z| (0..side).map(move |x| Vector2::new(x, z)))
            .find(|candidate| !self.levels.iter().any(|level| level.plot == *candidate))
            .unwrap_or(Vector2::new(0, 0))
    }

    /// Marks the parked chunk of sub-level `id` as ready to be networked.
    pub fn mark_ready(&mut self, id: u32) {
        if let Some(level) = self.levels.iter_mut().find(|level| level.id == id) {
            level.ready = true;
        }
    }

    /// Players still tracking sub-level `id`, for telling them it is gone.
    #[must_use]
    pub fn tracking_players(&self, id: u32) -> Vec<Uuid> {
        self.levels
            .iter()
            .find(|level| level.id == id)
            .map(|level| level.tracking.iter().copied().collect())
            .unwrap_or_default()
    }

    pub fn remove(&mut self, id: u32) -> bool {
        let Some(index) = self.levels.iter().position(|level| level.id == id) else {
            return false;
        };
        self.levels.swap_remove(index);
        self.physics.remove(id)
    }

    pub fn apply_velocity_change(&mut self, id: u32, delta: [f32; 3]) -> bool {
        self.physics.apply_velocity_change(id, delta)
    }

    /// The parked chunk of sub-level `id`, for preparing its content and
    /// placing riders.
    #[must_use]
    pub fn chunk_of(&self, id: u32) -> Option<Vector2<i32>> {
        self.levels
            .iter()
            .find(|level| level.id == id)
            .map(|level| level.chunk)
    }

    /// Every parked chunk of sub-level `id` that holds content.
    #[must_use]
    pub fn chunks_of(&self, id: u32) -> Option<Vec<Vector2<i32>>> {
        self.levels
            .iter()
            .find(|level| level.id == id)
            .map(|level| level.chunks.clone())
    }

    /// The plot-local block bounds of sub-level `id`.
    #[must_use]
    pub fn bounds_of(&self, id: u32) -> Option<[i32; 6]> {
        self.levels
            .iter()
            .find(|level| level.id == id)
            .map(|level| level.bounds)
    }

    /// The body pose of sub-level `id`, for mapping content back to the world.
    #[must_use]
    pub fn pose_of(&self, id: u32) -> Option<sable::Pose> {
        let level = self.levels.iter().find(|level| level.id == id)?;
        self.physics
            .snapshots()
            .into_iter()
            .find(|body| body.id == level.id)
            .map(|body| pose_from_body(&body, &level.pivot))
    }

    /// Converts a position reported in a sub-level's parked coordinates to
    /// its world position. Sable clients standing on a sub-level send
    /// positions in that sub-level's local frame; without this the server
    /// would place the player millions of blocks away.
    #[must_use]
    pub fn parked_to_world(&self, position: Vector3<f64>) -> Option<Vector3<f64>> {
        let chunk = Vector2::new(
            (position.x.floor() as i32).div_euclid(16),
            (position.z.floor() as i32).div_euclid(16),
        );
        if !Self::is_plot_chunk(chunk) {
            return None;
        }
        let plot = Vector2::new(
            (chunk.x >> PLOT_LOG_SIZE) - PLOTYARD_ORIGIN,
            (chunk.y >> PLOT_LOG_SIZE) - PLOTYARD_ORIGIN,
        );
        let level = self.levels.iter().find(|level| level.plot == plot)?;
        let pivot = level.pivot;
        Some(Vector3::new(
            level.last_pose.position[0] + position.x - pivot.x,
            level.last_pose.position[1] + position.y - pivot.y,
            level.last_pose.position[2] + position.z - pivot.z,
        ))
    }

    /// Whether `chunk` lies inside the plotyard, where sub-levels park their
    /// content.
    #[must_use]
    pub fn is_plot_chunk(chunk: Vector2<i32>) -> bool {
        let in_plotyard = |axis: i32| {
            let plot = (axis >> PLOT_LOG_SIZE) - PLOTYARD_ORIGIN;
            (0..1 << PLOT_LOG_SIZE).contains(&plot)
        };
        in_plotyard(chunk.x) && in_plotyard(chunk.y)
    }

    /// Whether `player` tracks the sub-level whose plot contains `chunk`.
    /// The entity tracker uses this to show parked entities to Sable clients.
    #[must_use]
    pub fn player_tracks_chunk(&self, player: Uuid, chunk: Vector2<i32>) -> bool {
        let in_plotyard = |axis: i32| {
            let plot = (axis >> PLOT_LOG_SIZE) - PLOTYARD_ORIGIN;
            (0..1 << PLOT_LOG_SIZE).contains(&plot)
        };
        if !in_plotyard(chunk.x) || !in_plotyard(chunk.y) {
            return false;
        }
        self.levels.iter().any(|level| {
            level.ready
                && level.tracking.contains(&player)
                && (chunk.x >> PLOT_LOG_SIZE) == (PLOTYARD_ORIGIN + level.plot.x)
                && (chunk.y >> PLOT_LOG_SIZE) == (PLOTYARD_ORIGIN + level.plot.y)
        })
    }

    /// Everything `/sublevel list` shows, per sub-level: the body, the parked
    /// chunk, whether the content is ready, and how many players track it.
    #[must_use]
    pub fn info(&self) -> Vec<(BodySnapshot, Vector2<i32>, bool, usize)> {
        self.physics
            .snapshots()
            .into_iter()
            .filter_map(|body| {
                self.levels
                    .iter()
                    .find(|level| level.id == body.id)
                    .map(|level| (body, level.chunk, level.ready, level.tracking.len()))
            })
            .collect()
    }

    /// Bodies with their half extents, for the debug channel.
    #[must_use]
    pub fn debug_snapshots(&self) -> Vec<(BodySnapshot, [f32; 3])> {
        self.physics
            .snapshots()
            .into_iter()
            .filter_map(|body| {
                self.levels
                    .iter()
                    .find(|level| level.id == body.id)
                    .map(|level| (body, level.half_extents))
            })
            .collect()
    }

    /// Steps the simulation and syncs every client. Runs from the world tick,
    /// before the entity tracker. Returns the payloads for the debug channel.
    pub fn tick(&mut self, world: &World) -> Vec<Vec<u8>> {
        let bodies = self.physics.step();
        self.tick = self.tick.wrapping_add(1);
        let ms_since_last = self.note_send_time();

        let players: Vec<Arc<Player>> = world.players.load().to_vec();
        self.update_tracking(world, &bodies, &players);
        self.send_movement(&bodies, &players, ms_since_last);
        self.debug_messages()
    }

    fn note_send_time(&mut self) -> i32 {
        let now = Instant::now();
        let ms_since_last = self
            .last_send
            .and_then(|last| i32::try_from(now.duration_since(last).as_millis()).ok())
            .unwrap_or(TICK_MS);
        self.last_send = Some(now);
        ms_since_last
    }

    /// Records every body's pose, then starts tracking for players that came
    /// into range (with a full sync) and ends it for players that left.
    /// Sable clients only: they are the ones that understand the protocol.
    fn update_tracking(&mut self, world: &World, bodies: &[BodySnapshot], players: &[Arc<Player>]) {
        // A reconnecting player keeps their uuid, but the new client has no
        // sub-levels: forget them so they get a fresh full sync instead of
        // snapshots for sub-levels they never tracked.
        let online = |uuid: &Uuid| players.iter().any(|player| player.gameprofile.id == *uuid);
        for level in &mut self.levels {
            level.tracking.retain(online);
        }
        self.updating.retain(online);

        let mut syncs: Vec<(u32, Uuid)> = Vec::new();
        let mut untracks: Vec<(i64, Uuid)> = Vec::new();
        for body in bodies {
            let Some(level) = self.levels.iter_mut().find(|level| level.id == body.id) else {
                continue;
            };
            let pose = pose_from_body(body, &level.pivot);
            level.last_pose = pose;
            if !level.ready {
                continue;
            }
            for player in players {
                if !is_sable_client(player) {
                    continue;
                }
                let uuid = player.gameprofile.id;
                let within = within_range(player, &pose.position);
                if within && !level.tracking.contains(&uuid) {
                    level.tracking.insert(uuid);
                    syncs.push((level.id, uuid));
                } else if !within && level.tracking.contains(&uuid) {
                    level.tracking.remove(&uuid);
                    untracks.push((level.plot_long(), uuid));
                }
            }
        }

        for (plot, uuid) in &untracks {
            if let Some(player) = find_player(players, *uuid) {
                send_payload(
                    player,
                    sable::STOP_TRACKING,
                    &encode(sable::encode_plot_ref(*plot)),
                );
            }
        }
        for (id, uuid) in &syncs {
            let Some(player) = find_player(players, *uuid) else {
                continue;
            };
            let Some(level) = self.levels.iter().find(|level| level.id == *id) else {
                continue;
            };
            let Some(body) = bodies.iter().find(|body| body.id == *id) else {
                continue;
            };
            send_full_sync(world, player, level, body, self.tick);
        }
    }

    /// Sable's per-tick movement updates: a snapshot bundle per tracking
    /// player with moving sub-levels, a stop notice when one comes to rest.
    fn send_movement(
        &mut self,
        bodies: &[BodySnapshot],
        players: &[Arc<Player>],
        ms_since_last: i32,
    ) {
        let mut tickets: FxHashMap<Uuid, Vec<Ticket>> = FxHashMap::default();
        for level in &mut self.levels {
            if !level.ready || level.tracking.is_empty() {
                continue;
            }
            let Some(ticket) = level.movement_ticket(bodies) else {
                continue;
            };
            for uuid in &level.tracking {
                tickets.entry(*uuid).or_default().push(ticket);
            }
        }

        // Players that were updating but have nothing left to send get one
        // final "stopped" info and leave the set.
        let settled: Vec<Uuid> = self
            .updating
            .iter()
            .copied()
            .filter(|uuid| !tickets.contains_key(uuid))
            .collect();
        for uuid in &settled {
            if let Some(player) = find_player(players, *uuid) {
                send_payload(
                    player,
                    sable::SNAPSHOT_INFO,
                    &encode(sable::encode_snapshot_info(ms_since_last, self.tick, true)),
                );
            }
        }
        self.updating.retain(|uuid| !settled.contains(uuid));

        for (uuid, list) in &tickets {
            let Some(player) = find_player(players, *uuid) else {
                continue;
            };
            let info = encode(sable::encode_snapshot_info(ms_since_last, self.tick, false));
            for ticket in list.iter().filter(|ticket| !ticket.moving) {
                send_payload(
                    player,
                    sable::STOP_MOVING,
                    &encode(sable::encode_plot_ref(ticket.plot)),
                );
            }
            let entries: Vec<sable::SnapshotEntry> = list.iter().filter_map(|t| t.entry).collect();
            for batch in entries.chunks(sable::SNAPSHOT_BATCH) {
                send_bundle(
                    player,
                    &[
                        encode_payload(player, sable::SNAPSHOT_INFO, &info),
                        encode_payload(
                            player,
                            sable::SNAPSHOT,
                            &encode(sable::encode_snapshot(self.tick, batch)),
                        ),
                    ],
                );
            }
            self.updating.insert(*uuid);
        }
    }

    /// Tells players the sub-level is gone, then removes it.
    pub fn remove_and_notify(&mut self, world: &World, id: u32) -> bool {
        let tracking = self.tracking_players(id);
        let Some(plot) = self
            .levels
            .iter()
            .find(|level| level.id == id)
            .map(SubLevel::plot_long)
        else {
            return false;
        };
        let players = world.players.load();
        for uuid in &tracking {
            if let Some(player) = players.iter().find(|player| player.gameprofile.id == *uuid) {
                send_payload(
                    player,
                    sable::STOP_TRACKING,
                    &encode(sable::encode_plot_ref(plot)),
                );
            }
        }
        self.remove(id)
    }

    /// Wireframe payloads for the Pumpkin Sublevels mod: a full refresh
    /// every few seconds so late joiners converge, transforms at 10 Hz.
    fn debug_messages(&mut self) -> Vec<Vec<u8>> {
        self.debug_tick += 1;
        let refresh = self.debug_tick.is_multiple_of(DEBUG_REFRESH_TICKS);
        let transforms = self.debug_tick.is_multiple_of(DEBUG_TRANSFORM_TICKS);
        if !refresh && !transforms {
            return Vec::new();
        }
        self.debug_snapshots()
            .into_iter()
            .map(|(body, half_extents)| {
                if refresh {
                    encode_debug_create(&body, half_extents)
                } else {
                    encode_debug_transform(&body)
                }
            })
            .collect()
    }
}

thread_local! {
    /// Set while the entity tracker sends packets about an entity parked in a
    /// sub-level: Sable clients read one extra boolean on such packets.
    static SENDING_PARKED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Runs `send` with every Sable-modified entity packet it emits flagged as
/// "actually in a sub-level".
pub fn with_parked_entity<R>(send: impl FnOnce() -> R) -> R {
    SENDING_PARKED.with(|flag| flag.set(true));
    let result = send();
    SENDING_PARKED.with(|flag| flag.set(false));
    result
}

/// Whether packets sent right now describe an entity parked in a sub-level.
#[must_use]
pub fn sending_parked_entity() -> bool {
    SENDING_PARKED.with(std::cell::Cell::get)
}

/// Keeps a sub-level's parked chunk loaded with a full-chunk ticket.
fn ticket_plot_chunk(level: &pumpkin_world::level::Level, chunk: Vector2<i32>) {
    let mut loading = level
        .chunk_loading
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    loading.add_ticket(chunk, ChunkLoading::FULL_CHUNK_LEVEL);
    loading.send_change();
}

/// Every chunk the world region `min..max` spans.
fn region_chunks(min: Vector3<i32>, max: Vector3<i32>) -> Vec<Vector2<i32>> {
    let x_chunks = min.x.div_euclid(16)..=max.x.div_euclid(16);
    let z_chunks = min.z.div_euclid(16)..=max.z.div_euclid(16);
    x_chunks
        .flat_map(|x| z_chunks.clone().map(move |z| Vector2::new(x, z)))
        .collect()
}

fn find_player(players: &[Arc<Player>], uuid: Uuid) -> Option<&Arc<Player>> {
    players.iter().find(|player| player.gameprofile.id == uuid)
}

/// The plot block the pose maps to: the centre chunk's middle block at the
/// plot origin height. Sable renders every content block relative to it.
const fn plot_center_block(center_chunk: Vector2<i32>) -> Vector3<i32> {
    Vector3::new(
        center_chunk.x * 16 + 8,
        PLOT_ORIGIN_Y,
        center_chunk.y * 16 + 8,
    )
}

/// The absolute parked-world bounds of content whose extents relative to the
/// plot centre block are `bounds`.
const fn parked_bounds(center: Vector3<i32>, bounds: &[i32; 6]) -> [i32; 6] {
    [
        center.x + bounds[0],
        center.y + bounds[1],
        center.z + bounds[2],
        center.x + bounds[3],
        center.y + bounds[4],
        center.z + bounds[5],
    ]
}

/// Every chunk the content with these absolute parked bounds occupies, as in
/// Sable's full sync: one chunk packet per parked chunk.
fn content_chunks(bounds: &[i32; 6]) -> Vec<Vector2<i32>> {
    let x_chunks = bounds[0].div_euclid(16)..=bounds[3].div_euclid(16);
    let z_chunks = bounds[2].div_euclid(16)..=bounds[5].div_euclid(16);
    x_chunks
        .flat_map(|x| z_chunks.clone().map(move |z| Vector2::new(x, z)))
        .collect()
}

fn is_sable_client(player: &Player) -> bool {
    player
        .client
        .java()
        .is_some_and(|java| java.has_mod_channel(sable::DETECT_CHANNEL))
}

fn within_range(player: &Player, position: &[f64; 3]) -> bool {
    let pos = player.get_entity().pos.load();
    let dx = pos.x - position[0];
    let dy = pos.y - position[1];
    let dz = pos.z - position[2];
    dx.mul_add(dx, dy.mul_add(dy, dz * dz)) < TRACKING_RANGE * TRACKING_RANGE
}

fn encode(result: Result<Vec<u8>, pumpkin_protocol::ser::WritingError>) -> Vec<u8> {
    result.unwrap_or_default()
}

fn encode_payload(player: &Player, channel: &str, data: &[u8]) -> Bytes {
    let packet = CCustomPayload::new(channel, data);
    JavaClient::serialize_packet_for_version(&packet, player.client.java_version())
        .unwrap_or_default()
}

/// A vanilla bundle: a delimiter packet before and after its members. Every
/// packet keeps its own frame; the bundle only promises order.
fn send_bundle(player: &Player, members: &[Bytes]) {
    let Some(java) = player.client.java() else {
        return;
    };
    let Ok(delimiter) =
        JavaClient::serialize_packet_for_version(&CBundleDelimiter, player.client.java_version())
    else {
        return;
    };
    // The members go out as plain sequential packets: TCP keeps their
    // order, and the bundle delimiters bought nothing but kick bugs.
    let _ = delimiter;
    for member in members {
        java.try_enqueue_packet(member.clone());
    }
}

fn send_payload(player: &Player, channel: &str, data: &[u8]) {
    let Some(java) = player.client.java() else {
        return;
    };
    java.try_enqueue_packet(encode_payload(player, channel, data));
}

/// Sable's full sync: start tracking, one chunk packet per content chunk,
/// finalize, in a bundle.
fn send_full_sync(
    world: &World,
    player: &Player,
    level: &SubLevel,
    body: &BodySnapshot,
    tick: i32,
) {
    let version = player.client.java_version();
    let mut members = Vec::with_capacity(level.chunks.len() + 2);
    members.push(encode_payload(
        player,
        sable::START_TRACKING,
        &encode(sable::encode_start_tracking(
            level.plot_long(),
            level.uuid,
            &level.last_pose,
            &pose_from_body(body, &level.pivot),
            level.bounds,
            tick,
        )),
    ));
    for chunk in &level.chunks {
        let Some(chunk) = world.level.read_chunk_sync(chunk, Arc::clone) else {
            debug!("sublevel: plot chunk of {} is not readable yet", level.id);
            return;
        };
        let mut chunk_buf = Vec::with_capacity(32 * 1024);
        let chunk_packet = chunk_buf
            .write_var_int(&VarInt(CChunkData::to_id(version)))
            .and_then(|()| CChunkData(&chunk).write_packet_data(&mut chunk_buf, &version))
            .map(|()| Bytes::from(chunk_buf));
        match chunk_packet {
            Ok(bytes) => members.push(bytes),
            Err(error) => {
                warn!(
                    "sublevel: could not encode the plot chunk of {}: {error}",
                    level.id
                );
                return;
            }
        }
    }
    members.push(encode_payload(
        player,
        sable::FINALIZE,
        &encode(sable::encode_plot_ref(level.plot_long())),
    ));
    send_bundle(player, &members);
}

/// The wire pose: the body sits on the sub-level's pivot, so the position
/// is that pivot's world position, as in Sable's centre-of-mass poses.
fn pose_from_body(body: &BodySnapshot, pivot: &Vector3<f64>) -> sable::Pose {
    sable::Pose {
        position: [
            f64::from(body.position[0]),
            f64::from(body.position[1]),
            f64::from(body.position[2]),
        ],
        orientation: body.rotation,
        rotation_point: [pivot.x, pivot.y, pivot.z],
    }
}

fn encode_debug_create(body: &BodySnapshot, half_extents: [f32; 3]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(57);
    buf.put_u8(DEBUG_MSG_CREATE);
    buf.put_u32_le(body.id);
    put_vec3(&mut buf, &body.position);
    put_quat(&mut buf, &body.rotation);
    put_vec3(&mut buf, &half_extents);
    put_vec3(&mut buf, &body.linear_velocity);
    buf
}

fn encode_debug_transform(body: &BodySnapshot) -> Vec<u8> {
    let mut buf = Vec::with_capacity(45);
    buf.put_u8(DEBUG_MSG_TRANSFORM);
    buf.put_u32_le(body.id);
    put_vec3(&mut buf, &body.position);
    put_quat(&mut buf, &body.rotation);
    put_vec3(&mut buf, &body.linear_velocity);
    buf
}

/// Tells debug-channel clients a sub-level is gone.
#[must_use]
pub fn encode_debug_remove(id: u32) -> Vec<u8> {
    let mut buf = Vec::with_capacity(5);
    buf.put_u8(DEBUG_MSG_REMOVE);
    buf.put_u32_le(id);
    buf
}

fn put_vec3(buf: &mut Vec<u8>, values: &[f32; 3]) {
    for value in values {
        buf.put_f32_le(*value);
    }
}

fn put_quat(buf: &mut Vec<u8>, values: &[f32; 4]) {
    for value in values {
        buf.put_f32_le(*value);
    }
}

impl World {
    fn lock_sublevels(&self) -> std::sync::MutexGuard<'_, SubLevelManager> {
        self.sublevels
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// Spawns a sub-level whose plot origin sits at `pose`, and starts
    /// preparing its parked chunk. Returns the sub-level id.
    pub fn spawn_sublevel(self: &Arc<Self>, pose: Vector3<f64>, half_extents: [f32; 3]) -> u32 {
        let (id, _) = self.lock_sublevels().spawn(pose, half_extents);
        let world = Arc::clone(self);
        tokio::spawn(async move { world.prepare_sublevel_chunk(id).await });
        id
    }

    /// Loads the parked chunk of a sub-level and lays a stone platform in it
    /// with its top face at the plot origin, so the pose places it exactly.
    pub async fn prepare_sublevel_chunk(self: Arc<Self>, id: u32) {
        let Some(chunk) = self.lock_sublevels().chunk_of(id) else {
            return;
        };
        if !self.await_plot_chunks(&[chunk]).await {
            return;
        }

        let platform_y = PLOT_ORIGIN_Y - 1;
        for dx in 0..16 {
            for dz in 0..16 {
                let pos = BlockPos(Vector3::new(
                    chunk.x * 16 + dx,
                    platform_y,
                    chunk.y * 16 + dz,
                ));
                self.set_block_state(&pos, Block::STONE.default_state.id, BlockFlags::NOTIFY_ALL);
            }
        }
        self.lock_sublevels().mark_ready(id);
        info!(
            "sublevel {id} ready at plot chunk ({}, {})",
            chunk.x, chunk.y
        );
    }

    /// Steps sub-levels and broadcasts the debug channel.
    pub fn tick_sublevels(&self) {
        let debug = self.lock_sublevels().tick(self);
        if debug.is_empty() {
            return;
        }
        // Sable clients get the wireframe too: it shows the physics body
        // next to the rendered blocks, which makes alignment bugs obvious.
        for player in self.players.load().iter() {
            for data in &debug {
                send_payload(player, DEBUG_CHANNEL, data);
            }
        }
    }

    /// Removes a sub-level, telling every client that tracked it.
    pub fn remove_sublevel(&self, id: u32) -> bool {
        let removed = self.lock_sublevels().remove_and_notify(self, id);
        if removed {
            let message = encode_debug_remove(id);
            for player in self.players.load().iter() {
                if is_sable_client(player) {
                    continue;
                }
                send_payload(player, DEBUG_CHANNEL, &message);
            }
        }
        removed
    }

    /// Places an armor stand on a sub-level's platform. It keeps its parked
    /// coordinates; Sable clients relocate it with the sub-level.
    pub fn summon_sublevel_rider(self: &Arc<Self>, id: u32) -> Option<Vector3<f64>> {
        let chunk = self.lock_sublevels().chunk_of(id)?;
        let pos = Vector3::new(
            f64::from(chunk.x * 16 + 8) + 0.5,
            f64::from(PLOT_ORIGIN_Y),
            f64::from(chunk.y * 16 + 8) + 0.5,
        );
        let entity = from_type(&EntityType::ARMOR_STAND, pos, self, Uuid::new_v4());
        self.spawn_entity(entity);
        Some(pos)
    }

    /// Freezes a world region into a new sub-level: the blocks, block
    /// entities and entities are copied into a fresh plot, the region is
    /// cleared, and a physics body carries the copy from where the region
    /// was. `min`/`max` are inclusive block corners. Returns the sub-level id.
    pub fn assemble_sublevel(
        self: &Arc<Self>,
        min: Vector3<i32>,
        max: Vector3<i32>,
    ) -> Result<u32, String> {
        let (min, max) = (
            Vector3::new(min.x.min(max.x), min.y.min(max.y), min.z.min(max.z)),
            Vector3::new(min.x.max(max.x), min.y.max(max.y), min.z.max(max.z)),
        );
        let size = max - min + Vector3::new(1, 1, 1);
        if size.x > MAX_ASSEMBLE_SPAN
            || size.z > MAX_ASSEMBLE_SPAN
            || i64::from(size.x) * i64::from(size.y) * i64::from(size.z) > MAX_ASSEMBLE_VOLUME
        {
            return Err(format!(
                "region is {}x{}x{} blocks; assembly stays within \
                 {MAX_ASSEMBLE_SPAN}x{MAX_ASSEMBLE_SPAN} wide and \
                 {MAX_ASSEMBLE_VOLUME} blocks",
                size.x, size.y, size.z
            ));
        }
        if !self.is_in_build_limit(BlockPos(min)) || !self.is_in_build_limit(BlockPos(max)) {
            return Err("region reaches outside the build limit".to_string());
        }

        // The anchor is the block the pose maps onto, so the copy renders
        // exactly where the region was.
        let anchor = min
            + Vector3::new(
                (max.x - min.x) / 2,
                (max.y - min.y) / 2,
                (max.z - min.z) / 2,
            );
        let bounds = [
            min.x - anchor.x,
            min.y - anchor.y,
            min.z - anchor.z,
            max.x - anchor.x,
            max.y - anchor.y,
            max.z - anchor.z,
        ];
        // The body sits at the region's centre; the anchor is the integer
        // block under it, so the pivot gets whatever fraction is left over.
        let pose = Vector3::new(
            f64::from(min.x) + f64::from(size.x) / 2.0,
            f64::from(min.y) + f64::from(size.y) / 2.0,
            f64::from(min.z) + f64::from(size.z) / 2.0,
        );
        let pivot_offset = pose - anchor.to_f64();
        let id = self.lock_sublevels().assemble(pose, bounds, pivot_offset);
        let world = Arc::clone(self);
        tokio::spawn(async move { world.copy_region_into_plot(id, min, max).await });
        Ok(id)
    }

    /// Copies a region into a sub-level's parked chunks, parks its entities,
    /// clears the original, then networks the sub-level.
    async fn copy_region_into_plot(self: Arc<Self>, id: u32, min: Vector3<i32>, max: Vector3<i32>) {
        let (chunks, anchor, center_chunk) = {
            let manager = self.lock_sublevels();
            let Some(chunks) = manager.chunks_of(id) else {
                return;
            };
            let Some(bounds) = manager.bounds_of(id) else {
                return;
            };
            let Some(center_chunk) = manager.chunk_of(id) else {
                return;
            };
            // The bounds are absolute parked coordinates; undo their offset
            // from the plot centre to get the region's world anchor back.
            let center = plot_center_block(center_chunk);
            let anchor = Vector3::new(
                min.x + center.x - bounds[0],
                min.y + center.y - bounds[1],
                min.z + center.z - bounds[2],
            );
            (chunks, anchor, center_chunk)
        };
        // The source region's chunks must stay loaded too, or the copy reads
        // air out of chunks the server unloaded. They are only held, never
        // cleared: only the plot chunks carry parked content.
        let mut held = chunks.clone();
        held.extend(region_chunks(min, max));
        if !self.await_plot_chunks(&held).await {
            return;
        }

        // A plot chunk generates like any other chunk; empty it so the
        // sub-level carries nothing but the region.
        for chunk in &chunks {
            self.clear_parked_chunk(*chunk);
        }
        let center = plot_center_block(center_chunk);
        let copy_flags = BlockFlags::SKIP_BLOCK_ADDED_CALLBACK | BlockFlags::UPDATE_KNOWN_SHAPE;
        let mut tight: Option<[i32; 6]> = None;
        for x in min.x..=max.x {
            for y in min.y..=max.y {
                for z in min.z..=max.z {
                    let pos = BlockPos(Vector3::new(x, y, z));
                    let state = self.get_block_state_id(&pos);
                    if state == Block::AIR.default_state.id {
                        continue;
                    }
                    let parked = center + pos.0 - anchor;
                    if !self.is_in_build_limit(BlockPos(parked)) {
                        warn!(
                            "sublevel: parked copy of ({}, {}, {}) landed outside the build limit; region skipped",
                            pos.0.x, pos.0.y, pos.0.z
                        );
                        return;
                    }
                    self.set_block_state(&BlockPos(parked), state, copy_flags);
                    tight = Some(tight.map_or(
                        [parked.x, parked.y, parked.z, parked.x, parked.y, parked.z],
                        |boxed| {
                            [
                                boxed[0].min(parked.x),
                                boxed[1].min(parked.y),
                                boxed[2].min(parked.z),
                                boxed[3].max(parked.x),
                                boxed[4].max(parked.y),
                                boxed[5].max(parked.z),
                            ]
                        },
                    ));
                    if let Some(block_entity) = self.get_block_entity(&pos) {
                        self.restore_block_entity_at(&block_entity, BlockPos(parked));
                    }
                }
            }
        }

        self.park_entities_in_region(min, max, anchor, center);
        let clear_flags = BlockFlags::NOTIFY_ALL
            | BlockFlags::SKIP_DROPS
            | BlockFlags::FORCE_STATE
            | BlockFlags::SKIP_BLOCK_ENTITY_REPLACED_CALLBACK;
        for x in min.x..=max.x {
            for y in min.y..=max.y {
                for z in min.z..=max.z {
                    let pos = BlockPos(Vector3::new(x, y, z));
                    if self.get_block_state_id(&pos) != Block::AIR.default_state.id {
                        self.set_block_state(&pos, Block::AIR.default_state.id, clear_flags);
                    }
                }
            }
        }

        // Sable's bounds are tight to the content; empty margins of the
        // region must not widen the client's render or collision box.
        let mut manager = self.lock_sublevels();
        if let Some(tight) = tight {
            manager.tighten_bounds(id, tight);
        }
        manager.mark_ready(id);
        info!(
            "sublevel {id} assembled from region ({}, {}, {})..({}, {}, {})",
            min.x, min.y, min.z, max.x, max.y, max.z
        );
    }

    /// Restores a block entity at a new position: the copied NBT keeps its
    /// contents, only the coordinates change.
    fn restore_block_entity_at(&self, block_entity: &Arc<dyn BlockEntity>, position: BlockPos) {
        let mut nbt = NbtCompound::new();
        block_entity.write_internal(&mut nbt);
        nbt.put_int("x", position.0.x);
        nbt.put_int("y", position.0.y);
        nbt.put_int("z", position.0.z);
        if let Some(restored) = block_entity_from_nbt(&nbt) {
            self.add_block_entity(restored);
            self.add_block_entity_nbt(position, &nbt);
        }
    }

    /// Moves every entity standing in the region to its parked position.
    /// Players stay: their position is client authoritative.
    fn park_entities_in_region(
        &self,
        min: Vector3<i32>,
        max: Vector3<i32>,
        anchor: Vector3<i32>,
        center: Vector3<i32>,
    ) {
        let center_f64 = Vector3::new(
            f64::from(center.x),
            f64::from(center.y),
            f64::from(center.z),
        );
        let anchor_f64 = Vector3::new(
            f64::from(anchor.x),
            f64::from(anchor.y),
            f64::from(anchor.z),
        );
        for entity in self.entities.load().iter() {
            let entity = entity.get_entity();
            if entity.entity_type == &EntityType::PLAYER {
                continue;
            }
            let pos = entity.pos.load();
            let block = pos.floor_to_i32();
            if block.x < min.x
                || block.x > max.x
                || block.y < min.y
                || block.y > max.y
                || block.z < min.z
                || block.z > max.z
            {
                continue;
            }
            entity.set_pos(center_f64 + pos - anchor_f64);
        }
    }

    /// Tickets every content chunk and waits for it to generate.
    async fn await_plot_chunks(&self, chunks: &[Vector2<i32>]) -> bool {
        for chunk in chunks {
            ticket_plot_chunk(&self.level, *chunk);
        }
        let deadline = Instant::now() + PLOT_CHUNK_TIMEOUT;
        for chunk in chunks {
            while self.level.read_chunk_sync(chunk, |_| ()).is_none() {
                if Instant::now() >= deadline {
                    warn!(
                        "sublevel: plot chunk ({}, {}) did not generate",
                        chunk.x, chunk.y
                    );
                    return false;
                }
                tokio::time::sleep(Duration::from_millis(250)).await;
            }
        }
        true
    }

    /// Writes air over an entire freshly generated plot chunk.
    fn clear_parked_chunk(self: &Arc<Self>, chunk: Vector2<i32>) {
        if !SubLevelManager::is_plot_chunk(chunk) {
            warn!("sublevel: refusing to clear {chunk:?}, it is outside the plotyard");
            return;
        }
        // A generated plot chunk can contain naturally placed chests; their
        // contents must not scatter into the plot on the way out.
        let flags = BlockFlags::SKIP_BLOCK_ADDED_CALLBACK
            | BlockFlags::UPDATE_KNOWN_SHAPE
            | BlockFlags::SKIP_DROPS
            | BlockFlags::SKIP_BLOCK_ENTITY_REPLACED_CALLBACK;
        for x in 0..16 {
            for y in self.get_bottom_y()..=self.get_top_y() {
                for z in 0..16 {
                    let pos = BlockPos(Vector3::new(chunk.x * 16 + x, y, chunk.y * 16 + z));
                    if self.get_block_state_id(&pos) != Block::AIR.default_state.id {
                        self.set_block_state(&pos, Block::AIR.default_state.id, flags);
                    }
                }
            }
        }
    }

    /// Melt a sub-level back into the world at its current position: the
    /// parked blocks, block entities and entities return as real ones and
    /// the sub-level is removed. Only an unrotated sub-level can return.
    pub fn disassemble_sublevel(self: &Arc<Self>, id: u32) -> Result<u32, String> {
        let manager = self.lock_sublevels();
        let Some(bounds) = manager.bounds_of(id) else {
            return Err(format!("No sub-level {id}"));
        };
        let Some(pose) = manager.pose_of(id) else {
            return Err(format!("No sub-level {id}"));
        };
        let Some(chunks) = manager.chunks_of(id) else {
            return Err(format!("No sub-level {id}"));
        };
        let Some(center_chunk) = manager.chunk_of(id) else {
            return Err(format!("No sub-level {id}"));
        };
        drop(manager);
        // Blocks cannot be written back rotated; wait for the sub-level to
        // come level again.
        let [qx, qy, qz, qw] = pose.orientation;
        if qx.abs() > 0.001 || qy.abs() > 0.001 || qz.abs() > 0.001 || (qw - 1.0).abs() > 0.001 {
            return Err("sub-level is rotated; bring it level first".to_string());
        }
        // Rendered position = pose + (parked - rotation point), so the world
        // target of the bounds is the pose moved by their offset from the
        // pivot - the same mapping the client renders with.
        let origin = Vector3::new(
            pose.position[0] - pose.rotation_point[0],
            pose.position[1] - pose.rotation_point[1],
            pose.position[2] - pose.rotation_point[2],
        );
        let min = (origin
            + Vector3::new(
                f64::from(bounds[0]),
                f64::from(bounds[1]),
                f64::from(bounds[2]),
            ))
        .floor_to_i32();
        let max = (origin
            + Vector3::new(
                f64::from(bounds[3]),
                f64::from(bounds[4]),
                f64::from(bounds[5]),
            ))
        .floor_to_i32();
        if !self.is_in_build_limit(BlockPos(min)) || !self.is_in_build_limit(BlockPos(max)) {
            return Err("sub-level has drifted outside the build limit".to_string());
        }

        let world = Arc::clone(self);
        tokio::spawn(async move { world.copy_plot_into_region(id, chunks, min, max).await });
        Ok(id)
    }

    /// The second half of [`World::disassemble_sublevel`], after the checks.
    async fn copy_plot_into_region(
        self: Arc<Self>,
        id: u32,
        chunks: Vec<Vector2<i32>>,
        min: Vector3<i32>,
        max: Vector3<i32>,
    ) {
        // Hold the target world chunks as well: without a player nearby the
        // server unloads them, and the write-back would land nowhere.
        let mut held = chunks.clone();
        held.extend(region_chunks(min, max));
        if !self.await_plot_chunks(&held).await {
            return;
        }
        let (bounds, pivot, pose_vec) = {
            let manager = self.lock_sublevels();
            let Some(bounds) = manager.bounds_of(id) else {
                return;
            };
            let Some(pose) = manager.pose_of(id) else {
                return;
            };
            let pose_vec = Vector3::new(pose.position[0], pose.position[1], pose.position[2]);
            let pivot = Vector3::new(
                pose.rotation_point[0],
                pose.rotation_point[1],
                pose.rotation_point[2],
            );
            (bounds, pivot, pose_vec)
        };

        // The bounds are absolute parked coordinates; the world target is the
        // pose moved by their offset from the pivot - the mapping the client
        // renders with.
        let origin = pose_vec - pivot;
        let flags = BlockFlags::NOTIFY_ALL | BlockFlags::SKIP_DROPS;
        for bx in bounds[0]..=bounds[3] {
            for by in bounds[1]..=bounds[4] {
                for bz in bounds[2]..=bounds[5] {
                    let parked = BlockPos(Vector3::new(bx, by, bz));
                    let state = self.get_block_state_id(&parked);
                    if state == Block::AIR.default_state.id {
                        continue;
                    }
                    let world_pos = BlockPos(Vector3::new(
                        (origin.x + f64::from(parked.0.x)).floor() as i32,
                        (origin.y + f64::from(parked.0.y)).floor() as i32,
                        (origin.z + f64::from(parked.0.z)).floor() as i32,
                    ));
                    self.set_block_state(&world_pos, state, flags);
                    if let Some(block_entity) = self.get_block_entity(&parked) {
                        self.restore_block_entity_at(&block_entity, world_pos);
                    }
                }
            }
        }

        // Entities keep fractional positions; move them by the same offset
        // the blocks took.
        let parked_min = Vector3::new(bounds[0], bounds[1], bounds[2]);
        let parked_max = Vector3::new(bounds[3], bounds[4], bounds[5]);
        for entity in self.entities.load().iter() {
            let entity = entity.get_entity();
            let pos = entity.pos.load();
            let block = pos.floor_to_i32();
            if block.x < parked_min.x
                || block.x > parked_max.x
                || block.y < parked_min.y
                || block.y > parked_max.y
                || block.z < parked_min.z
                || block.z > parked_max.z
            {
                continue;
            }
            entity.set_pos(pose_vec + pos - pivot);
        }

        // Leave the plot empty for whoever reuses it.
        for bx in bounds[0]..=bounds[3] {
            for by in bounds[1]..=bounds[4] {
                for bz in bounds[2]..=bounds[5] {
                    let parked = BlockPos(Vector3::new(bx, by, bz));
                    self.set_block_state(
                        &parked,
                        Block::AIR.default_state.id,
                        BlockFlags::SKIP_BLOCK_ADDED_CALLBACK
                            | BlockFlags::UPDATE_KNOWN_SHAPE
                            | BlockFlags::SKIP_DROPS
                            | BlockFlags::SKIP_BLOCK_ENTITY_REPLACED_CALLBACK,
                    );
                }
            }
        }
        if self.remove_sublevel(id) {
            info!(
                "sublevel {id} disassembled into region ({}, {}, {})..({}, {}, {})",
                min.x, min.y, min.z, max.x, max.y, max.z
            );
        }
    }

    /// Whether `player` tracks the sub-level whose plot contains `chunk`.
    pub fn player_tracks_sublevel_chunk(&self, player: Uuid, chunk: Vector2<i32>) -> bool {
        self.lock_sublevels().player_tracks_chunk(player, chunk)
    }

    /// Converts a player-reported position in sub-level-local (parked)
    /// coordinates to its world position, if it lies in the plotyard.
    #[must_use]
    pub fn to_world_position(&self, position: Vector3<f64>) -> Vector3<f64> {
        self.lock_sublevels()
            .parked_to_world(position)
            .unwrap_or(position)
    }

    /// Whether the entity stands in the plotyard, i.e. inside some
    /// sub-level's parked content.
    pub fn is_parked_in_sublevel(&self, entity: &crate::entity::Entity) -> bool {
        SubLevelManager::is_plot_chunk(entity.chunk_pos.load())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plots_are_parked_inside_the_plotyard() {
        let mut manager = SubLevelManager::new();
        let (first, first_chunk) = manager.spawn(Vector3::new(0.0, 64.0, 0.0), [8.0, 0.5, 8.0]);
        let (second, second_chunk) = manager.spawn(Vector3::new(0.0, 64.0, 0.0), [8.0, 0.5, 8.0]);
        assert_ne!(first, second);

        // The first plot's centre chunk sits 64 chunks into the plotyard.
        let corner = PLOTYARD_ORIGIN << PLOT_LOG_SIZE;
        assert_eq!(first_chunk, Vector2::new(corner + 64, corner + 64));
        // The second allocation takes the next plot over.
        assert_eq!(second_chunk, Vector2::new(corner + 128 + 64, corner + 64));
        // The platform fills its chunk's middle 16x16, one layer below the
        // plot origin, in absolute parked coordinates.
        let base_x = (corner + 128 + 64) * 16;
        let base_z = (corner + 64) * 16;
        assert_eq!(
            manager.bounds_of(second),
            Some([
                base_x,
                PLOT_ORIGIN_Y - 1,
                base_z,
                base_x + 15,
                PLOT_ORIGIN_Y - 1,
                base_z + 15,
            ])
        );

        assert!(manager.remove(first));
        let (third, _) = manager.spawn(Vector3::new(0.0, 64.0, 0.0), [8.0, 0.5, 8.0]);
        assert_eq!(third, first + 2);
    }

    #[test]
    fn plotyard_chunks_are_recognised() {
        let corner = PLOTYARD_ORIGIN << PLOT_LOG_SIZE;
        assert!(SubLevelManager::is_plot_chunk(Vector2::new(corner, corner)));
        assert!(SubLevelManager::is_plot_chunk(Vector2::new(
            corner + 64,
            corner + 64
        )));
        assert!(!SubLevelManager::is_plot_chunk(Vector2::new(
            corner - 1,
            corner
        )));
        assert!(!SubLevelManager::is_plot_chunk(Vector2::new(0, 0)));
    }

    #[test]
    fn assembled_content_covers_its_bounds_with_chunks() {
        let center = 1280064 * 16 + 8;
        // A 16x16x1 platform around the plot centre sits in one chunk.
        assert_eq!(
            content_chunks(&[
                center - 8,
                PLOT_ORIGIN_Y - 1,
                center - 8,
                center + 7,
                PLOT_ORIGIN_Y - 1,
                center + 7
            ]),
            vec![Vector2::new(1280064, 1280064)]
        );
        // A 33-block-wide region spans three chunks per axis, centred on the
        // plot's middle chunk.
        let chunks = content_chunks(&[center - 16, 0, center - 16, center + 16, 0, center + 16]);
        assert_eq!(chunks.len(), 9);
        // Bounds crossing a chunk border by one block cover both chunks.
        let straddling = content_chunks(&[center + 7, 0, center + 7, center + 8, 0, center + 8]);
        assert!(straddling.contains(&Vector2::new(1280064, 1280064)));
        assert!(straddling.contains(&Vector2::new(1280065, 1280065)));
        assert!(chunks.contains(&Vector2::new(1280063, 1280063)));
        assert!(chunks.contains(&Vector2::new(1280065, 1280065)));
    }

    #[test]
    fn bounds_travel_between_frames() {
        let center = Vector3::new(20481032, PLOT_ORIGIN_Y, 20481032);
        // Region-relative extents become absolute parked bounds.
        assert_eq!(
            parked_bounds(center, &[-2, -2, -2, 2, 3, 2]),
            [20481030, 126, 20481030, 20481034, 131, 20481034]
        );
    }

    #[test]
    fn assembled_regions_render_where_they_stood() {
        // assemble(): the anchor maps onto the plot centre, so a world block
        // and its parked copy render at the same place.
        let min = Vector3::new(-40, 70, -30);
        let max = Vector3::new(-9, 75, -25);
        let anchor = min
            + Vector3::new(
                (max.x - min.x) / 2,
                (max.y - min.y) / 2,
                (max.z - min.z) / 2,
            );
        let mut manager = SubLevelManager::new();
        let pose = Vector3::new(
            f64::from(anchor.x),
            f64::from(anchor.y),
            f64::from(anchor.z),
        );
        let id = manager.assemble(
            pose,
            [
                min.x - anchor.x,
                min.y - anchor.y,
                min.z - anchor.z,
                max.x - anchor.x,
                max.y - anchor.y,
                max.z - anchor.z,
            ],
            pose - anchor.to_f64(),
        );

        let bounds = manager.bounds_of(id).expect("assembled sub-level exists");
        let center_chunk = manager.chunk_of(id).expect("plot centre exists");
        let center = plot_center_block(center_chunk);
        // The bounds are the region's parked copy, in absolute coordinates.
        assert_eq!(
            bounds,
            [
                center.x - 15,
                center.y - 2,
                center.z - 2,
                center.x + 16,
                center.y + 3,
                center.z + 3,
            ]
        );
        // Round trip every corner: world -> parked -> rendered == world.
        for corner in [min, max, anchor] {
            let parked = center + corner - anchor;
            let rendered = Vector3::new(
                pose.x + f64::from(parked.x - center.x),
                pose.y + f64::from(parked.y - center.y),
                pose.z + f64::from(parked.z - center.z),
            );
            assert_eq!(rendered, corner.to_f64());
        }
    }

    #[test]
    fn debug_layouts_match_the_client_mod() {
        let body = BodySnapshot {
            id: 7,
            position: [1.0, 64.0, -2.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            linear_velocity: [0.5, 0.0, 0.0],
            angular_velocity: [0.0; 3],
        };
        let create = encode_debug_create(&body, [8.0, 0.5, 8.0]);
        assert_eq!(create.len(), 57);
        assert_eq!(create[0], DEBUG_MSG_CREATE);
        assert_eq!(&create[1..5], &[7, 0, 0, 0]);
        assert_eq!(&create[5..9], &1.0f32.to_le_bytes());

        let transform = encode_debug_transform(&body);
        assert_eq!(transform.len(), 45);
        assert_eq!(transform[0], DEBUG_MSG_TRANSFORM);

        assert_eq!(encode_debug_remove(9), [DEBUG_MSG_REMOVE, 9, 0, 0, 0]);
    }

    #[test]
    fn parked_entities_are_visible_only_to_trackers() {
        let mut manager = SubLevelManager::new();
        let (id, chunk) = manager.spawn(Vector3::new(0.0, 64.0, 0.0), [8.0, 0.5, 8.0]);
        let (other, other_chunk) = manager.spawn(Vector3::new(0.0, 64.0, 0.0), [8.0, 0.5, 8.0]);
        let tracker = Uuid::from_u128(1);
        manager.mark_ready(id);
        manager.mark_ready(other);
        manager.levels[0].tracking.insert(tracker);

        assert!(manager.player_tracks_chunk(tracker, chunk));
        // A different plot in the same plotyard is not this sub-level.
        assert!(!manager.player_tracks_chunk(tracker, other_chunk));
        // Outside the plotyard nothing is a sub-level.
        assert!(!manager.player_tracks_chunk(tracker, Vector2::new(0, 0)));
        assert!(!manager.player_tracks_chunk(Uuid::from_u128(2), chunk));
    }
}
