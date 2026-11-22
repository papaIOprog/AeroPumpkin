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
    /// The parked chunk holding the content.
    chunk: Vector2<i32>,
    /// Plot-local block bounds of the content, inclusive.
    bounds: [i32; 6],
    half_extents: [f32; 3],
    /// Set once the parked chunk holds the content and can be sent.
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
        // Block indices of the content in plot space: the box spans
        // `[-half, +half)` on x and z, and its top face is the plot origin.
        let half_x = half_extents[0].ceil() as i32;
        let height = (2.0 * half_extents[1]).ceil() as i32;
        self.levels.push(SubLevel {
            id,
            uuid: Uuid::new_v4(),
            plot,
            chunk: center,
            bounds: [-half_x, -height, -half_x, half_x - 1, -1, half_x - 1],
            half_extents,
            ready: false,
            last_pose: sable::Pose::default(),
            networked_pose: sable::Pose::default(),
            stop_sent: false,
            tracking: FxHashSet::default(),
        });
        (id, center)
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

    /// The plot-local block bounds of sub-level `id`.
    #[cfg(test)]
    #[must_use]
    pub fn bounds_of(&self, id: u32) -> Option<[i32; 6]> {
        self.levels
            .iter()
            .find(|level| level.id == id)
            .map(|level| level.bounds)
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
        let mut syncs: Vec<(u32, Uuid)> = Vec::new();
        let mut untracks: Vec<(i64, Uuid)> = Vec::new();
        for body in bodies {
            let pose = pose_from_body(body);
            let Some(level) = self.levels.iter_mut().find(|level| level.id == body.id) else {
                continue;
            };
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

fn find_player(players: &[Arc<Player>], uuid: Uuid) -> Option<&Arc<Player>> {
    players.iter().find(|player| player.gameprofile.id == uuid)
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

/// A vanilla bundle: the delimiter packet before and after its members.
fn send_bundle(player: &Player, members: &[Bytes]) {
    let Some(java) = player.client.java() else {
        return;
    };
    let Ok(delimiter) =
        JavaClient::serialize_packet_for_version(&CBundleDelimiter, player.client.java_version())
    else {
        return;
    };
    let mut bundle =
        Vec::with_capacity(delimiter.len() * 2 + members.iter().map(Bytes::len).sum::<usize>());
    bundle.put_slice(&delimiter);
    for member in members {
        bundle.put_slice(member);
    }
    bundle.put_slice(&delimiter);
    java.try_enqueue_packet(Bytes::from(bundle));
}

fn send_payload(player: &Player, channel: &str, data: &[u8]) {
    let Some(java) = player.client.java() else {
        return;
    };
    java.try_enqueue_packet(encode_payload(player, channel, data));
}

/// Sable's full sync: start tracking, the content chunk, finalize, in a bundle.
fn send_full_sync(
    world: &World,
    player: &Player,
    level: &SubLevel,
    body: &BodySnapshot,
    tick: i32,
) {
    let Some(chunk) = world.level.read_chunk_sync(&level.chunk, Arc::clone) else {
        debug!("sublevel: plot chunk of {} is not readable yet", level.id);
        return;
    };
    let version = player.client.java_version();
    let mut chunk_buf = Vec::with_capacity(32 * 1024);
    let chunk_packet = chunk_buf
        .write_var_int(&VarInt(CChunkData::to_id(version)))
        .and_then(|()| CChunkData(&chunk).write_packet_data(&mut chunk_buf, &version))
        .map(|()| Bytes::from(chunk_buf));
    let chunk_packet = match chunk_packet {
        Ok(bytes) => bytes,
        Err(error) => {
            warn!(
                "sublevel: could not encode the plot chunk of {}: {error}",
                level.id
            );
            return;
        }
    };
    send_bundle(
        player,
        &[
            encode_payload(
                player,
                sable::START_TRACKING,
                &encode(sable::encode_start_tracking(
                    level.plot_long(),
                    level.uuid,
                    &level.last_pose,
                    &pose_from_body(body),
                    level.bounds,
                    tick,
                )),
            ),
            chunk_packet,
            encode_payload(
                player,
                sable::FINALIZE,
                &encode(sable::encode_plot_ref(level.plot_long())),
            ),
        ],
    );
}

fn pose_from_body(body: &BodySnapshot) -> sable::Pose {
    sable::Pose {
        position: [
            f64::from(body.position[0]),
            f64::from(body.position[1]),
            f64::from(body.position[2]),
        ],
        orientation: body.rotation,
        rotation_point: [0.0; 3],
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
        let level = &self.level;
        ticket_plot_chunk(level, chunk);

        let deadline = Instant::now() + PLOT_CHUNK_TIMEOUT;
        while level.read_chunk_sync(&chunk, |_| ()).is_none() {
            if Instant::now() >= deadline {
                warn!(
                    "sublevel {id}: plot chunk ({}, {}) did not generate",
                    chunk.x, chunk.y
                );
                return;
            }
            tokio::time::sleep(Duration::from_millis(250)).await;
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
        for player in self.players.load().iter() {
            if is_sable_client(player) {
                continue;
            }
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

    /// Whether `player` tracks the sub-level whose plot contains `chunk`.
    pub fn player_tracks_sublevel_chunk(&self, player: Uuid, chunk: Vector2<i32>) -> bool {
        self.lock_sublevels().player_tracks_chunk(player, chunk)
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
        // A one-block platform spans block y -1 only.
        assert_eq!(manager.bounds_of(second), Some([-8, -1, -8, 7, -1, 7]));

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
