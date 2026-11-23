//! Server-side sub-levels: plots of real chunks parked at remote coordinates,
//! placed in the world by a physics body.
//!
//! The model and the wire protocol follow Sable so that the Sable client mod
//! renders Pumpkin's sub-levels: a sub-level's content is a set of ordinary
//! chunks at a "plot" coordinate, streamed with vanilla chunk packets, and only
//! the pose travels over custom payloads. Entities keep their parked
//! coordinates; Sable clients relocate anything inside a tracked plot. Sable
//! is the only client the sub-level system supports.
//!
//! Everything here runs sequentially on the world tick thread; the physics
//! step never touches rayon (see `pumpkin-sublevel`).

pub mod aero;
pub mod sable;

use std::sync::Arc;
use std::time::{Duration, Instant};

use bytes::Bytes;
use pumpkin_nbt::compound::NbtCompound;
use pumpkin_protocol::codec::var_int::VarInt;
use pumpkin_protocol::java::client::play::{CBundleDelimiter, CCustomPayload, CParticle};
use pumpkin_protocol::ser::NetworkWriteExt;
use pumpkin_protocol::{ClientPacket, MultiVersionJavaPacket};
use pumpkin_sublevel::{
    AeroInfo, Air, BodySnapshot, Content, Impact, ImpactTarget, Step, SurroundingsRequest, Voxel,
};
use pumpkin_util::math::vector2::Vector2;
use pumpkin_util::math::vector3::Vector3;
use rustc_hash::{FxHashMap, FxHashSet};
use tracing::{debug, info, warn};
use uuid::Uuid;

use super::World;
use super::block_entity_from_nbt;
use crate::block::blocks::ice;
use crate::block::blocks::redstone::bell::ring_bell;
use crate::block::entities::BlockEntity;
use crate::entity::EntityBase;
use crate::entity::player::Player;
use crate::entity::tnt::TNTEntity;
use crate::net::java::JavaClient;
use crate::net::java::chunk_data::CChunkData;
use pumpkin_data::block_properties::{HorizontalFacing, blocks_movement};
use pumpkin_data::entity::EntityType;
use pumpkin_data::sound::{Sound, SoundCategory};
use pumpkin_data::tag::{self, Taggable};
use pumpkin_data::{Block, BlockDirection};
use pumpkin_util::math::position::BlockPos;
use pumpkin_world::chunk_system::ChunkLoading;
use pumpkin_world::world::BlockFlags;
use rand::RngExt;

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
    /// Where the body sits relative to the region's anchor block when the
    /// region was assembled, in blocks. The physics frames the content by it.
    from_anchor: Vector3<f64>,
    /// Plot-local block bounds of the content, inclusive.
    bounds: [i32; 6],
    /// Set once the parked chunks hold the content and can be sent.
    ready: bool,
    last_pose: sable::Pose,
    networked_pose: sable::Pose,
    stop_sent: bool,
    tracking: FxHashSet<Uuid>,
    /// Terrain chunks held loaded around the body for its surroundings.
    surroundings_chunks: Vec<Vector2<i32>>,
    /// The world region being copied into the plot, inclusive corners,
    /// while the copy runs: its blocks are still in the world but are this
    /// sub-level's already, so nothing may sample them as ground.
    source: Option<[[i32; 3]; 2]>,
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

/// What copying a world region into its plot produced.
struct ParkedCopy {
    /// Parked bounds tight to the copied content, if it copied anything.
    bounds: Option<[i32; 6]>,
}

/// The side a bell was hit from: the horizontal direction from its centre
/// to the hit.
fn hit_side(from_centre: Vector3<f64>) -> HorizontalFacing {
    if from_centre.x.abs() > from_centre.z.abs() {
        if from_centre.x > 0.0 {
            HorizontalFacing::East
        } else {
            HorizontalFacing::West
        }
    } else if from_centre.z > 0.0 {
        HorizontalFacing::South
    } else {
        HorizontalFacing::North
    }
}

/// How a block family sounds. Vanilla gives every block one of its
/// `SoundType`s in code; Pumpkin has no such table, so the families here
/// come from the block's tags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SoundFamily {
    Stone,
    Wood,
    Wool,
    Sand,
    Gravel,
    Glass,
    Grass,
    Metal,
    Slime,
    Snow,
}

impl SoundFamily {
    fn of(block: &Block) -> Self {
        use tag::Block::{
            C_GLASS_BLOCKS, C_GRAVELS, MINECRAFT_ANVIL, MINECRAFT_CONCRETE_POWDER, MINECRAFT_DIRT,
            MINECRAFT_ICE, MINECRAFT_LEAVES, MINECRAFT_LOGS, MINECRAFT_PLANKS, MINECRAFT_RAILS,
            MINECRAFT_SAND, MINECRAFT_SNOW, MINECRAFT_WOODEN_SLABS, MINECRAFT_WOODEN_STAIRS,
            MINECRAFT_WOOL, MINECRAFT_WOOL_CARPETS,
        };
        if block.has_tag(&MINECRAFT_PLANKS)
            || block.has_tag(&MINECRAFT_LOGS)
            || block.has_tag(&MINECRAFT_WOODEN_SLABS)
            || block.has_tag(&MINECRAFT_WOODEN_STAIRS)
        {
            Self::Wood
        } else if block.has_tag(&MINECRAFT_WOOL) || block.has_tag(&MINECRAFT_WOOL_CARPETS) {
            Self::Wool
        } else if block.has_tag(&MINECRAFT_SAND) || block.has_tag(&MINECRAFT_CONCRETE_POWDER) {
            Self::Sand
        } else if block.has_tag(&MINECRAFT_SNOW) {
            Self::Snow
        } else if block.has_tag(&C_GRAVELS) || block.has_tag(&MINECRAFT_DIRT) {
            Self::Gravel
        } else if block.has_tag(&C_GLASS_BLOCKS) || block.has_tag(&MINECRAFT_ICE) {
            Self::Glass
        } else if block.has_tag(&MINECRAFT_LEAVES) || *block == Block::GRASS_BLOCK {
            Self::Grass
        } else if block.has_tag(&MINECRAFT_ANVIL) || block.has_tag(&MINECRAFT_RAILS) {
            Self::Metal
        } else if *block == Block::SLIME_BLOCK {
            Self::Slime
        } else {
            Self::Stone
        }
    }

    const fn step(self) -> Sound {
        match self {
            Self::Stone => Sound::BlockStoneStep,
            Self::Wood => Sound::BlockWoodStep,
            Self::Wool => Sound::BlockWoolStep,
            Self::Sand => Sound::BlockSandStep,
            Self::Gravel => Sound::BlockGravelStep,
            Self::Glass => Sound::BlockGlassStep,
            Self::Grass => Sound::BlockGrassStep,
            Self::Metal => Sound::BlockMetalStep,
            Self::Slime => Sound::BlockSlimeBlockStep,
            Self::Snow => Sound::BlockSnowStep,
        }
    }

    const fn hit(self) -> Sound {
        match self {
            Self::Stone => Sound::BlockStoneHit,
            Self::Wood => Sound::BlockWoodHit,
            Self::Wool => Sound::BlockWoolHit,
            Self::Sand => Sound::BlockSandHit,
            Self::Gravel => Sound::BlockGravelHit,
            Self::Glass => Sound::BlockGlassHit,
            Self::Grass => Sound::BlockGrassHit,
            Self::Metal => Sound::BlockMetalHit,
            Self::Slime => Sound::BlockSlimeBlockHit,
            Self::Snow => Sound::BlockSnowHit,
        }
    }

    const fn place(self) -> Sound {
        match self {
            Self::Stone => Sound::BlockStonePlace,
            Self::Wood => Sound::BlockWoodPlace,
            Self::Wool => Sound::BlockWoolPlace,
            Self::Sand => Sound::BlockSandPlace,
            Self::Gravel => Sound::BlockGravelPlace,
            Self::Glass => Sound::BlockGlassPlace,
            Self::Grass => Sound::BlockGrassPlace,
            Self::Metal => Sound::BlockMetalPlace,
            Self::Slime => Sound::BlockSlimeBlockPlace,
            Self::Snow => Sound::BlockSnowPlace,
        }
    }
}

/// Whether a block's aerodynamic role depends on the redstone reaching it.
fn needs_signal(block: &Block) -> bool {
    block.has_tag(&tag::Block::MINECRAFT_CAMPFIRES)
        || *block == Block::PISTON
        || *block == Block::STICKY_PISTON
}

pub struct SubLevelManager {
    physics: pumpkin_sublevel::SubLevelSystem,
    levels: Vec<SubLevel>,
    /// Sable's interpolation tick, advanced once per server tick.
    tick: i32,
    last_send: Option<Instant>,
    /// Players still receiving per-tick snapshot info.
    updating: FxHashSet<Uuid>,
    /// Sub-levels whose parked blocks changed since their last scan.
    dirty: FxHashSet<u32>,
    /// Blocks of sub-levels that sounded on their last knock, and when.
    recent_sounds: FxHashMap<(u32, [i32; 3]), i32>,
}

impl Default for SubLevelManager {
    fn default() -> Self {
        Self::new()
    }
}

impl SubLevelManager {
    /// Manages sub-levels flying in the overworld's air.
    #[must_use]
    pub fn new() -> Self {
        Self::with_air(Air::default())
    }

    /// Manages sub-levels flying in the air of a dimension whose sea level
    /// and build limit are `sea_level` and `top`.
    #[must_use]
    pub fn with_air(air: Air) -> Self {
        Self {
            physics: pumpkin_sublevel::SubLevelSystem::new(air),
            levels: Vec::new(),
            tick: 0,
            last_send: None,
            updating: FxHashSet::default(),
            dirty: FxHashSet::default(),
            recent_sounds: FxHashMap::default(),
        }
    }

    /// Allocates a plot and a body for a `width` by `width` stone slab whose
    /// top face sits at `pose`, the plot origin's world position. The slab is
    /// laid into the parked chunk asynchronously by
    /// [`World::prepare_sublevel_chunk`]; until then the sub-level is not
    /// networked. Returns the new id and the parked chunk.
    pub fn spawn(&mut self, pose: Vector3<f64>, width: i32) -> (u32, Vector2<i32>) {
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
        let id = self.physics.spawn([pose_f32.x, pose_f32.y, pose_f32.z]);
        // The slab is centred on the plot centre block with its top face at
        // the plot origin. Bounds are absolute parked-world block
        // coordinates, tight to the content and inclusive, as in Sable: the
        // client sizes its render sections and collision box from them.
        let pivot = plot_center_block(center);
        let half = width / 2;
        self.levels.push(SubLevel {
            id,
            uuid: Uuid::new_v4(),
            plot,
            chunk: center,
            chunks: vec![center],
            pivot: pivot.to_f64(),
            from_anchor: Vector3::new(0.0, 0.0, 0.0),
            bounds: [
                pivot.x - half,
                PLOT_ORIGIN_Y - 1,
                pivot.z - half,
                pivot.x - half + width - 1,
                PLOT_ORIGIN_Y - 1,
                pivot.z - half + width - 1,
            ],
            ready: false,
            last_pose: sable::Pose::default(),
            networked_pose: sable::Pose::default(),
            stop_sent: false,
            tracking: FxHashSet::default(),
            surroundings_chunks: Vec::new(),
            source: None,
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
        source: [[i32; 3]; 2],
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
        let pose_f32 = pose.to_f32_lossy();
        let id = self.physics.spawn([pose_f32.x, pose_f32.y, pose_f32.z]);
        self.levels.push(SubLevel {
            id,
            uuid: Uuid::new_v4(),
            plot,
            chunk: center,
            chunks: content_chunks(&parked),
            pivot,
            from_anchor: pivot_offset,
            bounds: parked,
            ready: false,
            last_pose: sable::Pose::default(),
            networked_pose: sable::Pose::default(),
            stop_sent: false,
            tracking: FxHashSet::default(),
            surroundings_chunks: Vec::new(),
            source: Some(source),
        });
        id
    }

    /// Notes that sub-level `id`'s region is copied: its blocks are out of
    /// the world, or staying in it for good.
    pub fn end_assembly(&mut self, id: u32) {
        if let Some(level) = self.levels.iter_mut().find(|level| level.id == id) {
            level.source = None;
        }
    }

    /// The world regions still being copied into plots, inclusive corners.
    #[must_use]
    pub fn assembling_regions(&self) -> Vec<[[i32; 3]; 2]> {
        self.levels
            .iter()
            .filter_map(|level| level.source)
            .collect()
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

    /// The parked block the body of sub-level `id` sits on.
    #[must_use]
    pub fn pivot_of(&self, id: u32) -> Option<Vector3<f64>> {
        self.levels
            .iter()
            .find(|level| level.id == id)
            .map(|level| level.pivot)
    }

    /// Where the body of sub-level `id` sat relative to its region's anchor
    /// block when the region was assembled.
    #[must_use]
    pub fn from_anchor_of(&self, id: u32) -> Option<Vector3<f64>> {
        self.levels
            .iter()
            .find(|level| level.id == id)
            .map(|level| level.from_anchor)
    }

    /// The parked block that was the region's anchor, the block the
    /// content's grid coordinates count from.
    #[must_use]
    pub fn anchor_of(&self, id: u32) -> Option<Vector3<f64>> {
        self.levels
            .iter()
            .find(|level| level.id == id)
            .map(|level| level.pivot - level.from_anchor)
    }

    /// Schedules a rescan of every ready sub-level whose parked content
    /// contains `pos`, after a block there was edited, and a new sample of
    /// the world for every body whose surroundings contain it.
    pub fn note_edit(&mut self, pos: &BlockPos) {
        let [x, y, z] = [pos.0.x, pos.0.y, pos.0.z];
        for level in &mut self.levels {
            let [x0, y0, z0, x1, y1, z1] = level.bounds;
            if level.ready && x >= x0 && x <= x1 && y >= y0 && y <= y1 && z >= z0 && z <= z1 {
                self.dirty.insert(level.id);
            }
        }
        self.physics.note_edit([x, y, z]);
    }

    pub const fn is_empty(&self) -> bool {
        self.levels.is_empty()
    }

    pub fn take_dirty(&mut self) -> Vec<u32> {
        self.dirty.drain().collect()
    }

    /// Whether sub-level `id`'s block `block` may sound its knock on tick
    /// `tick`: not if it sounded within the last `quiet` ticks.
    pub fn note_sound(&mut self, id: u32, block: [i32; 3], tick: i32, quiet: i32) -> bool {
        self.recent_sounds
            .retain(|_, sounded| tick - *sounded < quiet);
        self.recent_sounds
            .insert((id, block), tick)
            .is_none_or(|sounded| tick - sounded >= quiet)
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
        let pose = &level.last_pose;
        // The same mapping the client renders with: the pivot's world
        // position plus the offset from it, turned by the pose.
        let offset = rotate(pose.orientation, position - pivot);
        Some(Vector3::new(
            pose.position[0] + offset.x,
            pose.position[1] + offset.y,
            pose.position[2] + offset.z,
        ))
    }

    /// Hands sub-level `id` its content once its blocks are parked; the
    /// body then takes their mass and shape and waits for its surroundings.
    pub fn set_content(&mut self, id: u32, content: Content) -> bool {
        self.physics.set_content(id, content)
    }

    /// Delivers the sampled world for sub-level `id`'s pending request.
    pub fn set_surroundings(
        &mut self,
        id: u32,
        request: SurroundingsRequest,
        voxels: &[Voxel],
    ) -> bool {
        self.physics.set_surroundings(id, request, voxels)
    }

    /// Notes that sub-level `id`'s pending request came to nothing.
    pub fn cancel_surroundings(&mut self, id: u32) {
        self.physics.cancel_surroundings(id);
    }

    /// Scales how hard sub-level `id`'s propellers push, from 0 to 1.
    pub fn set_throttle(&mut self, id: u32, throttle: f32) -> bool {
        self.physics.set_throttle(id, throttle)
    }

    /// What the air does to sub-level `id`.
    #[must_use]
    pub fn aero_info(&self, id: u32) -> Option<AeroInfo> {
        self.physics.aero_info(id)
    }

    /// The terrain chunks held loaded around sub-level `id`.
    #[must_use]
    pub fn surroundings_chunks_of(&self, id: u32) -> Option<Vec<Vector2<i32>>> {
        self.levels
            .iter()
            .find(|level| level.id == id)
            .map(|level| level.surroundings_chunks.clone())
    }

    pub fn set_surroundings_chunks(&mut self, id: u32, chunks: Vec<Vector2<i32>>) {
        if let Some(level) = self.levels.iter_mut().find(|level| level.id == id) {
            level.surroundings_chunks = chunks;
        }
    }

    /// Forgets sub-level `id`, dropping its terrain chunks with it.
    pub fn forget_surroundings_chunks(&mut self, id: u32) -> Option<Vec<Vector2<i32>>> {
        self.levels
            .iter_mut()
            .find(|level| level.id == id)
            .map(|level| std::mem::take(&mut level.surroundings_chunks))
    }

    /// The parked position of a block of sub-level `id`'s content, given in
    /// grid coordinates counted from its anchor block.
    #[must_use]
    pub fn parked_block(&self, id: u32, block: [i32; 3]) -> Option<BlockPos> {
        let level = self.levels.iter().find(|level| level.id == id)?;
        let anchor = (level.pivot - level.from_anchor).floor_to_i32();
        Some(BlockPos(
            anchor + Vector3::new(block[0], block[1], block[2]),
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

    /// Steps the simulation, returning the world the bodies now need
    /// around them and what the step produced. Runs from the world tick;
    /// the lock is free afterwards so the host can act on both.
    pub fn step_physics(&mut self) -> (Vec<(u32, SurroundingsRequest)>, Step) {
        let requests = self.physics.take_surroundings_requests();
        (requests, self.physics.step())
    }

    /// Syncs every client to the bodies after a step. Runs from the world
    /// tick, before the entity tracker.
    pub fn sync(&mut self, world: &Arc<World>, bodies: &[BodySnapshot]) {
        self.tick = self.tick.wrapping_add(1);
        let ms_since_last = self.note_send_time();

        let players: Vec<Arc<Player>> = world.players.load().to_vec();
        self.update_tracking(world, bodies, &players);
        self.send_movement(bodies, &players, ms_since_last);
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

    /// Tells players the sub-level is gone, lets its terrain go, then
    /// removes it.
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
        if let Some(chunks) = self.forget_surroundings_chunks(id) {
            for chunk in chunks {
                unticket_chunk(&world.level, chunk);
            }
        }
        self.remove(id)
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

/// Keeps a chunk loaded with a full-chunk ticket.
fn ticket_chunk(level: &pumpkin_world::level::Level, chunk: Vector2<i32>) {
    let mut loading = level
        .chunk_loading
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    loading.add_ticket(chunk, ChunkLoading::FULL_CHUNK_LEVEL);
    loading.send_change();
}

/// Gives up the full-chunk ticket [`ticket_chunk`] took on a chunk.
fn unticket_chunk(level: &pumpkin_world::level::Level, chunk: Vector2<i32>) {
    let mut loading = level
        .chunk_loading
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    loading.remove_ticket(chunk, ChunkLoading::FULL_CHUNK_LEVEL);
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

/// Turns `vector` by the quaternion `(x, y, z, w)`.
fn rotate(quaternion: [f32; 4], vector: Vector3<f64>) -> Vector3<f64> {
    let [x, y, z, w] = quaternion.map(f64::from);
    let axis = Vector3::new(x, y, z);
    let twice = axis.cross(&vector) * 2.0;
    vector.add(&(twice * w)).add(&axis.cross(&twice))
}

impl World {
    fn lock_sublevels(&self) -> std::sync::MutexGuard<'_, SubLevelManager> {
        self.sublevels
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// Spawns a stone slab sub-level, `width` blocks on a side and one block
    /// thick, whose top face sits at `pose`, and starts preparing its parked
    /// chunk. Returns the sub-level id.
    pub fn spawn_sublevel(self: &Arc<Self>, pose: Vector3<f64>, width: i32) -> u32 {
        let (id, _) = self.lock_sublevels().spawn(pose, width);
        let world = Arc::clone(self);
        tokio::spawn(async move { world.prepare_sublevel_chunk(id, width).await });
        id
    }

    /// Loads the parked chunk of a slab sub-level, lays the stone in it with
    /// its top face at the plot origin, so the pose places it exactly, and
    /// hands the slab to the physics as its content.
    pub async fn prepare_sublevel_chunk(self: Arc<Self>, id: u32, width: i32) {
        let (chunk, pivot) = {
            let manager = self.lock_sublevels();
            let Some(chunk) = manager.chunk_of(id) else {
                return;
            };
            let Some(pivot) = manager.pivot_of(id) else {
                return;
            };
            (chunk, pivot)
        };
        if !self.await_plot_chunks(&[chunk]).await {
            return;
        }

        let slab_y = PLOT_ORIGIN_Y - 1;
        let half = width / 2;
        let stone = Block::STONE.default_state.id;
        let flags = BlockFlags::NOTIFY_ALL;
        let mut voxels = Vec::with_capacity((width * width) as usize);
        for dx in 0..width {
            for dz in 0..width {
                let x = pivot.x as i32 - half + dx;
                let z = pivot.z as i32 - half + dz;
                let pos = BlockPos(Vector3::new(x, slab_y, z));
                self.set_block_state(&pos, stone, flags);
                voxels.push(aero::block_voxel(
                    [x - pivot.x as i32, -1, z - pivot.z as i32],
                    &Block::STONE,
                    Block::STONE.default_state,
                    0,
                ));
            }
        }
        let mut manager = self.lock_sublevels();
        manager.set_content(
            id,
            Content {
                pivot_offset: [0.0; 3],
                voxels,
            },
        );
        manager.mark_ready(id);
        info!(
            "sublevel {id} ready at plot chunk ({}, {})",
            chunk.x, chunk.y
        );
    }

    /// Steps sub-levels and syncs every client: sub-levels whose blocks were
    /// edited are read again, world that has gone missing around a body is
    /// sampled by [`World::load_sublevel_surroundings`], and what the bodies
    /// hit is dealt with by [`World::handle_impacts`].
    pub fn tick_sublevels(self: &Arc<Self>) {
        let (dirty, requests, step) = {
            let mut manager = self.lock_sublevels();
            self.has_sublevels
                .store(!manager.is_empty(), std::sync::atomic::Ordering::Relaxed);
            let (requests, step) = manager.step_physics();
            (manager.take_dirty(), requests, step)
        };
        for id in dirty {
            self.rescan_sublevel(id);
        }
        for (id, request) in requests {
            let world = Arc::clone(self);
            tokio::spawn(async move { world.load_sublevel_surroundings(id, request).await });
        }
        self.handle_impacts(&step.impacts);
        self.lock_sublevels().sync(self, &step.bodies);
    }

    /// Called for every block set inside the plotyard, and for every block
    /// set anywhere while sub-levels exist: their bodies collide with the
    /// world, so it has to follow its edits.
    pub(crate) fn note_sublevel_edit(&self, pos: &BlockPos) {
        self.lock_sublevels().note_edit(pos);
    }

    /// Removes a sub-level, telling every client that tracked it.
    pub fn remove_sublevel(&self, id: u32) -> bool {
        self.lock_sublevels().remove_and_notify(self, id)
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
        let id = self.lock_sublevels().assemble(
            pose,
            bounds,
            pivot_offset,
            [[min.x, min.y, min.z], [max.x, max.y, max.z]],
        );
        let world = Arc::clone(self);
        tokio::spawn(async move { world.copy_region_into_plot(id, min, max).await });
        Ok(id)
    }

    /// Copies a region into a sub-level's parked chunks, parks its entities,
    /// clears the original, then networks the sub-level.
    async fn copy_region_into_plot(self: Arc<Self>, id: u32, min: Vector3<i32>, max: Vector3<i32>) {
        let (chunks, anchor, center_chunk, from_anchor) = {
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
            let Some(from_anchor) = manager.from_anchor_of(id) else {
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
            (chunks, anchor, center_chunk, from_anchor)
        };
        // The source region's chunks must stay loaded too, or the copy reads
        // air out of chunks the server unloaded. They are only held, never
        // cleared: only the plot chunks carry parked content.
        let mut held = chunks.clone();
        held.extend(region_chunks(min, max));
        if !self.await_plot_chunks(&held).await {
            self.lock_sublevels().end_assembly(id);
            return;
        }

        // A plot chunk generates like any other chunk; empty it so the
        // sub-level carries nothing but the region.
        for chunk in &chunks {
            self.clear_parked_chunk(*chunk);
        }
        let center = plot_center_block(center_chunk);
        let ParkedCopy { bounds } = match self.copy_blocks_into_plot(min, max, anchor, center) {
            Ok(copied) => copied,
            Err(outside) => {
                warn!(
                    "sublevel: parked copy of ({}, {}, {}) landed outside the build limit; region skipped",
                    outside.x, outside.y, outside.z
                );
                self.lock_sublevels().end_assembly(id);
                return;
            }
        };
        self.park_entities_in_region(min, max, anchor, center);
        self.clear_region(min, max);
        self.lock_sublevels().end_assembly(id);

        // Sable's bounds are tight to the content; empty margins of the
        // region must not widen the client's render or collision box.
        let mut manager = self.lock_sublevels();
        if let Some(bounds) = bounds {
            manager.tighten_bounds(id, bounds);
        }
        let pivot_offset = [
            from_anchor.x as f32,
            from_anchor.y as f32,
            from_anchor.z as f32,
        ];
        drop(manager);
        // The content is read back from the plot, where redstone signals
        // are the ones the burners and propellers will actually see. Its
        // grid counts from the parked anchor, the plot centre block.
        if let Some(voxels) = self.content_from_plot(id, center) {
            let mut manager = self.lock_sublevels();
            manager.set_content(
                id,
                Content {
                    pivot_offset,
                    voxels,
                },
            );
            manager.mark_ready(id);
        }
        info!(
            "sublevel {id} assembled from region ({}, {}, {})..({}, {}, {})",
            min.x, min.y, min.z, max.x, max.y, max.z
        );
    }

    /// The blocks parked in sub-level `id`'s plot as voxels, counted from
    /// `anchor`, the parked block the content's grid counts from (see
    /// [`SubLevelManager::anchor_of`]). `None` if nothing is parked there.
    fn content_from_plot(&self, id: u32, anchor: Vector3<i32>) -> Option<Vec<Voxel>> {
        let bounds = self.lock_sublevels().bounds_of(id)?;
        let mut voxels = Vec::new();
        for x in bounds[0]..=bounds[3] {
            for y in bounds[1]..=bounds[4] {
                for z in bounds[2]..=bounds[5] {
                    let pos = BlockPos(Vector3::new(x, y, z));
                    let (block, state) = self.get_block_and_state(&pos);
                    if state.is_air() {
                        continue;
                    }
                    let signal = if needs_signal(block) {
                        aero::signal_at(self, pos)
                    } else {
                        0
                    };
                    voxels.push(aero::block_voxel(
                        [x - anchor.x, y - anchor.y, z - anchor.z],
                        block,
                        state,
                        signal,
                    ));
                }
            }
        }
        Some(voxels)
    }

    /// Reads sub-level `id`'s parked blocks again and hands them to the
    /// physics, after a block of the sub-level was edited or broke off.
    fn rescan_sublevel(self: &Arc<Self>, id: u32) {
        let Some(anchor) = self
            .lock_sublevels()
            .anchor_of(id)
            .map(|anchor| anchor.floor_to_i32())
        else {
            return;
        };
        let Some(voxels) = self.content_from_plot(id, anchor) else {
            return;
        };
        let mut manager = self.lock_sublevels();
        if voxels.is_empty() {
            drop(manager);
            self.remove_sublevel(id);
            return;
        }
        let pivot_offset = manager.from_anchor_of(id).map_or([0.0; 3], |offset| {
            [offset.x as f32, offset.y as f32, offset.z as f32]
        });
        manager.set_content(
            id,
            Content {
                pivot_offset,
                voxels,
            },
        );
    }

    /// Deals with what the sub-levels hit this tick: fragile blocks break,
    /// TNT primes, bells ring, knocks hard for their mass sound, and the
    /// sub-levels that lost blocks are read again.
    fn handle_impacts(self: &Arc<Self>, impacts: &[Impact]) {
        let mut rescans: FxHashSet<u32> = FxHashSet::default();
        for impact in impacts {
            // The sub-level's own block that hit, in its plot. Its lookup is
            // a statement of its own: changing a parked block locks the
            // manager again.
            let own = self
                .lock_sublevels()
                .parked_block(impact.sublevel, impact.block);
            if own.is_some_and(|pos| self.hit_block(&pos, impact)) {
                rescans.insert(impact.sublevel);
            }
            match impact.target {
                ImpactTarget::World([x, y, z]) => {
                    let pos = BlockPos(Vector3::new(x, y, z));
                    self.hit_block(&pos, impact);
                }
                ImpactTarget::SubLevel { id, block } => {
                    let Some(pos) = self.lock_sublevels().parked_block(id, block) else {
                        continue;
                    };
                    if self.hit_block(&pos, impact) {
                        rescans.insert(id);
                    }
                }
            }
        }
        for id in rescans {
            self.rescan_sublevel(id);
        }
        self.sound_impacts(impacts);
    }

    /// Lets a block take the consequences of being hit: fragile blocks break,
    /// TNT primes, bells ring. Returns whether the block changed.
    fn hit_block(self: &Arc<Self>, pos: &BlockPos, impact: &Impact) -> bool {
        /// Sable's `FragileBlockCallback` threshold, in blocks per second.
        const FRAGILE_SPEED: f32 = 4.0;
        /// Sable's `ExplosiveBlockCallback` threshold.
        const TNT_SPEED: f32 = 5.0;
        /// Sable's `BellBlockCallback` threshold, inherited from fragile blocks.
        const BELL_SPEED: f32 = FRAGILE_SPEED;

        let speed = impact.speed;
        let (block, state) = self.get_block_and_state(pos);
        if *block == Block::TNT && speed >= TNT_SPEED {
            self.set_block_state(
                pos,
                Block::AIR.default_state.id,
                BlockFlags::NOTIFY_ALL | BlockFlags::SKIP_DROPS,
            );
            self.spawn_entity(TNTEntity::primed(self, pos, 4));
            return true;
        }
        if speed < FRAGILE_SPEED {
            return false;
        }
        if *block == Block::BELL && speed >= BELL_SPEED {
            // The bell swings from the side that was hit. One parked on a
            // sub-level is rung where it stands, but heard where it was hit.
            let centre = pos.to_centered_f64();
            let from = Vector3::new(
                f64::from(impact.point[0]),
                f64::from(impact.point[1]),
                f64::from(impact.point[2]),
            );
            let parked = SubLevelManager::is_plot_chunk(pos.chunk_and_chunk_relative_position().0);
            ring_bell(*pos, self, Some(hit_side(from - centre)), None);
            if parked {
                self.play_sound_fine(Sound::BlockBellUse, SoundCategory::Blocks, &from, 1.0, 2.0);
            }
            return false;
        }
        if aero::collision_shape(state).is_solid()
            && aero::block_voxel([0; 3], block, state, 0).material.fragile
        {
            let melts = (*block == Block::ICE || *block == Block::FROSTED_ICE) && {
                let below = pos.offset(BlockDirection::Down.to_offset());
                let (below_block, below_state) = self.get_block_and_state(&below);
                blocks_movement(below_state, below_block.id) || below_state.is_liquid()
            };
            if melts {
                ice::melt(self, pos);
            } else {
                // Sable destroys the block with its drops.
                self.set_block_state(pos, Block::AIR.default_state.id, BlockFlags::NOTIFY_ALL);
            }
            return true;
        }
        false
    }

    /// Sounds the knocks that were hard for the lighter body's mass, as
    /// Sable's `processCollisionEffects` does: the hit block's particles and
    /// step, hit and place sounds at the contact, a block sounding once per
    /// two ticks.
    fn sound_impacts(self: &Arc<Self>, impacts: &[Impact]) {
        /// Sable plays a contact's sounds when its force, the impulse over
        /// the substep, exceeds this multiple of the smaller mass.
        const FORCE_PER_MASS: f32 = 25.0;
        /// A block that just sounded stays quiet this many ticks.
        const QUIET_TICKS: i32 = 2;
        /// Sable caps the collisions it reports per tick.
        const MAX_PER_TICK: usize = 100;
        const VOLUME: f32 = 0.08;

        let tick = self.lock_sublevels().tick;
        let mut sounded = 0;
        for impact in impacts {
            if sounded >= MAX_PER_TICK || impact.impulse <= 0.0 {
                break;
            }
            let lighter = impact.mass.min(impact.other_mass);
            if impact.impulse / pumpkin_sublevel::SUBSTEP_DT <= FORCE_PER_MASS * lighter {
                continue;
            }
            if !self
                .lock_sublevels()
                .note_sound(impact.sublevel, impact.block, tick, QUIET_TICKS)
            {
                continue;
            }
            let state = match impact.target {
                ImpactTarget::World([x, y, z]) => {
                    self.get_block_state_id(&BlockPos(Vector3::new(x, y, z)))
                }
                ImpactTarget::SubLevel { id, block } => self
                    .lock_sublevels()
                    .parked_block(id, block)
                    .map_or(Block::AIR.default_state.id, |pos| {
                        self.get_block_state_id(&pos)
                    }),
            };
            let block = Block::from_state_id(state);
            if *block == Block::AIR {
                continue;
            }
            sounded += 1;
            let at = Vector3::new(
                f64::from(impact.point[0]),
                f64::from(impact.point[1]),
                f64::from(impact.point[2]),
            );
            self.block_particles(state, at, 2);
            let family = SoundFamily::of(block);
            for (sound, pitch) in [
                (family.step(), 0.4..0.8),
                (family.hit(), 0.0..0.4),
                (family.place(), 0.3..0.7),
            ] {
                let pitch = rand::rng().random_range(pitch);
                self.play_sound_fine(sound, SoundCategory::Blocks, &at, VOLUME, pitch);
            }
        }
    }

    /// A few chips of a block state flying off a point.
    fn block_particles(&self, state: pumpkin_data::BlockStateId, at: Vector3<f64>, count: i32) {
        let mut data = Vec::new();
        if VarInt(i32::from(state.as_u16())).encode(&mut data).is_err() {
            return;
        }
        let packet = CParticle::new(
            false,
            false,
            at,
            Vector3::new(0.0, 0.0, 0.0),
            0.0,
            count,
            VarInt(pumpkin_data::particle::Particle::Block.to_id() as i32),
            &data,
        );
        self.broadcast_packet_all(&packet);
    }

    /// Copies a world region's blocks and block entities into the plot
    /// around `center`, where the anchor block maps onto the plot centre
    /// block. `Err` carries the world block whose parked copy fell outside
    /// the build limit.
    fn copy_blocks_into_plot(
        self: &Arc<Self>,
        min: Vector3<i32>,
        max: Vector3<i32>,
        anchor: Vector3<i32>,
        center: Vector3<i32>,
    ) -> Result<ParkedCopy, Vector3<i32>> {
        let flags = BlockFlags::SKIP_BLOCK_ADDED_CALLBACK | BlockFlags::UPDATE_KNOWN_SHAPE;
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
                        return Err(pos.0);
                    }
                    self.set_block_state(&BlockPos(parked), state, flags);
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
        Ok(ParkedCopy { bounds: tight })
    }

    /// Empties a world region of the blocks an assembly copied out of it,
    /// leaving nothing behind: no drops, no spilled block entity contents.
    fn clear_region(self: &Arc<Self>, min: Vector3<i32>, max: Vector3<i32>) {
        let flags = BlockFlags::NOTIFY_ALL
            | BlockFlags::SKIP_DROPS
            | BlockFlags::FORCE_STATE
            | BlockFlags::SKIP_BLOCK_ENTITY_REPLACED_CALLBACK;
        for x in min.x..=max.x {
            for y in min.y..=max.y {
                for z in min.z..=max.z {
                    let pos = BlockPos(Vector3::new(x, y, z));
                    if self.get_block_state_id(&pos) != Block::AIR.default_state.id {
                        self.set_block_state(&pos, Block::AIR.default_state.id, flags);
                    }
                }
            }
        }
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
            ticket_chunk(&self.level, *chunk);
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

    /// Samples the world around sub-level `id` for its physics: holds the
    /// chunks of the requested region loaded, describes every block in it
    /// and hands the result to the body. The region's chunks replace those
    /// held for the previous sample.
    pub async fn load_sublevel_surroundings(
        self: Arc<Self>,
        id: u32,
        request: SurroundingsRequest,
    ) {
        let min = Vector3::new(request.min[0], request.min[1], request.min[2]);
        let max = Vector3::new(request.max[0], request.max[1], request.max[2]);
        // No assembled content reaches further than the widest region plus
        // the physics' margin, and a body is never in the plotyard; anything
        // else is garbage that must not turn into a chunk request.
        let span = max - min;
        if span.x > 4 * MAX_ASSEMBLE_SPAN
            || span.y > 4 * MAX_ASSEMBLE_SPAN
            || span.z > 4 * MAX_ASSEMBLE_SPAN
            || SubLevelManager::is_plot_chunk(Vector2::new(
                min.x.div_euclid(16),
                min.z.div_euclid(16),
            ))
        {
            warn!(
                "sublevel: refusing to sample a {}x{}x{} region around sublevel {id}",
                span.x, span.y, span.z
            );
            self.lock_sublevels().cancel_surroundings(id);
            return;
        }
        let chunks = region_chunks(min, max);
        let previous = self
            .lock_sublevels()
            .surroundings_chunks_of(id)
            .unwrap_or_default();
        let held = chunks
            .iter()
            .copied()
            .filter(|chunk| !previous.contains(chunk))
            .collect::<Vec<_>>();
        for chunk in previous.into_iter().filter(|chunk| !chunks.contains(chunk)) {
            unticket_chunk(&self.level, chunk);
        }
        // Recorded before the wait, so a sub-level removed meanwhile still
        // lets go of them.
        self.lock_sublevels().set_surroundings_chunks(id, chunks);
        if !self.await_plot_chunks(&held).await {
            self.lock_sublevels().cancel_surroundings(id);
            return;
        }

        // Blocks of a region still being assembled belong to the sub-level
        // they are becoming, not to the world around anyone.
        let assembling = self.lock_sublevels().assembling_regions();
        let inside = |pos: &BlockPos| {
            let block = [pos.0.x, pos.0.y, pos.0.z];
            assembling.iter().any(|region| {
                (0..3).all(|axis| block[axis] >= region[0][axis] && block[axis] <= region[1][axis])
            })
        };
        let mut voxels = Vec::new();
        for x in min.x..=max.x {
            for y in min.y..=max.y {
                for z in min.z..=max.z {
                    let pos = BlockPos(Vector3::new(x, y, z));
                    if inside(&pos) {
                        continue;
                    }
                    let (block, state) = self.get_block_and_state(&pos);
                    if state.is_air() {
                        continue;
                    }
                    voxels.push(aero::block_voxel(
                        [x - min.x, y - min.y, z - min.z],
                        block,
                        state,
                        0,
                    ));
                }
            }
        }
        let mut manager = self.lock_sublevels();
        if !manager.set_surroundings(id, request, &voxels) {
            manager.cancel_surroundings(id);
        }
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
        let (first, first_chunk) = manager.spawn(Vector3::new(0.0, 64.0, 0.0), 16);
        let (second, second_chunk) = manager.spawn(Vector3::new(0.0, 64.0, 0.0), 16);
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
        let (third, _) = manager.spawn(Vector3::new(0.0, 64.0, 0.0), 16);
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
            [[min.x, min.y, min.z], [max.x, max.y, max.z]],
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
        // The physics frames the content by the body's offset from the
        // anchor block in region coordinates, never by the parked pivot.
        let from_anchor = manager
            .from_anchor_of(id)
            .expect("assembled sub-level exists");
        assert_eq!(from_anchor, pose - anchor.to_f64());
        assert!(from_anchor.x < 1.0 && from_anchor.y < 1.0 && from_anchor.z < 1.0);
    }

    #[test]
    fn parked_positions_follow_a_turned_pose() {
        let mut manager = SubLevelManager::new();
        let (id, chunk) = manager.spawn(Vector3::new(0.0, 64.0, 0.0), 16);
        let pivot = manager.pivot_of(id).expect("pivot of a spawned sub-level");
        // Turned a quarter turn around the pivot, which now sits elsewhere.
        let half = std::f32::consts::FRAC_1_SQRT_2;
        manager.levels[0].last_pose = sable::Pose {
            position: [10.0, 64.0, -5.0],
            orientation: [0.0, half, 0.0, half],
            rotation_point: [pivot.x, pivot.y, pivot.z],
        };
        // One block east of the pivot is one block north of it once turned.
        let world = manager
            .parked_to_world(pivot.add(&Vector3::new(1.0, 0.0, 0.0)))
            .expect("position inside the plotyard");
        assert!((world.x - 10.0).abs() < 1.0e-3);
        assert!((world.y - 64.0).abs() < 1.0e-3);
        assert!((world.z + 6.0).abs() < 1.0e-3, "{world:?}");
        // The pivot itself stays put.
        let centre = manager
            .parked_to_world(pivot)
            .expect("pivot inside the plotyard");
        assert!((centre.x - 10.0).abs() < 1.0e-3 && (centre.z + 5.0).abs() < 1.0e-3);
        let _ = chunk;
    }

    #[test]
    fn parked_entities_are_visible_only_to_trackers() {
        let mut manager = SubLevelManager::new();
        let (id, chunk) = manager.spawn(Vector3::new(0.0, 64.0, 0.0), 16);
        let (other, other_chunk) = manager.spawn(Vector3::new(0.0, 64.0, 0.0), 16);
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
