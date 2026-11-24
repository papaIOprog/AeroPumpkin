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
pub mod attachment;
pub mod sable;

use std::collections::VecDeque;
use std::sync::Arc;
use std::time::{Duration, Instant};

use bytes::Bytes;
use pumpkin_nbt::compound::NbtCompound;
use pumpkin_protocol::codec::var_int::VarInt;
use pumpkin_protocol::java::client::play::{
    CBlockUpdate, CBundleDelimiter, CCustomPayload, CParticle,
};
use pumpkin_protocol::ser::NetworkWriteExt;
use pumpkin_protocol::{ClientPacket, MultiVersionJavaPacket};
use pumpkin_sublevel::{
    AeroInfo, Air, Alignment, BodySnapshot, Content, Impact, ImpactTarget, Step,
    SurroundingsRequest, Voxel,
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
use pumpkin_data::block_rotation::Rotation;
use pumpkin_data::entity::EntityType;
use pumpkin_data::sound::{Sound, SoundCategory};
use pumpkin_data::tag::{self, Taggable};
use pumpkin_data::{Block, BlockDirection, BlockStateId};
use pumpkin_util::math::position::BlockPos;
use pumpkin_world::chunk_system::ChunkLoading;
use pumpkin_world::world::BlockFlags;
use rand::RngExt;

use crate::entity::r#type::from_type;
use crate::mods::glue::{GlueEntity, GlueKind, glue_boxes, play_named_sound, with_glue_by_id};
use crate::mods::packets::write_block_pos;
use crate::mods::simulated::physics_assembler;
use crate::mods::simulated::physics_staff::{self, Action};
use crate::mods::simulated::{
    ASSEMBLER_FAILED, FLICK_ASSEMBLER_LEVER, PHYSICS_STAFF_BEAM, PHYSICS_STAFF_DRAG_SESSIONS,
    PHYSICS_STAFF_LOCKS,
};

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
/// Simulated's `maxBlocksMoved`: the most blocks an assembler takes.
const MAX_ASSEMBLE_BLOCKS: usize = 128_000;
/// Simulated's `disassemblyDegreeTolerance` in radians, and the distance in
/// blocks within which a sub-level counts as placed.
const MAX_ALIGN_ANGLE: f32 = 4.0 * std::f32::consts::PI / 180.0;
const MAX_ALIGN_DISTANCE: f32 = 0.2;
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
    /// Set while the sub-level is driven level and onto block centres to be
    /// melted back into the world.
    disassembling: Option<Disassembly>,
    /// The block Sable's heat map grows from, in the content's grid: the
    /// piece holding it keeps the sub-level when the content comes apart.
    root: Option<[i32; 3]>,
    /// Set while a piece of the sub-level is moving to a plot of its own.
    splitting: bool,
    /// The sub-level a piece was split off from and the pose it had in that
    /// one's frame, until the sub-level's first full sync: Sable's
    /// `ClientboundRecentlySplitSubLevelPacket`.
    split_from: Option<(u32, sable::Pose)>,
}

/// A piece of a sub-level's content that holds together: its blocks, their
/// mass, and their grid bounds.
struct Piece {
    voxels: Vec<Voxel>,
    mass: f32,
    bounds: [i32; 6],
}

/// The pieces of a content: blocks joined through faces and edges, as
/// Sable's heat map joins them. One piece for content that holds together.
fn pieces_of(voxels: &[Voxel]) -> Vec<Piece> {
    let mut place: FxHashMap<[i32; 3], usize> = FxHashMap::default();
    for (index, voxel) in voxels.iter().enumerate() {
        place.insert(voxel.position, index);
    }
    let mut visited = vec![false; voxels.len()];
    let mut pieces = Vec::new();
    for start in 0..voxels.len() {
        if visited[start] {
            continue;
        }
        visited[start] = true;
        let mut piece = Piece {
            voxels: Vec::new(),
            mass: 0.0,
            bounds: [i32::MAX, i32::MAX, i32::MAX, i32::MIN, i32::MIN, i32::MIN],
        };
        let mut stack = vec![start];
        while let Some(index) = stack.pop() {
            let voxel = &voxels[index];
            piece.voxels.push(voxel.clone());
            piece.mass += voxel.mass;
            for axis in 0..3 {
                piece.bounds[axis] = piece.bounds[axis].min(voxel.position[axis]);
                piece.bounds[axis + 3] = piece.bounds[axis + 3].max(voxel.position[axis]);
            }
            for offset in ASSEMBLY_NEIGHBOURS {
                let next = [
                    voxel.position[0] + offset.x,
                    voxel.position[1] + offset.y,
                    voxel.position[2] + offset.z,
                ];
                if let Some(&neighbour) = place.get(&next)
                    && !visited[neighbour]
                {
                    visited[neighbour] = true;
                    stack.push(neighbour);
                }
            }
        }
        pieces.push(piece);
    }
    pieces
}

/// The piece that keeps the sub-level when its content comes apart, as
/// Sable's heat map decides it: the one holding the block the heat map grows
/// from. With that block gone every piece would part ways, so the largest
/// stays. Index into `pieces`, which has more than one.
fn kept_piece(pieces: &[Piece], root: Option<[i32; 3]>) -> usize {
    if let Some(root) = root
        && let Some(kept) = pieces
            .iter()
            .position(|piece| piece.voxels.iter().any(|voxel| voxel.position == root))
    {
        return kept;
    }
    pieces
        .iter()
        .enumerate()
        .max_by_key(|(_, piece)| piece.voxels.len())
        .map_or(0, |(index, _)| index)
}

/// Sable's `sub_level_punch_strength_multiplier`.
const PUNCH_STRENGTH: f64 = 2.1;
/// Sable's `sub_level_punch_downward_strength_multiplier`.
const DOWNWARD_PUNCH_STRENGTH: f64 = 0.175;

/// Sable's `punchCurve`: the impulse a punch gives a body of `mass`
/// along the punched line, in block masses times blocks per second.
fn punch_curve(mass: f64) -> f64 {
    // Falloff scale and exponent at and above one mass, the slope at one,
    // and the impulse at none.
    let (s, e, k, p) = (2.0, 0.5, 0.8, 1.75);
    let u = mass - 1.0;
    if mass < 1.0 {
        (((p + k - 2.0) * u + k - 1.0) * u + 1.0) * mass
    } else {
        let g = k / (s * e);
        let inverse_e = 1.0 / (e - 1.0);
        s * ((u + g.powf(inverse_e)).powf(e) - g.powf(e * inverse_e)) + 1.0
    }
}

/// Sable's `ServerboundPunchSubLevelPacket`: the punched block, where the hit
/// lies from the sub-level's place in world axes, and the direction.
struct Punch;

impl Punch {
    /// Reads the payload: a packed block position and two vectors.
    fn read(data: &[u8]) -> Option<(Vector3<i32>, Vector3<f64>, Vector3<f64>)> {
        if data.len() != 8 + 6 * 8 {
            return None;
        }
        let long = i64::from_be_bytes(data[..8].try_into().ok()?);
        let block = Vector3::new(
            (long >> 38) as i32,
            (long << 52 >> 52) as i32,
            (long << 26 >> 38) as i32,
        );
        let vector = |offset: usize| -> Option<f64> {
            Some(f64::from_be_bytes(
                data[offset..offset + 8].try_into().ok()?,
            ))
        };
        let local = Vector3::new(vector(8)?, vector(16)?, vector(24)?);
        let direction = Vector3::new(vector(32)?, vector(40)?, vector(48)?);
        Some((block, local, direction))
    }
}

/// `vector` as a unit vector, or `None` if it has no length.
fn normalize(vector: Vector3<f64>) -> Option<Vector3<f64>> {
    let length = vector.length();
    (length > 1.0e-6).then(|| Vector3::new(vector.x / length, vector.y / length, vector.z / length))
}

/// A block a sub-level hit this tick: the hardest speed it was hit at, a
/// point it was hit from, and the sub-levels it belongs to.
struct Hit {
    speed: f32,
    point: [f32; 3],
    owners: FxHashSet<u32>,
}

impl Hit {
    fn new(impact: &Impact) -> Self {
        Self {
            speed: impact.speed,
            point: impact.point,
            owners: FxHashSet::default(),
        }
    }

    const fn harder(&mut self, speed: f32) {
        self.speed = self.speed.max(speed);
    }
}

/// A knock hard enough to be heard: the sub-level's block that hit, where,
/// with how much force, and the lighter of the two masses involved.
struct Knock {
    sublevel: u32,
    block: [i32; 3],
    point: [f32; 3],
    force: f32,
    lighter: f32,
}

/// A sub-level on its way to being disassembled.
struct Disassembly {
    ticks: u32,
    /// Ticks in a row the sub-level has held its alignment.
    ready_ticks: u32,
    /// Where the assembler that melts it stands, for the sound it makes.
    stop_sound: Option<Vector3<f64>>,
}

/// A player dragging a sub-level with Simulated's physics staff: its
/// `DragSession`.
struct Drag {
    sublevel: u32,
    /// Where the staff's goal lies from the player's eyes.
    goal: Vector3<f64>,
    /// The point of the sub-level the staff holds, in the plot's
    /// coordinates: where the other players' clients end its beam.
    anchor: Vector3<f64>,
}

/// What the players have not heard of the physics staff since it changed.
#[derive(Default, Clone, Copy)]
pub struct StaffNews {
    /// The locked sub-levels changed.
    pub locks: bool,
    /// Who drags what changed.
    pub sessions: bool,
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

/// Parked block changes per sub-level, waiting for its players.
type PlotChanges = FxHashMap<u32, Vec<(BlockPos, BlockStateId)>>;

/// The eighteen ways two blocks of a contraption touch: the six faces and
/// the twelve edges. Corners never connect.
const ASSEMBLY_NEIGHBOURS: [Vector3<i32>; 18] = [
    Vector3::new(1, 0, 0),
    Vector3::new(-1, 0, 0),
    Vector3::new(0, 1, 0),
    Vector3::new(0, -1, 0),
    Vector3::new(0, 0, 1),
    Vector3::new(0, 0, -1),
    Vector3::new(1, 1, 0),
    Vector3::new(1, -1, 0),
    Vector3::new(-1, 1, 0),
    Vector3::new(-1, -1, 0),
    Vector3::new(1, 0, 1),
    Vector3::new(1, 0, -1),
    Vector3::new(-1, 0, 1),
    Vector3::new(-1, 0, -1),
    Vector3::new(0, 1, 1),
    Vector3::new(0, 1, -1),
    Vector3::new(0, -1, 1),
    Vector3::new(0, -1, -1),
];

/// The quarter turn a pose is closest to: how many clockwise quarter turns
/// it is from upright, from 0 to 3.
struct QuarterTurn {
    turns: u8,
}

impl QuarterTurn {
    /// Vanilla's rotation that turns blocks the same way.
    const fn rotation(self) -> Rotation {
        Rotation::from_index(self.turns)
    }

    /// The upright orientation at this quarter turn, as the quaternion
    /// `(x, y, z, w)` the physics steers to.
    fn upright(self) -> [f32; 4] {
        // A clockwise quarter turn seen from above is a right-handed turn
        // the other way about the up axis.
        let half = -f32::from(self.turns) * std::f32::consts::FRAC_PI_2;
        [0.0, (half * 0.5).sin(), 0.0, (half * 0.5).cos()]
    }
}

fn quarter_turn([_qx, qy, _qz, qw]: [f32; 4]) -> QuarterTurn {
    let yaw = 2.0 * qy.atan2(qw);
    let turns = (-yaw / std::f32::consts::FRAC_PI_2).round().rem_euclid(4.0) as u8;
    QuarterTurn { turns }
}

/// Where within its block the pivot comes to rest once the sub-level is
/// turned by `turns` clockwise quarter turns, given where it sat at
/// assembly: the pivot's offset from its anchor, turned with it, as a
/// fraction of a block per axis.
fn pivot_place(from_anchor: [f64; 3], turns: u8) -> [f32; 3] {
    let turned = placed(from_anchor, turns);
    std::array::from_fn(|axis| (turned[axis] as f32).rem_euclid(1.0))
}

/// Where the pivot of a sub-level at `position` is written back to the world
/// once turned by `turns` clockwise quarter turns: on the block grid its
/// alignment meant for, not where the body rests. Within the alignment's
/// tolerance of a block boundary the live pose rounds a block off.
fn placed_pivot(position: [f64; 3], from_anchor: Vector3<f64>, turns: u8) -> [f64; 3] {
    let place = pivot_place([from_anchor.x, from_anchor.y, from_anchor.z], turns);
    std::array::from_fn(|axis| {
        (position[axis] - f64::from(place[axis])).round() + f64::from(place[axis])
    })
}

/// Where a point of a sub-level's parked space lands once the sub-level is
/// turned by `turns` clockwise quarter turns: the same map as
/// [`Rotation::rotate_offset`].
fn placed(local: [f64; 3], turns: u8) -> [f64; 3] {
    let (mut x, mut z) = (local[0], local[2]);
    for _ in 0..turns {
        (x, z) = (-z, x);
    }
    [x, local[1], z]
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
    /// Parked blocks that changed since their sub-level's players last heard.
    pending_updates: FxHashMap<u32, Vec<(BlockPos, BlockStateId)>>,
    /// Sub-levels whose bounds changed since their players last heard.
    pending_bounds: FxHashSet<u32>,
    /// The sub-level each player last reported standing in, by player.
    riders: FxHashMap<Uuid, u32>,
    /// Ticks left before each player may punch a sub-level again.
    punch_cooldowns: FxHashMap<Uuid, i32>,
    /// Simulated's `PhysicsStaffServerHandler.draggingSessions`, by player.
    drags: FxHashMap<Uuid, Drag>,
    staff_news: StaffNews,
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
            pending_updates: FxHashMap::default(),
            pending_bounds: FxHashSet::default(),
            riders: FxHashMap::default(),
            punch_cooldowns: FxHashMap::default(),
            drags: FxHashMap::default(),
            staff_news: StaffNews::default(),
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
            root: None,
            splitting: false,
            split_from: None,
            disassembling: None,
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
        source: Option<[[i32; 3]; 2]>,
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
            source,
            disassembling: None,
            root: None,
            splitting: false,
            split_from: None,
        });
        id
    }

    /// Sable's heat map split: a piece of sub-level `id`'s parked blocks,
    /// `piece` being its absolute parked bounds, becomes a sub-level of its
    /// own, with the same pose and pivot so the piece stays exactly where it
    /// is. The plot it moves to is empty until [`World::move_piece`] fills
    /// it. Returns the new sub-level and how far its blocks move in the
    /// plotyard.
    pub fn split(&mut self, id: u32, piece: [i32; 6]) -> Option<(u32, Vector3<i32>)> {
        let level = self.levels.iter().find(|level| level.id == id)?;
        let old_center = plot_center_block(level.chunk);
        let from_anchor = level.from_anchor;
        let pose = self.pose_of(id)?;
        let relative = [
            piece[0] - old_center.x,
            piece[1] - old_center.y,
            piece[2] - old_center.z,
            piece[3] - old_center.x,
            piece[4] - old_center.y,
            piece[5] - old_center.z,
        ];
        let new_id = self.assemble(
            Vector3::new(pose.position[0], pose.position[1], pose.position[2]),
            relative,
            from_anchor,
            None,
        );
        // Turned as the old one is, so the piece keeps its place.
        self.physics
            .set_body_pose(new_id, pose.position, pose.orientation);
        let new_center = plot_center_block(self.chunk_of(new_id)?);
        if let Some(level) = self.levels.iter_mut().find(|level| level.id == id) {
            level.splitting = true;
        }
        // Sable's recently split packet carries the piece's pose in the old
        // sub-level's frame, for the client to continue its motion from. The
        // piece starts on the old sub-level's pivot, so there it sits on the
        // rotation point.
        if let Some(level) = self.levels.iter_mut().find(|level| level.id == new_id) {
            level.split_from = Some((
                id,
                sable::Pose {
                    position: pose.rotation_point,
                    rotation_point: [level.pivot.x, level.pivot.y, level.pivot.z],
                    ..sable::Pose::default()
                },
            ));
        }
        Some((new_id, new_center - old_center))
    }

    /// The block Sable's heat map grows from in sub-level `id`, in its
    /// content's grid.
    #[must_use]
    pub fn root_of(&self, id: u32) -> Option<[i32; 3]> {
        self.levels
            .iter()
            .find(|level| level.id == id)
            .and_then(|level| level.root)
    }

    /// Whether a piece of sub-level `id` is on its way to a plot of its own.
    pub fn is_splitting(&self, id: u32) -> bool {
        self.levels
            .iter()
            .find(|level| level.id == id)
            .is_some_and(|level| level.splitting)
    }

    /// Notes that sub-level `id`'s pieces have left its plot.
    pub fn end_split(&mut self, id: u32) {
        if let Some(level) = self.levels.iter_mut().find(|level| level.id == id) {
            level.splitting = false;
        }
    }

    /// Starts driving sub-level `id` to the alignment its disassembly
    /// waits for. The sound of its stopping, `stop_sound`, plays where it
    /// is placed.
    pub fn start_disassembly(&mut self, id: u32, stop_sound: Option<Vector3<f64>>) {
        if let Some(level) = self.levels.iter_mut().find(|level| level.id == id) {
            level.disassembling = Some(Disassembly {
                ticks: 0,
                ready_ticks: 0,
                stop_sound,
            });
        }
    }

    /// Where the sound of sub-level `id`'s stopping is to play.
    pub fn stop_sound_of(&self, id: u32) -> Option<Vector3<f64>> {
        self.levels
            .iter()
            .find(|level| level.id == id)?
            .disassembling
            .as_ref()?
            .stop_sound
    }

    /// Stops a disassembly under way and the steering that went with it.
    pub fn cancel_disassembly(&mut self, id: u32) {
        if let Some(level) = self.levels.iter_mut().find(|level| level.id == id) {
            level.disassembling = None;
        }
        self.physics.set_alignment(id, None);
    }

    /// The sub-levels being disassembled.
    pub fn disassembling(&self) -> Vec<u32> {
        self.levels
            .iter()
            .filter(|level| level.disassembling.is_some())
            .map(|level| level.id)
            .collect()
    }

    /// One tick of sub-level `id`'s disassembly: how many ticks it has
    /// taken and how many it has held its alignment for.
    pub fn disassembly_tick(&mut self, id: u32) -> Option<(u32, u32)> {
        let level = self.levels.iter_mut().find(|level| level.id == id)?;
        let disassembly = level.disassembling.as_mut()?;
        disassembly.ticks += 1;
        let aligned = self
            .physics
            .alignment_error(id)
            .is_some_and(|(angle, distance)| {
                angle <= MAX_ALIGN_ANGLE && distance <= MAX_ALIGN_DISTANCE
            });
        disassembly.ready_ticks = if aligned {
            disassembly.ready_ticks + 1
        } else {
            0
        };
        Some((disassembly.ticks, disassembly.ready_ticks))
    }

    /// Steers sub-level `id` to `alignment` until told otherwise.
    pub fn set_alignment(&mut self, id: u32, alignment: Option<Alignment>) -> bool {
        self.physics.set_alignment(id, alignment)
    }

    /// Sub-level `id`'s body after the last step.
    #[must_use]
    pub fn snapshot_of(&self, id: u32) -> Option<BodySnapshot> {
        self.physics
            .snapshots()
            .into_iter()
            .find(|body| body.id == id)
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
        if let Some(level) = self.levels.iter_mut().find(|level| level.id == id)
            && level.bounds != bounds
        {
            level.chunks = content_chunks(&bounds);
            level.bounds = bounds;
            self.pending_bounds.insert(id);
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
        // A physics staff lock goes with its sub-level.
        if self.physics.is_locked(id) {
            self.staff_news.locks = true;
        }
        self.physics.remove(id)
    }

    /// The sub-level whose parked content holds the block at `position`.
    pub fn sublevel_at(&self, position: &BlockPos) -> Option<u32> {
        let plot = Self::plot_of_chunk(position.chunk_position())?;
        self.levels
            .iter()
            .find(|level| level.ready && level.plot == plot)
            .map(|level| level.id)
    }

    /// The players tracking the sub-level whose plot holds `chunk`.
    pub fn tracking_players_of_chunk(&self, chunk: Vector2<i32>) -> Vec<Uuid> {
        let Some(plot) = Self::plot_of_chunk(chunk) else {
            return Vec::new();
        };
        self.levels
            .iter()
            .find(|level| level.plot == plot)
            .map(|level| level.tracking.iter().copied().collect())
            .unwrap_or_default()
    }

    fn id_of(&self, uuid: Uuid) -> Option<u32> {
        self.levels
            .iter()
            .find(|level| level.uuid == uuid)
            .map(|level| level.id)
    }

    /// Simulated's `toggleLock`: locks sub-level `uuid` where it stands, or
    /// lets it go. Whether it is locked now; `None` for no such sub-level.
    pub fn toggle_lock(&mut self, uuid: Uuid) -> Option<bool> {
        let locked = self.physics.toggle_lock(self.id_of(uuid)?)?;
        self.staff_news.locks = true;
        Some(locked)
    }

    /// The locked sub-levels by uuid: what `PhysicsStaffLocksPacket` lists.
    pub fn locked(&self) -> Vec<Uuid> {
        self.levels
            .iter()
            .filter(|level| self.physics.is_locked(level.id))
            .map(|level| level.uuid)
            .collect()
    }

    /// Simulated's `removeLock`: a lock on sub-level `id` gives way.
    pub fn let_lock_go(&mut self, id: u32) {
        if self.physics.is_locked(id) {
            self.physics.let_go_id(id);
            self.staff_news.locks = true;
        }
    }

    /// Simulated's `drag`: `player` drags sub-level `uuid` by the point
    /// `anchor`, in the plot's coordinates, towards the goal `goal` from
    /// their eyes, which are at `eye`, turned to `orientation`. A session
    /// keeps the sub-level it began with, and a lock gives way.
    pub fn drag(
        &mut self,
        player: Uuid,
        uuid: Uuid,
        goal: Vector3<f64>,
        anchor: Vector3<f64>,
        orientation: [f32; 4],
        eye: Vector3<f64>,
    ) -> bool {
        let Some(id) = self.id_of(uuid) else {
            return false;
        };
        self.let_lock_go(id);
        let fresh = !self.drags.contains_key(&player);
        let sublevel = self.drags.get(&player).map_or(id, |drag| drag.sublevel);
        self.drags.insert(
            player,
            Drag {
                sublevel,
                goal,
                anchor,
            },
        );
        self.staff_news.sessions |= fresh;
        let Some(pivot) = self.pivot_of(sublevel) else {
            return false;
        };
        // The body's own frame counts from the pivot.
        let held = (anchor - pivot).to_f32_lossy();
        let reach = (eye + goal).to_f32_lossy();
        self.physics.drag(
            sublevel,
            [held.x, held.y, held.z],
            [reach.x, reach.y, reach.z],
            orientation,
        )
    }

    /// Whether any player drags a sub-level.
    #[must_use]
    pub fn dragging(&self) -> bool {
        !self.drags.is_empty()
    }

    /// Simulated's `stopDragging`.
    pub fn stop_dragging(&mut self, player: Uuid) {
        if self.drags.remove(&player).is_some() {
            self.staff_news.sessions = true;
        }
    }

    /// One tick of Simulated's drag sessions, before the physics: each
    /// goal follows its player's eyes, which `eyes` gives for a player who
    /// holds the staff. A session ends when its player is gone or holds
    /// something else, or when its sub-level is.
    pub fn step_drags(&mut self, mut eyes: impl FnMut(Uuid) -> Option<Vector3<f64>>) {
        let mut ended = Vec::new();
        for (player, drag) in &self.drags {
            let follows = eyes(*player).is_some_and(|eye| {
                let reach = (eye + drag.goal).to_f32_lossy();
                self.physics
                    .set_drag_goal(drag.sublevel, [reach.x, reach.y, reach.z])
            });
            if !follows {
                ended.push(*player);
            }
        }
        if !ended.is_empty() {
            for player in ended {
                self.drags.remove(&player);
            }
            self.staff_news.sessions = true;
        }
    }

    /// The drag sessions as `PhysicsStaffDragSessionsPacket` lists them.
    pub fn drag_sessions(&self) -> Vec<(Uuid, Vector3<f64>)> {
        self.drags
            .iter()
            .map(|(player, drag)| (*player, drag.anchor))
            .collect()
    }

    /// What the players have not heard of the physics staff; nothing until
    /// it changes again.
    pub fn take_staff_news(&mut self) -> StaffNews {
        std::mem::take(&mut self.staff_news)
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

    /// Notes that the block at `pos` is now `state`: a ready sub-level whose
    /// plot holds it is rescanned and its players told of the block, and
    /// every body whose surroundings contain it gets a new sample of the
    /// world.
    pub fn note_edit(&mut self, pos: &BlockPos, state: BlockStateId) {
        let [x, y, z] = [pos.0.x, pos.0.y, pos.0.z];
        // By plot, not by bounds: a block placed against the content lies
        // outside them until the rescan, and a piece a split moves out must
        // leave the clients' view although it no longer lies within them.
        let chunk = Vector2::new(x.div_euclid(16), z.div_euclid(16));
        if let Some(plot) = Self::plot_of_chunk(chunk)
            && let Some(level) = self
                .levels
                .iter_mut()
                .find(|level| level.ready && level.plot == plot)
        {
            self.dirty.insert(level.id);
            self.pending_updates
                .entry(level.id)
                .or_default()
                .push((*pos, state));
        }
        self.physics.note_edit([x, y, z]);
    }

    /// The parked block changes and bounds changes the players tracking each
    /// sub-level have not been told of yet, by sub-level.
    pub fn take_plot_changes(&mut self) -> (PlotChanges, Vec<(u32, [i32; 6])>) {
        let bounds = self
            .pending_bounds
            .iter()
            .filter_map(|id| {
                self.levels
                    .iter()
                    .find(|level| level.id == *id)
                    .map(|level| (*id, level.bounds))
            })
            .collect();
        self.pending_bounds.clear();
        (std::mem::take(&mut self.pending_updates), bounds)
    }

    pub const fn is_empty(&self) -> bool {
        self.levels.is_empty()
    }

    pub fn take_dirty(&mut self) -> Vec<u32> {
        self.dirty.drain().collect()
    }

    /// Schedules a rescan of sub-level `id`.
    pub fn note_rescan(&mut self, id: u32) {
        self.dirty.insert(id);
    }

    /// Whether sub-level `id`'s block `block` may sound its knock on tick
    /// `tick`: not if it sounded within the last `quiet` ticks.
    /// Sable's punch: sub-level `target` is shoved at the hit, its place
    /// plus `local`, towards `direction`, and sub-level `standing`, the one
    /// the player stands on at `position`, is shoved back the other way; a
    /// punch on the world shoves `standing` alone. Both legs of the punch
    /// get the strength of the leg that meets the heavier mass. Returns
    /// where the punch landed in the world.
    pub fn punch(
        &mut self,
        target: Option<u32>,
        standing: Option<u32>,
        position: Vector3<f64>,
        local: Vector3<f64>,
        direction: Vector3<f64>,
    ) -> Option<Vector3<f64>> {
        // How hard the punch lands on a side: the lighter mass along the
        // punched line feels it more.
        let strength = |id: u32, point: Vector3<f64>, along: Vector3<f64>| {
            self.physics
                .inverse_normal_mass(
                    id,
                    [point.x as f32, point.y as f32, point.z as f32],
                    [along.x as f32, along.y as f32, along.z as f32],
                )
                .filter(|inverse| *inverse > 0.0)
                .map_or(0.0, |inverse| {
                    punch_curve(1.0 / f64::from(inverse)) * PUNCH_STRENGTH
                })
        };
        let hit = target
            .and_then(|id| self.physics.pivot_of(id))
            .map(|pivot| {
                Vector3::new(
                    f64::from(pivot[0]),
                    f64::from(pivot[1]),
                    f64::from(pivot[2]),
                )
            })
            .map(|pivot| pivot + local);
        let back = Vector3::new(-direction.x, -direction.y, -direction.z);
        // The shoves to give, as sub-level, point, direction and strength.
        let shoves: Vec<(u32, Vector3<f64>, Vector3<f64>, f64)> = match (target, standing, hit) {
            (Some(target), None, Some(hit)) => {
                vec![(target, hit, direction, strength(target, hit, direction))]
            }
            (Some(target), Some(standing), Some(hit)) => {
                let scalar =
                    strength(target, hit, direction).min(strength(standing, position, back));
                vec![
                    (standing, position, back, scalar),
                    (target, hit, direction, scalar),
                ]
            }
            (None, Some(standing), _) => {
                vec![(standing, position, back, strength(standing, position, back))]
            }
            _ => Vec::new(),
        };
        for (id, point, along, scalar) in shoves {
            self.physics.punch(
                id,
                [point.x as f32, point.y as f32, point.z as f32],
                [
                    (along.x * scalar) as f32,
                    (along.y * scalar) as f32,
                    (along.z * scalar) as f32,
                ],
            );
        }
        hit
    }

    /// The sub-level `player` last reported standing in, if any.
    #[must_use]
    pub fn ridden_by(&self, player: Uuid) -> Option<u32> {
        self.riders.get(&player).copied()
    }

    /// Notes that `player`'s position was reported in `position`: inside a
    /// sub-level's plot, they stand on that sub-level; anywhere else, on
    /// the world.
    pub fn note_rider(&mut self, player: Uuid, position: Vector3<f64>) {
        let chunk = Vector2::new(
            (position.x.floor() as i32).div_euclid(16),
            (position.z.floor() as i32).div_euclid(16),
        );
        if !Self::is_plot_chunk(chunk) {
            self.riders.remove(&player);
            return;
        }
        let plot = Vector2::new(
            (chunk.x >> PLOT_LOG_SIZE) - PLOTYARD_ORIGIN,
            (chunk.y >> PLOT_LOG_SIZE) - PLOTYARD_ORIGIN,
        );
        if let Some(level) = self.levels.iter().find(|level| level.plot == plot) {
            self.riders.insert(player, level.id);
        }
    }

    /// The plot a chunk of the plotyard belongs to, each plot being a
    /// square of `1 << PLOT_LOG_SIZE` chunks.
    fn plot_of_chunk(chunk: Vector2<i32>) -> Option<Vector2<i32>> {
        Self::is_plot_chunk(chunk).then(|| {
            Vector2::new(
                (chunk.x >> PLOT_LOG_SIZE) - PLOTYARD_ORIGIN,
                (chunk.y >> PLOT_LOG_SIZE) - PLOTYARD_ORIGIN,
            )
        })
    }

    /// The sub-level whose parked content holds `block`, if any.
    #[must_use]
    pub fn containing(&self, block: &BlockPos) -> Option<u32> {
        let [x, y, z] = [block.0.x, block.0.y, block.0.z];
        self.levels
            .iter()
            .find(|level| {
                let [x0, y0, z0, x1, y1, z1] = level.bounds;
                x >= x0 && x <= x1 && y >= y0 && y <= y1 && z >= z0 && z <= z1
            })
            .map(|level| level.id)
    }

    /// The orientation of sub-level `id`'s body as a quaternion, `(x, y, z, w)`.
    #[must_use]
    pub fn orientation_of(&self, id: u32) -> Option<[f32; 4]> {
        self.snapshot_of(id).map(|body| body.rotation)
    }

    /// Whether `player` may punch a sub-level now, with `cooldown` ticks
    /// between punches: not if their last one was more recent than that.
    pub fn try_punch(&mut self, player: Uuid, cooldown: i32) -> bool {
        if self
            .punch_cooldowns
            .get(&player)
            .is_some_and(|left| *left > 0)
        {
            return false;
        }
        self.punch_cooldowns.insert(player, cooldown);
        true
    }

    /// Forgets the blocks that sounded before `tick`.
    pub fn forget_sounds_before(&mut self, tick: i32) {
        self.recent_sounds.retain(|_, sounded| *sounded >= tick);
    }

    /// Whether sub-level `id`'s block `block` may sound its knock on tick
    /// `tick`: not if it has not been forgotten since it last did.
    pub fn note_sound(&mut self, block: (u32, [i32; 3]), tick: i32) -> bool {
        self.recent_sounds.insert(block, tick).is_none()
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
        let plot = Self::plot_of_chunk(chunk)?;
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
        // Sable's heat map grows from the first block placed in a sub-level.
        // The piece holding that block keeps the sub-level when the content
        // comes apart; with the block gone, the first block of what remains
        // takes over.
        if let Some(level) = self.levels.iter_mut().find(|level| level.id == id)
            && !content
                .voxels
                .iter()
                .any(|voxel| Some(voxel.position) == level.root)
        {
            level.root = content.voxels.first().map(|voxel| voxel.position);
        }
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

        for left in self.punch_cooldowns.values_mut() {
            *left -= 1;
        }
        self.punch_cooldowns.retain(|_, left| *left > 0);
        let players: Vec<Arc<Player>> = world.players.load().to_vec();
        self.update_tracking(world, bodies, &players);
        self.send_movement(bodies, &players, ms_since_last);
        self.send_plot_changes(&players);
    }

    /// Tells the players tracking a sub-level of its parked blocks that
    /// changed and, as Sable's `change_bounds_sublevel`, of its new bounds.
    /// Nobody's view reaches the plotyard, so the usual block updates
    /// never arrive there.
    fn send_plot_changes(&mut self, players: &[Arc<Player>]) {
        let (updates, bounds) = self.take_plot_changes();
        for (id, changes) in updates {
            let Some(level) = self.levels.iter().find(|level| level.id == id) else {
                continue;
            };
            for uuid in &level.tracking {
                let Some(player) = find_player(players, *uuid) else {
                    continue;
                };
                for (pos, state) in &changes {
                    send_packet(
                        player,
                        &CBlockUpdate::new(*pos, VarInt(i32::from(state.as_u16()))),
                    );
                }
            }
        }
        for (id, bounds) in bounds {
            let Some(level) = self.levels.iter().find(|level| level.id == id) else {
                continue;
            };
            let payload = encode(sable::encode_change_bounds(level.plot_long(), bounds));
            for uuid in &level.tracking {
                if let Some(player) = find_player(players, *uuid) {
                    send_payload(player, sable::CHANGE_BOUNDS, &payload);
                }
            }
        }
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
            // A piece split off another sub-level follows that one's motion
            // until its own snapshots arrive, for players who track both.
            let split_from = level.split_from.and_then(|(from, pose)| {
                let origin = self.levels.iter().find(|other| other.id == from)?;
                origin
                    .tracking
                    .contains(uuid)
                    .then_some((origin.uuid, pose))
            });
            send_full_sync(world, player, level, body, self.tick, split_from);
        }
        // The split is news only to the players a sub-level starts with.
        for level in &mut self.levels {
            if level.ready {
                level.split_from = None;
            }
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

/// The absolute parked bounds of blocks counted from `anchor`: the lowest
/// and highest block on each axis, inclusive.
fn tight_bounds(anchor: Vector3<i32>, voxels: &[Voxel]) -> [i32; 6] {
    let mut tight = [i32::MAX, i32::MAX, i32::MAX, i32::MIN, i32::MIN, i32::MIN];
    let anchor = [anchor.x, anchor.y, anchor.z];
    for voxel in voxels {
        for axis in 0..3 {
            let parked = anchor[axis] + voxel.position[axis];
            tight[axis] = tight[axis].min(parked);
            tight[axis + 3] = tight[axis + 3].max(parked);
        }
    }
    tight
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

fn send_packet<P: ClientPacket + MultiVersionJavaPacket>(player: &Player, packet: &P) {
    let Some(java) = player.client.java() else {
        return;
    };
    if let Ok(bytes) =
        JavaClient::serialize_packet_for_version(packet, player.client.java_version())
    {
        java.try_enqueue_packet(bytes);
    }
}

fn send_payload(player: &Player, channel: &str, data: &[u8]) {
    let Some(java) = player.client.java() else {
        return;
    };
    java.try_enqueue_packet(encode_payload(player, channel, data));
}

/// The same for a mod's channel, which only a client that announced it
/// may be sent on.
fn send_mod_payload(player: &Player, channel: &str, data: &[u8]) {
    let Some(java) = player.client.java() else {
        return;
    };
    if !java.has_mod_channel(channel) {
        return;
    }
    java.try_enqueue_packet(encode_payload(player, channel, data));
}

/// Sable's full sync: start tracking, one chunk packet per content chunk,
/// finalize, in a bundle; a sub-level split off another one comes with what
/// Sable's recently split packet says about it.
fn send_full_sync(
    world: &World,
    player: &Player,
    level: &SubLevel,
    body: &BodySnapshot,
    tick: i32,
    split_from: Option<(Uuid, sable::Pose)>,
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
    if let Some((origin, pose)) = split_from {
        members.push(encode_payload(
            player,
            sable::RECENTLY_SPLIT,
            &encode(sable::encode_recently_split(level.uuid, origin, &pose)),
        ));
    }
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
        let (dirty, requests, step, news) = {
            let mut manager = self.lock_sublevels();
            self.has_sublevels
                .store(!manager.is_empty(), std::sync::atomic::Ordering::Relaxed);
            if manager.dragging() {
                let players = self.players.load();
                manager.step_drags(|uuid| {
                    players
                        .iter()
                        .find(|player| {
                            player.gameprofile.id == uuid && physics_staff::holding(player)
                        })
                        .map(|player| player.eye_position())
                });
            }
            let (requests, step) = manager.step_physics();
            (
                manager.take_dirty(),
                requests,
                step,
                manager.take_staff_news(),
            )
        };
        for id in dirty {
            self.rescan_sublevel(id);
        }
        for (id, request) in requests {
            let world = Arc::clone(self);
            tokio::spawn(async move { world.load_sublevel_surroundings(id, request).await });
        }
        for id in &step.recovered {
            warn!("sublevel {id}: left the numbers; put back at rest where it last was");
        }
        for id in &step.gone {
            warn!("sublevel {id}: left the heights sub-levels may exist at; removed");
            self.remove_sublevel(*id);
        }
        self.handle_impacts(&step.impacts);
        self.lock_sublevels().sync(self, &step.bodies);
        self.step_disassemblies();
        self.tell_staff_news(news);
    }

    /// Tells every player what changed about the physics staff: Simulated's
    /// `sendLocksToClients` and `sendDragSessionsToClients`.
    fn tell_staff_news(&self, news: StaffNews) {
        if !news.locks && !news.sessions {
            return;
        }
        let dimension = self.dimension.minecraft_name;
        let manager = self.lock_sublevels();
        let (locks, sessions) = (
            news.locks
                .then(|| physics_staff::encode_locks(dimension, &manager.locked())),
            news.sessions
                .then(|| physics_staff::encode_sessions(dimension, &manager.drag_sessions())),
        );
        for player in self.players.load().iter() {
            if let Some(locks) = &locks {
                send_mod_payload(player, PHYSICS_STAFF_LOCKS, locks);
            }
            if let Some(sessions) = &sessions {
                send_mod_payload(player, PHYSICS_STAFF_DRAG_SESSIONS, sessions);
            }
        }
    }

    /// Simulated's `PhysicsStaffServerHandler.sendAllData`, for a player
    /// who joined: the locks and the drag sessions of this dimension.
    pub fn send_staff_data(&self, player: &Player) {
        let dimension = self.dimension.minecraft_name;
        let manager = self.lock_sublevels();
        send_mod_payload(
            player,
            PHYSICS_STAFF_LOCKS,
            &physics_staff::encode_locks(dimension, &manager.locked()),
        );
        send_mod_payload(
            player,
            PHYSICS_STAFF_DRAG_SESSIONS,
            &physics_staff::encode_sessions(dimension, &manager.drag_sessions()),
        );
    }

    /// Simulated's `PhysicsStaffServerHandler.drag`, as the staff's drag
    /// packet has it.
    pub fn staff_drag(
        &self,
        player: &Arc<Player>,
        sub_level: Uuid,
        goal: Vector3<f64>,
        anchor: Vector3<f64>,
        orientation: [f32; 4],
    ) {
        let news = {
            let mut manager = self.lock_sublevels();
            manager.drag(
                player.gameprofile.id,
                sub_level,
                goal,
                anchor,
                orientation,
                player.eye_position(),
            );
            manager.take_staff_news()
        };
        self.tell_staff_news(news);
    }

    /// Simulated's `PhysicsStaffActionPacket`: `Lock` locks or frees
    /// `sub_level`, with a beam from the player's eyes to `location` for
    /// the players watching, and `StopDrag` ends the player's drag.
    pub fn staff_action(
        &self,
        player: &Arc<Player>,
        action: Action,
        sub_level: Uuid,
        location: Vector3<f64>,
    ) {
        let news = {
            let mut manager = self.lock_sublevels();
            match action {
                Action::Lock => {
                    manager.toggle_lock(sub_level);
                }
                Action::StopDrag => manager.stop_dragging(player.gameprofile.id),
                Action::StartDrag => {}
            }
            manager.take_staff_news()
        };
        if action == Action::Lock {
            let beam =
                physics_staff::encode_beam(player.gameprofile.id, player.eye_position(), location);
            let beam = CCustomPayload::new(PHYSICS_STAFF_BEAM, &beam);
            self.send_to_chunk_watchers_except(
                BlockPos::floored_v(location).chunk_position(),
                player.gameprofile.id,
                &beam,
            );
        }
        self.tell_staff_news(news);
    }

    /// Sends `packet` to the players watching the chunk `chunk`, all but
    /// `except`. A plot's chunk reaches the players tracking its sub-level.
    fn send_to_chunk_watchers_except(
        &self,
        chunk: Vector2<i32>,
        except: Uuid,
        packet: &CCustomPayload<'_>,
    ) {
        if SubLevelManager::is_plot_chunk(chunk) {
            let players = self.players.load();
            for uuid in self.lock_sublevels().tracking_players_of_chunk(chunk) {
                if uuid != except
                    && let Some(player) = find_player(&players, uuid)
                {
                    player.try_send_client_packet(packet);
                }
            }
        } else {
            self.broadcast_to_chunk_except(chunk, &[except], packet);
        }
    }

    /// Simulated's `PhysicsAssemblerBlockEntity.assembleOrDisassemble`:
    /// the assembler at `position`, in the world or parked in a sub-level,
    /// takes the contraption its sticky side touches into a sub-level, or
    /// gives its sub-level back to the world.
    pub fn assembler_used(self: &Arc<Self>, position: &BlockPos) {
        let inside = self.lock_sublevels().sublevel_at(position);
        let state = self.get_block_state_id(position);
        // The lever holds where it was put until the server says otherwise.
        let mut flick = Vec::with_capacity(9);
        let _ = write_block_pos(&mut flick, *position);
        let _ = flick.write_bool(inside.is_none());
        self.tell_assembler(position, FLICK_ASSEMBLER_LEVER, &flick);
        let failed = inside.map_or_else(
            || self.assembler_assemble(position, physics_assembler::sticky_facing(state)),
            |id| self.assembler_disassemble(id, position),
        );
        if failed {
            let mut failed = Vec::with_capacity(8);
            let _ = write_block_pos(&mut failed, *position);
            self.tell_assembler(position, ASSEMBLER_FAILED, &failed);
        }
    }

    /// Tells the players watching the assembler at `position` `data` on
    /// `channel`.
    fn tell_assembler(&self, position: &BlockPos, channel: &str, data: &[u8]) {
        let packet = CCustomPayload::new(channel, data);
        self.send_to_chunk_watchers_except(position.chunk_position(), Uuid::nil(), &packet);
    }

    /// The assembler at `position` takes the contraption its sticky side
    /// `sticky` touches. Whether it could not, as `assemblyFailed` says.
    fn assembler_assemble(self: &Arc<Self>, position: &BlockPos, sticky: BlockDirection) -> bool {
        let start = position.0 + sticky.to_offset();
        // Simulated's `assembleFromSingleBlock`: with nothing there,
        // nothing happens.
        let (_, state) = self.get_block_and_state(&BlockPos(start));
        if state.is_air() {
            return false;
        }
        match self.assemble_connected(start) {
            Ok(_) => {
                play_named_sound(
                    self,
                    "simulated:block.physics_assembler.assemble",
                    SoundCategory::Blocks,
                    &position.to_centered_f64(),
                    1.0,
                    1.0,
                    None,
                );
                false
            }
            Err(error) => {
                debug!("sublevel: the assembler at {position:?} could not assemble: {error}");
                true
            }
        }
    }

    /// The assembler at `position`, parked in sub-level `id`, gives the
    /// sub-level back to the world. Whether it could not.
    fn assembler_disassemble(self: &Arc<Self>, id: u32, position: &BlockPos) -> bool {
        let news = {
            let mut manager = self.lock_sublevels();
            // Simulated's `startDisassembling`: a physics staff lock gives way.
            manager.let_lock_go(id);
            manager.take_staff_news()
        };
        self.tell_staff_news(news);
        let stop_sound = self
            .lock_sublevels()
            .parked_to_world(position.to_centered_f64());
        self.disassemble_sublevel(id, stop_sound).is_err()
    }

    /// Called for every block set inside the plotyard, and for every block
    /// set anywhere while sub-levels exist: their bodies collide with the
    /// world, so it has to follow its edits.
    pub(crate) fn note_sublevel_edit(&self, pos: &BlockPos, state: BlockStateId) {
        self.lock_sublevels().note_edit(pos, state);
    }

    /// The same for every block a command changed in one batch: `/fill`
    /// writes its blocks without `set_block_state`.
    pub fn note_sublevel_edits(&self, changes: &[(BlockPos, BlockStateId)]) {
        let concerns = self
            .has_sublevels
            .load(std::sync::atomic::Ordering::Relaxed)
            || changes.iter().any(|(pos, _)| {
                SubLevelManager::is_plot_chunk(pos.chunk_and_chunk_relative_position().0)
            });
        if !concerns {
            return;
        }
        let mut manager = self.lock_sublevels();
        for (pos, state) in changes {
            manager.note_edit(pos, *state);
        }
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
        self.assemble_region(min, max, None, Vec::new())
    }

    /// Freezes the contraption around the block at `start` into a new
    /// sub-level, as Simulated's assembler gathers one: every block held to
    /// it by glue, by slime or honey, or by being fastened to what it
    /// touches, up to [`MAX_ASSEMBLE_BLOCKS`]. The glue that held it
    /// together goes along. Returns the sub-level id.
    pub fn assemble_connected(self: &Arc<Self>, start: Vector3<i32>) -> Result<u32, String> {
        let (blocks, glue) = self.gathered(start)?;
        let (mut min, mut max) = (blocks[0], blocks[0]);
        for block in &blocks {
            min.x = min.x.min(block.x);
            min.y = min.y.min(block.y);
            min.z = min.z.min(block.z);
            max.x = max.x.max(block.x);
            max.y = max.y.max(block.y);
            max.z = max.z.max(block.z);
        }
        let include: FxHashSet<BlockPos> = blocks.iter().copied().map(BlockPos).collect();
        self.assemble_region(min, max, Some(include), glue)
    }

    /// The blocks of the contraption around `start`, and the ids of the glue
    /// entities that held it together: Simulated's
    /// `SimAssemblyContraption.searchMovedStructure`, which is Create's
    /// `Contraption.moveBlock`. A block that cannot be moved is left out,
    /// unless it is the one the contraption is gathered around.
    fn gathered(
        self: &Arc<Self>,
        start: Vector3<i32>,
    ) -> Result<(Vec<Vector3<i32>>, Vec<i32>), String> {
        let start_pos = BlockPos(start);
        if !self.is_in_build_limit(start_pos) {
            return Err("the block is outside the build limit".to_owned());
        }
        if !self.chunk_loaded(start) {
            return Err("the block's chunk is not loaded".to_owned());
        }
        let (block, state) = self.get_block_and_state(&start_pos);
        if state.is_air() {
            return Err("there is no block to assemble there".to_owned());
        }
        if !attachment::movement_allowed(block, state) {
            return Err("that block cannot be moved".to_owned());
        }
        let glue = glue_boxes(self);
        // Simulated's addInitialHoneyGlue: the honey glue over the first
        // block goes along, whether or not it holds anything.
        let mut used: Vec<i32> = glue
            .iter()
            .filter(|glue| glue.kind == GlueKind::Honey && glue.contains(start_pos))
            .map(|glue| glue.entity_id)
            .collect();
        let mut blocks: Vec<Vector3<i32>> = Vec::new();
        let mut seen: FxHashSet<BlockPos> = FxHashSet::default();
        // A brittle first block is not searched around.
        let mut frontier = VecDeque::new();
        if !attachment::brittle(block, state) {
            frontier.push_back(start);
        }
        while let Some(at) = frontier.pop_front() {
            let pos = BlockPos(at);
            if !seen.insert(pos) {
                continue;
            }
            if !self.is_in_build_limit(pos) {
                continue;
            }
            if !self.chunk_loaded(at) {
                return Err("the contraption reaches unloaded chunks".to_owned());
            }
            let (block, state) = self.get_block_and_state(&pos);
            if state.is_air() {
                continue;
            }
            if !attachment::movement_allowed(block, state) {
                return Err("a block of the contraption cannot be moved".to_owned());
            }
            // The halves of a double chest go together.
            if let Some(towards) = attachment::chest_half(block, state) {
                let half = pos.offset(towards.to_offset());
                if !seen.contains(&half) {
                    frontier.push_back(half.0);
                }
            }
            for offset in ASSEMBLY_NEIGHBOURS {
                let next = BlockPos(at + offset);
                if seen.contains(&next) {
                    continue;
                }
                let (next_block, next_state) = self.get_block_and_state(&next);
                if !attachment::movement_allowed(next_block, next_state) {
                    continue;
                }
                let mut holding = glue.iter().filter(|glue| glue.holds(pos, next)).peekable();
                if holding.peek().is_some() {
                    for entity_id in holding.map(|glue| glue.entity_id) {
                        if !used.contains(&entity_id) {
                            used.push(entity_id);
                        }
                    }
                    frontier.push_back(next.0);
                } else if attachment::takes(block, state, next_block, next_state, offset) {
                    frontier.push_back(next.0);
                }
            }
            blocks.push(at);
            if blocks.len() > MAX_ASSEMBLE_BLOCKS {
                return Err(format!(
                    "the contraption is over {MAX_ASSEMBLE_BLOCKS} blocks"
                ));
            }
        }
        if blocks.is_empty() {
            return Err("there is nothing to assemble".to_owned());
        }
        Ok((blocks, used))
    }

    /// Whether the chunk holding `pos` is loaded.
    fn chunk_loaded(&self, pos: Vector3<i32>) -> bool {
        self.level
            .is_chunk_loaded(&Vector2::new(pos.x >> 4, pos.z >> 4))
    }

    /// Freezes a world region into a sub-level, or with `include` only the
    /// listed blocks of it, with the glue entities `glue` by id.
    fn assemble_region(
        self: &Arc<Self>,
        min: Vector3<i32>,
        max: Vector3<i32>,
        include: Option<FxHashSet<BlockPos>>,
        glue: Vec<i32>,
    ) -> Result<u32, String> {
        let (min, max) = (
            Vector3::new(min.x.min(max.x), min.y.min(max.y), min.z.min(max.z)),
            Vector3::new(min.x.max(max.x), min.y.max(max.y), min.z.max(max.z)),
        );
        let size = max - min + Vector3::new(1, 1, 1);
        if size.x > MAX_ASSEMBLE_SPAN
            || size.z > MAX_ASSEMBLE_SPAN
            || (include.is_none()
                && i64::from(size.x) * i64::from(size.y) * i64::from(size.z) > MAX_ASSEMBLE_VOLUME)
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
            Some([[min.x, min.y, min.z], [max.x, max.y, max.z]]),
        );
        let world = Arc::clone(self);
        tokio::spawn(async move {
            world
                .copy_region_into_plot(id, min, max, include, glue)
                .await;
        });
        Ok(id)
    }

    /// Copies a region into a sub-level's parked chunks, parks its entities
    /// and its glue, clears the original, then networks the sub-level.
    async fn copy_region_into_plot(
        self: Arc<Self>,
        id: u32,
        min: Vector3<i32>,
        max: Vector3<i32>,
        include: Option<FxHashSet<BlockPos>>,
        glue: Vec<i32>,
    ) {
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
        let ParkedCopy { bounds } = match self.copy_blocks_into_plot(
            min,
            max,
            anchor,
            center,
            include.as_ref(),
        ) {
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
        // Another assembly of the same region may have taken its blocks
        // first: with nothing to carry, the sub-level goes.
        let Some(bounds) = bounds else {
            warn!("sublevel {id}: assembled nothing; the region was already taken");
            self.lock_sublevels().end_assembly(id);
            self.remove_sublevel(id);
            return;
        };
        self.park_entities_in_region(min, max, anchor, center, include.as_ref());
        self.park_glue(&glue, center.to_f64() - anchor.to_f64());
        self.clear_region(min, max, include.as_ref());
        self.lock_sublevels().end_assembly(id);

        // Sable's bounds are tight to the content; empty margins of the
        // region must not widen the client's render or collision box.
        self.lock_sublevels().tighten_bounds(id, bounds);
        let pivot_offset = [
            from_anchor.x as f32,
            from_anchor.y as f32,
            from_anchor.z as f32,
        ];
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
        // A block placed against the content is the sub-level's next, so the
        // read reaches one block past the bounds; they tighten to what is
        // found.
        for x in bounds[0] - 1..=bounds[3] + 1 {
            for y in bounds[1] - 1..=bounds[4] + 1 {
                for z in bounds[2] - 1..=bounds[5] + 1 {
                    let pos = BlockPos(Vector3::new(x, y, z));
                    let (block, state) = self.get_block_and_state(&pos);
                    if state.is_air() {
                        continue;
                    }
                    let signal = if aero::heater(block) {
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
        // Sable's heat map: blocks that no longer hold together part ways.
        // Every piece but the one keeping the sub-level becomes a sub-level of
        // its own, and a piece without mass goes. While pieces are on their
        // way, the content is read as it is.
        let mut pieces = if manager.is_splitting(id) {
            Vec::new()
        } else {
            pieces_of(&voxels)
        };
        if pieces.len() > 1 {
            let kept = kept_piece(&pieces, manager.root_of(id));
            pieces.swap(kept, 0);
            let mut kept = pieces.swap_remove(0).voxels;
            let mut moves = Vec::new();
            let mut discards = Vec::new();
            for piece in pieces {
                let bounds = parked_bounds(anchor, &piece.bounds);
                if piece.mass <= 0.0 {
                    discards.push(bounds);
                } else if let Some((new_id, delta)) = manager.split(id, bounds) {
                    moves.push((new_id, delta, bounds));
                } else {
                    kept.extend(piece.voxels);
                }
            }
            manager.tighten_bounds(id, tight_bounds(anchor, &kept));
            manager.set_content(
                id,
                Content {
                    pivot_offset,
                    voxels: kept,
                },
            );
            drop(manager);
            // Its own statement: the edits take the lock again.
            for bounds in discards {
                self.discard_piece(bounds);
            }
            for (new_id, delta, bounds) in moves {
                let world = Arc::clone(self);
                tokio::spawn(async move { world.move_piece(id, new_id, delta, bounds).await });
            }
            return;
        }
        // Sable keeps its bounds tight to the content: a block gone from an
        // edge shrinks the client's render and collision box.
        manager.tighten_bounds(id, tight_bounds(anchor, &voxels));
        manager.set_content(
            id,
            Content {
                pivot_offset,
                voxels,
            },
        );
    }

    /// Moves a piece of sub-level `old`, the blocks in its absolute parked
    /// `bounds`, into sub-level `new`'s plot, `delta` blocks over in the
    /// plotyard, then gives the new sub-level its content, the old one's
    /// motion, and lets both be seen.
    async fn move_piece(
        self: Arc<Self>,
        old: u32,
        new: u32,
        delta: Vector3<i32>,
        bounds: [i32; 6],
    ) {
        let Some(chunks) = self.lock_sublevels().chunks_of(new) else {
            return;
        };
        if !self.await_plot_chunks(&chunks).await {
            warn!("sublevel {new}: its plot chunks never loaded; the piece stays with {old}");
            self.lock_sublevels().remove(new);
            self.lock_sublevels().end_split(old);
            return;
        }
        for chunk in &chunks {
            self.clear_parked_chunk(*chunk);
        }
        let copy = BlockFlags::SKIP_BLOCK_ADDED_CALLBACK | BlockFlags::UPDATE_KNOWN_SHAPE;
        let clear = BlockFlags::NOTIFY_ALL
            | BlockFlags::SKIP_DROPS
            | BlockFlags::FORCE_STATE
            | BlockFlags::SKIP_BLOCK_ENTITY_REPLACED_CALLBACK;
        for x in bounds[0]..=bounds[3] {
            for y in bounds[1]..=bounds[4] {
                for z in bounds[2]..=bounds[5] {
                    let parked = BlockPos(Vector3::new(x, y, z));
                    let state = self.get_block_state_id(&parked);
                    if state == Block::AIR.default_state.id {
                        continue;
                    }
                    let to = BlockPos(parked.0 + delta);
                    self.set_block_state(&to, state, copy);
                    if let Some(block_entity) = self.get_block_entity(&parked) {
                        self.restore_block_entity_at(&block_entity, to);
                    }
                    self.set_block_state(&parked, Block::AIR.default_state.id, clear);
                }
            }
        }
        // Entities parked on the piece move with it.
        for entity in self.entities.load().iter() {
            let entity = entity.get_entity();
            let pos = entity.pos.load();
            let block = pos.floor_to_i32();
            if block.x >= bounds[0]
                && block.x <= bounds[3]
                && block.y >= bounds[1]
                && block.y <= bounds[4]
                && block.z >= bounds[2]
                && block.z <= bounds[5]
            {
                entity.set_pos(pos + delta.to_f64());
            }
        }
        let anchor = self
            .lock_sublevels()
            .anchor_of(new)
            .map(|anchor| anchor.floor_to_i32());
        let Some(anchor) = anchor else {
            return;
        };
        let Some(voxels) = self.content_from_plot(new, anchor) else {
            return;
        };
        let mut manager = self.lock_sublevels();
        if voxels.is_empty() {
            drop(manager);
            self.remove_sublevel(new);
            self.lock_sublevels().end_split(old);
            return;
        }
        let pivot_offset = manager.from_anchor_of(new).map_or([0.0; 3], |offset| {
            [offset.x as f32, offset.y as f32, offset.z as f32]
        });
        manager.set_content(
            new,
            Content {
                pivot_offset,
                voxels,
            },
        );
        manager.physics.inherit_velocity(old, new);
        manager.mark_ready(new);
        manager.end_split(old);
        manager.note_rescan(old);
        drop(manager);
        info!("sublevel {new} split off sublevel {old}");
    }

    /// Destroys a massless piece of a sub-level, with its drops.
    fn discard_piece(self: &Arc<Self>, bounds: [i32; 6]) {
        for x in bounds[0]..=bounds[3] {
            for y in bounds[1]..=bounds[4] {
                for z in bounds[2]..=bounds[5] {
                    let pos = BlockPos(Vector3::new(x, y, z));
                    if self.get_block_state_id(&pos) != Block::AIR.default_state.id {
                        self.set_block_state(
                            &pos,
                            Block::AIR.default_state.id,
                            BlockFlags::NOTIFY_ALL,
                        );
                    }
                }
            }
        }
    }

    /// Deals with what the sub-levels hit this tick: fragile blocks break,
    /// TNT primes, bells ring, knocks hard for their mass sound, and the
    /// sub-levels that lost blocks are read again.
    fn handle_impacts(self: &Arc<Self>, impacts: &[Impact]) {
        // One entry per block hit this tick, with the hardest hit on it, a
        // point that hit it and the sub-levels it belongs to: a platform
        // settling on ground reports thousands of contacts a tick, and a
        // block only needs breaking once. Parked blocks are looked up here
        // in one go: a lookup is a statement of its own elsewhere, as
        // changing a parked block locks the manager again.
        let mut hits: FxHashMap<BlockPos, Hit> = FxHashMap::default();
        let mut knocks = Vec::new();
        {
            let manager = self.lock_sublevels();
            for impact in impacts {
                if let Some(pos) = manager.parked_block(impact.sublevel, impact.block) {
                    let hit = hits.entry(pos).or_insert_with(|| Hit::new(impact));
                    hit.harder(impact.speed);
                    hit.owners.insert(impact.sublevel);
                }
                let target = match impact.target {
                    ImpactTarget::World([x, y, z]) => Some(BlockPos(Vector3::new(x, y, z))),
                    ImpactTarget::SubLevel { id, block } => manager.parked_block(id, block),
                };
                if let Some(pos) = target {
                    let hit = hits.entry(pos).or_insert_with(|| Hit::new(impact));
                    hit.harder(impact.speed);
                    if let ImpactTarget::SubLevel { id, .. } = impact.target {
                        hit.owners.insert(id);
                    }
                }
                if impact.impulse > 0.0 {
                    knocks.push(Knock {
                        sublevel: impact.sublevel,
                        block: impact.block,
                        point: impact.point,
                        force: impact.impulse / pumpkin_sublevel::SUBSTEP_DT,
                        lighter: impact.mass.min(impact.other_mass),
                    });
                }
            }
        }
        let mut rescans: FxHashSet<u32> = FxHashSet::default();
        for (pos, hit) in hits {
            if self.hit_block(&pos, hit.speed, hit.point) {
                rescans.extend(hit.owners);
            }
        }
        for id in rescans {
            self.rescan_sublevel(id);
        }
        self.sound_impacts(&knocks);
    }

    /// Lets a block take the consequences of being hit at `speed` on its
    /// side towards `point`: fragile blocks break, TNT primes, bells ring.
    /// Returns whether the block changed.
    fn hit_block(self: &Arc<Self>, pos: &BlockPos, speed: f32, point: [f32; 3]) -> bool {
        /// Sable's `FragileBlockCallback` threshold, in blocks per second.
        const FRAGILE_SPEED: f32 = 4.0;
        /// Sable's `ExplosiveBlockCallback` threshold.
        const TNT_SPEED: f32 = 5.0;

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
        // A bell's threshold is the fragile blocks' 4 m/s.
        if *block == Block::BELL {
            // The bell swings from the side that was hit. One parked on a
            // sub-level is rung where it stands, but heard where it was hit.
            let centre = pos.to_centered_f64();
            let from = Vector3::new(
                f64::from(point[0]),
                f64::from(point[1]),
                f64::from(point[2]),
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
    /// two ticks, and so many collisions per tick at most.
    fn sound_impacts(&self, knocks: &[Knock]) {
        /// Sable plays a contact's sounds when its force, the impulse over
        /// the substep, exceeds this multiple of the smaller mass.
        const FORCE_PER_MASS: f32 = 25.0;
        /// A block that just sounded stays quiet this many ticks.
        const QUIET_TICKS: i32 = 2;
        /// Sable caps the collisions it reports per tick.
        const MAX_PER_TICK: usize = 100;
        const VOLUME: f32 = 0.08;

        let mut manager = self.lock_sublevels();
        let tick = manager.tick;
        manager.forget_sounds_before(tick - QUIET_TICKS);
        let mut sounded = 0;
        for knock in knocks {
            if sounded >= MAX_PER_TICK {
                break;
            }
            if knock.force <= FORCE_PER_MASS * knock.lighter {
                continue;
            }
            if !manager.note_sound((knock.sublevel, knock.block), tick) {
                continue;
            }
            let Some(pos) = manager.parked_block(knock.sublevel, knock.block) else {
                continue;
            };
            let state = self.get_block_state_id(&pos);
            let block = Block::from_state_id(state);
            if *block == Block::AIR {
                continue;
            }
            sounded += 1;
            let at = Vector3::new(
                f64::from(knock.point[0]),
                f64::from(knock.point[1]),
                f64::from(knock.point[2]),
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
        include: Option<&FxHashSet<BlockPos>>,
    ) -> Result<ParkedCopy, Vector3<i32>> {
        let flags = BlockFlags::SKIP_BLOCK_ADDED_CALLBACK | BlockFlags::UPDATE_KNOWN_SHAPE;
        let mut tight: Option<[i32; 6]> = None;
        for x in min.x..=max.x {
            for y in min.y..=max.y {
                for z in min.z..=max.z {
                    let pos = BlockPos(Vector3::new(x, y, z));
                    if include.is_some_and(|include| !include.contains(&pos)) {
                        continue;
                    }
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
    fn clear_region(
        self: &Arc<Self>,
        min: Vector3<i32>,
        max: Vector3<i32>,
        include: Option<&FxHashSet<BlockPos>>,
    ) {
        let flags = BlockFlags::NOTIFY_ALL
            | BlockFlags::SKIP_DROPS
            | BlockFlags::FORCE_STATE
            | BlockFlags::SKIP_BLOCK_ENTITY_REPLACED_CALLBACK;
        for x in min.x..=max.x {
            for y in min.y..=max.y {
                for z in min.z..=max.z {
                    let pos = BlockPos(Vector3::new(x, y, z));
                    if include.is_some_and(|include| !include.contains(&pos)) {
                        continue;
                    }
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

    /// Brings the glue parked in a sub-level's `bounds` back into the
    /// world, its box turned and moved the way the blocks go: Simulated's
    /// `disassembleSubLevel`. The blocks' map is the pose's `pose`, the
    /// pivot `pivot` and `turns` clockwise quarter turns.
    fn unpark_glue(self: &Arc<Self>, bounds: [i32; 6], pivot: [f64; 3], pose: [f64; 3], turns: u8) {
        let low = [
            f64::from(bounds[0]),
            f64::from(bounds[1]),
            f64::from(bounds[2]),
        ];
        let high = [
            f64::from(bounds[3]) + 1.0,
            f64::from(bounds[4]) + 1.0,
            f64::from(bounds[5]) + 1.0,
        ];
        for entity in self.entities.load().iter() {
            let Some(glue) = entity.cast_any().downcast_ref::<GlueEntity>() else {
                continue;
            };
            let [min_x, min_y, min_z, max_x, max_y, max_z] = glue.bounds();
            let outside = |axis: usize| {
                [max_x, max_y, max_z][axis] <= low[axis]
                    || [min_x, min_y, min_z][axis] >= high[axis]
            };
            if (0..3).any(outside) {
                continue;
            }
            let lands = |corner: [f64; 3]| {
                std::array::from_fn::<f64, 3, _>(|axis| {
                    let local = [
                        corner[0] - pivot[0],
                        corner[1] - pivot[1],
                        corner[2] - pivot[2],
                    ];
                    (pose[axis] + placed(local, turns)[axis]).floor()
                })
            };
            let from = lands([min_x, min_y, min_z]);
            let to = lands([max_x, max_y, max_z]);
            let kind = glue.kind();
            glue.remove();
            GlueEntity::spawn(
                self,
                kind,
                [
                    from[0].min(to[0]),
                    from[1].min(to[1]),
                    from[2].min(to[2]),
                    from[0].max(to[0]),
                    from[1].max(to[1]),
                    from[2].max(to[2]),
                ],
            );
        }
    }

    /// Moves the glue entities `glue` into a sub-level's plot by the offset
    /// its blocks moved: Simulated's `assembleFromSingleBlock`.
    fn park_glue(self: &Arc<Self>, glue: &[i32], offset: Vector3<f64>) {
        for id in glue {
            with_glue_by_id(self, *id, |glue| {
                let (kind, bounds) = (glue.kind(), glue.bounds());
                glue.remove();
                GlueEntity::spawn(
                    self,
                    kind,
                    [
                        bounds[0] + offset.x,
                        bounds[1] + offset.y,
                        bounds[2] + offset.z,
                        bounds[3] + offset.x,
                        bounds[4] + offset.y,
                        bounds[5] + offset.z,
                    ],
                );
            });
        }
    }

    /// Moves every entity standing in the region to its parked position.
    /// Players stay: their position is client authoritative. With `include`,
    /// only entities on the contraption's own blocks go. Glue goes by its
    /// own box, in [`World::park_glue`].
    fn park_entities_in_region(
        &self,
        min: Vector3<i32>,
        max: Vector3<i32>,
        anchor: Vector3<i32>,
        center: Vector3<i32>,
        include: Option<&FxHashSet<BlockPos>>,
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
            // Glue goes by its own box, in `park_glue`.
            if entity.cast_any().downcast_ref::<GlueEntity>().is_some() {
                continue;
            }
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
            if include.is_some_and(|include| {
                !include.contains(&BlockPos(block))
                    && !include.contains(&BlockPos(block - Vector3::new(0, 1, 0)))
            }) {
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

    /// Melt a sub-level back into the world where it stands, as Simulated's
    /// assembler does: not at once, but once the sub-level has been turned
    /// to the nearest quarter turn and brought onto block centres, which it
    /// is driven to over the next ticks. One drifting out of the world,
    /// moving too fast or too far from the ground is refused. The sound of
    /// its stopping plays at `stop_sound`.
    pub fn disassemble_sublevel(
        self: &Arc<Self>,
        id: u32,
        stop_sound: Option<Vector3<f64>>,
    ) -> Result<u32, String> {
        self.check_disassembly(id)?;
        let Some(pose) = self.lock_sublevels().pose_of(id) else {
            return Err(format!("No sub-level {id}"));
        };
        let quarter = quarter_turn(pose.orientation);
        // The pivot's place within its block, as it was at assembly, turned
        // with the sub-level.
        let offset = self
            .lock_sublevels()
            .from_anchor_of(id)
            .map_or([0.5; 3], |offset| {
                pivot_place([offset.x, offset.y, offset.z], quarter.turns)
            });
        let mut manager = self.lock_sublevels();
        manager.start_disassembly(id, stop_sound);
        manager.set_alignment(
            id,
            Some(Alignment {
                orientation: quarter.upright(),
                offset,
            }),
        );
        Ok(id)
    }

    /// Whether a sub-level may be disassembled where it is.
    fn check_disassembly(&self, id: u32) -> Result<(), String> {
        /// Simulated's `disassemblyMaxVelocity`, and its angular limit.
        const MAX_SPEED: f32 = 5.0;
        const MAX_ANGULAR_SPEED: f32 = std::f32::consts::FRAC_PI_2;

        let manager = self.lock_sublevels();
        let Some(pose) = manager.pose_of(id) else {
            return Err(format!("No sub-level {id}"));
        };
        let Some(bounds) = manager.bounds_of(id) else {
            return Err(format!("No sub-level {id}"));
        };
        let Some(body) = manager.snapshot_of(id) else {
            return Err(format!("No sub-level {id}"));
        };
        drop(manager);
        if body.linear_velocity.iter().map(|v| v * v).sum::<f32>() > MAX_SPEED * MAX_SPEED
            || body.angular_velocity.iter().map(|v| v * v).sum::<f32>()
                > MAX_ANGULAR_SPEED * MAX_ANGULAR_SPEED
        {
            return Err("sub-level is moving too fast to disassemble".to_string());
        }
        // The corners of the content once turned to the quarter turn it
        // will be placed at.
        let turns = quarter_turn(pose.orientation).turns;
        let mut world: Option<[[f64; 3]; 2]> = None;
        for corner in 0..8 {
            let mut local = [0.0f64; 3];
            for axis in 0..3 {
                let edge = if corner & (1 << axis) == 0 {
                    bounds[axis]
                } else {
                    bounds[axis + 3] + 1
                };
                local[axis] = f64::from(edge) - pose.rotation_point[axis];
            }
            let place = placed(local, turns);
            let at = [
                (pose.position[0] + place[0]).floor() as i32,
                (pose.position[1] + place[1]).floor() as i32,
                (pose.position[2] + place[2]).floor() as i32,
            ];
            if !self.is_in_build_limit(BlockPos(Vector3::new(at[0], at[1], at[2]))) {
                return Err("sub-level has drifted outside the build limit".to_string());
            }
            world = Some(match world {
                None => [at.map(f64::from), at.map(f64::from)],
                Some([[x0, y0, z0], [x1, y1, z1]]) => [
                    [
                        x0.min(f64::from(at[0])),
                        y0.min(f64::from(at[1])),
                        z0.min(f64::from(at[2])),
                    ],
                    [
                        x1.max(f64::from(at[0])),
                        y1.max(f64::from(at[1])),
                        z1.max(f64::from(at[2])),
                    ],
                ],
            });
        }
        if let Some(world) = world
            && !self.ground_near(world)
        {
            return Err("sub-level is too far from the ground to disassemble".to_string());
        }
        Ok(())
    }

    /// Whether any block stands within a section's margin of the world
    /// region: Simulated's test against disassembly in mid-air. Read off
    /// the chunk sections, which know whether they hold anything.
    fn ground_near(&self, [[x0, y0, z0], [x1, y1, z1]]: [[f64; 3]; 2]) -> bool {
        /// The margin around the region, in sections.
        const MARGIN: i32 = 1;

        let low = |value: f64| (value.floor() as i32 - MARGIN * 16).max(-30_000_000);
        let high = |value: f64| (value.ceil() as i32 + MARGIN * 16).min(30_000_000);
        let bottom = self.get_bottom_y();
        let top = self.get_top_y();
        for x in (low(x0) >> 4)..=(high(x1) >> 4) {
            for z in (low(z0) >> 4)..=(high(z1) >> 4) {
                let Some(chunk) = self.level.loaded_chunks.get(&Vector2::new(x, z)) else {
                    continue;
                };
                let sections = chunk
                    .section
                    .block_sections
                    .read()
                    .unwrap_or_else(std::sync::PoisonError::into_inner);
                let min_y = chunk.section.min_y;
                for (index, section) in sections.iter().enumerate() {
                    let base = min_y + index as i32 * 16;
                    if base + 16 < low(y0).max(bottom) || base > high(y1).min(top) {
                        continue;
                    }
                    if !section.is_empty() {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// One tick of every disassembly under way: a sub-level that has held
    /// its alignment for a few ticks is placed; one that cannot reach it in
    /// time is let go.
    fn step_disassemblies(self: &Arc<Self>) {
        /// Simulated's `maxDisassemblyTicks` times five, and the ticks a
        /// sub-level must stay aligned before it is placed.
        const MAX_TICKS: u32 = 100;
        const READY_TICKS: u32 = 5;

        // Its own statement: the loop body takes the lock again.
        let disassembling = self.lock_sublevels().disassembling();
        for id in disassembling {
            let Some((ticks, ready)) = self.lock_sublevels().disassembly_tick(id) else {
                continue;
            };
            if ticks > MAX_TICKS {
                warn!("sublevel {id}: could not align to disassemble");
                self.lock_sublevels().cancel_disassembly(id);
            } else if ready > READY_TICKS {
                self.place_into_world(id);
            }
        }
    }

    /// Ends a sub-level's alignment and melts it back into the world.
    fn place_into_world(self: &Arc<Self>, id: u32) {
        let stop_sound = self.lock_sublevels().stop_sound_of(id);
        self.lock_sublevels().cancel_disassembly(id);
        if let Err(error) = self.check_disassembly(id) {
            warn!("sublevel {id}: {error}");
            return;
        }
        if let Some(at) = stop_sound {
            // Simulated's `disassembleSubLevel` sounds the contraption to a stop.
            play_named_sound(
                self,
                "simulated:block.physics_assembler.disassemble",
                SoundCategory::Blocks,
                &at,
                1.0,
                1.0,
                None,
            );
        }
        let Some(chunks) = self.lock_sublevels().chunks_of(id) else {
            return;
        };
        let world = Arc::clone(self);
        tokio::spawn(async move { world.copy_plot_into_region(id, chunks).await });
    }

    async fn copy_plot_into_region(self: Arc<Self>, id: u32, chunks: Vec<Vector2<i32>>) {
        // Hold the target world chunks as well: without a player nearby the
        // server unloads them, and the write-back would land nowhere.
        if !self.await_plot_chunks(&chunks).await {
            return;
        }
        let (bounds, pivot, pose, turns, rotation) = {
            let manager = self.lock_sublevels();
            let Some(bounds) = manager.bounds_of(id) else {
                return;
            };
            let Some(pose) = manager.pose_of(id) else {
                return;
            };
            let Some(from_anchor) = manager.from_anchor_of(id) else {
                return;
            };
            let quarter = quarter_turn(pose.orientation);
            let pivot = [
                pose.rotation_point[0],
                pose.rotation_point[1],
                pose.rotation_point[2],
            ];
            let placed = placed_pivot(pose.position, from_anchor, quarter.turns);
            (bounds, pivot, placed, quarter.turns, quarter.rotation())
        };

        // A parked block lands where the client renders it: turned by the
        // quarter turn and moved by the pose.
        let flags = BlockFlags::NOTIFY_ALL | BlockFlags::SKIP_DROPS;
        for bx in bounds[0]..=bounds[3] {
            for by in bounds[1]..=bounds[4] {
                for bz in bounds[2]..=bounds[5] {
                    let parked = BlockPos(Vector3::new(bx, by, bz));
                    let state = self.get_block_state_id(&parked);
                    if state == Block::AIR.default_state.id {
                        continue;
                    }
                    let place = placed(
                        [
                            f64::from(bx) - pivot[0],
                            f64::from(by) - pivot[1],
                            f64::from(bz) - pivot[2],
                        ],
                        turns,
                    );
                    let world_pos = BlockPos(Vector3::new(
                        (pose[0] + place[0]).floor() as i32,
                        (pose[1] + place[1]).floor() as i32,
                        (pose[2] + place[2]).floor() as i32,
                    ));
                    let state = pumpkin_data::BlockState::from_id(state);
                    self.set_block_state(&world_pos, state.rotate(rotation).id, flags);
                    if let Some(block_entity) = self.get_block_entity(&parked) {
                        self.restore_block_entity_at(&block_entity, world_pos);
                    }
                }
            }
        }

        self.unpark_glue(bounds, pivot, pose, turns);

        // Entities keep fractional positions; move them the way the blocks
        // went.
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
            let place = placed(
                [pos.x - pivot[0], pos.y - pivot[1], pos.z - pivot[2]],
                turns,
            );
            entity.set_pos(Vector3::new(
                pose[0] + place[0],
                pose[1] + place[1],
                pose[2] + place[2],
            ));
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
            info!("sublevel {id} disassembled");
        }
    }

    /// Whether `player` tracks the sub-level whose plot contains `chunk`.
    pub fn player_tracks_sublevel_chunk(&self, player: Uuid, chunk: Vector2<i32>) -> bool {
        self.lock_sublevels().player_tracks_chunk(player, chunk)
    }

    /// Converts a player-reported position in sub-level-local (parked)
    /// coordinates to its world position, if it lies in the plotyard, and
    /// remembers which sub-level the player stands on.
    pub fn to_world_position(&self, player: Uuid, position: Vector3<f64>) -> Vector3<f64> {
        let mut manager = self.lock_sublevels();
        manager.note_rider(player, position);
        manager.parked_to_world(position).unwrap_or(position)
    }

    /// Whether `player`'s reach covers the block at `position`, plus
    /// `additional_range`. A Sable client names a sub-level's blocks in the
    /// plot's coordinates, millions of blocks out: those are in reach where
    /// their sub-level shows them.
    pub fn block_within_reach(
        &self,
        player: &Player,
        position: &BlockPos,
        additional_range: f64,
    ) -> bool {
        self.lock_sublevels()
            .parked_to_world(position.0.to_f64() + Vector3::new(0.5, 0.5, 0.5))
            .map_or_else(
                || player.can_interact_with_block_at(position, additional_range),
                |shown| {
                    player.can_interact_with_block_at(
                        &BlockPos(shown.floor_to_i32()),
                        additional_range,
                    )
                },
            )
    }

    /// Sable's `ServerboundPunchSubLevelPacket`: the player punched the block
    /// `data` names, at `local` from a sub-level's place and towards
    /// `direction`. A sub-level is shoved; the one the player stands on is
    /// shoved back, and takes the shove when the player punched the world.
    pub fn punch_sublevel(&self, player: &Arc<Player>, data: &[u8]) {
        /// Sable's `sub_level_punch_cooldown_ticks`.
        const COOLDOWN: i32 = 3;

        let Some((block, local, direction)) = Punch::read(data) else {
            return;
        };
        // Sable: only a player with something to push against may punch.
        let entity = &player.living_entity.entity;
        if !(entity.on_ground.load(std::sync::atomic::Ordering::Relaxed)
            || entity.is_in_water()
            || player.is_flying())
        {
            return;
        }
        let Some(direction) = normalize(direction) else {
            return;
        };
        let uuid = player.gameprofile.id;
        let hit = {
            let mut manager = self.lock_sublevels();
            let standing = manager.ridden_by(uuid);
            let target = manager.containing(&BlockPos(block));
            // Nobody punches the sub-level they stand on, nor twice in a row.
            if standing == target || !manager.try_punch(uuid, COOLDOWN) {
                return;
            }
            // Standing on a sub-level, the client sends the direction in its
            // frame.
            let direction = standing
                .and_then(|id| manager.orientation_of(id))
                .map_or(direction, |orientation| rotate(orientation, direction));
            // A downward punch is weakened, so it cannot lift the player by
            // shoving what they stand on.
            let downward = direction.y < 0.0;
            let direction = Vector3::new(
                direction.x,
                direction.y
                    * if downward {
                        DOWNWARD_PUNCH_STRENGTH
                    } else {
                        1.0
                    },
                direction.z,
            );
            manager.punch(target, standing, entity.pos.load(), local, direction)
        };
        // The punch costs the hand its swing for a moment, and throws chips
        // of the block punched where it landed.
        let held = player.inventory.held_item();
        player.set_item_cooldown(held.get_item().registry_key, COOLDOWN);
        let count = i32::try_from((rand::random::<f64>() * 3.0) as u64).unwrap_or(0);
        let at = hit.unwrap_or(Vector3::new(
            f64::from(block.x) + 0.5,
            f64::from(block.y) + 0.5,
            f64::from(block.z) + 0.5,
        )) - direction * 0.1;
        self.block_particles(self.get_block_state_id(&BlockPos(block)), at, count);
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
            Some([[min.x, min.y, min.z], [max.x, max.y, max.z]]),
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

    #[test]
    fn tight_bounds_reach_from_the_lowest_to_the_highest_block() {
        // Blocks scattered round the anchor: the bounds run from the lowest
        // to the highest on each axis, whichever block each is on.
        let voxels = [
            Voxel::cube([3, -2, 0], 1.0),
            Voxel::cube([-1, 4, 2], 1.0),
            Voxel::cube([0, 0, -5], 1.0),
        ];
        assert_eq!(
            tight_bounds(Vector3::new(100, 50, -7), &voxels),
            [99, 48, -12, 103, 54, -5]
        );
        // Every chunk of a 33-block-wide region, three per axis.
        assert_eq!(
            content_chunks(&tight_bounds(
                Vector3::new(0, 0, 0),
                &[
                    Voxel::cube([-16, 0, -16], 1.0),
                    Voxel::cube([16, 0, 16], 1.0),
                ]
            ))
            .len(),
            9
        );
    }

    #[test]
    fn punch_payload_reads_a_packed_position_and_two_vectors() -> Result<(), &'static str> {
        // BlockPos.asLong packs x in 26 bits, z in 26 and y in 12.
        let packed: i64 = (i64::from(-44) & 0x3FF_FFFF) << 38
            | (i64::from(7) & 0x3FF_FFFF) << 12
            | i64::from(-2 & 0xFFF);
        let mut data = packed.to_be_bytes().to_vec();
        for value in [1.5f64, -0.25, 2.0, 0.0, 0.0, -1.0] {
            data.extend_from_slice(&value.to_be_bytes());
        }
        let (block, local, direction) = Punch::read(&data).ok_or("payload")?;
        assert_eq!(block, Vector3::new(-44, -2, 7));
        assert_eq!(local, Vector3::new(1.5, -0.25, 2.0));
        assert_eq!(direction, Vector3::new(0.0, 0.0, -1.0));
        // Truncated or padded payloads are refused.
        assert!(Punch::read(&data[..55]).is_none());
        data.push(0);
        assert!(Punch::read(&data).is_none());
        Ok(())
    }

    #[test]
    fn punches_move_light_bodies_more_than_heavy_ones() {
        // Sable's curve: no mass, no shove; one mass, the base impulse.
        assert_eq!(punch_curve(0.0), 0.0);
        assert!((punch_curve(1.0) - 1.0).abs() < 1.0e-9);
        // And the velocity change it buys falls as mass grows.
        let velocity = |mass: f64| punch_curve(mass) / mass;
        assert!(velocity(10.0) < velocity(1.0) && velocity(1000.0) < velocity(10.0));
    }

    #[test]
    fn pieces_hold_together_through_faces_and_edges_only() {
        // Two blocks touching on a face, two on an edge, and one touching
        // the first only at a corner: three pieces, the corner one alone.
        let voxels = [
            Voxel::cube([0, 0, 0], 1.0),
            Voxel::cube([1, 0, 0], 1.0),
            Voxel::cube([2, 1, 0], 1.0),
            Voxel::cube([2, 2, 2], 0.5),
        ];
        let mut pieces = pieces_of(&voxels);
        pieces.sort_by_key(|piece| piece.bounds[0]);
        assert_eq!(pieces.len(), 2);
        assert_eq!((pieces[0].voxels.len(), pieces[1].voxels.len()), (3, 1));
        assert_eq!((pieces[0].mass, pieces[1].mass), (3.0, 0.5));
        assert_eq!(pieces[0].bounds, [0, 0, 0, 2, 1, 0]);
        assert_eq!(pieces[1].bounds, [2, 2, 2, 2, 2, 2]);
        // Held together, a content is one piece.
        assert_eq!(pieces_of(&voxels[..3]).len(), 1);
    }

    #[test]
    fn a_split_piece_stays_where_it_was() -> Result<(), &'static str> {
        // A sub-level turned a quarter and placed off the pivot, split in
        // two: the piece's blocks map to the same world places from the new
        // plot as they did from the old one.
        let mut manager = SubLevelManager::new();
        let half = std::f32::consts::FRAC_PI_4;
        let pose = Vector3::new(10.0, 70.25, -4.5);
        let bounds = [-2, -1, -2, 1, 0, 1];
        let old = manager.assemble(pose, bounds, Vector3::new(0.5, 0.5, 0.5), None);
        let old_pivot = manager.anchor_of(old).expect("old sub-level");
        manager.physics.set_body_pose(
            old,
            [pose.x, pose.y, pose.z],
            [0.0, half.sin(), 0.0, half.cos()],
        );
        let old_pose = manager.pose_of(old).expect("pose");
        let piece = parked_bounds(old_pivot.floor_to_i32(), &[-2, -1, -2, -2, 0, 1]);
        let (new, delta) = manager.split(old, piece).expect("split");
        let new_pivot = manager
            .anchor_of(new)
            .expect("new sub-level")
            .floor_to_i32();
        let new_pose = manager.pose_of(new).expect("pose");
        for block in [[-2, -1, -2], [-2, 0, 1]] {
            let old_at = old_pivot.floor_to_i32() + Vector3::new(block[0], block[1], block[2]);
            let new_at = new_pivot + Vector3::new(block[0], block[1], block[2]);
            assert_eq!(new_at, old_at + delta);
            let place = |pose: &sable::Pose, pivot: Vector3<i32>, at: Vector3<i32>| {
                let [x, y, z] = pose.position;
                Vector3::new(x, y, z) + rotate(pose.orientation, at.to_f64() - pivot.to_f64())
            };
            assert_eq!(
                place(&old_pose, old_pivot.floor_to_i32(), old_at),
                place(&new_pose, new_pivot, new_at)
            );
        }
        // The old sub-level waits for the piece until it has moved.
        assert!(manager.is_splitting(old));
        manager.end_split(old);
        assert!(!manager.is_splitting(old));
        // What the recently split packet carries: the piece sits on the old
        // sub-level's rotation point, in the old one's frame, and keeps its
        // own.
        let new_level = manager
            .levels
            .iter()
            .find(|level| level.id == new)
            .ok_or("piece")?;
        let (from, split_pose) = new_level.split_from.ok_or("split origin")?;
        assert_eq!(from, old);
        assert_eq!(split_pose.position, old_pose.rotation_point);
        let pivot =
            manager.anchor_of(new).ok_or("piece")? + manager.from_anchor_of(new).ok_or("piece")?;
        assert_eq!(split_pose.rotation_point, [pivot.x, pivot.y, pivot.z]);
        // Sable's client continues the origin's motion from there: placed by
        // the origin's pose, the pose maps onto the piece's own.
        let local = [
            split_pose.position[0] - old_pose.rotation_point[0],
            split_pose.position[1] - old_pose.rotation_point[1],
            split_pose.position[2] - old_pose.rotation_point[2],
        ];
        let carried = rotate(
            old_pose.orientation,
            Vector3::new(local[0], local[1], local[2]),
        );
        for axis in 0..3 {
            let carried = [carried.x, carried.y, carried.z][axis] + old_pose.position[axis];
            assert!(
                (carried - new_pose.position[axis]).abs() < 1.0e-4,
                "{axis}: {carried}"
            );
        }
        Ok(())
    }

    #[test]
    fn the_piece_holding_the_heat_map_root_keeps_the_sub_level() {
        // A short, heavy piece and a longer, light one.
        let voxels = [
            Voxel::cube([0, 0, 0], 5.0),
            Voxel::cube([1, 0, 0], 5.0),
            Voxel::cube([5, 0, 0], 0.5),
            Voxel::cube([6, 0, 0], 0.5),
            Voxel::cube([7, 0, 0], 0.5),
        ];
        let pieces = pieces_of(&voxels);
        assert_eq!(pieces.len(), 2);
        // Whichever piece the heat map grows from stays, mass aside.
        assert_eq!(kept_piece(&pieces, Some([1, 0, 0])), 0);
        assert_eq!(kept_piece(&pieces, Some([5, 0, 0])), 1);
        // With the root gone every piece would part ways, so the largest
        // stays: the longer one, not the heavier one.
        assert_eq!(kept_piece(&pieces, Some([3, 0, 0])), 1);
        assert_eq!(kept_piece(&pieces, None), 1);
    }

    #[test]
    fn the_heat_map_root_follows_the_content() {
        let mut manager = SubLevelManager::new();
        let (id, _) = manager.spawn(Vector3::new(0.0, 64.0, 0.0), 4);
        let content = |voxels: [Voxel; 2]| Content {
            pivot_offset: [0.0; 3],
            voxels: voxels.to_vec(),
        };
        // Sable's heat map grows from the first block placed.
        manager.set_content(
            id,
            content([Voxel::cube([0, 0, 0], 1.0), Voxel::cube([1, 0, 0], 1.0)]),
        );
        assert_eq!(manager.root_of(id), Some([0, 0, 0]));
        // A rescan that keeps the block keeps the root.
        manager.set_content(
            id,
            content([Voxel::cube([1, 0, 0], 1.0), Voxel::cube([0, 0, 0], 1.0)]),
        );
        assert_eq!(manager.root_of(id), Some([0, 0, 0]));
        // With the block gone, the first block of what remains takes over.
        manager.set_content(
            id,
            content([Voxel::cube([2, 0, 0], 1.0), Voxel::cube([1, 0, 0], 1.0)]),
        );
        assert_eq!(manager.root_of(id), Some([2, 0, 0]));
        assert_eq!(manager.root_of(id + 1), None);
    }

    #[test]
    fn a_block_changed_in_the_plot_reaches_the_sub_level_whatever_the_bounds() {
        let mut manager = SubLevelManager::new();
        let (id, chunk) = manager.spawn(Vector3::new(0.0, 64.0, 0.0), 16);
        manager.mark_ready(id);
        let centre = plot_center_block(chunk);
        // In the plot, but above the slab: a block placed on it, or a piece a
        // split has just moved out, once the bounds tightened.
        let stone = Block::STONE.default_state.id;
        let placed = BlockPos(centre + Vector3::new(0, 3, 0));
        manager.note_edit(&placed, stone);
        let (updates, _) = manager.take_plot_changes();
        assert_eq!(
            updates.get(&id).map(Vec::as_slice),
            Some(&[(placed, stone)][..])
        );
        assert_eq!(manager.take_dirty(), vec![id]);
        // Another plot's block is nobody's.
        manager.note_edit(
            &BlockPos(centre + Vector3::new(16 << PLOT_LOG_SIZE, 3, 0)),
            stone,
        );
        assert_eq!(
            manager.take_plot_changes(),
            (PlotChanges::default(), Vec::new())
        );
        assert!(manager.take_dirty().is_empty());
    }

    #[test]
    fn quarter_turns_match_vanilla_rotations() {
        let half = std::f32::consts::FRAC_PI_4;
        for (quaternion, turns) in [
            ([0.0, 0.0, 0.0, 1.0], 0),
            ([0.0, half.sin(), 0.0, half.cos()], 3),
            ([0.0, 1.0, 0.0, 0.0], 2),
            ([0.0, -half.sin(), 0.0, half.cos()], 1),
        ] {
            let quarter = quarter_turn(quaternion);
            assert_eq!(quarter.turns, turns, "{quaternion:?}");
            let (x, z) = Rotation::from_index(turns).rotate_offset(3, 5);
            assert_eq!(
                placed([3.0, 7.0, 5.0], turns),
                [f64::from(x), 7.0, f64::from(z)]
            );
            // The upright orientation at that quarter turn is that quarter turn.
            assert_eq!(quarter_turn(quarter.upright()).turns, turns);
        }
    }
}
