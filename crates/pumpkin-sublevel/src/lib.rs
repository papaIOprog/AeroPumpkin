//! Native sub-level physics: rigid bodies that give a sub-level its pose.
//!
//! A sub-level is a plot of real chunks parked at remote coordinates (see
//! `pumpkin-core`'s `world::sublevel`); this crate only simulates the body
//! whose transform places that plot in the world. It knows nothing about
//! chunks, blocks or entities: the host describes a sub-level with a
//! [`Content`] of block-sized [`Voxel`]s and hands it the [`Voxel`]s of the
//! world around the body, its surroundings, whenever it moves.
//!
//! A sub-level stays where it was spawned until it has both content and
//! surroundings. Then it flies: it falls under [`GRAVITY`], bumps into the
//! blocks around it and into other sub-levels, rests on them, and the
//! balloons, thrusters and lift surfaces in its content act on it (see
//! `aero`). Every hit is reported as an [`Impact`] for the host to break
//! fragile blocks and the like.
//!
//! Threading: everything here runs sequentially on the world tick thread.
//! `rapier3d` is built without the `parallel` feature, so stepping never
//! dispatches onto rayon and cannot compete with the chunk generation pool.

mod aero;
mod voxel;

use std::collections::{HashMap, HashSet};
use std::num::NonZeroUsize;
use std::sync::Mutex;

use aero::PersistentForce;
pub use aero::{
    Air, Analysed, Content, GRAVITY, THRUSTER_AIRFLOW, THRUSTER_THRUST, UNIVERSAL_DRAG,
};

pub use voxel::{FloatingMaterial, Material, MaterialBox, Voxel, VoxelKind, VoxelShape};

use rapier3d::na::{Matrix3, Quaternion, UnitQuaternion, Vector3};
use rapier3d::prelude::*;

/// Sable gives sub-level colliders a friction of 0.525, which rapier
/// averages with the ground's 0.5 into 0.5125, then multiplies by the two
/// blocks' own friction. Rapier combines friction by multiplying the two
/// colliders' values, so each side carries a square root's share of the
/// pair's value.
const CONTENT_FRICTION: f32 = 0.525;
const GROUND_VS_CONTENT_FRICTION: f32 = 0.5125;

/// Seconds per world tick; each tick is [`SUBSTEPS`] physics steps, as in
/// Sable.
pub const TICK_DT: f32 = 1.0 / 20.0;
pub const SUBSTEPS: usize = 2;
pub const SUBSTEP_DT: f32 = TICK_DT / SUBSTEPS as f32;

/// The world is collided this many blocks beyond a sub-level's own blocks,
/// so a sub-level meets the terrain it is about to touch.
const SURROUNDINGS_MARGIN: f32 = 4.0;
/// A sub-level's velocity carries it this far ahead of its blocks when
/// deciding what surroundings to ask for.
const SURROUNDINGS_LOOKAHEAD: f32 = 0.25;
const SURROUNDINGS_LOOKAHEAD_CAP: f32 = 20.0;
/// Surroundings stay valid until the sub-level's blocks leave the inner
/// half of the margin.
const SURROUNDINGS_REFRESH: f32 = SURROUNDINGS_MARGIN / 2.0;
/// Sable's `sub_level_remove_min` and `sub_level_remove_max`: a sub-level
/// whose blocks pass either height is removed.
const REMOVE_MIN_Y: f32 = -10_000.0;
const REMOVE_MAX_Y: f32 = 100_000.0;
/// Contacts slower than this, in blocks per second, are resting rather than
/// hits, and go unreported.
const IMPACT_MIN_SPEED: f32 = 0.5;

/// A sub-level's own colliders meet the static world and other sub-levels,
/// never their own body; the static world only meets sub-levels.
fn content_groups() -> InteractionGroups {
    InteractionGroups::new(Group::GROUP_1, Group::GROUP_1 | Group::GROUP_2)
}
const STATIC_GROUPS: InteractionGroups = InteractionGroups::new(Group::GROUP_2, Group::GROUP_1);

/// Sable's integration parameters: two 25 ms steps per tick, stiff and
/// well-damped contacts, a thorough solver, and islands too small to sleep.
fn integration_parameters() -> IntegrationParameters {
    IntegrationParameters {
        dt: SUBSTEP_DT,
        contact_natural_frequency: 40.0,
        contact_damping_ratio: 5.0,
        num_solver_iterations: NonZeroUsize::new(18).unwrap_or(NonZeroUsize::MIN),
        num_internal_pgs_iterations: 2,
        num_internal_stabilization_iterations: 2,
        min_island_size: 128,
        max_ccd_substeps: 3,
        normalized_prediction_distance: 0.005,
        normalized_max_corrective_velocity: 50.0,
        normalized_allowed_linear_error: 0.0025,
        ..Default::default()
    }
}

/// A view of one body after a step, in blocks and blocks per second.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodySnapshot {
    pub id: u32,
    pub position: [f32; 3],
    /// Body rotation as a quaternion, `(x, y, z, w)`.
    pub rotation: [f32; 4],
    pub linear_velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
}

/// What the host should sample around a sub-level: every block in the
/// inclusive block range, as [`Voxel`]s counted from `min`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SurroundingsRequest {
    pub min: [i32; 3],
    pub max: [i32; 3],
}

/// What a sub-level's block ran into, for the host to act on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Impact {
    pub sublevel: u32,
    /// Where the block met what it hit, in the world.
    pub point: [f32; 3],
    /// How fast the block and what it hit were closing on each other, in
    /// blocks per second, when the substep began.
    pub speed: f32,
    /// The whole contact's impulse over the substep, in block masses times
    /// blocks per second, and the masses of the two bodies that met (the
    /// world's is infinite): a knock hard for its mass sounds.
    pub impulse: f32,
    pub mass: f32,
    pub other_mass: f32,
    /// The sub-level's block that hit, in its grid coordinates.
    pub block: [i32; 3],
    /// What it hit.
    pub target: ImpactTarget,
}

/// The far side of an [`Impact`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImpactTarget {
    /// A block of the world, in world coordinates.
    World([i32; 3]),
    /// A block of another sub-level, in that sub-level's grid coordinates.
    SubLevel { id: u32, block: [i32; 3] },
}

/// What one tick of the simulation produced.
#[derive(Debug, Clone, PartialEq)]
pub struct Step {
    pub bodies: Vec<BodySnapshot>,
    pub impacts: Vec<Impact>,
    /// Sub-levels whose bodies lost their footing in the numbers this tick,
    /// a pose or velocity that is not a number or far outside any world,
    /// and were put back where they last were, at rest.
    pub recovered: Vec<u32>,
    /// Sub-levels whose blocks left the heights Sable lets sub-levels exist
    /// at.
    pub gone: Vec<u32>,
}

/// Beyond this, in blocks or blocks per second, a body's place or motion is
/// no world's: Sable drops sub-levels that leave its height limits.
const SANITY_LIMIT: f32 = 1.0e6;

fn sane(values: [f32; 3]) -> bool {
    values
        .iter()
        .all(|value| value.is_finite() && value.abs() < SANITY_LIMIT)
}

/// Where a sub-level is being steered to, as Sable's disassembly drives
/// it: turned to a given orientation and brought onto the blocks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Alignment {
    /// The orientation as a quaternion, `(x, y, z, w)`.
    pub orientation: [f32; 4],
    /// Where within its block the pivot comes to rest once there, per
    /// axis: a pivot at a block's centre has `[0.5; 3]`, one at a corner
    /// `[0.0; 3]`. The pivot is brought onto that place.
    pub offset: [f32; 3],
}

/// How the alignment steers a sub-level: a damped spring of so many radians
/// per second natural frequency, critically damped. Sable's own gains are
/// joint-motor regularisation, which a free body cannot take over as they
/// are; these settle within a few ticks, and a stiffer spring oscillates
/// under the physics substep.
const ALIGN_TURN_FREQUENCY: f32 = 20.0;
const ALIGN_TURN_DAMPING: f32 = 1.0;
const ALIGN_SHIFT_FREQUENCY: f32 = 20.0;
const ALIGN_SHIFT_DAMPING: f32 = 1.0;
/// The most the alignment's springs accelerate a sub-level towards where
/// it should go, in radians per square second for the turn and blocks per
/// square second for the shift. Sable's turn motors are stiff and its
/// linear ones gentle; the damping is left alone, as it only ever calms.
const MAX_ALIGN_TURN: f32 = 60.0;
const MAX_ALIGN_SHIFT: f32 = 20.0;

/// What the creative physics staff does to a sub-level, as Simulated's
/// `PhysicsStaffServerHandler` keeps it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Hold {
    /// Held in place where it was locked.
    Locked,
    /// Pulled so that the point `anchor` of its content, in the body's
    /// own coordinates, comes to `goal` in the world, while the body is
    /// turned to `orientation`, a quaternion `(x, y, z, w)`.
    Dragged {
        anchor: Vector3<f32>,
        goal: Vector3<f32>,
        orientation: Quaternion<f32>,
    },
}

/// Simulated's `physics_staff_linear_stiffness` and
/// `physics_staff_linear_damping`: the gains of the motors that pull a
/// dragged sub-level.
const STAFF_LINEAR_STIFFNESS: f32 = 2650.0;
const STAFF_LINEAR_DAMPING: f32 = 125.0;
/// Simulated's `physics_staff_angular_stiffness` and
/// `physics_staff_angular_damping`.
const STAFF_ANGULAR_STIFFNESS: f32 = 10_000.0;
const STAFF_ANGULAR_DAMPING: f32 = 850.0;

/// What a collider is: a box of a sub-level's content, or of the static
/// surroundings sampled for one. Encoded in rapier's `user_data`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tag {
    Content { sublevel: u32, index: u32 },
    Static { sublevel: u32, index: u32 },
}

impl Tag {
    fn encode(self) -> u128 {
        let (kind, sublevel, index): (u32, u32, u32) = match self {
            Self::Content { sublevel, index } => (1, sublevel, index),
            Self::Static { sublevel, index } => (2, sublevel, index),
        };
        (u128::from(kind) << 64) | (u128::from(sublevel) << 32) | u128::from(index)
    }

    fn decode(data: u128) -> Option<Self> {
        let sublevel = u32::try_from((data >> 32) & 0xFFFF_FFFF).ok()?;
        let index = u32::try_from(data & 0xFFFF_FFFF).ok()?;
        match (data >> 64) & 0xFFFF_FFFF {
            1 => Some(Self::Content { sublevel, index }),
            2 => Some(Self::Static { sublevel, index }),
            _ => None,
        }
    }
}

struct SubLevel {
    id: u32,
    body: RigidBodyHandle,
    /// Colliders carrying the body's mass, one per box of content.
    colliders: Vec<ColliderHandle>,
    content: Option<Analysed>,
    /// The pivot's offset from the content grid's corner, in blocks.
    pivot_offset: [f32; 3],
    /// The static world sampled around the body: its colliders and the
    /// region they fill.
    surroundings: Option<Surroundings>,
    surroundings_pending: Option<SurroundingsRequest>,
    /// A world block within the surroundings changed since they were taken.
    refresh: bool,
    /// Forces working on the body for the whole tick, figured out once.
    persistent: Vec<PersistentForce>,
    /// Set while the body is steered to an orientation and onto the blocks.
    alignment: Option<Alignment>,
    /// Set while the physics staff holds the body.
    hold: Option<Hold>,
    /// The constraint that holds it, while one does.
    hold_joint: Option<ImpulseJointHandle>,
    /// The body's pose at the start of the tick, to fall back on.
    last_pose: (Isometry<f32>, bool),
    throttle: f32,
    /// The body feels gravity and air.
    active: bool,
}

struct Surroundings {
    region: SurroundingsRequest,
    colliders: Vec<ColliderHandle>,
    /// The world blocks holding liquid, which the body floats in.
    liquids: HashSet<[i32; 3]>,
}

/// Records the contact manifolds between sub-levels and what they hit while
/// the pipeline steps, with the velocities the bodies entered the substep
/// with.
#[derive(Default)]
struct ImpactCollector {
    manifolds: Mutex<Vec<RecordedManifold>>,
    before: HashMap<RigidBodyHandle, (Vector<f32>, Vector<f32>)>,
}

struct RecordedManifold {
    first: ColliderHandle,
    second: ColliderHandle,
    /// World-space contact points on the faces that met.
    points: Vec<[f32; 3]>,
    /// World-space contact normal, pointing out of the first collider.
    normal: [f32; 3],
    /// The two bodies' linear and angular velocities before the substep.
    velocities: [(Vector<f32>, Vector<f32>); 2],
    /// The whole manifold's impulse over the substep.
    impulse: f32,
}

impl EventHandler for ImpactCollector {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        _event: CollisionEvent,
        _contact_pair: Option<&ContactPair>,
    ) {
    }

    fn handle_contact_force_event(
        &self,
        _dt: Real,
        _bodies: &RigidBodySet,
        colliders: &ColliderSet,
        contact_pair: &ContactPair,
        _total_force_magnitude: Real,
    ) {
        let Some(first) = colliders.get(contact_pair.collider1) else {
            return;
        };
        // By the time this runs the solver has already stilled the bodies,
        // so the speed of the hit comes from what they brought to the step.
        let entering = |body: Option<RigidBodyHandle>| {
            body.and_then(|body| self.before.get(&body).copied())
                .unwrap_or_default()
        };
        let mut manifolds = self
            .manifolds
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        for manifold in &contact_pair.manifolds {
            let mut impulse = 0.0;
            let points = manifold
                .contacts()
                .iter()
                .filter(|contact| {
                    impulse += contact.data.impulse;
                    contact.data.impulse > 0.0
                })
                .map(|contact| {
                    let point = first.position() * contact.local_p1;
                    [point.x, point.y, point.z]
                })
                .collect::<Vec<_>>();
            if points.is_empty() {
                continue;
            }
            manifolds.push(RecordedManifold {
                first: contact_pair.collider1,
                second: contact_pair.collider2,
                points,
                normal: [
                    manifold.data.normal.x,
                    manifold.data.normal.y,
                    manifold.data.normal.z,
                ],
                velocities: [
                    entering(manifold.data.rigid_body1),
                    entering(manifold.data.rigid_body2),
                ],
                impulse,
            });
        }
    }
}

pub struct SubLevelSystem {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    islands: IslandManager,
    broad_phase: BroadPhaseMultiSap,
    narrow_phase: NarrowPhase,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    pipeline: PhysicsPipeline,
    query_pipeline: QueryPipeline,
    gravity: Vector<f32>,
    integration: IntegrationParameters,
    air: Air,
    /// The fixed body the staff's constraints tie a sub-level to.
    ground: RigidBodyHandle,
    sublevels: Vec<SubLevel>,
    next_id: u32,
    impacts: ImpactCollector,
    /// Collider friction values that multiply to Sable's pair frictions.
    content_friction_factor: f32,
    static_friction_factor: f32,
}

impl Default for SubLevelSystem {
    fn default() -> Self {
        Self::new(Air::default())
    }
}

impl SubLevelSystem {
    #[must_use]
    pub fn new(air: Air) -> Self {
        let content_friction_factor = CONTENT_FRICTION.sqrt();
        let mut system = Self {
            content_friction_factor,
            static_friction_factor: GROUND_VS_CONTENT_FRICTION / content_friction_factor,
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            islands: IslandManager::new(),
            broad_phase: BroadPhaseMultiSap::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            pipeline: PhysicsPipeline::new(),
            query_pipeline: QueryPipeline::new(),
            gravity: Vector::new(0.0, -GRAVITY, 0.0),
            integration: integration_parameters(),
            air,
            sublevels: Vec::new(),
            ground: RigidBodyHandle::invalid(),
            next_id: 1,
            impacts: ImpactCollector::default(),
        };
        system.ground = system.bodies.insert(RigidBodyBuilder::fixed().build());
        system
    }

    /// Spawns a massless dynamic body at `position`. It stays where it is
    /// until [`Self::set_content`] gives it blocks to carry.
    pub fn spawn(&mut self, position: [f32; 3]) -> u32 {
        let id = self.next_id;
        self.next_id += 1;

        let iso = Isometry::translation(position[0], position[1], position[2]);
        let body = RigidBodyBuilder::dynamic()
            .position(iso)
            .gravity_scale(0.0)
            .linear_damping(UNIVERSAL_DRAG)
            .angular_damping(UNIVERSAL_DRAG)
            .ccd_enabled(true)
            .build();
        let body_handle = self.bodies.insert(body);
        // Sable's bodies never sleep: the air acts on them every tick.
        if let Some(body) = self.bodies.get_mut(body_handle) {
            *body.activation_mut() = RigidBodyActivation::cannot_sleep();
        }

        self.sublevels.push(SubLevel {
            id,
            body: body_handle,
            colliders: Vec::new(),
            content: None,
            pivot_offset: [0.0; 3],
            surroundings: None,
            surroundings_pending: None,
            refresh: false,
            persistent: Vec::new(),
            alignment: None,
            hold: None,
            hold_joint: None,
            last_pose: (Isometry::identity(), false),
            throttle: 1.0,
            active: false,
        });
        id
    }

    /// Removes a sub-level's body and its surroundings.
    pub fn remove(&mut self, id: u32) -> bool {
        let Some(index) = self.sublevels.iter().position(|s| s.id == id) else {
            return false;
        };
        self.clear_surroundings(index);
        let sublevel = self.sublevels.swap_remove(index);
        self.bodies.remove(
            sublevel.body,
            &mut self.islands,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            true,
        );
        true
    }

    /// Gives sub-level `id` its content: the body takes the mass and shape
    /// of the blocks, and their balloons, thrusters and lift surfaces start
    /// acting on it once its surroundings arrive.
    pub fn set_content(&mut self, id: u32, content: Content) -> bool {
        let Some(index) = self.sublevels.iter().position(|s| s.id == id) else {
            return false;
        };
        for collider in self.sublevels[index].colliders.drain(..) {
            self.colliders
                .remove(collider, &mut self.islands, &mut self.bodies, true);
        }
        // The surroundings were sized for the old footprint.
        self.clear_surroundings(index);
        self.sublevels[index].surroundings_pending = None;
        self.sublevels[index].active = false;

        let pivot_offset = content.pivot_offset;
        let previous = self.sublevels[index].content.take();
        let mut analysed = content.analyse();
        if let Some(previous) = &previous {
            analysed.inherit(previous);
        }
        let body = self.sublevels[index].body;
        let mut colliders = Vec::with_capacity(analysed.boxes.len());
        for (index_of_box, boxed) in analysed.boxes.iter().enumerate() {
            let collider = ColliderBuilder::cuboid(
                boxed.half_extents[0],
                boxed.half_extents[1],
                boxed.half_extents[2],
            )
            .translation(vector![boxed.centre[0], boxed.centre[1], boxed.centre[2]])
            .mass(boxed.mass)
            .friction(self.content_friction_factor * boxed.material.friction)
            .friction_combine_rule(CoefficientCombineRule::Multiply)
            .restitution(boxed.material.restitution)
            .restitution_combine_rule(CoefficientCombineRule::Max)
            .collision_groups(content_groups())
            .active_events(ActiveEvents::CONTACT_FORCE_EVENTS)
            .contact_force_event_threshold(0.0)
            .user_data(
                Tag::Content {
                    sublevel: id,
                    index: index_of_box as u32,
                }
                .encode(),
            )
            .build();
            let handle = self
                .colliders
                .insert_with_parent(collider, body, &mut self.bodies);
            colliders.push(handle);
        }
        // Keep the mass current before the next step; the air forces read
        // it against the body's weight.
        if let Some(body) = self.bodies.get_mut(body) {
            body.recompute_mass_properties_from_colliders(&self.colliders);
        }
        let sublevel = &mut self.sublevels[index];
        sublevel.colliders = colliders;
        sublevel.pivot_offset = pivot_offset;
        sublevel.content = Some(analysed);
        self.activate(index);
        true
    }

    /// Sub-levels whose surroundings are missing or no longer reach around
    /// their blocks, with the region the host should sample for each. The
    /// host answers each with [`Self::set_surroundings`] or
    /// [`Self::cancel_surroundings`].
    pub fn take_surroundings_requests(&mut self) -> Vec<(u32, SurroundingsRequest)> {
        let mut requests = Vec::new();
        for sublevel in &mut self.sublevels {
            let Some(analysed) = &sublevel.content else {
                continue;
            };
            let Some(body) = self.bodies.get(sublevel.body) else {
                continue;
            };
            let blocks = world_bounds(body, analysed, sublevel.pivot_offset);
            let left_behind = sublevel.surroundings.as_ref().is_none_or(|surroundings| {
                let region = surroundings.region;
                (0..3).any(|axis| {
                    blocks.0[axis] < region.min[axis] as f32 + SURROUNDINGS_REFRESH
                        || blocks.1[axis] > region.max[axis] as f32 + 1.0 - SURROUNDINGS_REFRESH
                })
            });
            if sublevel.surroundings_pending.is_some() {
                // Past the inner half of what it was given, a body runs off
                // the edge of that world: it waits where it is until the
                // sample arrives. Its colliders stay, so nothing falls
                // through it meanwhile.
                if left_behind && let Some(body) = self.bodies.get_mut(sublevel.body) {
                    body.set_body_type(RigidBodyType::KinematicPositionBased, true);
                }
                continue;
            }
            if !left_behind && !sublevel.refresh {
                continue;
            }
            let request = SurroundingsRequest {
                min: expand_bounds(body, blocks, -1.0),
                max: expand_bounds(body, blocks, 1.0),
            };
            sublevel.surroundings_pending = Some(request);
            // A sample taken from here on sees every change so far; one
            // during its wait sets the flag again.
            sublevel.refresh = false;
            requests.push((sublevel.id, request));
        }
        requests
    }

    /// Notes that the world block at `pos` changed. A sub-level whose
    /// surroundings cover it asks for them again; the old ones stay until
    /// the new sample arrives.
    pub fn note_edit(&mut self, pos: [i32; 3]) {
        for sublevel in &mut self.sublevels {
            if sublevel.surroundings.as_ref().is_some_and(|surroundings| {
                (0..3).all(|axis| {
                    pos[axis] >= surroundings.region.min[axis]
                        && pos[axis] <= surroundings.region.max[axis]
                })
            }) {
                sublevel.refresh = true;
            }
        }
    }

    /// Delivers the sampled world for a pending request: `voxels` are the
    /// blocks of the region, counted from the request's `min`, and become
    /// the static ground and walls the sub-level collides with. A delivery
    /// that answers no pending request is dropped.
    pub fn set_surroundings(
        &mut self,
        id: u32,
        request: SurroundingsRequest,
        voxels: &[Voxel],
    ) -> bool {
        let Some(index) = self.sublevels.iter().position(|s| s.id == id) else {
            return false;
        };
        if self.sublevels[index].surroundings_pending != Some(request) {
            return false;
        }
        self.sublevels[index].surroundings_pending = None;
        self.clear_surroundings(index);

        let origin = [
            request.min[0] as f32,
            request.min[1] as f32,
            request.min[2] as f32,
        ];
        let liquids: HashSet<[i32; 3]> = voxels
            .iter()
            .filter(|voxel| voxel.liquid)
            .map(|voxel| std::array::from_fn(|axis| request.min[axis] + voxel.position[axis]))
            .collect();
        let mut colliders = Vec::new();
        for (index_of_box, boxed) in voxel::collision_boxes(voxels, origin)
            .into_iter()
            .enumerate()
        {
            let collider = ColliderBuilder::cuboid(
                boxed.half_extents[0],
                boxed.half_extents[1],
                boxed.half_extents[2],
            )
            .translation(vector![boxed.centre[0], boxed.centre[1], boxed.centre[2]])
            .friction(self.static_friction_factor * boxed.material.friction)
            .friction_combine_rule(CoefficientCombineRule::Multiply)
            .restitution(boxed.material.restitution)
            .restitution_combine_rule(CoefficientCombineRule::Max)
            .collision_groups(STATIC_GROUPS)
            .user_data(
                Tag::Static {
                    sublevel: id,
                    index: index_of_box as u32,
                }
                .encode(),
            )
            .build();
            colliders.push(self.colliders.insert(collider));
        }
        self.sublevels[index].surroundings = Some(Surroundings {
            region: request,
            colliders,
            liquids,
        });
        // Set going again, if it had to wait past the edge of the old ones.
        if let Some(body) = self.bodies.get_mut(self.sublevels[index].body) {
            body.set_body_type(RigidBodyType::Dynamic, true);
        }
        self.activate(index);
        true
    }

    /// Notes that the host will not answer a sub-level's pending request,
    /// so it can be asked again.
    pub fn cancel_surroundings(&mut self, id: u32) {
        if let Some(sublevel) = self.sublevels.iter_mut().find(|s| s.id == id) {
            sublevel.surroundings_pending = None;
        }
    }

    fn clear_surroundings(&mut self, index: usize) {
        if let Some(surroundings) = self.sublevels[index].surroundings.take() {
            for collider in surroundings.colliders {
                self.colliders
                    .remove(collider, &mut self.islands, &mut self.bodies, false);
            }
        }
    }

    /// Lets gravity and the air act on a sub-level once it has both its
    /// content and a world to stand on.
    fn activate(&mut self, index: usize) {
        let sublevel = &self.sublevels[index];
        if sublevel.active || sublevel.content.is_none() || sublevel.surroundings.is_none() {
            return;
        }
        let Some(body) = self.bodies.get_mut(sublevel.body) else {
            return;
        };
        body.set_gravity_scale(1.0, true);
        body.wake_up(true);
        self.sublevels[index].active = true;
    }

    /// Changes a body's velocity by `delta`, in blocks per second.
    pub fn apply_velocity_change(&mut self, id: u32, delta: [f32; 3]) -> bool {
        let Some(sublevel) = self.sublevels.iter().find(|s| s.id == id) else {
            return false;
        };
        let Some(body) = self.bodies.get_mut(sublevel.body) else {
            return false;
        };
        let delta: Vector<f32> = delta.into();
        let impulse = body.mass() * delta;
        body.apply_impulse(impulse, true);
        true
    }

    /// Starts or stops steering sub-level `id` to `alignment`: turned to
    /// the orientation and brought onto the blocks, a little every substep,
    /// until it is cleared.
    pub fn set_alignment(&mut self, id: u32, alignment: Option<Alignment>) -> bool {
        let Some(sublevel) = self.sublevels.iter_mut().find(|s| s.id == id) else {
            return false;
        };
        sublevel.alignment = alignment;
        true
    }

    /// How far sub-level `id` still is from its alignment: the angle to
    /// the orientation in radians, and the distance to the nearest place
    /// the pivot may rest, in blocks.
    #[must_use]
    pub fn alignment_error(&self, id: u32) -> Option<(f32, f32)> {
        let sublevel = self.sublevels.iter().find(|s| s.id == id)?;
        let alignment = sublevel.alignment?;
        let body = self.bodies.get(sublevel.body)?;
        let [x, y, z, w] = alignment.orientation;
        let target = rapier3d::na::UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z));
        let angle = body.rotation().angle_to(&target);
        let position = body.translation();
        let distance = (0..3)
            .map(|axis| (position[axis] - alignment.offset[axis]).rem_euclid(1.0))
            .map(|offset| offset.min(1.0 - offset))
            .fold(0.0f32, f32::max);
        Some((angle, distance))
    }

    /// Places sub-level `id`'s body at `position` with `rotation`.
    pub fn set_body_pose(&mut self, id: u32, position: [f64; 3], rotation: [f32; 4]) -> bool {
        let Some(body) = self
            .body_of(id)
            .and_then(|handle| self.bodies.get_mut(handle))
        else {
            return false;
        };
        let rotation = Quaternion::new(rotation[3], rotation[0], rotation[1], rotation[2]);
        body.set_position(
            Isometry::from_parts(
                rapier3d::na::Translation::from(Vector3::new(
                    position[0] as f32,
                    position[1] as f32,
                    position[2] as f32,
                )),
                rapier3d::na::UnitQuaternion::new_normalize(rotation),
            ),
            true,
        );
        true
    }

    /// Sub-level `to` takes up the motion of sub-level `from` at the
    /// place they share, as a piece split off from a sub-level does: the
    /// velocity at its centre of mass, and the spin.
    pub fn inherit_velocity(&mut self, from: u32, to: u32) -> bool {
        let (Some(source), Some(target)) = (self.body_of(from), self.body_of(to)) else {
            return false;
        };
        let Some(motion) = self.bodies.get(source).and_then(|source| {
            let target = self.bodies.get(target)?;
            Some((
                *source.linvel(),
                *source.angvel(),
                target.center_of_mass() - source.center_of_mass(),
            ))
        }) else {
            return false;
        };
        let Some(target) = self.bodies.get_mut(target) else {
            return false;
        };
        target.set_linvel(motion.0 + motion.1.cross(&motion.2), true);
        target.set_angvel(motion.1, true);
        true
    }

    /// The body's place in the world, which the pose reports.
    #[must_use]
    pub fn pivot_of(&self, id: u32) -> Option<[f32; 3]> {
        let sublevel = self.sublevels.iter().find(|s| s.id == id)?;
        self.bodies
            .get(sublevel.body)
            .map(|body| (*body.translation()).into())
    }

    /// How readily a push at the world `point` along `direction` moves
    /// sub-level `id`: Sable's `getInverseNormalMass`, the inverse of the
    /// mass a punch along that line meets.
    #[must_use]
    pub fn inverse_normal_mass(
        &self,
        id: u32,
        point: [f32; 3],
        direction: [f32; 3],
    ) -> Option<f32> {
        let sublevel = self.sublevels.iter().find(|s| s.id == id)?;
        let body = self.bodies.get(sublevel.body)?;
        let normal = Vector3::from(direction).try_normalize(1.0e-6)?;
        let lever = (Point::from(point) - body.center_of_mass()).cross(&normal);
        let properties = &body.mass_properties().local_mprops;
        let frame = *properties
            .principal_inertia_local_frame
            .to_rotation_matrix()
            .matrix();
        let inverses = Vector3::from_fn(|axis, _| 1.0 / properties.principal_inertia()[axis]);
        let inverse = frame * Matrix3::from_diagonal(&inverses) * frame.transpose();
        let rotation = *body.rotation().to_rotation_matrix().matrix();
        let inverse_world = rotation * inverse * rotation.transpose();
        Some(lever.dot(&(inverse_world * lever)) + properties.inv_mass)
    }

    /// Locks sub-level `id` in place where it stands, or lets it go when
    /// it is locked: Simulated's `toggleLock`. Whether it is locked now;
    /// `None` for no such sub-level.
    pub fn toggle_lock(&mut self, id: u32) -> Option<bool> {
        let index = self.sublevels.iter().position(|s| s.id == id)?;
        if self.sublevels[index].hold == Some(Hold::Locked) {
            self.let_go(index);
            Some(false)
        } else {
            self.lock(index);
            Some(true)
        }
    }

    /// Whether sub-level `id` is locked in place.
    #[must_use]
    pub fn is_locked(&self, id: u32) -> bool {
        self.sublevels
            .iter()
            .find(|s| s.id == id)
            .is_some_and(|s| s.hold == Some(Hold::Locked))
    }

    /// Lets sub-level `id` go, locked or dragged as it was.
    pub fn let_go_id(&mut self, id: u32) -> bool {
        let Some(index) = self
            .sublevels
            .iter()
            .position(|s| s.id == id && s.hold.is_some())
        else {
            return false;
        };
        self.let_go(index);
        true
    }

    /// What the staff does to every sub-level, with the sub-level's id.
    pub fn holds(&self) -> impl Iterator<Item = (u32, Hold)> + '_ {
        self.sublevels.iter().filter_map(|s| Some((s.id, s.hold?)))
    }

    fn lock(&mut self, index: usize) {
        self.let_go(index);
        let body_handle = self.sublevels[index].body;
        let Some(body) = self.bodies.get(body_handle) else {
            return;
        };
        // Held where it stands: the joint's frames coincide only at this pose.
        let joint = FixedJointBuilder::new()
            .local_frame1(*body.position())
            .local_frame2(Isometry::identity());
        let handle = self
            .impulse_joints
            .insert(self.ground, body_handle, joint, true);
        let sublevel = &mut self.sublevels[index];
        sublevel.hold = Some(Hold::Locked);
        sublevel.hold_joint = Some(handle);
    }

    fn let_go(&mut self, index: usize) {
        let sublevel = &mut self.sublevels[index];
        sublevel.hold = None;
        if let Some(joint) = sublevel.hold_joint.take() {
            self.impulse_joints.remove(joint, false);
        }
    }

    /// Drags sub-level `id`: the point `anchor` of its content, in the
    /// body's own coordinates, is pulled to `goal` in the world while the
    /// body is turned to `orientation`, a quaternion `(x, y, z, w)`, as
    /// Simulated's `drag`. A lock gives way.
    pub fn drag(
        &mut self,
        id: u32,
        anchor: [f32; 3],
        goal: [f32; 3],
        orientation: [f32; 4],
    ) -> bool {
        let Some(index) = self.sublevels.iter().position(|s| s.id == id) else {
            return false;
        };
        self.let_go(index);
        self.sublevels[index].hold = Some(Hold::Dragged {
            anchor: anchor.into(),
            goal: goal.into(),
            orientation: Quaternion::new(
                orientation[3],
                orientation[0],
                orientation[1],
                orientation[2],
            ),
        });
        true
    }

    /// Where the staff drags sub-level `id`'s anchor to now.
    pub fn set_drag_goal(&mut self, id: u32, goal: [f32; 3]) -> bool {
        let goal = goal.into();
        let Some(sublevel) = self.sublevels.iter_mut().find(|s| s.id == id) else {
            return false;
        };
        match sublevel.hold.as_mut() {
            Some(Hold::Dragged { goal: at, .. }) => *at = goal,
            _ => return false,
        }
        true
    }

    /// Re-ties every dragged sub-level for this substep.
    fn retie_dragged(&mut self) {
        for index in 0..self.sublevels.len() {
            if self.sublevels[index].active
                && matches!(self.sublevels[index].hold, Some(Hold::Dragged { .. }))
            {
                self.retie(index);
            }
        }
    }

    /// Re-ties a dragged sub-level for this substep, as Sable re-creates
    /// the constraint every physics tick. Only the staff's frame carries
    /// the orientation, so the angular motors turn the body into it, and
    /// the linear ones aim at the goal turned into that frame.
    fn retie(&mut self, index: usize) {
        let sublevel = &mut self.sublevels[index];
        let Hold::Dragged {
            anchor,
            goal,
            orientation,
        } = sublevel.hold.unwrap_or(Hold::Locked)
        else {
            return;
        };
        let (anchor, goal, orientation) = (anchor, goal, orientation);
        let body = sublevel.body;
        if let Some(joint) = sublevel.hold_joint.take() {
            self.impulse_joints.remove(joint, false);
        }
        let orientation = UnitQuaternion::new_normalize(orientation);
        let joint = GenericJointBuilder::new(JointAxesMask::empty())
            .local_frame1(Isometry::from_parts(Translation::identity(), orientation))
            .local_frame2(Isometry::from_parts(
                Translation::from(anchor),
                UnitQuaternion::identity(),
            ));
        let handle = self.impulse_joints.insert(self.ground, body, joint, true);
        self.sublevels[index].hold_joint = Some(handle);
        let Some(joint) = self.impulse_joints.get_mut(handle, false) else {
            return;
        };
        for axis in [JointAxis::AngX, JointAxis::AngY, JointAxis::AngZ] {
            joint.data.set_motor_position(
                axis,
                0.0,
                STAFF_ANGULAR_STIFFNESS,
                STAFF_ANGULAR_DAMPING,
            );
        }
        let local_goal = orientation.conjugate() * goal;
        for (axis, target) in [JointAxis::LinX, JointAxis::LinY, JointAxis::LinZ]
            .into_iter()
            .zip(local_goal.iter())
        {
            joint.data.set_motor_position(
                axis,
                *target,
                STAFF_LINEAR_STIFFNESS,
                STAFF_LINEAR_DAMPING,
            );
        }
    }

    /// Shoves sub-level `id` with an impulse at a world `point`, as a punch
    /// does.
    pub fn punch(&mut self, id: u32, point: [f32; 3], impulse: [f32; 3]) -> bool {
        let Some(sublevel) = self.sublevels.iter().find(|s| s.id == id) else {
            return false;
        };
        let Some(body) = self.bodies.get_mut(sublevel.body) else {
            return false;
        };
        body.apply_impulse_at_point(impulse.into(), Point::from(point), true);
        true
    }

    /// Scales how hard sub-level `id`'s propellers push, from 0 to 1 on top
    /// of their rpm; 1 by default.
    pub fn set_throttle(&mut self, id: u32, throttle: f32) -> bool {
        let Some(sublevel) = self.sublevels.iter_mut().find(|s| s.id == id) else {
            return false;
        };
        sublevel.throttle = throttle.clamp(0.0, 1.0);
        true
    }

    /// What the air does to sub-level `id`, or `None` if it has no content.
    #[must_use]
    pub fn aero_info(&self, id: u32) -> Option<AeroInfo> {
        let sublevel = self.sublevels.iter().find(|s| s.id == id)?;
        let content = sublevel.content.as_ref()?;
        let body = self.bodies.get(sublevel.body)?;
        let pressure = self.air.pressure(body.translation().y);
        Some(AeroInfo {
            mass: body.mass(),
            active: sublevel.active,
            throttle: sublevel.throttle,
            balloons: content
                .balloons
                .iter()
                .map(|balloon| BalloonInfo {
                    capacity: balloon.capacity,
                    fill: balloon.fill,
                    lift: balloon.lift(pressure),
                })
                .collect(),
            thrusters: content.thrusters.len(),
            surfaces: content.surfaces.len(),
        })
    }

    /// Advances the simulation by one world tick.
    pub fn step(&mut self) -> Step {
        let air = self.air;
        for sublevel in &mut self.sublevels {
            if let Some(content) = sublevel.content.as_mut() {
                content.advance(TICK_DT);
            }
        }
        let mut recovered = Vec::new();
        for sublevel in &mut self.sublevels {
            if !sublevel.active {
                continue;
            }
            if let Some(body) = self.bodies.get(sublevel.body)
                && sane((*body.translation()).into())
            {
                sublevel.last_pose = (*body.position(), true);
            }
        }
        for _ in 0..SUBSTEPS {
            self.retie_dragged();
            for sublevel in &mut self.sublevels {
                if !sublevel.active {
                    continue;
                }
                let Some(body) = self.bodies.get_mut(sublevel.body) else {
                    continue;
                };
                let Some(content) = sublevel.content.as_ref() else {
                    continue;
                };
                // The liquids' work follows the body as it moves: a thin
                // raft held to one tick-old forces rocks itself apart.
                sublevel.persistent.clear();
                if let Some(liquids) = sublevel
                    .surroundings
                    .as_ref()
                    .map(|surroundings| &surroundings.liquids)
                {
                    sublevel.persistent = content.buoyancy(body, liquids);
                }
                aero::apply_forces(
                    body,
                    content,
                    &sublevel.persistent,
                    sublevel.throttle,
                    air,
                    SUBSTEP_DT,
                );
                if let Some(alignment) = sublevel.alignment {
                    steer_to_alignment(body, alignment);
                }
            }
            // What the bodies bring to the substep, for gauging the hits in it.
            self.impacts.before.clear();
            for sublevel in &self.sublevels {
                if !sublevel.active {
                    continue;
                }
                if let Some(body) = self.bodies.get(sublevel.body) {
                    self.impacts
                        .before
                        .insert(sublevel.body, (*body.linvel(), *body.angvel()));
                }
            }
            self.pipeline.step(
                &self.gravity,
                &self.integration,
                &mut self.islands,
                &mut self.broad_phase,
                &mut self.narrow_phase,
                &mut self.bodies,
                &mut self.colliders,
                &mut self.impulse_joints,
                &mut self.multibody_joints,
                &mut self.ccd_solver,
                Some(&mut self.query_pipeline),
                &(),
                &self.impacts,
            );
            // Checked between the substeps: a body that just left the
            // numbers must not reach the next substep's broad phase, which
            // sizes its grid from the bodies' extents.
            for sublevel in &mut self.sublevels {
                let (pose, known) = sublevel.last_pose;
                let Some(body) = self.bodies.get_mut(sublevel.body) else {
                    continue;
                };
                let rotation = body.rotation();
                if sane((*body.translation()).into())
                    && sane((*body.linvel()).into())
                    && sane((*body.angvel()).into())
                    && rotation.coords.iter().all(|value| value.is_finite())
                {
                    continue;
                }
                body.set_position(pose, known);
                body.set_linvel(Vector::zeros(), known);
                body.set_angvel(Vector::zeros(), known);
                recovered.push(sublevel.id);
            }
        }
        Step {
            bodies: self.snapshots(),
            impacts: self.collect_impacts(),
            recovered,
            gone: self.past_height_limits(),
        }
    }

    /// Sub-levels whose blocks left the heights Sable lets sub-levels exist
    /// at.
    fn past_height_limits(&self) -> Vec<u32> {
        let mut gone = Vec::new();
        for sublevel in &self.sublevels {
            let (Some(analysed), Some(body)) = (&sublevel.content, self.bodies.get(sublevel.body))
            else {
                continue;
            };
            let blocks = world_bounds(body, analysed, sublevel.pivot_offset);
            if blocks.0[1] < REMOVE_MIN_Y || blocks.1[1] > REMOVE_MAX_Y {
                gone.push(sublevel.id);
            }
        }
        gone
    }

    /// Turns the contacts of the tick into impacts of sub-level blocks
    /// against the world or other sub-levels.
    fn collect_impacts(&self) -> Vec<Impact> {
        let manifolds = std::mem::take(
            &mut *self
                .impacts
                .manifolds
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner),
        );
        let mut impacts = Vec::new();
        for manifold in manifolds {
            let Some(first) = self.colliders.get(manifold.first) else {
                continue;
            };
            let Some(second) = self.colliders.get(manifold.second) else {
                continue;
            };
            let (Some(tag_first), Some(tag_second)) =
                (Tag::decode(first.user_data), Tag::decode(second.user_data))
            else {
                continue;
            };
            let (content, other, side) = match (tag_first, tag_second) {
                (content @ Tag::Content { .. }, other) => (content, other, 0),
                (other, content @ Tag::Content { .. }) => (content, other, 1),
                (Tag::Static { .. }, Tag::Static { .. }) => continue,
            };
            let Tag::Content {
                sublevel,
                index: box_index,
            } = content
            else {
                continue;
            };
            // The manifold normal points out of the first collider; make it
            // point out of the sub-level's box.
            self.describe_impacts(sublevel, box_index, &manifold, side, other, &mut impacts);
            // Two sub-levels meeting hit each other.
            if let Tag::Content {
                sublevel: other,
                index: other_box,
            } = other
            {
                self.describe_impacts(other, other_box, &manifold, 1 - side, content, &mut impacts);
            }
        }
        impacts
    }

    fn body_of(&self, sublevel: u32) -> Option<RigidBodyHandle> {
        self.sublevels
            .iter()
            .find(|s| s.id == sublevel)
            .map(|sublevel| sublevel.body)
    }

    /// The impacts of one contact manifold, one per block of the box
    /// `box_index` of sub-level `sublevel`'s, the `side`th collider of the
    /// pair, that met `other`: Sable collides block by block, and every
    /// block of the box on the face that touched within the contact patch
    /// touched too. The speed of a hit is how fast the two sides closed.
    fn describe_impacts(
        &self,
        sublevel: u32,
        box_index: u32,
        manifold: &RecordedManifold,
        side: usize,
        other: Tag,
        impacts: &mut Vec<Impact>,
    ) {
        let Some(sublevel) = self.sublevels.iter().find(|s| s.id == sublevel) else {
            return;
        };
        let Some(boxed) = sublevel
            .content
            .as_ref()
            .and_then(|content| content.boxes.get(box_index as usize))
        else {
            return;
        };
        let Some(body) = self.bodies.get(sublevel.body) else {
            return;
        };
        let (before_linear, before_angular) = manifold.velocities[side];
        // The manifold normal points out of the pair's first collider.
        let flip = if side == 0 { 1.0 } else { -1.0 };
        let outward: Vector<f32> = Vector::from(manifold.normal) * flip;
        // The other side's motion, when it is a sub-level as well.
        let other_body = match other {
            Tag::Content { sublevel, .. } => self.body_of(sublevel),
            Tag::Static { .. } => None,
        };
        let other_motion = other_body
            .filter(|handle| *handle != sublevel.body)
            .and_then(|handle| self.bodies.get(handle))
            .map(|other| (other.center_of_mass(), manifold.velocities[1 - side]));
        let other_mass = other_body
            .and_then(|handle| self.bodies.get(handle))
            .map_or(f32::INFINITY, RigidBody::mass);
        let mass = body.mass();
        let Some(patch) = contact_patch(
            boxed,
            body.position(),
            sublevel.pivot_offset.into(),
            manifold,
            outward,
        ) else {
            return;
        };

        let position = body.position();
        let pivot: Vector<f32> = sublevel.pivot_offset.into();
        let centre_of_mass = body.center_of_mass();
        for u in patch.first[0]..=patch.last[0] {
            for v in patch.first[1]..=patch.last[1] {
                let mut block = [0; 3];
                block[patch.axis] = patch.face_block;
                block[patch.tangents[0]] = u;
                block[patch.tangents[1]] = v;
                // The block's centre, brought onto the face.
                let mut local = [0.0; 3];
                for a in 0..3 {
                    local[a] = if a == patch.axis {
                        patch.face_offset
                    } else {
                        (block[a] as f32 + 0.5) - pivot[a]
                    };
                }
                let point = position * point![local[0], local[1], local[2]];
                let mut velocity = before_linear + before_angular.cross(&(point - centre_of_mass));
                if let Some((com, (linear, angular))) = other_motion {
                    velocity -= linear + angular.cross(&(point - com));
                }
                let speed = velocity.dot(&outward);
                if speed < IMPACT_MIN_SPEED {
                    continue;
                }
                // Half a block out from the face is the block it hit.
                let into = point + outward * 0.5;
                let target = match other {
                    Tag::Static { .. } => ImpactTarget::World([
                        into.x.floor() as i32,
                        into.y.floor() as i32,
                        into.z.floor() as i32,
                    ]),
                    Tag::Content {
                        sublevel: other,
                        index: other_box,
                    } => {
                        let Some(block) = self.block_at(other, other_box, [into.x, into.y, into.z])
                        else {
                            continue;
                        };
                        ImpactTarget::SubLevel { id: other, block }
                    }
                };
                impacts.push(Impact {
                    sublevel: sublevel.id,
                    point: [point.x, point.y, point.z],
                    speed,
                    impulse: manifold.impulse,
                    mass,
                    other_mass,
                    block,
                    target,
                });
            }
        }
    }
    /// The block of sub-level `sublevel`'s box `box_index` that a world
    /// point lies in, in grid coordinates.
    fn block_at(&self, sublevel: u32, box_index: u32, world: [f32; 3]) -> Option<[i32; 3]> {
        let sublevel = self.sublevels.iter().find(|s| s.id == sublevel)?;
        let boxed = sublevel.content.as_ref()?.boxes.get(box_index as usize)?;
        let body = self.bodies.get(sublevel.body)?;
        let local = body
            .position()
            .inverse_transform_point(&point![world[0], world[1], world[2]]);
        let pivot: Vector<f32> = sublevel.pivot_offset.into();
        let block = [
            (local.x + pivot.x).floor() as i32,
            (local.y + pivot.y).floor() as i32,
            (local.z + pivot.z).floor() as i32,
        ];
        Some(std::array::from_fn(|axis| {
            block[axis].clamp(boxed.blocks[0][axis], boxed.blocks[1][axis])
        }))
    }

    /// Snapshots without stepping, for listing.
    #[must_use]
    pub fn snapshots(&self) -> Vec<BodySnapshot> {
        self.sublevels
            .iter()
            .filter_map(|sublevel| {
                let body = self.bodies.get(sublevel.body)?;
                let rotation = body.rotation();
                Some(BodySnapshot {
                    id: sublevel.id,
                    position: (*body.translation()).into(),
                    rotation: [rotation.i, rotation.j, rotation.k, rotation.w],
                    linear_velocity: (*body.linvel()).into(),
                    angular_velocity: (*body.angvel()).into(),
                })
            })
            .collect()
    }
}

/// The part of a box's face that a contact manifold touched.
struct Patch {
    /// The face's axis, and the box's blocks along it next to the face.
    axis: usize,
    face_block: i32,
    /// Where the face lies along the axis, from the pivot.
    face_offset: f32,
    /// The face's other two axes, and the first and last block along each
    /// that the contact patch covers.
    tangents: [usize; 2],
    first: [i32; 2],
    last: [i32; 2],
}

/// Which blocks of `boxed`, placed by `position`, met a contact with
/// normal `outward` (pointing out of the box) at the points of `manifold`.
/// The face is the box's along the normal; the contact points bound the
/// patch of it that touched, in grid coordinates.
fn contact_patch(
    boxed: &MaterialBox,
    position: &Isometry<f32>,
    pivot: Vector<f32>,
    manifold: &RecordedManifold,
    outward: Vector<f32>,
) -> Option<Patch> {
    let local_outward = position.rotation.inverse().transform_vector(&outward);
    let axis = (0..3)
        .max_by(|a, b| local_outward[*a].abs().total_cmp(&local_outward[*b].abs()))
        .unwrap_or(0);
    let tangents = match axis {
        0 => [1, 2],
        1 => [0, 2],
        _ => [0, 1],
    };
    let side = if local_outward[axis] >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let face_block = if side > 0.0 {
        boxed.blocks[1][axis]
    } else {
        boxed.blocks[0][axis]
    };
    let face_offset = boxed.centre[axis] + side * boxed.half_extents[axis];

    let mut low = [f32::MAX; 2];
    let mut high = [f32::MIN; 2];
    for point in &manifold.points {
        let local = position.inverse_transform_point(&point![point[0], point[1], point[2]]) + pivot;
        for (i, tangent) in tangents.into_iter().enumerate() {
            low[i] = low[i].min(local[tangent]);
            high[i] = high[i].max(local[tangent]);
        }
    }
    let mut first = [0; 2];
    let mut last = [0; 2];
    for i in 0..2 {
        let tangent = tangents[i];
        first[i] = (low[i].floor() as i32).max(boxed.blocks[0][tangent]);
        last[i] = (high[i].ceil() as i32 - 1).min(boxed.blocks[1][tangent]);
    }
    (first[0] <= last[0] && first[1] <= last[1]).then_some(Patch {
        axis,
        face_block,
        face_offset,
        tangents,
        first,
        last,
    })
}

/// What a balloon does right now, for display.
#[derive(Debug, Clone, PartialEq)]
pub struct BalloonInfo {
    pub capacity: u32,
    /// Lifting gas in the balloon, in cubic metres.
    pub fill: f32,
    /// What the gas lifts, in block masses.
    pub lift: f32,
}

/// A sub-level's content worked out for the physics, for display.
#[derive(Debug, Clone, PartialEq)]
pub struct AeroInfo {
    pub mass: f32,
    /// The body feels gravity and air.
    pub active: bool,
    pub throttle: f32,
    pub balloons: Vec<BalloonInfo>,
    pub thrusters: usize,
    pub surfaces: usize,
}

/// One substep of steering a body to `alignment`: turned to the
/// orientation and brought onto block centres, each with its own damped
/// spring.
fn steer_to_alignment(body: &mut RigidBody, alignment: Alignment) {
    let [x, y, z, w] = alignment.orientation;
    let target = rapier3d::na::UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z));
    let properties = &body.mass_properties().local_mprops;
    let frame = *properties
        .principal_inertia_local_frame
        .to_rotation_matrix()
        .matrix();
    let inertia =
        frame * Matrix3::from_diagonal(&properties.principal_inertia()) * frame.transpose();
    let rotation = *body.rotation().to_rotation_matrix().matrix();
    let inertia_world = rotation * inertia * rotation.transpose();

    // Turn towards the orientation. Sable's joint motors cannot pull harder
    // than the solver lets them; the spring gives the same leeway by only
    // ever seeing a bounded error, so a sub-level lying on its side comes
    // round in a swing rather than in one step.
    let turn_kp = ALIGN_TURN_FREQUENCY * ALIGN_TURN_FREQUENCY;
    let turn_kd = 2.0 * ALIGN_TURN_DAMPING * ALIGN_TURN_FREQUENCY;
    let error: Vector3<f32> = (target * body.rotation().inverse()).scaled_axis();
    let spring = (error * turn_kp).cap_magnitude(MAX_ALIGN_TURN);
    let angular_acceleration: Vector3<f32> = spring - *body.angvel() * turn_kd;
    body.add_torque(inertia_world * angular_acceleration, true);

    // Slide onto the blocks: the pivot to its place within them.
    let shift_kp = ALIGN_SHIFT_FREQUENCY * ALIGN_SHIFT_FREQUENCY;
    let shift_kd = 2.0 * ALIGN_SHIFT_DAMPING * ALIGN_SHIFT_FREQUENCY;
    let position = body.translation();
    let offset = Vector3::from(alignment.offset);
    let goal = (position - offset).map(f32::round) + offset;
    let spring = ((goal - position) * shift_kp).cap_magnitude(MAX_ALIGN_SHIFT);
    let acceleration: Vector3<f32> = spring - *body.linvel() * shift_kd;
    body.add_force(acceleration * body.mass(), true);
}

/// The sub-level's blocks in world coordinates: the lowest and highest
/// points its grid bounds reach once the body has turned.
fn world_bounds(
    body: &RigidBody,
    analysed: &Analysed,
    pivot_offset: [f32; 3],
) -> ([f32; 3], [f32; 3]) {
    let mut min = [f32::MAX; 3];
    let mut max = [f32::MIN; 3];
    let Some(bounds) = analysed.bounds else {
        let position = body.translation();
        return (
            [position.x, position.y, position.z],
            [position.x, position.y, position.z],
        );
    };
    for corner in 0..8 {
        let mut local = [0.0; 3];
        for axis in 0..3 {
            let edge = if corner & (1 << axis) == 0 {
                bounds[0][axis] as f32
            } else {
                bounds[1][axis] as f32 + 1.0
            };
            local[axis] = edge - pivot_offset[axis];
        }
        let world = body.position() * point![local[0], local[1], local[2]];
        for axis in 0..3 {
            min[axis] = min[axis].min(world[axis]);
            max[axis] = max[axis].max(world[axis]);
        }
    }
    (min, max)
}

/// One side of the surroundings request: the sub-level's blocks grown by
/// the margin and by how far the body's velocity carries it.
fn expand_bounds(body: &RigidBody, blocks: ([f32; 3], [f32; 3]), direction: f32) -> [i32; 3] {
    let velocity = body.linvel();
    let mut edge = [0; 3];
    for axis in 0..3 {
        let drift = (velocity[axis] * SURROUNDINGS_LOOKAHEAD)
            .clamp(-SURROUNDINGS_LOOKAHEAD_CAP, SURROUNDINGS_LOOKAHEAD_CAP);
        let value = if direction < 0.0 {
            blocks.0[axis] - SURROUNDINGS_MARGIN + drift.min(0.0)
        } else {
            blocks.1[axis] + SURROUNDINGS_MARGIN + drift.max(0.0)
        };
        edge[axis] = if direction < 0.0 {
            value.floor() as i32
        } else {
            value.ceil() as i32
        };
    }
    edge
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() < 0.03,
            "expected ~{expected}, got {actual}"
        );
    }

    fn solid(x: i32, y: i32, z: i32) -> Voxel {
        Voxel::cube([x, y, z], 1.0)
    }

    fn wool(x: i32, y: i32, z: i32) -> Voxel {
        Voxel::cube([x, y, z], 0.25).with_kind(VoxelKind::Envelope)
    }

    fn slab(width: i32, depth: i32) -> Content {
        let mut voxels = Vec::new();
        for x in 0..width {
            for z in 0..depth {
                voxels.push(solid(x, 0, z));
            }
        }
        Content {
            pivot_offset: [width as f32 / 2.0, 0.5, depth as f32 / 2.0],
            voxels,
        }
    }

    /// A 5x5x5 wool shell around a 3x3x3 space, with a heater in its floor.
    fn balloon(output: f32) -> Content {
        let mut voxels = Vec::new();
        for x in -2i32..=2 {
            for y in -2i32..=3 {
                for z in -2i32..=2 {
                    if x.abs() == 2 || y.abs() == 2 || z.abs() == 2 {
                        voxels.push(wool(x, y, z));
                    }
                }
            }
        }
        voxels.push(Voxel::cube([0, -1, 0], 1.0).with_kind(VoxelKind::Heater { output }));
        Content {
            pivot_offset: [0.5; 3],
            voxels,
        }
    }

    /// Flat ground at `top`, filling whatever the request asks for.
    /// Every cell of a request, in order.
    fn request_region(request: &SurroundingsRequest) -> impl Iterator<Item = [i32; 3]> + use<'_> {
        (request.min[0]..=request.max[0]).flat_map(move |x| {
            (request.min[1]..=request.max[1])
                .flat_map(move |y| (request.min[2]..=request.max[2]).map(move |z| [x, y, z]))
        })
    }

    fn flat_ground(request: SurroundingsRequest, top: i32) -> Vec<Voxel> {
        let mut voxels = Vec::new();
        for x in request.min[0]..=request.max[0] {
            for y in request.min[1]..=request.max[1] {
                for z in request.min[2]..=request.max[2] {
                    if y < top {
                        voxels.push(Voxel::cube(
                            [x - request.min[0], y - request.min[1], z - request.min[2]],
                            2.0,
                        ));
                    }
                }
            }
        }
        voxels
    }

    /// Content, then matching flat ground, for sub-level `id`.
    fn land_on(system: &mut SubLevelSystem, id: u32, content: Content, top: i32) {
        assert!(system.set_content(id, content));
        let requests = system.take_surroundings_requests();
        assert_eq!(requests.len(), 1);
        let (_, request) = requests[0];
        assert!(system.set_surroundings(id, request, &flat_ground(request, top)));
    }

    /// The host's part of every tick: answer whatever surroundings the
    /// system asks for with flat ground.
    fn serve(system: &mut SubLevelSystem, top: i32) {
        serve_with(system, |request| flat_ground(*request, top));
    }

    fn serve_with(system: &mut SubLevelSystem, build: impl Fn(&SurroundingsRequest) -> Vec<Voxel>) {
        for (id, request) in system.take_surroundings_requests() {
            assert!(system.set_surroundings(id, request, &build(&request)));
        }
    }

    /// Steps until the body settles or `ticks` pass; returns its snapshot.
    fn settle(system: &mut SubLevelSystem, id: u32, ticks: usize) -> BodySnapshot {
        let mut body = system.snapshots()[0];
        for _ in 0..ticks {
            serve(system, 58);
            let step = system.step();
            body = step
                .bodies
                .into_iter()
                .find(|body| body.id == id)
                .unwrap_or(body);
        }
        body
    }

    #[test]
    fn a_body_outgrowing_its_surroundings_waits_for_the_next_sample() {
        // A slab given nothing to stand on falls; past the inner half of
        // what it was given, with the next sample still outstanding, it
        // waits instead of running off the edge of that world.
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 64.0, 0.0]);
        assert!(system.set_content(id, slab(4, 4)));
        let (_, request) = system.take_surroundings_requests().swap_remove(0);
        assert!(system.set_surroundings(id, request, &[]));
        let mut outstanding = Vec::new();
        let mut lowest = f32::MAX;
        for _ in 0..200 {
            let requests = system.take_surroundings_requests();
            if !requests.is_empty() {
                outstanding = requests;
            }
            system.step();
            lowest = lowest.min(system.snapshots()[0].position[1]);
        }
        assert_eq!(outstanding.len(), 1);
        assert!(lowest > request.min[1] as f32, "{lowest}");
        // Served, it carries on down to the ground.
        for (id, request) in outstanding {
            assert!(system.set_surroundings(id, request, &flat_ground(request, 50)));
        }
        // `settle` serves flat ground with its top at 58.
        let body = settle(&mut system, id, 200);
        assert!(
            body.position[1] > 58.0 && body.position[1] < 59.5,
            "{body:?}"
        );
    }

    #[test]
    fn a_sub_level_past_sable_height_limits_is_reported_gone() {
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 64.0, 0.0]);
        assert!(system.set_content(id, slab(4, 4)));
        assert!(system.step().gone.is_empty());
        assert!(system.set_body_pose(id, [0.0, -20_000.5, 0.0], [0.0, 0.0, 0.0, 1.0]));
        assert_eq!(system.step().gone, vec![id]);
    }

    #[test]
    fn a_locked_body_stays_put_until_let_go() {
        // Shoved and locked, the body keeps its place against the shove and
        // the staff; let go, a shove moves it again.
        let mut system = SubLevelSystem::new(Air::default());
        let id = airborne(&mut system, slab(4, 4), [0.0, 64.5, 0.0]);
        assert!(system.drag(id, [0.0; 3], [0.0, 64.5, 0.0], [0.0, 0.0, 0.0, 1.0]));
        settle(&mut system, id, 2);
        assert_eq!(system.toggle_lock(id), Some(true));
        assert!(system.is_locked(id));
        let locked = system.snapshots()[0].position;
        assert!(system.punch(id, [1.9, 64.5, 1.9], [40.0, 0.0, 0.0]));
        settle(&mut system, id, 20);
        let held = system.snapshots()[0];
        assert!(held.linear_velocity.iter().all(|value| value.abs() < 0.01));
        for (axis, at) in locked.iter().enumerate() {
            assert!((held.position[axis] - at).abs() < 0.05, "{axis}");
        }
        assert_eq!(system.toggle_lock(id), Some(false));
        assert!(!system.is_locked(id));
        assert!(system.punch(id, [1.9, 64.5, 1.9], [40.0, 0.0, 0.0]));
        settle(&mut system, id, 20);
        let moved = system.snapshots()[0];
        assert!(moved.position[0] > locked[0] + 1.0, "{moved:?}");
    }

    #[test]
    fn a_dragged_body_follows_its_goal_held_up_and_turned() {
        // Nothing holds the body up but the staff: dragged to a point in the
        // air it settles there, turned as asked, and falls once let go.
        let mut system = SubLevelSystem::new(Air::default());
        let id = airborne(&mut system, slab(4, 4), [0.0, 64.5, 0.0]);
        let eighth = std::f32::consts::FRAC_PI_4;
        let half = [0.0, eighth.sin(), 0.0, eighth.cos()];
        let goal = [6.0, 80.0, -2.0];
        assert!(system.drag(id, [0.0; 3], goal, half));
        settle(&mut system, id, 120);
        let dragged = system.snapshots()[0];
        for (axis, at) in goal.iter().enumerate() {
            assert!(
                (dragged.position[axis] - at).abs() < 0.1,
                "{axis}: {dragged:?}"
            );
        }
        // A quarter turn about up: the body's x axis now points along the
        // world's negative z.
        let [x, y, z, w] = dragged.rotation;
        let x_axis_z = 2.0 * (x * z - y * w);
        assert!(x_axis_z < -0.95, "{dragged:?}");
        assert!(system.let_go_id(id));
        settle(&mut system, id, 40);
        assert!(system.snapshots()[0].position[1] < goal[1] - 5.0);
    }

    #[test]
    fn stepping_moves_a_shoved_body() {
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 64.0, 0.0]);
        assert!(system.set_content(id, slab(4, 4)));
        assert!(system.apply_velocity_change(id, [4.0, 0.0, 0.0]));

        let step = system.step();
        assert_eq!(step.bodies.len(), 1);
        // 4 blocks/s for one 50 ms tick; with no ground yet there is no
        // drag, so the shove carries over whole.
        assert_close(step.bodies[0].position[0], 0.2);
        assert_close(step.bodies[0].position[1], 64.0);
        assert_close(step.bodies[0].linear_velocity[0], 4.0);
    }

    #[test]
    fn overlapping_bodies_shove_each_other_apart() {
        let mut system = SubLevelSystem::new(Air::default());
        // Two single blocks spawned on the same spot are both sub-levels:
        // they collide, and the solver pushes them apart.
        let first = system.spawn([-39.5, 150.5, -39.5]);
        let second = system.spawn([-39.5, 150.5, -39.5]);
        assert!(system.set_content(first, slab(1, 1)));
        assert!(system.set_content(second, slab(1, 1)));
        for (_, request) in system.take_surroundings_requests() {
            // Nothing below them: they only have each other.
            assert!(system.set_surroundings(
                if system.aero_info(first).is_some_and(|info| !info.active) {
                    first
                } else {
                    second
                },
                request,
                &[],
            ));
        }
        for _ in 0..20 {
            system.step();
        }
        let bodies = system.snapshots();
        let apart = (bodies[0].position[0] - bodies[1].position[0]).abs()
            + (bodies[0].position[1] - bodies[1].position[1]).abs()
            + (bodies[0].position[2] - bodies[1].position[2]).abs();
        assert!(apart > 0.9, "only {apart} apart");
    }

    #[test]
    fn removing_a_body_retires_its_id() {
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 64.0, 0.0]);
        assert!(system.remove(id));

        assert!(system.snapshots().is_empty());
        assert!(!system.apply_velocity_change(id, [1.0, 0.0, 0.0]));
        assert!(!system.remove(id));
    }

    #[test]
    fn content_weighs_its_blocks_and_hovers_until_grounded() {
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 100.0, 0.0]);
        assert!(system.set_content(id, slab(4, 6)));
        assert!(
            system
                .aero_info(id)
                .is_some_and(|info| (info.mass - 24.0).abs() < 0.03 && !info.active)
        );
        for _ in 0..40 {
            assert_close(system.step().bodies[0].position[1], 100.0);
        }
    }

    #[test]
    fn content_rests_on_the_ground() {
        let mut system = SubLevelSystem::new(Air::default());
        // A 4x4 slab centred at y = 64.5 has its underside at 64.
        let id = system.spawn([10.3, 64.5, -7.2]);
        land_on(&mut system, id, slab(4, 4), 64);
        assert!(system.aero_info(id).is_some_and(|info| info.active));

        for _ in 0..120 {
            let body = system.step().bodies[0];
            assert_close(body.position[0], 10.3);
            assert_close(body.position[1], 64.5);
            assert_close(body.position[2], -7.2);
        }
    }

    #[test]
    fn content_falls_until_it_meets_the_ground() -> Result<(), &'static str> {
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 80.5, 0.0]);
        land_on(&mut system, id, slab(4, 4), 64);

        let mut hardest = 0.0f32;
        let mut lowest = f32::MAX;
        let mut landing_impacts = Vec::new();
        for _ in 0..200 {
            serve(&mut system, 64);
            let step = system.step();
            let body = &step.bodies[0];
            lowest = lowest.min(body.position[1]);
            hardest = hardest.max(
                step.impacts
                    .iter()
                    .map(|impact| impact.speed)
                    .fold(0.0, f32::max),
            );
            landing_impacts.extend(
                step.impacts
                    .iter()
                    .filter(|impact| impact.speed > 10.0)
                    .copied(),
            );
        }
        let body = system.snapshots()[0];
        // Falls about 15 blocks, lands with its underside on the ground,
        // and stays there.
        assert!(lowest >= 64.4, "sank to {lowest}");
        assert_close(body.position[1], 64.5);
        assert!(body.linear_velocity[1].abs() < 0.05);
        // The landing is reported as an impact into a world block.
        assert!(hardest > 10.0, "hardest impact {hardest}");
        // The landing comes with the momentum it transferred, the falling
        // body's mass, and the world's infinite one.
        let landing = landing_impacts.into_iter().next().ok_or("impact")?;
        assert!(landing.impulse > 0.0 && landing.impulse / SUBSTEP_DT > 25.0 * landing.mass);
        assert_eq!(landing.other_mass, f32::INFINITY);
        Ok(())
    }

    #[test]
    fn a_dropped_shell_lands_and_stays_put() {
        // A 9x7x7 wool shell dropped from 150 blocks up: it lands flat, on
        // the spot, and does not creep.
        let mut system = SubLevelSystem::new(Air::default());
        let mut voxels = Vec::new();
        for x in -4i32..=4 {
            for y in -3i32..=3 {
                for z in -4i32..=4 {
                    if x.abs() == 4 || y.abs() == 3 || z.abs() == 4 {
                        voxels.push(wool(x, y, z));
                    }
                }
            }
        }
        let id = system.spawn([-16.0, 208.5, -271.0]);
        land_on(
            &mut system,
            id,
            Content {
                pivot_offset: [0.5; 3],
                voxels,
            },
            58,
        );
        // The wool shell drags the air, so it falls at under ten blocks a
        // second: 150 blocks down takes a while.
        let body = settle(&mut system, id, 400);
        assert_close(body.position[0], -16.0);
        assert_close(body.position[1], 61.5);
        assert_close(body.position[2], -271.0);
        assert!(body.linear_velocity.iter().all(|value| value.abs() < 0.01));
        assert!(body.angular_velocity.iter().all(|value| value.abs() < 0.01));
    }

    #[test]
    fn surroundings_follow_a_moving_body() {
        let mut system = SubLevelSystem::new(Air::default());
        // Ground far below, so the body keeps the shove instead of braking
        // on it.
        let id = system.spawn([0.0, 200.5, 0.0]);
        land_on(&mut system, id, slab(4, 4), 64);
        assert!(system.take_surroundings_requests().is_empty());

        assert!(system.apply_velocity_change(id, [8.0, 0.0, 0.0]));
        let mut x = 0.0;
        for _ in 0..80 {
            x = system.step().bodies[0].position[0];
        }
        assert!(x > 6.0, "only reached {x}");
        let requests = system.take_surroundings_requests();
        assert_eq!(requests.len(), 1);
        let request = requests[0].1;
        assert!((request.min[0] + request.max[0]) as f32 / 2.0 - x < 8.0);
        assert!(system.set_surroundings(id, request, &flat_ground(request, 64)));
    }

    #[test]
    fn stale_surroundings_are_refused() {
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 64.5, 0.0]);
        assert!(system.set_content(id, slab(4, 4)));
        let requests = system.take_surroundings_requests();
        let (_, request) = requests[0];
        // A request that was cancelled is no longer pending.
        system.cancel_surroundings(id);
        assert!(!system.set_surroundings(id, request, &flat_ground(request, 64)));
        // Asking again yields the same request.
        assert_eq!(system.take_surroundings_requests(), vec![(id, request)]);
        assert!(system.set_surroundings(id, request, &flat_ground(request, 64)));
        // Ground delivered twice is not what was asked for either.
        assert!(!system.set_surroundings(id, request, &flat_ground(request, 64)));
    }

    #[test]
    fn a_hot_enough_balloon_lifts_its_ship() -> Result<(), &'static str> {
        // The shell weighs 98 blocks of wool, 24.5 masses. Its 26 cells of
        // hot air lift 39 at full heat, so a weak heater leaves it on the
        // ground and a proper one takes off.
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 66.5, 0.0]);
        land_on(&mut system, id, balloon(10.0), 64);
        for _ in 0..400 {
            serve(&mut system, 64);
            system.step();
        }
        let info = system.aero_info(id).ok_or("content set")?;
        // Gas creeps in with a nine second time constant, so after twenty
        // seconds it is nearly all there.
        assert!(
            (info.balloons[0].fill - 10.0).abs() < 0.2,
            "{}",
            info.balloons[0].fill
        );
        assert_close(system.snapshots()[0].position[1], 66.5);

        // Well clear of the first ship, since sub-levels now collide.
        let id = system.spawn([40.0, 66.5, 0.0]);
        land_on(&mut system, id, balloon(26.0), 64);
        let mut rising = false;
        // Airtight blocks drag the air, so the ship rises at a walking
        // pace once the gas has had its nine seconds.
        for _ in 0..800 {
            serve(&mut system, 64);
            let body = system.step().bodies[1];
            rising |= body.linear_velocity[1] > 0.05;
        }
        let body = system.snapshots()[1];
        assert!(rising, "never left the ground");
        assert!(body.position[1] > 80.0, "only reached {}", body.position[1]);
        // The gas sits over the middle of the shell, so it rises straight,
        // give or take the shuffle of leaving the ground.
        assert!(
            (body.position[0] - 40.0).abs() < 0.5,
            "{}",
            body.position[0]
        );
        assert!(body.position[2].abs() < 0.5, "{}", body.position[2]);
        Ok(())
    }

    #[test]
    fn thrusters_push_along_their_facing_under_throttle() {
        let mut system = SubLevelSystem::new(Air::default());
        // Two pistons facing +x, flush with the deck at the back corners:
        // their push evens out, so the ship moves straight.
        let mut content = slab(4, 4);
        for z in [0, 3] {
            content.voxels[z as usize].kind = VoxelKind::Thruster {
                direction: [1.0, 0.0, 0.0],
                rpm: 256.0,
            };
        }
        let id = system.spawn([0.0, 64.5, 0.0]);
        land_on(&mut system, id, content, 64);

        // Propellers spin as soon as they have rpm, unless throttled down.
        assert!(system.set_throttle(id, 0.0));
        for _ in 0..20 {
            serve(&mut system, 64);
            system.step();
        }
        assert_close(system.snapshots()[0].linear_velocity[0], 0.0);

        assert!(system.set_throttle(id, 1.0));
        for _ in 0..10 {
            serve(&mut system, 64);
            system.step();
        }
        let body = system.snapshots()[0];
        assert!(body.linear_velocity[0] > 1.0, "{}", body.linear_velocity[0]);
        // The ship stays on the ground and level; friction at its underside
        // pitches it down a touch, as braking would.
        assert!(
            (body.position[1] - 64.5).abs() < 0.1,
            "{}",
            body.position[1]
        );
        assert_close(body.position[2], 0.0);
        assert!(body.rotation[0].abs() < 0.05 && body.rotation[2].abs() < 0.01);
    }

    #[test]
    fn an_off_centre_thruster_turns_the_body() {
        let mut system = SubLevelSystem::new(Air::default());
        // One piston at a corner of the deck, flush with it, pushes only
        // that corner: the ship yaws.
        let mut content = slab(4, 4);
        content.voxels[0].kind = VoxelKind::Thruster {
            direction: [1.0, 0.0, 0.0],
            rpm: 256.0,
        };
        let id = system.spawn([0.0, 64.5, 0.0]);
        land_on(&mut system, id, content, 64);
        assert!(system.set_throttle(id, 1.0));
        for _ in 0..10 {
            serve(&mut system, 64);
            system.step();
        }
        let body = system.snapshots()[0];
        assert!(
            body.angular_velocity[1].abs() > 0.01,
            "{:?}",
            body.angular_velocity
        );
    }

    #[test]
    fn a_ship_stopped_by_a_wall_reports_where_it_hit() -> Result<(), &'static str> {
        let mut system = SubLevelSystem::new(Air::default());
        // A 4x4 slab on the ground, shoved into a five block pillar that
        // stands three blocks ahead of its east face.
        let id = system.spawn([0.0, 64.5, 0.0]);
        assert!(system.set_content(id, slab(4, 4)));
        let pillar = |request: &SurroundingsRequest| {
            let mut voxels = flat_ground(*request, 64);
            for y in 64..=68 {
                voxels.push(Voxel::cube(
                    [5 - request.min[0], y - request.min[1], -request.min[2]],
                    2.0,
                ));
            }
            voxels
        };
        serve_with(&mut system, pillar);
        assert!(system.aero_info(id).is_some_and(|info| info.active));
        assert!(system.apply_velocity_change(id, [8.0, 0.0, 0.0]));

        let mut hit: Option<Impact> = None;
        for _ in 0..40 {
            serve_with(&mut system, pillar);
            for impact in system.step().impacts {
                if matches!(impact.target, ImpactTarget::World(_))
                    && impact.point[1] > 64.0
                    && hit.is_none_or(|known| impact.speed > known.speed)
                {
                    hit = Some(impact);
                }
            }
        }
        let hit = hit.ok_or("no impact reported")?;
        // The slab spans -2..2 and the pillar sits at x = 5: the ship rolls
        // three blocks, then its east face meets the pillar's west face.
        assert!(hit.point[0] > 3.9 && hit.point[0] < 5.1, "{:?}", hit.point);
        assert!(hit.speed > 2.0, "{}", hit.speed);
        let ImpactTarget::World([x, _, z]) = hit.target else {
            return Err("the ship hit something other than the world");
        };
        assert_eq!((x, z), (5, 0));
        assert_eq!(hit.block, [3, 0, 2]);
        Ok(())
    }

    #[test]
    fn a_cube_shatters_the_canopy_it_lands_on() {
        // A 9x7x9 stone cube falls from y 203 onto an oak whose 7x7 canopy
        // is one leaf thick at y 158, over a stone floor at y 150. The cube
        // covers 36 of the leaves: every one of them breaks, and with the
        // world sampled again the cube goes on to the floor.
        let mut system = SubLevelSystem::new(Air::default());
        let mut voxels = Vec::new();
        for x in -4i32..=4 {
            for y in -3..=3 {
                for z in -4..=4 {
                    voxels.push(Voxel::cube([x, y, z], 2.0));
                }
            }
        }
        // Blocks sit on world blocks, as an assembled region's do.
        let id = system.spawn([0.5, 203.5, 0.5]);
        assert!(system.set_content(
            id,
            Content {
                pivot_offset: [0.5; 3],
                voxels,
            }
        ));
        let fragile = Material {
            fragile: true,
            ..Material::DEFAULT
        };
        // The leaves still standing, in world coordinates.
        let mut leaves: Vec<[i32; 3]> = (-1..=5)
            .flat_map(|x| (-1..=5).map(move |z| [x, 158, z]))
            .collect();
        let world = |leaves: &[[i32; 3]], request: &SurroundingsRequest| {
            let inside = |[x, y, z]: [i32; 3]| {
                (0..3).all(|axis| {
                    request.min[axis] <= [x, y, z][axis] && [x, y, z][axis] <= request.max[axis]
                })
            };
            let relative =
                |[x, y, z]: [i32; 3]| [x - request.min[0], y - request.min[1], z - request.min[2]];
            let mut voxels = Vec::new();
            for x in request.min[0]..=request.max[0] {
                for z in request.min[2]..=request.max[2] {
                    if inside([x, 150, z]) {
                        voxels.push(Voxel::cube(relative([x, 150, z]), 2.0));
                    }
                }
            }
            for y in 151..=157 {
                if inside([5, y, 5]) {
                    voxels.push(Voxel::cube(relative([5, y, 5]), 0.5));
                }
            }
            for leaf in leaves {
                if inside(*leaf) {
                    voxels.push(Voxel::cube_of(relative(*leaf), 0.25, fragile));
                }
            }
            voxels
        };
        serve_with(&mut system, |request| world(&leaves, request));
        assert!(system.aero_info(id).is_some_and(|info| info.active));

        let mut shattered = std::collections::HashSet::new();
        for _ in 0..300 {
            serve_with(&mut system, |request| world(&leaves, request));
            for impact in system.step().impacts {
                // The host's part: a leaf hit hard enough breaks, and the
                // world around the body is sampled again.
                if impact.speed >= 4.0
                    && let ImpactTarget::World(target) = impact.target
                    && let Some(at) = leaves.iter().position(|leaf| *leaf == target)
                {
                    leaves.swap_remove(at);
                    shattered.insert(target);
                    system.note_edit(target);
                }
            }
        }
        let under: std::collections::HashSet<[i32; 3]> = (-1..=4)
            .flat_map(|x| (-1..=4).map(move |z| [x, 158, z]))
            .collect();
        assert_eq!(shattered, under);
        assert_eq!(leaves.len(), 7 * 7 - 36);
        // With the canopy gone from under it, the cube comes to rest on the
        // floor, where it started falling from, give or take the tip.
        let body = system.snapshots()[0];
        assert_close(body.position[1], 154.5);
        for horizontal in [body.position[0], body.position[2]] {
            assert!((horizontal - 0.5).abs() < 0.2, "{horizontal}");
        }
    }

    #[test]
    fn two_sub_levels_meeting_report_hitting_each_other() -> Result<(), &'static str> {
        // A stone cube dropped onto a slab that is a sub-level too: each
        // reports hitting the other, at the speed they closed.
        let mut system = SubLevelSystem::new(Air::default());
        let slab = (0..4)
            .flat_map(|x| {
                (0..4).map(move |z| {
                    Voxel::cube_of(
                        [x, 0, z],
                        1.0,
                        Material {
                            fragile: true,
                            ..Material::DEFAULT
                        },
                    )
                })
            })
            .collect::<Vec<_>>();
        let slab_id = system.spawn([0.0, 64.5, 0.0]);
        assert!(system.set_content(
            slab_id,
            Content {
                pivot_offset: [2.0, 0.5, 2.0],
                voxels: slab,
            }
        ));
        let cube = (0..2)
            .flat_map(|x| {
                (0..2).flat_map(move |y| (0..2).map(move |z| Voxel::cube([x, y, z], 2.0)))
            })
            .collect::<Vec<_>>();
        let cube_id = system.spawn([0.0, 70.5, 0.0]);
        assert!(system.set_content(
            cube_id,
            Content {
                pivot_offset: [1.0; 3],
                voxels: cube,
            }
        ));

        let mut cube_hit_slab = 0.0f32;
        let mut slab_hit_cube = 0.0f32;
        for _ in 0..120 {
            serve(&mut system, 64);
            for impact in system.step().impacts {
                match (impact.sublevel, impact.target) {
                    (sublevel, ImpactTarget::SubLevel { id, .. })
                        if sublevel == cube_id && id == slab_id =>
                    {
                        cube_hit_slab = cube_hit_slab.max(impact.speed);
                    }
                    (sublevel, ImpactTarget::SubLevel { id, .. })
                        if sublevel == slab_id && id == cube_id =>
                    {
                        slab_hit_cube = slab_hit_cube.max(impact.speed);
                    }
                    _ => {}
                }
            }
        }
        assert!(cube_hit_slab > 5.0, "{cube_hit_slab}");
        assert!(
            (cube_hit_slab - slab_hit_cube).abs() < 0.01,
            "{cube_hit_slab}"
        );
        // The two-block cube comes to rest on the slab.
        let cube = system
            .snapshots()
            .into_iter()
            .find(|body| body.id == cube_id)
            .ok_or("cube")?;
        assert_close(cube.position[1], 66.0);
        Ok(())
    }

    fn floating_block(material: FloatingMaterial) -> Content {
        Content {
            pivot_offset: [0.5; 3],
            voxels: vec![Voxel::cube([0, 0, 0], 1.0).with_kind(VoxelKind::Floating {
                material,
                scale: 1.0,
            })],
        }
    }

    /// Content dropped into open air, with nothing to land on.
    fn airborne(system: &mut SubLevelSystem, content: Content, at: [f32; 3]) -> u32 {
        let id = system.spawn(at);
        assert!(system.set_content(id, content));
        let requests = system.take_surroundings_requests();
        assert_eq!(requests.len(), 1);
        let (_, request) = requests[0];
        assert!(system.set_surroundings(id, request, &[]));
        id
    }

    #[test]
    fn end_stone_and_levitite_hold_their_own_weight() {
        // One block of either lifts more than it weighs, but never enough
        // to rise: the lift gives way before its centre would accelerate
        // upwards. Sable's drag nudge leaves them sinking a slow tenth of a
        // block per second, which is Aeronautics' behaviour too.
        for material in [FloatingMaterial::END_STONE, FloatingMaterial::LEVITITE] {
            let mut system = SubLevelSystem::new(Air::default());
            let id = airborne(&mut system, floating_block(material), [0.0, 100.0, 0.0]);
            for _ in 0..200 {
                system.step();
            }
            let body = system.snapshots()[0];
            assert!(
                body.position[1] < 100.0 && body.position[1] > 98.5,
                "{material:?}: {:?}",
                body.position
            );
            assert!(
                (body.linear_velocity[1] + 0.13).abs() < 0.05,
                "{material:?}: {:?}",
                body.linear_velocity
            );
            system.remove(id);
        }
    }

    #[test]
    fn airtight_blocks_drag_the_air() -> Result<(), &'static str> {
        // A wool shell falls far slower than its weight alone would take
        // it: every airtight block drags. Stone of the same weight does not.
        let shell = |kind| {
            let mut voxels = Vec::new();
            for x in -4i32..=4 {
                for y in -3i32..=3 {
                    for z in -4i32..=4 {
                        if x.abs() == 4 || y.abs() == 3 || z.abs() == 4 {
                            voxels.push(Voxel::cube([x, y, z], 0.25).with_kind(kind));
                        }
                    }
                }
            }
            Content {
                pivot_offset: [0.5; 3],
                voxels,
            }
        };
        let mut system = SubLevelSystem::new(Air::default());
        // Within the air: the drag thins with the pressure, which is nil
        // near the build limit.
        let wool = airborne(&mut system, shell(VoxelKind::Envelope), [0.0, 100.0, 0.0]);
        let stone = airborne(&mut system, shell(VoxelKind::Solid), [100.0, 100.0, 0.0]);
        for _ in 0..300 {
            system.step();
        }
        let bodies = system.snapshots();
        let wool_body = bodies.iter().find(|body| body.id == wool).ok_or("wool")?;
        let stone_body = bodies.iter().find(|body| body.id == stone).ok_or("stone")?;
        assert!(
            wool_body.linear_velocity[1] > -10.0,
            "{:?}",
            wool_body.linear_velocity
        );
        assert!(
            stone_body.linear_velocity[1] < -40.0,
            "{:?}",
            stone_body.linear_velocity
        );
        Ok(())
    }

    #[test]
    fn sub_levels_float_in_liquids_by_their_density() -> Result<(), &'static str> {
        // Every block in a liquid is lifted by 10.5 per unit of volume
        // against its mass and dragged towards rest: planks (half a mass
        // per block) rise, stone (two) sinks, both slowly.
        let cube = |mass| {
            (0..2)
                .flat_map(|x| {
                    (0..2).flat_map(move |y| (0..2).map(move |z| Voxel::cube([x, y, z], mass)))
                })
                .collect::<Vec<_>>()
        };
        let mut system = SubLevelSystem::new(Air::default());
        let planks = system.spawn([0.0, 0.0, 0.0]);
        assert!(system.set_content(
            planks,
            Content {
                pivot_offset: [1.0; 3],
                voxels: cube(0.5),
            }
        ));
        let stone = system.spawn([100.0, 0.0, 0.0]);
        assert!(system.set_content(
            stone,
            Content {
                pivot_offset: [1.0; 3],
                voxels: cube(2.0),
            }
        ));
        let water = |request: &SurroundingsRequest| {
            request_region(request)
                .map(|cell| {
                    Voxel::liquid([
                        cell[0] - request.min[0],
                        cell[1] - request.min[1],
                        cell[2] - request.min[2],
                    ])
                })
                .collect::<Vec<_>>()
        };
        serve_with(&mut system, water);
        for _ in 0..100 {
            serve_with(&mut system, water);
            system.step();
        }
        let bodies = system.snapshots();
        let planks = bodies
            .iter()
            .find(|body| body.id == planks)
            .ok_or("planks")?;
        let stone = bodies.iter().find(|body| body.id == stone).ok_or("stone")?;
        assert!(
            planks.linear_velocity[1] > 0.3,
            "{:?}",
            planks.linear_velocity
        );
        assert!(planks.position[1] > 0.5, "{:?}", planks.position);
        assert!(
            stone.linear_velocity[1] < -0.5,
            "{:?}",
            stone.linear_velocity
        );
        assert!(
            stone.linear_velocity[1] > -10.0,
            "{:?}",
            stone.linear_velocity
        );
        Ok(())
    }

    #[test]
    fn alignment_turns_a_quarter_and_centres_the_pivot() -> Result<(), &'static str> {
        // A slab resting off-centre, told to turn a quarter: it turns to
        // within Sable's tolerance and settles with its pivot back on its
        // block corner, while staying on the ground.
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([10.3, 64.5, -7.2]);
        land_on(&mut system, id, slab(4, 4), 64);
        let half = std::f32::consts::FRAC_PI_4;
        // The slab's pivot sits at a block corner.
        assert!(system.set_alignment(
            id,
            Some(Alignment {
                orientation: [0.0, half.sin(), 0.0, half.cos()],
                offset: [0.0, 0.5, 0.0],
            })
        ));
        let mut settled = 0;
        for _ in 0..80 {
            serve(&mut system, 64);
            system.step();
            let (angle, distance) = system.alignment_error(id).ok_or("aligning")?;
            settled = if angle <= 0.07 && distance <= 0.2 {
                settled + 1
            } else {
                0
            };
        }
        assert!(settled >= 40, "held its alignment for only {settled} ticks");
        let body = system.snapshots()[0];
        assert_close(body.position[1], 64.5);
        assert!(
            (body.position[0] - 10.0).abs() < 0.2,
            "{}",
            body.position[0]
        );
        assert!(
            (body.position[2] - -7.0).abs() < 0.2,
            "{}",
            body.position[2]
        );

        // Let go, the slab stays where it was put.
        assert!(system.set_alignment(id, None));
        assert!(system.alignment_error(id).is_none());
        Ok(())
    }

    #[test]
    fn a_punch_off_centre_turns_the_body_and_meets_more_mass() -> Result<(), &'static str> {
        // A 3x3 slab of 1-mass blocks on the ground. Its centre of mass is
        // the body's place: a punch there only slides it; the same punch at
        // a corner also yaws it, and Sable's inverse normal mass, which sets
        // how hard a punch lands, is the larger for it.
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 64.5, 0.0]);
        land_on(&mut system, id, slab(3, 3), 64);
        let centre = [0.0, 64.5, 0.0];
        let corner = [1.4, 64.5, 1.4];
        let straight = system
            .inverse_normal_mass(id, centre, [1.0, 0.0, 0.0])
            .ok_or("body")?;
        let turned = system
            .inverse_normal_mass(id, corner, [1.0, 0.0, 0.0])
            .ok_or("body")?;
        assert!((straight - 1.0 / 9.0).abs() < 1.0e-4, "{straight}");
        assert!(turned > straight, "{turned}");

        assert!(system.punch(id, centre, [9.0, 0.0, 0.0]));
        let body = system.snapshots()[0];
        assert_close(body.linear_velocity[0], 1.0);
        assert!(body.angular_velocity.iter().all(|value| value.abs() < 0.01));
        // The yaw a corner punch gives: the lever times the impulse, over
        // the slab's inertia about the vertical (1/6 per block, plus each
        // block's distance from the centre), about 0.93 rad/s.
        assert!(system.punch(id, corner, [9.0, 0.0, 0.0]));
        let body = system.snapshots()[0];
        assert_close(body.linear_velocity[0], 2.0);
        assert_close(body.angular_velocity[1], 0.93);
        Ok(())
    }

    #[test]
    fn collider_tags_round_trip() {
        for tag in [
            Tag::Content {
                sublevel: 7,
                index: 3,
            },
            Tag::Static {
                sublevel: 70_000,
                index: 0,
            },
        ] {
            assert_eq!(Tag::decode(tag.encode()), Some(tag));
        }
        assert_eq!(Tag::decode(0), None);
    }
}
