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
const SUBSTEP_DT: f32 = TICK_DT / SUBSTEPS as f32;

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
    /// Contact point in the world.
    pub point: [f32; 3],
    /// How fast the sub-level's block was moving into what it hit, in
    /// blocks per second.
    pub speed: f32,
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
}

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
            let points = manifold
                .contacts()
                .iter()
                .filter(|contact| contact.data.impulse > 0.0)
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
        Self {
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
            next_id: 1,
            impacts: ImpactCollector::default(),
        }
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
            if sublevel.surroundings_pending.is_some() {
                continue;
            }
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
        for _ in 0..SUBSTEPS {
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
        }
        Step {
            bodies: self.snapshots(),
            impacts: self.collect_impacts(),
        }
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
    fn content_falls_until_it_meets_the_ground() {
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 80.5, 0.0]);
        land_on(&mut system, id, slab(4, 4), 64);

        let mut hardest = 0.0f32;
        let mut lowest = f32::MAX;
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
        }
        let body = system.snapshots()[0];
        // Falls about 15 blocks, lands with its underside on the ground,
        // and stays there.
        assert!(lowest >= 64.4, "sank to {lowest}");
        assert_close(body.position[1], 64.5);
        assert!(body.linear_velocity[1].abs() < 0.05);
        // The landing is reported as an impact into a world block.
        assert!(hardest > 10.0, "hardest impact {hardest}");
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
