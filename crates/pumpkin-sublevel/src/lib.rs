//! Native sub-level physics: rigid bodies that give a sub-level its pose.
//!
//! A sub-level is a plot of real chunks parked at remote coordinates (see
//! `pumpkin-core`'s `world::sublevel`); this crate only simulates the body
//! whose transform places that plot in the world. It knows nothing about
//! chunks, blocks or entities: the host describes a sub-level with a
//! [`Content`] of block-sized [`Voxel`]s and hands it [`Ground`] sampled
//! around the body.
//!
//! A sub-level stays where it was spawned until it has both content and
//! ground. Then it flies: it falls under [`GRAVITY`], rests on the ground,
//! and the balloons, thrusters and lift surfaces in its content act on it
//! (see `aero`). Test platforms that never get content keep hovering.
//!
//! Threading: everything here runs sequentially on the world tick thread.
//! `rapier3d` is built without the `parallel` feature, so stepping never
//! dispatches onto rayon and cannot compete with the chunk generation pool.

mod aero;

pub use aero::{
    Air, Content, GRAVITY, THRUSTER_AIRFLOW, THRUSTER_THRUST, UNIVERSAL_DRAG, Voxel, VoxelKind,
};

use rapier3d::prelude::*;

/// Seconds per world tick, the fixed step the simulation advances by.
pub const TICK_DT: f32 = 1.0 / 20.0;

/// Ground is sampled this many blocks beyond a sub-level's footprint.
const GROUND_MARGIN: i32 = 8;
/// Coulomb friction between a sub-level and the ground.
const GROUND_FRICTION: f32 = 0.5;
/// The underside of a sub-level's boxes is probed for ground this often.
const GROUND_SAMPLE_SPACING: f32 = 2.0;
/// How often the sinking corners of a sub-level are pushed back per tick.
/// Corner pushes swing the body, so the passes take a while to agree.
const GROUND_CONTACT_ITERATIONS: usize = 32;

/// A sub-level's own colliders never meet each other or other sub-levels:
/// two assembled regions may be spawned overlapping, and contact resolution
/// would shove both off the exact spot their blocks came from. The ground
/// is not a collider at all (see [`resolve_ground`]), so nothing else needs
/// to reach them.
const CONTENT_GROUPS: InteractionGroups = InteractionGroups::new(Group::GROUP_1, Group::GROUP_2);

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

/// The terrain under a sub-level, one height per block column.
#[derive(Debug, Clone, PartialEq)]
pub struct Ground {
    /// Block column of the middle sample.
    pub center: [i32; 2],
    /// Samples reach this many blocks out from the centre on each axis.
    pub radius: i32,
    /// Terrain heights row by row (z, then x), `2 * radius + 1` to a side.
    pub heights: Vec<f32>,
}

impl Ground {
    /// The terrain height at a horizontal position, between samples.
    /// `None` outside the sampled area.
    #[must_use]
    fn height_at(&self, x: f32, z: f32) -> Option<f32> {
        let span = (self.radius * 2) as f32;
        // Sample j sits at j blocks from the first sample, which is a
        // block centre `radius` blocks out from the middle one.
        let fx = x - (self.center[0] as f32 + 0.5) + self.radius as f32;
        let fz = z - (self.center[1] as f32 + 0.5) + self.radius as f32;
        if !(0.0..=span).contains(&fx) || !(0.0..=span).contains(&fz) {
            return None;
        }
        let side = (self.radius * 2 + 1) as usize;
        let column = fx.floor() as usize;
        let row = fz.floor() as usize;
        let sample = |row: usize, column: usize| self.heights[row * side + column];
        let tx = fx - column as f32;
        let tz = fz - row as f32;
        let north = sample(row, column) * (1.0 - tx) + sample(row, (column + 1).min(side - 1)) * tx;
        let south = sample((row + 1).min(side - 1), column) * (1.0 - tx)
            + sample((row + 1).min(side - 1), (column + 1).min(side - 1)) * tx;
        Some(north * (1.0 - tz) + south * tz)
    }
}

/// The ground the host should sample for a sub-level, as in [`Ground`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GroundRequest {
    pub center: [i32; 2],
    pub radius: i32,
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

struct SubLevel {
    id: u32,
    body: RigidBodyHandle,
    /// The colliders carrying the body's mass: the placeholder box, or the
    /// content's boxes.
    colliders: Vec<ColliderHandle>,
    content: Option<aero::Analysed>,
    /// The terrain under the body, as the host last sampled it.
    ground: Option<Ground>,
    /// Half the width of the ground the content needs, in blocks.
    ground_radius: i32,
    /// The host was asked for this ground and has not answered yet.
    ground_pending: Option<GroundRequest>,
    throttle: f32,
    active: bool,
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
}

impl Default for SubLevelSystem {
    fn default() -> Self {
        Self::new(Air::default())
    }
}

impl SubLevelSystem {
    #[must_use]
    pub fn new(air: Air) -> Self {
        // One integration per world tick instead of rapier's 60 Hz default.
        let integration = IntegrationParameters {
            dt: TICK_DT,
            ..Default::default()
        };
        Self {
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
            integration,
            air,
            sublevels: Vec::new(),
            next_id: 1,
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
            .build();
        let body_handle = self.bodies.insert(body);

        self.sublevels.push(SubLevel {
            id,
            body: body_handle,
            colliders: Vec::new(),
            content: None,
            ground: None,
            ground_radius: 0,
            ground_pending: None,
            throttle: 0.0,
            active: false,
        });
        id
    }

    /// Removes a sub-level's body.
    pub fn remove(&mut self, id: u32) -> bool {
        let Some(index) = self.sublevels.iter().position(|s| s.id == id) else {
            return false;
        };
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
    /// acting on it once ground arrives.
    pub fn set_content(&mut self, id: u32, content: Content) -> bool {
        let Some(index) = self.sublevels.iter().position(|s| s.id == id) else {
            return false;
        };
        for collider in self.sublevels[index].colliders.drain(..) {
            self.colliders
                .remove(collider, &mut self.islands, &mut self.bodies, true);
        }
        // The old ground was sized for the old footprint; whatever is in
        // flight for it is stale too.
        let sublevel = &mut self.sublevels[index];
        sublevel.ground = None;
        sublevel.ground_pending = None;
        sublevel.active = false;

        let analysed = content.analyse();
        let body = sublevel.body;
        let mut reach = 0.0f32;
        let mut colliders = Vec::with_capacity(analysed.boxes.len());
        for (centre, half_extents, mass) in &analysed.boxes {
            reach = reach
                .max((centre[0] + half_extents[0]).abs())
                .max((centre[0] - half_extents[0]).abs())
                .max((centre[2] + half_extents[2]).abs())
                .max((centre[2] - half_extents[2]).abs());
            let collider =
                ColliderBuilder::cuboid(half_extents[0], half_extents[1], half_extents[2])
                    .translation(vector![centre[0], centre[1], centre[2]])
                    .mass(*mass)
                    .collision_groups(CONTENT_GROUPS)
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
        sublevel.content = Some(analysed);
        sublevel.ground_radius = reach.ceil() as i32 + GROUND_MARGIN;
        self.activate(index);
        true
    }

    /// Sub-levels whose ground is missing or has been left behind, with the
    /// ground the host should sample for each. The host answers each with
    /// [`Self::set_ground`] or [`Self::cancel_ground`].
    pub fn take_ground_requests(&mut self) -> Vec<(u32, GroundRequest)> {
        let mut requests = Vec::new();
        for sublevel in &mut self.sublevels {
            let Some(body) = self.bodies.get(sublevel.body) else {
                continue;
            };
            if sublevel.content.is_none() || sublevel.ground_pending.is_some() {
                continue;
            }
            let position = body.translation();
            let left_behind = match &sublevel.ground {
                None => true,
                Some(ground) => {
                    let refresh = (sublevel.ground_radius / 2).max(1) as f32;
                    let dx = position.x - (ground.center[0] as f32 + 0.5);
                    let dz = position.z - (ground.center[1] as f32 + 0.5);
                    dx.abs() > refresh || dz.abs() > refresh
                }
            };
            if left_behind {
                let request = GroundRequest {
                    center: [position.x.floor() as i32, position.z.floor() as i32],
                    radius: sublevel.ground_radius,
                };
                sublevel.ground_pending = Some(request);
                requests.push((sublevel.id, request));
            }
        }
        requests
    }

    /// Delivers the ground for a pending request, as sampled by the host.
    /// Ground that answers no pending request is dropped.
    pub fn set_ground(&mut self, id: u32, ground: Ground) -> bool {
        let Some(index) = self.sublevels.iter().position(|s| s.id == id) else {
            return false;
        };
        let side = usize::try_from(ground.radius * 2 + 1).unwrap_or(0);
        if ground.heights.len() != side * side {
            return false;
        }
        if self.sublevels[index].ground_pending
            != Some(GroundRequest {
                center: ground.center,
                radius: ground.radius,
            })
        {
            return false;
        }
        self.sublevels[index].ground_pending = None;
        self.sublevels[index].ground = Some(ground);
        self.activate(index);
        true
    }

    /// Notes that the host will not answer a sub-level's pending ground
    /// request, so it can be asked again.
    pub fn cancel_ground(&mut self, id: u32) {
        if let Some(sublevel) = self.sublevels.iter_mut().find(|s| s.id == id) {
            sublevel.ground_pending = None;
        }
    }

    /// Lets gravity and the air act on a sub-level once it has both its
    /// content and ground to stand on.
    fn activate(&mut self, index: usize) {
        let sublevel = &self.sublevels[index];
        if sublevel.active || sublevel.content.is_none() || sublevel.ground.is_none() {
            return;
        }
        let Some(body) = self.bodies.get_mut(sublevel.body) else {
            return;
        };
        body.set_gravity_scale(1.0, true);
        body.set_linear_damping(UNIVERSAL_DRAG);
        body.set_angular_damping(UNIVERSAL_DRAG);
        body.wake_up(true);
        self.sublevels[index].active = true;
    }

    /// Changes a body's velocity by `delta`, in blocks per second. This is
    /// the user-facing "shove" from the test command.
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

    /// Sets how hard sub-level `id`'s thrusters push, from 0 to 1.
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

    /// Advances the simulation by one world tick and returns every body.
    pub fn step(&mut self) -> Vec<BodySnapshot> {
        let dt = self.integration.dt;
        let air = self.air;
        for sublevel in &mut self.sublevels {
            if !sublevel.active {
                continue;
            }
            let Some(body) = self.bodies.get_mut(sublevel.body) else {
                continue;
            };
            let Some(content) = sublevel.content.as_mut() else {
                continue;
            };
            aero::apply_aero(body, content, sublevel.throttle, air, dt);
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
            &(),
        );
        for sublevel in &self.sublevels {
            if !sublevel.active {
                continue;
            }
            let (Some(content), Some(ground)) = (&sublevel.content, &sublevel.ground) else {
                continue;
            };
            if let Some(body) = self.bodies.get_mut(sublevel.body) {
                resolve_ground(body, content, ground);
            }
        }
        self.snapshots()
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

/// Ground contact for one sub-level, after its step: every low corner of
/// every box that ended up inside the terrain stops sinking, drags against
/// the ground and lifts the body back out. A body at rest ends each tick
/// exactly on the surface, so it never jitters.
fn resolve_ground(body: &mut RigidBody, analysed: &aero::Analysed, ground: &Ground) {
    let position = *body.position();
    let up = Vector::y();
    // (point, penetration, normal impulse so far)
    let mut contacts: Vec<(Point<Real>, Real, Real)> = Vec::new();
    for (centre, half, _) in &analysed.boxes {
        // Long boxes are probed every couple of blocks, so a hill between
        // their corners still catches them.
        let steps_x = (half[0] / (GROUND_SAMPLE_SPACING / 2.0)).ceil().max(1.0) as usize;
        let steps_z = (half[2] / (GROUND_SAMPLE_SPACING / 2.0)).ceil().max(1.0) as usize;
        for step_x in 0..=steps_x {
            for step_z in 0..=steps_z {
                let fx = -half[0] + 2.0 * half[0] * step_x as f32 / steps_x as f32;
                let fz = -half[2] + 2.0 * half[2] * step_z as f32 / steps_z as f32;
                let probe = point![centre[0] + fx, centre[1] - half[1], centre[2] + fz];
                let probe = position.transform_point(&probe);
                let Some(height) = ground.height_at(probe.x, probe.z) else {
                    continue;
                };
                let penetration = height - probe.y;
                if penetration > 0.0 {
                    contacts.push((probe, penetration, 0.0));
                }
            }
        }
    }
    if contacts.is_empty() {
        return;
    }

    let props = body.mass_properties().clone();
    let inv_mass = props.effective_inv_mass.x;
    // Rapier turns a torque impulse `t` into spin `s * (s * t)` with this
    // `s`, so the same product gives a corner's response to an impulse.
    let spin = props.effective_world_inv_inertia_sqrt;
    let center = props.world_com;
    // How fast a unit impulse along `direction` at `point` moves that
    // point along `direction`.
    let responsiveness = |point: &Point<Real>, direction: &Vector<Real>| {
        let arm = point - center;
        let turn = (spin * (spin * arm.cross(direction))).cross(&arm);
        inv_mass + direction.dot(&turn)
    };

    // Projected Gauss-Seidel over the probes: each pass tops a probe's
    // push up to what its current sink calls for, or takes some back if
    // other probes' pushes already lifted it.
    for _ in 0..GROUND_CONTACT_ITERATIONS {
        for contact in &mut contacts {
            let (point, _, impulse) = contact;
            let sink = -body.velocity_at_point(point).y;
            let target = (*impulse + sink / responsiveness(point, &up)).max(0.0);
            let change = target - *impulse;
            if change != 0.0 {
                body.apply_impulse_at_point(up * change, *point, true);
                *impulse = target;
            }
        }
    }
    // Friction follows the normal impulses: a corner that only carries the
    // body's weight for one tick grips little, one that just absorbed a
    // landing grips hard.
    for (point, _, impulse) in &contacts {
        let velocity = body.velocity_at_point(point);
        let slide = velocity - up * velocity.y;
        let speed = slide.magnitude();
        if speed <= 0.0 || *impulse <= 0.0 {
            continue;
        }
        let direction = slide / speed;
        let stop = speed / responsiveness(point, &direction);
        let grip = stop.min(GROUND_FRICTION * *impulse);
        body.apply_impulse_at_point(-direction * grip, *point, true);
    }

    // Whatever still sank is pushed out outright, deepest corner first.
    let deepest = contacts
        .iter()
        .fold(0.0f32, |deepest, (_, penetration, _)| {
            deepest.max(*penetration)
        });
    if deepest > 0.0 {
        let mut translation = *body.translation();
        translation.y += deepest;
        body.set_translation(translation, true);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() < 0.02,
            "expected ~{expected}, got {actual}"
        );
    }

    fn solid(x: i32, y: i32, z: i32) -> Voxel {
        Voxel {
            position: [x, y, z],
            kind: VoxelKind::Solid,
            mass: 1.0,
        }
    }

    fn wool(x: i32, y: i32, z: i32) -> Voxel {
        Voxel {
            position: [x, y, z],
            kind: VoxelKind::Envelope,
            mass: 0.25,
        }
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
            for y in -2i32..=2 {
                for z in -2i32..=2 {
                    if x.abs() == 2 || y.abs() == 2 || z.abs() == 2 {
                        voxels.push(wool(x, y, z));
                    }
                }
            }
        }
        voxels.push(Voxel {
            position: [0, -1, 0],
            kind: VoxelKind::Heater { output },
            mass: 1.0,
        });
        Content {
            pivot_offset: [0.5; 3],
            voxels,
        }
    }

    /// Flat ground at `top`, covering the usual margin around a slab.
    fn flat_ground(center: [i32; 2], radius: i32, top: f32) -> Ground {
        let side = (radius * 2 + 1) as usize;
        Ground {
            center,
            radius,
            heights: vec![top; side * side],
        }
    }

    /// Content, then matching flat ground, for sub-level `id`.
    fn land_on(system: &mut SubLevelSystem, id: u32, content: Content, top: f32) {
        assert!(system.set_content(id, content));
        let requests = system.take_ground_requests();
        assert_eq!(requests.len(), 1);
        let (_, request) = requests[0];
        assert!(system.set_ground(id, flat_ground(request.center, request.radius, top)));
    }

    #[test]
    fn corner_impulse_response_matches_rapier() -> Result<(), &'static str> {
        // The ground solver sizes its impulses with this same formula, so
        // it has to predict exactly what rapier does with an impulse.
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 64.5, 0.0]);
        assert!(system.set_content(id, slab(4, 4)));
        let body = system.sublevels[0].body;
        let Some(body) = system.bodies.get_mut(body) else {
            return Err("body");
        };
        let props = body.mass_properties().clone();
        let point = point![2.0, 64.0, 1.5];
        let up = Vector::y();
        let before = body.velocity_at_point(&point);
        body.apply_impulse_at_point(up, point, true);
        let after = body.velocity_at_point(&point);
        let spin = props.effective_world_inv_inertia_sqrt;
        let arm = point - props.world_com;
        let predicted =
            props.effective_inv_mass.x + up.dot(&(spin * (spin * arm.cross(&up))).cross(&arm));
        assert!((after.y - before.y - predicted).abs() < 1.0e-4);
        Ok(())
    }

    #[test]
    fn stepping_moves_a_shoved_body() {
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 64.0, 0.0]);
        assert!(system.set_content(id, slab(4, 4)));
        assert!(system.apply_velocity_change(id, [4.0, 0.0, 0.0]));

        let snapshots = system.step();
        assert_eq!(snapshots.len(), 1);
        // 4 blocks/s for one 50 ms tick; with no ground yet there is no
        // drag, so the shove carries over whole.
        assert_close(snapshots[0].position[0], 0.2);
        assert_close(snapshots[0].position[1], 64.0);
        assert_close(snapshots[0].linear_velocity[0], 4.0);
    }

    #[test]
    fn overlapping_bodies_do_not_shove_each_other() {
        let mut system = SubLevelSystem::new(Air::default());
        // Two regions assembled from the same spot: without collision
        // groups, contact resolution pushes both half a block apart.
        let first = system.spawn([-39.5, 150.5, -39.5]);
        let second = system.spawn([-39.5, 150.5, -39.5]);
        assert!(system.set_content(first, slab(1, 1)));
        assert!(system.set_content(second, slab(1, 1)));

        for _ in 0..40 {
            let snapshots = system.step();
            for body in snapshots {
                assert_close(body.position[0], -39.5);
                assert_close(body.position[1], 150.5);
                assert_close(body.position[2], -39.5);
            }
        }
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
    fn content_weighs_its_blocks_and_hovers_until_grounded() -> Result<(), &'static str> {
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 100.0, 0.0]);
        assert!(system.set_content(id, slab(4, 6)));
        let info = system.aero_info(id).ok_or("content set")?;
        assert_close(info.mass, 24.0);
        assert!(!info.active);
        for _ in 0..40 {
            let snapshots = system.step();
            assert_close(snapshots[0].position[1], 100.0);
        }
        Ok(())
    }

    #[test]
    fn content_rests_on_the_ground() -> Result<(), &'static str> {
        let mut system = SubLevelSystem::new(Air::default());
        // A 4x4 slab centred at y = 64.5 has its underside at 64.
        let id = system.spawn([10.3, 64.5, -7.2]);
        land_on(&mut system, id, slab(4, 4), 64.0);
        assert!(system.aero_info(id).ok_or("content set")?.active);

        for _ in 0..120 {
            let snapshots = system.step();
            let body = &snapshots[0];
            assert_close(body.position[0], 10.3);
            assert_close(body.position[1], 64.5);
            assert_close(body.position[2], -7.2);
        }
        Ok(())
    }

    #[test]
    fn content_falls_until_it_meets_the_ground() {
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 80.5, 0.0]);
        land_on(&mut system, id, slab(4, 4), 64.0);

        let mut lowest = f32::MAX;
        for _ in 0..200 {
            let snapshots = system.step();
            lowest = lowest.min(snapshots[0].position[1]);
        }
        let body = system.snapshots()[0];
        // Falls about 15 blocks, lands with its underside on the ground,
        // and stays there.
        assert!(lowest >= 64.4, "sank to {lowest}");
        assert_close(body.position[1], 64.5);
        assert!(body.linear_velocity[1].abs() < 0.05);
    }

    #[test]
    fn uneven_ground_tilts_the_body() {
        let mut system = SubLevelSystem::new(Air::default());
        // A long slab dropped across a one block step: the low side lands
        // first and the body settles tilted towards it.
        let id = system.spawn([0.0, 70.5, 0.0]);
        assert!(system.set_content(id, slab(12, 2)));
        let (_, request) = system.take_ground_requests()[0];
        let side = (request.radius * 2 + 1) as usize;
        let heights = (0..side * side)
            .map(|index| {
                let column = index % side;
                if column < side / 2 { 66.0 } else { 64.0 }
            })
            .collect();
        assert!(system.set_ground(
            id,
            Ground {
                center: request.center,
                radius: request.radius,
                heights,
            }
        ));
        for _ in 0..200 {
            system.step();
        }
        let body = system.snapshots()[0];
        // Half the slab lies a block lower than the other half, so the body
        // comes to rest tilted towards it, halfway between the two.
        assert!(
            body.rotation[2].abs() > 0.01 && body.rotation[2].abs() < 0.2,
            "rotation {:?}",
            body.rotation
        );
        assert!(
            body.position[1] < 66.5 && body.position[1] > 64.5,
            "{}",
            body.position[1]
        );
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
            58.0,
        );
        let mut body = system.snapshots()[0];
        for _ in 0..200 {
            body = system.step()[0];
        }
        assert_close(body.position[0], -16.0);
        assert_close(body.position[1], 61.5);
        assert_close(body.position[2], -271.0);
        assert!(body.linear_velocity.iter().all(|value| value.abs() < 0.01));
        assert!(body.angular_velocity.iter().all(|value| value.abs() < 0.01));
    }

    #[test]
    fn ground_follows_a_moving_body() {
        let mut system = SubLevelSystem::new(Air::default());
        // Ground far below, so the body keeps the shove instead of braking
        // on it.
        let id = system.spawn([0.0, 200.5, 0.0]);
        land_on(&mut system, id, slab(4, 4), 64.0);
        assert!(system.take_ground_requests().is_empty());

        assert!(system.apply_velocity_change(id, [8.0, 0.0, 0.0]));
        let mut x = 0.0;
        for _ in 0..80 {
            x = system.step()[0].position[0];
        }
        assert!(x > 6.0, "only reached {x}");
        let requests = system.take_ground_requests();
        assert_eq!(requests.len(), 1);
        let request = requests[0].1;
        assert!((request.center[0] as f32 - x).abs() <= 1.0);
        assert!(system.set_ground(id, flat_ground(request.center, request.radius, 64.0)));
    }

    #[test]
    fn stale_ground_is_refused() {
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 64.5, 0.0]);
        assert!(system.set_content(id, slab(4, 4)));
        let requests = system.take_ground_requests();
        let (_, request) = requests[0];
        // A request that was cancelled is no longer pending.
        system.cancel_ground(id);
        assert!(!system.set_ground(id, flat_ground(request.center, request.radius, 64.0)));
        // Asking again yields the same request.
        assert_eq!(system.take_ground_requests(), vec![(id, request)]);
        // Ground of the wrong size is not what was asked for either.
        assert!(!system.set_ground(id, flat_ground(request.center, request.radius + 1, 64.0)));
        assert!(system.set_ground(id, flat_ground(request.center, request.radius, 64.0)));
    }

    #[test]
    fn a_hot_enough_balloon_lifts_its_ship() -> Result<(), &'static str> {
        // The shell weighs 98 blocks of wool, 24.5 masses. Its 26 cells of
        // hot air lift 39 at full heat, so a weak heater leaves it on the
        // ground and a proper one takes off.
        let mut system = SubLevelSystem::new(Air::default());
        let id = system.spawn([0.0, 66.5, 0.0]);
        land_on(&mut system, id, balloon(10.0), 64.0);
        for _ in 0..400 {
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

        let id = system.spawn([0.0, 66.5, 0.0]);
        land_on(&mut system, id, balloon(26.0), 64.0);
        let mut rising = false;
        for _ in 0..400 {
            let body = system.step()[1];
            rising |= body.linear_velocity[1] > 0.05;
        }
        let body = system.snapshots()[1];
        assert!(rising, "never left the ground");
        assert!(body.position[1] > 80.0, "only reached {}", body.position[1]);
        // The gas sits over the middle of the shell, so it rises straight,
        // give or take the shuffle of leaving the ground.
        assert!(body.position[0].abs() < 0.25, "{}", body.position[0]);
        assert!(body.position[2].abs() < 0.25, "{}", body.position[2]);
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
            };
        }
        let id = system.spawn([0.0, 64.5, 0.0]);
        land_on(&mut system, id, content, 64.0);

        // Nothing happens without throttle.
        for _ in 0..20 {
            system.step();
        }
        assert_close(system.snapshots()[0].linear_velocity[0], 0.0);

        assert!(system.set_throttle(id, 1.0));
        for _ in 0..10 {
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
        let mut content = slab(8, 8);
        content.voxels[0].kind = VoxelKind::Thruster {
            direction: [1.0, 0.0, 0.0],
        };
        let id = system.spawn([0.0, 64.5, 0.0]);
        land_on(&mut system, id, content, 64.0);
        assert!(system.set_throttle(id, 1.0));
        for _ in 0..10 {
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
    fn ground_between_samples_slopes() {
        let ground = Ground {
            center: [0, 0],
            radius: 1,
            // One row at 10, the next at 12: samples sit at block centres.
            heights: vec![10.0, 10.0, 10.0, 12.0, 12.0, 12.0, 14.0, 14.0, 14.0],
        };
        assert_eq!(ground.height_at(-0.5, -0.5), Some(10.0));
        assert_eq!(ground.height_at(-0.5, 0.5), Some(12.0));
        assert_eq!(ground.height_at(-0.5, 0.0), Some(11.0));
        assert_eq!(ground.height_at(0.25, 1.25), Some(13.5));
        assert_eq!(ground.height_at(5.0, 0.0), None);
    }
}
