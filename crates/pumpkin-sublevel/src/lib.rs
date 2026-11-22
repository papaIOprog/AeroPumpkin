//! Native sub-level physics: rigid bodies that own a region of Minecraft
//! blocks and carry mounted entities through the world at a dynamic position
//! and orientation.
//!
//! This mirrors the "sub-level" concept popularised by Sable, implemented
//! from scratch for Pumpkin. The crate is a pure simulation domain: it knows
//! nothing about chunks, blocks or entities. Riders are opaque ids with an
//! anchor point; the world glue resolves ids to entities and applies the
//! carried positions.
//!
//! Threading: everything here runs sequentially on the world tick thread.
//! `rapier3d` is built without the `parallel` feature, so stepping never
//! dispatches onto rayon and cannot compete with the chunk generation pool.

use rapier3d::prelude::*;

/// Identifies a mounted entity. Opaque to this crate; the world glue uses
/// Pumpkin entity ids.
pub type RiderId = i32;

/// Seconds per world tick, the fixed step the simulation advances by.
pub const TICK_DT: f32 = 1.0 / 20.0;

/// Density of stone-like content, kg per block.
const ROCK_DENSITY: f32 = 1500.0;

/// Damping applied so test platforms drift to rest instead of sliding forever.
const LINEAR_DAMPING: f32 = 0.5;
const ANGULAR_DAMPING: f32 = 2.0;

/// How often (in ticks) the world should broadcast a full create/refresh of
/// every body, so clients that joined late converge.
pub const REFRESH_INTERVAL_TICKS: u64 = 100;

/// Transform updates between refreshes; 10 Hz keeps the payload stream cheap.
pub const TRANSFORM_INTERVAL_TICKS: u64 = 2;

struct SubLevel {
    id: u32,
    body: RigidBodyHandle,
    /// Half extents of the content bounds in body-local space.
    half_extents: Vector<f32>,
    /// Mounted riders as (entity id, body-local anchor point).
    riders: Vec<(RiderId, Point<f32>)>,
}

/// A view of one sub-level after a step, used for syncing clients and
/// carrying entities.
#[derive(Debug, Clone, Copy)]
pub struct SubLevelSnapshot {
    pub id: u32,
    pub position: [f32; 3],
    /// Body rotation as a quaternion, `(x, y, z, w)`.
    pub rotation: [f32; 4],
    pub linear_velocity: [f32; 3],
    pub half_extents: [f32; 3],
    /// World-space positions the riders should be carried to, in mount order.
    pub riders: [(RiderId, [f32; 3]); 8],
    pub rider_count: usize,
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
    sublevels: Vec<SubLevel>,
    next_id: u32,
    tick: u64,
}

impl Default for SubLevelSystem {
    fn default() -> Self {
        Self::new()
    }
}

impl SubLevelSystem {
    #[must_use]
    pub fn new() -> Self {
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
            // Sub-levels hover for now; aerodynamics arrive with the physics
            // properties port.
            gravity: Vector::zeros(),
            integration,
            sublevels: Vec::new(),
            next_id: 1,
            tick: 0,
        }
    }

    /// Spawns a hovering dynamic platform with a box collider.
    ///
    /// `half_extents` are in blocks, from the platform centre. Content is
    /// treated as solid stone for mass purposes until real chunk capture
    /// lands.
    pub fn spawn_platform(&mut self, position: [f32; 3], half_extents: [f32; 3]) -> u32 {
        let id = self.next_id;
        self.next_id += 1;

        let iso = Isometry::translation(position[0], position[1], position[2]);
        let body = RigidBodyBuilder::dynamic()
            .position(iso)
            .gravity_scale(0.0)
            .linear_damping(LINEAR_DAMPING)
            .angular_damping(ANGULAR_DAMPING)
            .build();
        let body_handle = self.bodies.insert(body);

        let collider = ColliderBuilder::cuboid(half_extents[0], half_extents[1], half_extents[2])
            .density(ROCK_DENSITY)
            .build();
        self.colliders
            .insert_with_parent(collider, body_handle, &mut self.bodies);

        self.sublevels.push(SubLevel {
            id,
            body: body_handle,
            half_extents: half_extents.into(),
            riders: Vec::new(),
        });
        id
    }

    /// Removes a sub-level. Riders keep their last carried position.
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

    /// Mounts a rider, remembering where it stands relative to the body.
    /// A rider can only ride one sub-level; re-mounting moves it.
    pub fn mount(&mut self, id: u32, rider: RiderId, world_position: [f32; 3]) -> bool {
        let Some(index) = self.sublevels.iter().position(|s| s.id == id) else {
            return false;
        };
        for other in &mut self.sublevels {
            other.riders.retain(|(rider_id, _)| *rider_id != rider);
        }
        let sublevel = &mut self.sublevels[index];
        let Some(body) = self.bodies.get(sublevel.body) else {
            return false;
        };
        let anchor = body
            .position()
            .inverse_transform_point(&Point::from(world_position));
        sublevel.riders.push((rider, anchor));
        true
    }

    /// Detaches a rider, e.g. because its entity no longer exists.
    pub fn unmount(&mut self, rider: RiderId) {
        for sublevel in &mut self.sublevels {
            sublevel.riders.retain(|(rider_id, _)| *rider_id != rider);
        }
    }

    /// Advances the simulation by one world tick and returns snapshots of
    /// every live sub-level.
    pub fn step(&mut self) -> Vec<SubLevelSnapshot> {
        self.tick += 1;
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
        self.snapshots()
    }

    /// The tick counter, so the glue can cadence its broadcasts without
    /// keeping its own clock.
    #[must_use]
    pub const fn tick_count(&self) -> u64 {
        self.tick
    }

    /// Snapshots without stepping, for listing and late-joiner refreshes.
    #[must_use]
    pub fn snapshots(&self) -> Vec<SubLevelSnapshot> {
        let mut out = Vec::with_capacity(self.sublevels.len());
        for sublevel in &self.sublevels {
            let Some(body) = self.bodies.get(sublevel.body) else {
                continue;
            };
            let rotation = body.rotation();
            let mut riders = [(0, [0.0; 3]); 8];
            let mut rider_count = 0;
            for (rider, anchor) in &sublevel.riders {
                if rider_count == riders.len() {
                    break;
                }
                let world = body.position() * anchor;
                riders[rider_count] = (*rider, world.into());
                rider_count += 1;
            }
            out.push(SubLevelSnapshot {
                id: sublevel.id,
                position: (*body.translation()).into(),
                rotation: [rotation.i, rotation.j, rotation.k, rotation.w],
                linear_velocity: (*body.linvel()).into(),
                half_extents: sublevel.half_extents.into(),
                riders,
                rider_count,
            });
        }
        out
    }

    /// True when the current tick should broadcast full create messages.
    #[must_use]
    pub const fn should_refresh_clients(&self) -> bool {
        self.tick.is_multiple_of(REFRESH_INTERVAL_TICKS)
    }

    /// True when the current tick should broadcast transform messages.
    #[must_use]
    pub const fn should_send_transforms(&self) -> bool {
        self.tick.is_multiple_of(TRANSFORM_INTERVAL_TICKS)
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

    #[test]
    fn stepping_carries_mounted_riders() {
        let mut system = SubLevelSystem::new();
        let id = system.spawn_platform([0.0, 64.0, 0.0], [8.0, 0.5, 8.0]);
        assert!(system.mount(id, 7, [2.0, 64.5, 0.0]));

        assert!(system.apply_velocity_change(id, [4.0, 0.0, 0.0]));

        let snapshots = system.step();
        assert_eq!(snapshots.len(), 1);
        let snap = &snapshots[0];
        // 4 blocks/s for one 50 ms tick.
        assert_close(snap.position[0], 0.2);
        assert_close(snap.position[1], 64.0);
        assert_eq!(snap.rider_count, 1);
        assert_eq!(snap.riders[0].0, 7);
        assert_close(snap.riders[0].1[0], 2.2);
        assert_close(snap.riders[0].1[1], 64.5);
    }

    #[test]
    fn riders_rotate_with_the_body() {
        let mut system = SubLevelSystem::new();
        let id = system.spawn_platform([0.0, 0.0, 0.0], [8.0, 0.5, 8.0]);
        assert!(system.mount(id, 3, [2.0, 0.0, 0.0]));

        // Rotate the body a quarter turn around Y: +X maps to -Z.
        let quarter = Isometry::rotation(Vector::y() * (std::f32::consts::FRAC_PI_2));
        if let Some(sublevel) = system.sublevels.iter().find(|s| s.id == id)
            && let Some(body) = system.bodies.get_mut(sublevel.body)
        {
            body.set_position(quarter, true);
        }

        let snapshots = system.step();
        let snap = &snapshots[0];
        assert_eq!(snap.rider_count, 1);
        assert_close(snap.riders[0].1[0], 0.0);
        assert_close(snap.riders[0].1[2], -2.0);
    }

    #[test]
    fn removing_a_body_unmounts_and_reports() {
        let mut system = SubLevelSystem::new();
        let id = system.spawn_platform([0.0, 64.0, 0.0], [4.0, 0.5, 4.0]);
        assert!(system.mount(id, 11, [0.0, 64.5, 0.0]));
        assert!(system.remove(id));

        assert!(system.snapshots().is_empty());
        assert!(!system.apply_velocity_change(id, [1.0, 0.0, 0.0]));
        assert!(!system.mount(id, 11, [0.0, 64.5, 0.0]));
    }
}
