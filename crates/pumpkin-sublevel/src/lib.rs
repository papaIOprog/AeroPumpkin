//! Native sub-level physics: rigid bodies that give a sub-level its pose.
//!
//! A sub-level is a plot of real chunks parked at remote coordinates (see
//! `pumpkin-core`'s `world::sublevel`); this crate only simulates the body
//! whose transform places that plot in the world. It knows nothing about
//! chunks, blocks or entities.
//!
//! Threading: everything here runs sequentially on the world tick thread.
//! `rapier3d` is built without the `parallel` feature, so stepping never
//! dispatches onto rayon and cannot compete with the chunk generation pool.

use rapier3d::prelude::*;

/// Seconds per world tick, the fixed step the simulation advances by.
pub const TICK_DT: f32 = 1.0 / 20.0;

/// Density of stone-like content, kg per block.
const ROCK_DENSITY: f32 = 1500.0;

/// Damping applied so test platforms drift to rest instead of sliding forever.
const LINEAR_DAMPING: f32 = 0.5;
const ANGULAR_DAMPING: f32 = 2.0;

struct SubLevel {
    id: u32,
    body: RigidBodyHandle,
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
        }
    }

    /// Spawns a hovering dynamic body with a box collider of `half_extents`
    /// blocks, centred on `position`. Content is treated as solid stone for
    /// mass purposes.
    pub fn spawn(&mut self, position: [f32; 3], half_extents: [f32; 3]) -> u32 {
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

        // Sub-level bodies never touch each other: two assembled regions
        // may be spawned overlapping, and contact resolution would shove
        // both off the exact spot their blocks came from.
        let collider = ColliderBuilder::cuboid(half_extents[0], half_extents[1], half_extents[2])
            .density(ROCK_DENSITY)
            .collision_groups(InteractionGroups::new(Group::GROUP_1, Group::GROUP_2))
            .build();
        self.colliders
            .insert_with_parent(collider, body_handle, &mut self.bodies);

        self.sublevels.push(SubLevel {
            id,
            body: body_handle,
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

    /// Advances the simulation by one world tick and returns every body.
    pub fn step(&mut self) -> Vec<BodySnapshot> {
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
    fn stepping_moves_a_shoved_body() {
        let mut system = SubLevelSystem::new();
        let id = system.spawn([0.0, 64.0, 0.0], [8.0, 0.5, 8.0]);
        assert!(system.apply_velocity_change(id, [4.0, 0.0, 0.0]));

        let snapshots = system.step();
        assert_eq!(snapshots.len(), 1);
        // 4 blocks/s for one 50 ms tick.
        assert_close(snapshots[0].position[0], 0.2);
        assert_close(snapshots[0].position[1], 64.0);
        // Linear damping shaves a little off the first step.
        assert!(snapshots[0].linear_velocity[0] > 3.5);
    }

    #[test]
    fn overlapping_bodies_do_not_shove_each_other() {
        let mut system = SubLevelSystem::new();
        // Two regions assembled from the same spot: without collision
        // groups, contact resolution pushes both half a block apart.
        let _first = system.spawn([-39.5, 150.5, -39.5], [0.5, 0.5, 0.5]);
        let _second = system.spawn([-39.5, 150.5, -39.5], [0.5, 0.5, 0.5]);

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
        let mut system = SubLevelSystem::new();
        let id = system.spawn([0.0, 64.0, 0.0], [4.0, 0.5, 4.0]);
        assert!(system.remove(id));

        assert!(system.snapshots().is_empty());
        assert!(!system.apply_velocity_change(id, [1.0, 0.0, 0.0]));
        assert!(!system.remove(id));
    }
}
