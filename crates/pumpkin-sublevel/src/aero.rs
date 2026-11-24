//! Aerodynamics of an assembled sub-level: the lifting gas sealed inside
//! it, the thrusters that push it, the surfaces and floating blocks that
//! bite the air, and the liquids it floats in.
//!
//! The balloon, sail, floating-material and buoyancy behaviour follows
//! Create: Aeronautics and Sable. The air model, gravity and drag match the
//! defaults Sable ships for the overworld, so builds handle the same.

use std::collections::{HashMap, HashSet};

use rapier3d::math::{Point, Real, Vector};
use rapier3d::na::{Matrix3, Vector3};
use rapier3d::prelude::RigidBody;

use crate::voxel::{
    FloatingMaterial, MaterialBox, Voxel, VoxelKind, collision_boxes, solid_bounds,
};

/// Downward acceleration on a sub-level, in blocks per second squared.
pub const GRAVITY: f32 = 11.0;
/// The fraction of its velocity a sub-level loses to air drag per second.
pub const UNIVERSAL_DRAG: f32 = 0.09;
/// Lifting force of one cubic metre of lifting gas, in block masses.
pub const LIFT_PER_BLOCK: f32 = 1.5;
/// Seconds a balloon takes to fill with, or empty of, lifting gas.
pub const FILL_TIME: f32 = 9.0;
/// A balloon far from its target fill moves this much faster than the fill
/// time suggests.
const RESPONSIVENESS_BOOST: f32 = 5.0;
/// "Far" means this fraction of the balloon's capacity.
const RESPONSIVENESS_RANGE: f32 = 0.05;
/// Less lifting gas than this, in cubic metres, is none left: the balloon
/// is gone (`isValid`).
const GAS_GONE: f32 = 0.05;
/// How far up its column a heater looks for the envelope above it.
const HEATER_RANGE: i32 = 80;
/// Aeronautics' small propellers: a push of 1 pN per rpm and an airflow of
/// 0.1 m/s per rpm; the push fades out as the sub-level approaches the
/// airflow speed.
pub const THRUSTER_THRUST: f32 = 1.0;
pub const THRUSTER_AIRFLOW: f32 = 0.1;
/// How much of its target rpm a propeller gains per tick, as it spools up.
const PROPELLER_SPOOL: f32 = 0.15;
/// A propeller slower than this, in rpm, counts as standing still.
const PROPELLER_IDLE: f32 = 0.03;
/// Sable's `directionlessDragScalar` on a lift surface: the least drag that
/// keeps a surface from gaining speed from the air.
const SURFACE_DRAG: f32 = 0.068_882;
/// Air pressure thins by this fraction per block above sea level.
const PRESSURE_FALLOFF: f32 = 0.004;
/// Air pressure stops growing this far below sea level.
const MAX_PRESSURE: f32 = 1.5;
/// Pressure fades to nothing over this many blocks below the build limit.
const PRESSURE_CEILING_BAND: f32 = 40.0;
/// Pressure is sampled this far apart to find how it changes with height.
const PRESSURE_STEP: f32 = 0.1;
/// Below this pressure lifting gas lifts nothing.
const NO_AIR: f32 = 1.0e-5;
/// Sable nudges the velocity its floating drag works with by this much of a
/// substep against gravity before applying it.
const DRAG_GRAVITY_BIAS: f32 = 1.0 / 2.1;
/// How hard a liquid drags a block moving through it, and how fast a fully
/// submerged block would rise unopposed.
pub const LIQUID_DRAG: f32 = 1.7;
pub const LIQUID_LIFT: f32 = 10.5;
/// Sub-levels whose extents sum to less than this get their buoyancy
/// sampled at eight points per block rather than one.
const FINE_BUOYANCY_EXTENTS: i32 = 10;
/// Airtight blocks carry `simple_drag` at this strength (`balloon_drag.json`).
const AIRTIGHT_DRAG_SCALE: f32 = 0.33;

/// The air a sub-level flies in, fixed by the dimension's sea level and
/// build limit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Air {
    sea_level: i32,
    top: i32,
}

impl Default for Air {
    fn default() -> Self {
        Self::new(63, 320)
    }
}

impl Air {
    #[must_use]
    pub const fn new(sea_level: i32, top: i32) -> Self {
        Self { sea_level, top }
    }

    /// Relative air pressure at height `y`: 1 at sea level, denser below,
    /// thinner above, and gone at the build limit.
    #[must_use]
    pub fn pressure(&self, y: f32) -> f32 {
        let band = self.top as f32 - PRESSURE_CEILING_BAND;
        if y <= band {
            return (PRESSURE_FALLOFF * (self.sea_level as f32 - y))
                .exp()
                .min(MAX_PRESSURE);
        }
        // The last stretch of build height eases the air out instead of
        // cutting it off.
        let remaining = ((self.top as f32 - y) / PRESSURE_CEILING_BAND).clamp(0.0, 1.0);
        (PRESSURE_FALLOFF * (self.sea_level as f32 - band)).exp() * remaining * remaining
    }
}

/// Everything a sub-level is made of.
///
/// Positions are whole blocks counted from an anchor block; the body's pivot
/// sits `pivot_offset` blocks (each in `0..1`) from that anchor.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Content {
    pub pivot_offset: [f32; 3],
    pub voxels: Vec<Voxel>,
}

/// A heater and where its column of air ends: the cell it fills from.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Cast {
    cell: [i32; 3],
    output: f32,
}

/// What a balloon's cells are measured against.
#[derive(Clone)]
struct Measure {
    /// The air cells among the balloon's cells: what holds the gas.
    air: Vec<[i32; 3]>,
    pivot_offset: [f32; 3],
    casts: Vec<Cast>,
}

/// Lifting gas sealed into a sub-level. It fills towards what the heaters
/// heat, capped by the space the envelope encloses, and empties again
/// without heat or once the envelope leaks.
#[derive(Clone)]
pub struct Balloon {
    /// The cells the gas can reach, air and blocks alike, in grid coordinates.
    cells: Vec<[i32; 3]>,
    /// Cells let out of the balloon for good by a hole.
    vented: HashSet<[i32; 3]>,
    /// The air cells among `cells`, once measured.
    air: Vec<[i32; 3]>,
    /// The cell the heater highest up fills from; the gas stays with it.
    controller: [i32; 3],
    pub capacity: u32,
    /// Centre of the gas volume in the body frame.
    pub centre: [f32; 3],
    /// How the gas cells spread about their centre: Σ (p − centre)(p − centre)ᵀ.
    scatter: Matrix3<f32>,
    /// What the heaters inside heat, capped by the capacity.
    target: f32,
    pub fill: f32,
}

impl Balloon {
    fn new(cells: Vec<[i32; 3]>, controller: [i32; 3], fill: f32, measure: &Measure) -> Self {
        let mut balloon = Self {
            cells,
            vented: HashSet::new(),
            air: Vec::new(),
            controller,
            capacity: 0,
            centre: [0.0; 3],
            scatter: Matrix3::zeros(),
            target: 0.0,
            fill,
        };
        balloon.measure(measure);
        balloon
    }

    fn contains(&self, cell: [i32; 3]) -> bool {
        self.cells.contains(&cell) && !self.vented.contains(&cell)
    }

    /// Takes `cells` out of the balloon for good.
    fn vent(&mut self, cells: impl IntoIterator<Item = [i32; 3]>, measure: &Measure) {
        let cells: HashSet<[i32; 3]> = cells.into_iter().collect();
        self.cells.retain(|cell| !cells.contains(cell));
        self.vented.extend(cells);
        self.measure(measure);
    }

    /// Works out the gas cells, capacity, centre and spread from the cells,
    /// and the target from the heaters casting into them.
    fn measure(&mut self, measure: &Measure) {
        self.air = self
            .cells
            .iter()
            .copied()
            .filter(|cell| measure.air.contains(cell))
            .collect();
        self.capacity = self.air.len() as u32;
        if self.capacity > 0 {
            let mut centre = Vector::zeros();
            let mut squares = Matrix3::zeros();
            for cell in &self.air {
                let at = cell_centre(*cell, measure.pivot_offset);
                centre += at;
                squares += at * at.transpose();
            }
            centre /= self.capacity as f32;
            self.scatter = squares - centre * centre.transpose() * self.capacity as f32;
            self.centre = centre.into();
        }
        self.target = measure
            .casts
            .iter()
            .filter(|cast| self.contains(cast.cell))
            .map(|cast| cast.output)
            .sum::<f32>()
            .min(self.capacity as f32);
    }

    /// What holes let out: the layer each one opens into, and every layer
    /// below it that shares a column with a layer already let out. With the
    /// controller at the top, a hole vents the balloon from there down.
    fn vented_by(&self, holes: &[[i32; 3]]) -> HashSet<[i32; 3]> {
        let mut vented = self.vented.clone();
        let mut layers: Vec<i32> = Vec::new();
        for hole in holes {
            // The hole's own cell is open to the outside, not part of the balloon.
            vented.insert(*hole);
            for axis in 0..3 {
                for step in [-1, 1] {
                    let mut adjacent = *hole;
                    adjacent[axis] += step;
                    if self.contains(adjacent) && !layers.contains(&adjacent[1]) {
                        layers.push(adjacent[1]);
                    }
                }
            }
        }
        if layers.is_empty() {
            return vented;
        }
        let columns: HashMap<i32, HashSet<[i32; 2]>> =
            self.cells.iter().fold(HashMap::new(), |mut columns, cell| {
                columns
                    .entry(cell[1])
                    .or_default()
                    .insert([cell[0], cell[2]]);
                columns
            });
        while let Some(y) = layers.pop() {
            vented.extend(self.cells.iter().filter(|cell| cell[1] == y).copied());
            let Some(below) = columns.get(&(y - 1)) else {
                continue;
            };
            if columns[&y].is_disjoint(below) || vented.iter().any(|cell| cell[1] == y - 1) {
                continue;
            }
            layers.push(y - 1);
        }
        vented
    }

    fn advance(&mut self, dt: f32) {
        let difference = self.target - self.fill;
        if difference == 0.0 {
            return;
        }
        let mut nudge = difference / (FILL_TIME / dt);
        let distance = difference / (self.capacity as f32 * RESPONSIVENESS_RANGE);
        nudge *= 1.0 + RESPONSIVENESS_BOOST / (1.0 + 3.0 * distance * distance);
        self.fill += nudge;
    }

    /// What the gas lifts at this air pressure, in block masses.
    #[must_use]
    pub fn lift(&self, pressure: f32) -> f32 {
        self.fill * LIFT_PER_BLOCK * pressure
    }
}

/// A propeller: spools towards its rpm and pushes along its direction.
pub struct Thruster {
    pub centre: [f32; 3],
    pub direction: [f32; 3],
    pub rpm: f32,
    speed: f32,
}

pub struct LiftSurface {
    pub centre: [f32; 3],
    pub normal: [f32; 3],
    parallel_drag: f32,
    lift: f32,
}

/// Every block of a content carrying one floating material, acting as one.
pub struct FloatingCluster {
    pub material: FloatingMaterial,
    /// Σ scaleᵢ pᵢ over the blocks, pᵢ their centres from the pivot.
    weighted: Vector3<f32>,
    /// Σ scaleᵢ.
    pub scale: f32,
    /// Σ scaleᵢ pᵢpᵢᵀ over the blocks.
    squares: Matrix3<f32>,
}

impl FloatingCluster {
    fn new(material: FloatingMaterial) -> Self {
        Self {
            material,
            weighted: Vector::zeros(),
            scale: 0.0,
            squares: Matrix3::zeros(),
        }
    }

    fn add(&mut self, centre: Vector3<f32>, scale: f32) {
        self.weighted += centre * scale;
        self.squares += centre * centre.transpose() * scale;
        self.scale += scale;
    }

    /// The cluster about the body's centre of mass: its centre relative to
    /// it, and the position matrix Sable spreads the drag over.
    fn about(&self, com: Vector3<f32>) -> (Vector3<f32>, Matrix3<f32>) {
        let centre = self.weighted / self.scale - com;
        let scatter = self.squares - self.weighted * self.weighted.transpose() / self.scale;
        let matrix = (scatter + Matrix3::identity() * (self.scale / 6.0)) / self.scale;
        (centre, matrix)
    }
}

/// A block liquids can get at: not buried behind solid blocks on all sides.
pub struct Wettable {
    /// Centre in the body frame.
    pub centre: [f32; 3],
    pub volume: f32,
}

/// A force working on a body for a whole tick, like Sable's buoyancy.
pub struct PersistentForce {
    /// Where it acts, in the world.
    pub point: [f32; 3],
    pub force: [f32; 3],
}

/// A [`Content`] worked out for the physics.
pub struct Analysed {
    /// Boxes of material in the body frame, one collider each.
    pub boxes: Vec<MaterialBox>,
    /// Grid bounds of the colliding blocks, inclusive.
    pub bounds: Option<[[i32; 3]; 2]>,
    pub balloons: Vec<Balloon>,
    pub thrusters: Vec<Thruster>,
    pub surfaces: Vec<LiftSurface>,
    pub floating: Vec<FloatingCluster>,
    pub wettable: Vec<Wettable>,
    /// The envelope blocks, to tell a hole from an edit that keeps the seal.
    envelopes: HashSet<[i32; 3]>,
    measure: Measure,
    fine_buoyancy: bool,
}

impl Content {
    #[must_use]
    pub fn analyse(self) -> Analysed {
        let pivot_offset = self.pivot_offset;
        let layout = Layout::of(&self.voxels);
        let mut floating: Vec<FloatingCluster> = Vec::new();
        for voxel in &self.voxels {
            let (material, scale) = match voxel.kind {
                // Airtight blocks drag the air as a weak floating material.
                VoxelKind::Envelope => (FloatingMaterial::SIMPLE_DRAG, AIRTIGHT_DRAG_SCALE),
                VoxelKind::Floating { material, scale } => (material, scale),
                _ => continue,
            };
            let index = floating
                .iter()
                .position(|cluster| cluster.material == material)
                .unwrap_or_else(|| {
                    floating.push(FloatingCluster::new(material));
                    floating.len() - 1
                });
            floating[index].add(cell_centre(voxel.position, pivot_offset), scale);
        }
        let blocks: HashSet<[i32; 3]> = self.voxels.iter().map(|voxel| voxel.position).collect();
        let mut air = Vec::new();
        for x in layout.min[0]..=layout.max[0] {
            for y in layout.min[1]..=layout.max[1] {
                for z in layout.min[2]..=layout.max[2] {
                    if !blocks.contains(&[x, y, z]) {
                        air.push([x, y, z]);
                    }
                }
            }
        }
        let measure = Measure {
            air,
            pivot_offset,
            casts: casts(&layout),
        };
        let extents: i32 = (0..3)
            .map(|axis| (layout.max[axis] - layout.min[axis] + 1).max(0))
            .sum();
        Analysed {
            boxes: collision_boxes(
                &self.voxels,
                [-pivot_offset[0], -pivot_offset[1], -pivot_offset[2]],
            ),
            bounds: solid_bounds(&self.voxels),
            balloons: balloons(&layout, &measure),
            thrusters: self
                .voxels
                .iter()
                .filter_map(|voxel| match voxel.kind {
                    VoxelKind::Thruster { direction, rpm } => Some(Thruster {
                        centre: cell_centre(voxel.position, pivot_offset).into(),
                        direction,
                        rpm,
                        speed: 0.0,
                    }),
                    _ => None,
                })
                .collect(),
            surfaces: self
                .voxels
                .iter()
                .filter_map(|voxel| match voxel.kind {
                    VoxelKind::LiftSurface {
                        normal,
                        parallel_drag,
                        lift,
                    } => Some(LiftSurface {
                        centre: cell_centre(voxel.position, pivot_offset).into(),
                        normal,
                        parallel_drag,
                        lift,
                    }),
                    _ => None,
                })
                .collect(),
            floating,
            wettable: wettable(&layout, pivot_offset),
            envelopes: self
                .voxels
                .iter()
                .filter(|voxel| voxel.kind == VoxelKind::Envelope)
                .map(|voxel| voxel.position)
                .collect(),
            measure,
            fine_buoyancy: extents < FINE_BUOYANCY_EXTENTS,
        }
    }
}

impl Analysed {
    /// Carries the gas of `old`'s balloons over to these, as Aeronautics
    /// keeps balloons alive through edits: the gas stays with the balloon
    /// its controller ends up in, a hole lets the layer it opens into and
    /// everything below it out for good, and what is left sealed keeps its
    /// gas while it cools.
    pub(crate) fn inherit(&mut self, old: &Self) {
        let holes: Vec<[i32; 3]> = old.envelopes.difference(&self.envelopes).copied().collect();
        let mut leftovers: Vec<Balloon> = Vec::new();
        for previous in &old.balloons {
            let vented = previous.vented_by(&holes);
            for balloon in &mut self.balloons {
                balloon.vent(vented.iter().copied(), &self.measure);
            }
            // A fresh balloon holding the controller takes the gas, as
            // Aeronautics merges balloons that grow into one another.
            if let Some(balloon) = self
                .balloons
                .iter_mut()
                .find(|balloon| balloon.contains(previous.controller))
            {
                balloon.fill += previous.fill;
                continue;
            }
            // Nothing fresh holds it. What is still sealed keeps its gas
            // while it cools; what is open leaks away.
            let mut leftover = previous.clone();
            let taken: HashSet<[i32; 3]> = self
                .balloons
                .iter()
                .flat_map(|balloon| balloon.cells.iter().copied())
                .chain(vented.iter().copied())
                .collect();
            leftover.vent(taken, &self.measure);
            leftovers.push(leftover);
        }
        self.balloons.extend(leftovers);
        self.balloons
            .retain(|balloon| balloon.target > GAS_GONE || balloon.fill > GAS_GONE);
    }

    /// One game tick of the slow-changing state: the gas in the balloons
    /// and the propellers spooling towards their rpm.
    pub fn advance(&mut self, dt: f32) {
        self.balloons
            .retain(|balloon| balloon.target > GAS_GONE || balloon.fill > GAS_GONE);
        for balloon in &mut self.balloons {
            balloon.advance(dt);
        }
        for thruster in &mut self.thrusters {
            thruster.speed += (thruster.rpm - thruster.speed) * PROPELLER_SPOOL;
            if thruster.speed.abs() < PROPELLER_IDLE && thruster.rpm == 0.0 {
                thruster.speed = 0.0;
            }
        }
    }

    /// What the liquids around the body do to it for a tick (Sable's
    /// `compute_buoyancy`): every block overlapping a world liquid is
    /// dragged towards rest and lifted by the liquid it displaces.
    #[must_use]
    pub fn buoyancy(&self, body: &RigidBody, liquids: &HashSet<[i32; 3]>) -> Vec<PersistentForce> {
        if liquids.is_empty() || self.wettable.is_empty() {
            return Vec::new();
        }
        let rotation = *body.rotation().to_rotation_matrix().matrix();
        let position = *body.position();
        let (samples, half): (&[[f32; 3]], f32) = if self.fine_buoyancy {
            (
                &[
                    [-0.25, -0.25, -0.25],
                    [-0.25, -0.25, 0.25],
                    [-0.25, 0.25, -0.25],
                    [-0.25, 0.25, 0.25],
                    [0.25, -0.25, -0.25],
                    [0.25, -0.25, 0.25],
                    [0.25, 0.25, -0.25],
                    [0.25, 0.25, 0.25],
                ],
                0.25,
            )
        } else {
            (&[[0.0, 0.0, 0.0]], 0.5)
        };
        // How far the block's rotated cube reaches from its centre.
        let reach = rotation.abs() * Vector::new(0.5, 0.5, 0.5);
        let mut forces = Vec::new();
        for block in &self.wettable {
            let centre = position * point_of(block.centre);
            let low = centre - reach - Vector::repeat(half);
            let high = centre + reach + Vector::repeat(half);
            for x in low.x.floor() as i32..=high.x.floor() as i32 {
                for y in low.y.floor() as i32..=high.y.floor() as i32 {
                    for z in low.z.floor() as i32..=high.z.floor() as i32 {
                        if !liquids.contains(&[x, y, z]) {
                            continue;
                        }
                        for offset in samples {
                            let at = centre + vector_of(*offset);
                            let overlap = overlap_with_cell(at.coords, half, [x, y, z]);
                            if overlap <= 0.0 {
                                continue;
                            }
                            let velocity = body.velocity_at_point(&at);
                            let drag = -velocity * (LIQUID_DRAG * overlap);
                            let lift = Vector::new(0.0, LIQUID_LIFT * overlap * block.volume, 0.0);
                            forces.push(PersistentForce {
                                point: at.into(),
                                force: (drag + lift).into(),
                            });
                        }
                    }
                }
            }
        }
        forces
    }
}

/// The middle of block `position` in the body frame.
fn cell_centre(position: [i32; 3], pivot_offset: [f32; 3]) -> Vector3<f32> {
    Vector::new(
        position[0] as f32 + 0.5 - pivot_offset[0],
        position[1] as f32 + 0.5 - pivot_offset[1],
        position[2] as f32 + 0.5 - pivot_offset[2],
    )
}

/// Where the blocks of a content are, for the flood fills to find their way.
struct Layout<'a> {
    grid: HashMap<[i32; 3], &'a Voxel>,
    /// Per column, the highest envelope block. Above it the column is
    /// sealed; a cell is capped when an envelope sits above it.
    roofs: HashMap<[i32; 2], i32>,
    min: [i32; 3],
    max: [i32; 3],
}

impl<'a> Layout<'a> {
    fn of(voxels: &'a [Voxel]) -> Self {
        let Some(first) = voxels.first() else {
            return Self {
                grid: HashMap::new(),
                roofs: HashMap::new(),
                min: [0; 3],
                max: [-1; 3],
            };
        };
        let (mut min, mut max) = (first.position, first.position);
        for voxel in voxels {
            for axis in 0..3 {
                min[axis] = min[axis].min(voxel.position[axis]);
                max[axis] = max[axis].max(voxel.position[axis]);
            }
        }
        let mut roofs: HashMap<[i32; 2], i32> = HashMap::new();
        for voxel in voxels {
            if voxel.kind == VoxelKind::Envelope {
                let column = [voxel.position[0], voxel.position[2]];
                let roof = roofs.entry(column).or_insert(i32::MIN);
                *roof = (*roof).max(voxel.position[1]);
            }
        }
        Self {
            grid: voxels.iter().map(|voxel| (voxel.position, voxel)).collect(),
            roofs,
            min,
            max,
        }
    }

    fn inside(&self, cell: [i32; 3]) -> bool {
        (0..3).all(|axis| cell[axis] >= self.min[axis] && cell[axis] <= self.max[axis])
    }

    fn sealed(&self, cell: [i32; 3]) -> bool {
        self.grid
            .get(&cell)
            .is_some_and(|voxel| voxel.kind == VoxelKind::Envelope)
    }

    fn capped(&self, cell: [i32; 3]) -> bool {
        self.roofs
            .get(&[cell[0], cell[2]])
            .is_some_and(|roof| *roof > cell[1])
    }

    fn solid(&self, cell: [i32; 3]) -> bool {
        self.grid
            .get(&cell)
            .is_some_and(|voxel| voxel.shape.is_solid())
    }
}

/// The cells a flood reached, and whether it got out or lost its roof.
struct Flood {
    cells: Vec<[i32; 3]>,
    escaped: bool,
    capped: bool,
}

/// The cells reachable from `start` moving up and sideways through cells
/// that are not sealed and that `blocked` admits. Leaving the content or
/// reaching under open sky marks the flood as no balloon.
fn upward_flood(layout: &Layout<'_>, start: [i32; 3], blocked: &dyn Fn([i32; 3]) -> bool) -> Flood {
    let mut flood = Flood {
        cells: Vec::new(),
        escaped: false,
        capped: true,
    };
    let mut stack = vec![start];
    let mut seen = HashSet::from([start]);
    while let Some(cell) = stack.pop() {
        if !layout.inside(cell) {
            flood.escaped = true;
            continue;
        }
        if layout.sealed(cell) || blocked(cell) {
            continue;
        }
        flood.cells.push(cell);
        flood.capped &= layout.capped(cell);
        let mut above = cell;
        above[1] += 1;
        stack.push(above);
        for axis in [0, 2] {
            for step in [-1, 1] {
                let mut next = cell;
                next[axis] += step;
                if seen.insert(next) {
                    stack.push(next);
                }
            }
        }
    }
    flood
}

/// The cells of the balloon a heater filling from `cast` inflates:
/// everything reachable upwards and sideways under a roof, plus the pockets
/// hanging below it. `None` when the space is open to the sky.
fn balloon_cells(layout: &Layout<'_>, cast: [i32; 3]) -> Option<Vec<[i32; 3]>> {
    let graph = upward_flood(layout, cast, &|_| false);
    if graph.escaped || !graph.capped {
        return None;
    }
    let mut cells: HashSet<[i32; 3]> = graph.cells.into_iter().collect();
    let mut rejected: HashSet<[i32; 3]> = HashSet::new();
    loop {
        let mut grew = false;
        let layer: Vec<[i32; 3]> = cells.iter().copied().collect();
        for cell in layer {
            let mut below = cell;
            below[1] -= 1;
            if !layout.inside(below)
                || layout.sealed(below)
                || cells.contains(&below)
                || rejected.contains(&below)
            {
                continue;
            }
            let in_graph = |at: [i32; 3]| cells.contains(&at);
            let pocket = upward_flood(layout, below, &in_graph);
            if pocket.escaped || !pocket.capped {
                rejected.extend(pocket.cells);
                continue;
            }
            cells.extend(pocket.cells);
            grew = true;
        }
        if !grew {
            break;
        }
    }
    Some(cells.into_iter().collect())
}

/// Every heater's cast: the last cell of its column of air below the
/// envelope that caps it. Anything with a collision shape above the heater
/// stops the column, and must be an envelope; otherwise the heater heats
/// nothing.
fn casts(layout: &Layout<'_>) -> Vec<Cast> {
    let mut casts = Vec::new();
    for (&position, voxel) in &layout.grid {
        let VoxelKind::Heater { output } = voxel.kind else {
            continue;
        };
        if output <= 0.0 {
            continue;
        }
        for step in 1..=HEATER_RANGE {
            let mut cell = position;
            cell[1] += step;
            if !layout.inside(cell) {
                break;
            }
            if !layout.solid(cell) {
                continue;
            }
            if layout.sealed(cell) {
                cell[1] -= 1;
                casts.push(Cast { cell, output });
            }
            break;
        }
    }
    casts.sort_unstable_by_key(|cast| (cast.cell[1], cast.cell[0], cast.cell[2]));
    casts
}

/// A balloon in the making: its cells and the controller cell.
type Fill = (HashSet<[i32; 3]>, [i32; 3]);

/// The balloons of a content: the space each heater's column of air ends
/// in, merged when heaters share a space. The highest heater's cell is the
/// controller, the first one winning ties.
fn balloons(layout: &Layout<'_>, measure: &Measure) -> Vec<Balloon> {
    let mut fills: Vec<Option<Fill>> = Vec::new();
    let mut owner: HashMap<[i32; 3], usize> = HashMap::new();
    for cast in measure.casts.iter().rev() {
        let Some(cells) = balloon_cells(layout, cast.cell) else {
            continue;
        };
        let cells: HashSet<[i32; 3]> = cells.into_iter().collect();
        let touched: Vec<usize> = cells
            .iter()
            .filter_map(|cell| owner.get(cell).copied())
            .collect::<HashSet<_>>()
            .into_iter()
            .collect();
        let survivor = touched.first().copied().unwrap_or(fills.len());
        if survivor == fills.len() {
            fills.push(Some((cells.clone(), cast.cell)));
        }
        for other in touched {
            if other == survivor {
                continue;
            }
            if let Some((cells, _)) = fills[other].take()
                && let Some(fill) = fills[survivor].as_mut()
            {
                fill.0.extend(cells);
            }
            for index in owner.values_mut() {
                if *index == other {
                    *index = survivor;
                }
            }
        }
        for cell in &cells {
            owner.insert(*cell, survivor);
        }
    }
    fills
        .into_iter()
        .flatten()
        .map(|(cells, controller)| {
            Balloon::new(cells.into_iter().collect(), controller, 0.0, measure)
        })
        .collect()
}

/// The blocks liquids can reach: every block that is not buried behind
/// full cubes on all six sides.
fn wettable(layout: &Layout<'_>, pivot_offset: [f32; 3]) -> Vec<Wettable> {
    let mut wettable = Vec::new();
    for (&position, voxel) in &layout.grid {
        if !voxel.shape.is_solid() && !voxel.liquid {
            continue;
        }
        let buried = (0..3).all(|axis| {
            [-1, 1].into_iter().all(|step| {
                let mut next = position;
                next[axis] += step;
                layout
                    .grid
                    .get(&next)
                    .is_some_and(|voxel| voxel.shape.is_cube())
            })
        });
        if !buried {
            wettable.push(Wettable {
                centre: cell_centre(position, pivot_offset).into(),
                volume: voxel.volume,
            });
        }
    }
    wettable
}

/// How much of the box of half size `half` at `centre` lies inside the unit
/// cell at `cell`.
fn overlap_with_cell(centre: Vector3<f32>, half: f32, cell: [i32; 3]) -> f32 {
    let mut volume = 1.0;
    for axis in 0..3 {
        let low = (cell[axis] as f32).max(centre[axis] - half);
        let high = (cell[axis] as f32 + 1.0).min(centre[axis] + half);
        volume *= (high - low).max(0.0);
    }
    volume
}

/// Sable's drag along and across gravity: `v ↦ −horizontal·v_⊥ − vertical·v_∥`.
fn gravity_matrix(up: Vector3<f32>, vertical: f32, horizontal: f32) -> Matrix3<f32> {
    -horizontal * Matrix3::identity() + (horizontal - vertical) * (up * up.transpose())
}

/// Sable's `matrixThingy`: how a drag matrix `drag` over a position matrix
/// `avg` turns angular velocity into torque.
fn torque_matrix(avg: Matrix3<f32>, drag: Matrix3<f32>) -> Matrix3<f32> {
    let identity = Matrix3::identity();
    let part = |matrix: &Matrix3<f32>| matrix.trace() * identity - matrix;
    let product = drag * avg;
    part(&avg) * part(&drag) - part(&product)
}

/// One physics substep of lift, thrust, drag and buoyancy on a sub-level's
/// body, `dt` seconds long, queued as forces for the step. `persistent`
/// holds the forces working for the whole tick.
pub fn apply_forces(
    body: &mut RigidBody,
    analysed: &Analysed,
    persistent: &[PersistentForce],
    throttle: f32,
    air: Air,
    dt: f32,
) {
    // Rapier keeps forces and torques until they are cleared, so every
    // substep starts from none and queues what works until the next one.
    body.reset_forces(false);
    body.reset_torques(false);
    for force in persistent {
        body.add_force_at_point(vector_of(force.force), point_of(force.point), true);
    }

    let rotation = *body.rotation();
    let position = *body.position();
    for thruster in &analysed.thrusters {
        if throttle == 0.0 || thruster.speed == 0.0 {
            continue;
        }
        let point = position.transform_point(&point_of(thruster.centre));
        let facing = rotation.transform_vector(&vector_of(thruster.direction));
        // A negative rpm is a reversed propeller: it pushes the other way.
        let along = facing * thruster.speed.signum();
        let thrust = thruster.speed.abs() * THRUSTER_THRUST * throttle;
        let airflow = thruster.speed.abs() * THRUSTER_AIRFLOW;
        // A propeller only accelerates air up to its airflow: past that
        // speed it pushes the sub-level nowhere.
        let speed_along = body.velocity_at_point(&point).dot(&along);
        let remaining = if airflow.abs() <= 0.001 {
            1.0
        } else {
            ((airflow + speed_along) / airflow).clamp(0.0, 1.0)
        };
        let push = thrust * air.pressure(point.y) * remaining;
        body.add_force_at_point(along * push, point, true);
    }
    let inverse_rotation = rotation.inverse();
    for surface in &analysed.surfaces {
        let point = position.transform_point(&point_of(surface.centre));
        let normal = vector_of(surface.normal);
        let velocity = inverse_rotation.transform_vector(&body.velocity_at_point(&point));
        let along = normal.dot(&velocity);
        let across = (velocity - normal * along).magnitude();
        // Drag opposes the motion and lift follows the normal, both in step
        // with the speed of the air past the surface.
        let force = -(velocity * SURFACE_DRAG
            + normal * (along * surface.parallel_drag + across * surface.lift))
            * air.pressure(point.y);
        body.add_force_at_point(rotation.transform_vector(&force), point, true);
    }
    apply_balloons(body, analysed, air);
    apply_floating(body, analysed, air, dt);
}

/// Balloon lift: each gas cell lifts its share at the pressure where it
/// is, so the gas lifts at its centre, and the air thinning with height
/// turns the balloon upright.
fn apply_balloons(body: &mut RigidBody, analysed: &Analysed, air: Air) {
    if analysed.balloons.iter().all(|balloon| balloon.fill == 0.0) {
        return;
    }
    let rotation = *body.rotation().to_rotation_matrix().matrix();
    let position = *body.position();
    let up = Vector::new(0.0, 1.0, 0.0);
    for balloon in &analysed.balloons {
        if balloon.fill == 0.0 {
            continue;
        }
        let centre = position * point_of(balloon.centre);
        let pressure = air.pressure(centre.y);
        if pressure < NO_AIR {
            continue;
        }
        let force = up * (balloon.lift(pressure) * GRAVITY);
        // How the pressure changes with height, turned into the body's frame.
        let gradient = (air.pressure(centre.y + PRESSURE_STEP) - pressure) / PRESSURE_STEP;
        let gradient_local = rotation.transpose() * up * gradient;
        let turn = rotation * (balloon.scatter * gradient_local / balloon.capacity.max(1) as f32);
        body.add_force_at_point(force, centre, true);
        body.add_torque(turn.cross(&force), true);
    }
}

/// Floating blocks: their drag on the air and, for the lifting materials,
/// their lift, never enough to raise their own centre.
fn apply_floating(body: &mut RigidBody, analysed: &Analysed, air: Air, dt: f32) {
    if analysed.floating.is_empty() {
        return;
    }
    let rotation = *body.rotation().to_rotation_matrix().matrix();
    let com_local = body.mass_properties().local_mprops.local_com.coords;
    let up = Vector3::new(0.0, 1.0, 0.0);
    let gravity = -up * GRAVITY;
    // Sable nudges the velocity its drag works with against gravity.
    let velocity = body.linvel() + up * (GRAVITY * DRAG_GRAVITY_BIAS * dt);
    let angular = *body.angvel();
    let mut impulse = Vector3::zeros();
    let mut torque = Vector3::zeros();
    let mut lift_force = 0.0;
    let mut lift_moment = Vector3::zeros();
    for cluster in &analysed.floating {
        let (centre_local, matrix_local) = cluster.about(com_local);
        let centre = rotation * centre_local;
        let pressure = air.pressure((body.center_of_mass() + centre).y);
        let material = cluster.material;
        let (force, cluster_torque) = friction(
            cluster,
            material,
            &rotation,
            matrix_local,
            centre,
            pressure,
            velocity,
            angular,
            up,
        );
        impulse += force;
        torque += cluster_torque;

        let cluster_force = material.lift
            * if material.scale_with_pressure {
                pressure
            } else {
                1.0
            };
        if cluster_force <= 0.0 {
            continue;
        }
        let weighted = cluster_force * cluster.scale;
        if material.prevent_self_lift {
            lift_force += weighted;
            lift_moment += centre * cluster_force;
        } else {
            impulse -= gravity * weighted;
            torque += centre.cross(&(-gravity * weighted));
        }
    }
    if lift_force > 0.0 {
        let average = lift_moment / lift_force;
        let lifting_force = -gravity * lift_force;
        let lifting_torque = average.cross(&lifting_force);
        // The lift gives way before its centre accelerates upwards faster
        // than gravity pulls: levitite alone hovers, it cannot ascend.
        let properties = &body.mass_properties().local_mprops;
        let frame = *properties
            .principal_inertia_local_frame
            .to_rotation_matrix()
            .matrix();
        let squares = properties
            .inv_principal_inertia_sqrt
            .component_mul(&properties.inv_principal_inertia_sqrt);
        let inverse_inertia = frame * Matrix3::from_diagonal(&squares) * frame.transpose();
        let turn = rotation * inverse_inertia * rotation.transpose() * lifting_torque;
        let acceleration = turn.cross(&average) + lifting_force * properties.inv_mass;
        let along = gravity.dot(&acceleration);
        let scale = if along == 0.0 {
            1.0
        } else {
            (-gravity.norm_squared() / along).min(1.0)
        };
        impulse += lifting_force * scale;
        torque += lifting_torque * scale;
    }
    // Rapier advances a body by the mean of its velocities before and
    // after a substep, so a lift applied as an impulse before the step
    // cancels gravity only on paper and the body drifts up. As a force it
    // is integrated along with gravity and cancels exactly.
    body.add_force(impulse, true);
    body.add_torque(torque, true);
}

/// One cluster's drag on the air, as Sable's `FloatingBlockController`
/// works it out: slow friction near rest and fast friction when moving,
/// blended by how far the cluster's mean velocity is from its transition
/// speed. Returns the force and, about the centre of mass, the torque.
#[allow(clippy::too_many_arguments)]
fn friction(
    cluster: &FloatingCluster,
    material: FloatingMaterial,
    rotation: &Matrix3<f32>,
    matrix_local: Matrix3<f32>,
    centre: Vector3<f32>,
    pressure: f32,
    velocity: Vector3<f32>,
    angular: Vector3<f32>,
    up: Vector3<f32>,
) -> (Vector3<f32>, Vector3<f32>) {
    let friction_scale = (if material.scale_friction_with_gravity {
        GRAVITY
    } else {
        1.0
    }) * (if material.scale_with_pressure {
        pressure
    } else {
        1.0
    });
    let identity = Matrix3::identity();
    let matrix = *rotation * matrix_local * rotation.transpose();
    let speed_scale = if material.transition_speed > 0.0 {
        3.0 / (material.transition_speed * material.transition_speed)
    } else {
        0.0
    };
    let mean_velocity = velocity + angular.cross(&centre);
    let Some(matrix_inverse) = invert(matrix) else {
        return (Vector3::zeros(), Vector3::zeros());
    };
    let spinning = angular.norm_squared() * identity - angular * angular.transpose();
    let Some(shifted) = invert(matrix_inverse + speed_scale * spinning) else {
        return (Vector3::zeros(), Vector3::zeros());
    };
    let Some(shifted_inverse) = invert(shifted) else {
        return (Vector3::zeros(), Vector3::zeros());
    };
    let shifted_centre = shifted * (angular.cross(&mean_velocity) * speed_scale);
    let slow_scale = (shifted.determinant() / matrix.determinant())
        .max(0.0)
        .sqrt()
        * (-0.5
            * (speed_scale * mean_velocity.norm_squared()
                - shifted_centre.dot(&(shifted_inverse * shifted_centre))))
        .exp();
    let slow = gravity_matrix(
        up,
        material.slow_vertical_friction,
        material.slow_horizontal_friction,
    ) * (cluster.scale * friction_scale * slow_scale);
    let fast = gravity_matrix(
        up,
        material.fast_vertical_friction,
        material.fast_horizontal_friction,
    ) * (cluster.scale * friction_scale);
    let force = fast * mean_velocity + slow * (angular.cross(&shifted_centre) + mean_velocity);
    let torque = centre.cross(&force)
        + shifted_centre.cross(&(slow * mean_velocity))
        + (torque_matrix(matrix, fast) + torque_matrix(shifted, slow)) * angular;
    (force, torque)
}

fn invert(matrix: Matrix3<f32>) -> Option<Matrix3<f32>> {
    matrix.try_inverse()
}

const fn point_of(values: [f32; 3]) -> Point<Real> {
    Point::new(values[0], values[1], values[2])
}

const fn vector_of(values: [f32; 3]) -> Vector3<f32> {
    Vector::new(values[0], values[1], values[2])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn solid(x: i32, y: i32, z: i32) -> Voxel {
        Voxel::cube([x, y, z], 1.0)
    }

    fn wool(x: i32, y: i32, z: i32) -> Voxel {
        Voxel::cube([x, y, z], 0.25).with_kind(VoxelKind::Envelope)
    }

    fn heater(x: i32, y: i32, z: i32, output: f32) -> Voxel {
        Voxel::cube([x, y, z], 1.0).with_kind(VoxelKind::Heater { output })
    }

    fn shell(min: [i32; 3], max: [i32; 3]) -> Vec<Voxel> {
        let mut voxels = Vec::new();
        for x in min[0]..=max[0] {
            for y in min[1]..=max[1] {
                for z in min[2]..=max[2] {
                    let face = x == min[0]
                        || x == max[0]
                        || y == min[1]
                        || y == max[1]
                        || z == min[2]
                        || z == max[2];
                    if face {
                        voxels.push(wool(x, y, z));
                    }
                }
            }
        }
        voxels
    }

    fn analysed(voxels: Vec<Voxel>) -> Analysed {
        Content {
            pivot_offset: [0.5; 3],
            voxels,
        }
        .analyse()
    }

    /// A sealed 5x5x5 shell with a heater in the middle of its floor.
    fn heated_shell() -> Vec<Voxel> {
        let mut voxels = shell([-2, -2, -2], [2, 2, 2]);
        voxels.push(heater(0, -1, 0, 500.0));
        voxels
    }

    fn without(mut voxels: Vec<Voxel>, cell: [i32; 3]) -> Vec<Voxel> {
        voxels.retain(|voxel| voxel.position != cell);
        voxels
    }

    #[test]
    fn balloons_fill_the_space_heaters_can_reach() {
        let balloons = analysed(heated_shell()).balloons;
        assert_eq!(balloons.len(), 1);
        assert_eq!(balloons[0].capacity, 26);
        // The gas sits at the centre of the 26 empty cells: one block above
        // the middle in total, spread over all of them.
        assert!((balloons[0].centre[1] - 1.0 / 26.0).abs() < 1.0e-5);

        // A gap in the roof lets the gas out: no balloon at all. A gap in a
        // wall only loses the balloon from there down: the layers above it
        // still hold gas.
        assert!(
            analysed(without(heated_shell(), [0, 2, 0]))
                .balloons
                .is_empty()
        );
        let walled = analysed(without(heated_shell(), [2, 0, 0])).balloons;
        assert_eq!(walled.len(), 1);
        assert_eq!(walled[0].capacity, 9);

        // Without a heater the sealed space holds cold air and lifts nothing.
        assert!(
            analysed(without(heated_shell(), [0, -1, 0]))
                .balloons
                .is_empty()
        );
    }

    #[test]
    fn two_heaters_in_one_space_share_a_balloon() {
        let mut voxels = shell([-2, -2, -2], [2, 2, 2]);
        voxels.push(heater(-1, -1, 0, 300.0));
        voxels.push(heater(1, -1, 0, 200.0));
        let balloons = analysed(voxels).balloons;
        assert_eq!(balloons.len(), 1);
        assert_eq!(balloons[0].capacity, 25);
    }

    #[test]
    fn blocks_inside_a_balloon_take_its_space() {
        // A heater and a crate inside a 3x3x3 space: both take a cell.
        let mut voxels = heated_shell();
        voxels.push(solid(1, 1, 1));
        let balloons = analysed(voxels).balloons;
        assert_eq!(balloons.len(), 1);
        assert_eq!(balloons[0].capacity, 25);
    }

    #[test]
    fn a_heaters_column_of_air_ends_at_the_envelope_above_it() {
        // A crate right above the heater stops its column of air: no
        // balloon, though the shell is sealed.
        let mut voxels = heated_shell();
        voxels.push(solid(0, 1, 0));
        assert!(analysed(voxels.clone()).balloons.is_empty());

        // A funnel of wool over the heater leaves the column open to the
        // roof, and the shell fills through the funnel.
        voxels.pop();
        for x in -1..=1 {
            for z in -1..=1 {
                if (x, z) != (0, 0) {
                    voxels.push(wool(x, 1, z));
                }
            }
        }
        let balloons = analysed(voxels).balloons;
        assert_eq!(balloons.len(), 1);
        assert_eq!(balloons[0].capacity, 18);
    }

    #[test]
    fn pockets_hanging_below_a_balloon_join_it() {
        // A wool tube under the floor of the shell: its three cells are
        // capped too, so they fill along with the shell.
        let mut voxels = without(heated_shell(), [0, -2, 0]);
        for y in [-3, -4] {
            for [x, z] in [[-1, 0], [1, 0], [0, -1], [0, 1]] {
                voxels.push(wool(x, y, z));
            }
        }
        voxels.push(wool(0, -5, 0));
        let balloons = analysed(voxels).balloons;
        assert_eq!(balloons.len(), 1);
        assert_eq!(balloons[0].capacity, 29);
    }

    #[test]
    fn a_hole_in_the_floor_lets_only_the_bottom_layer_out() {
        let first = analysed(heated_shell());
        let mut second = analysed(without(heated_shell(), [0, -2, 0]));
        // Fresh, the shell still fills: a balloon only needs a roof, and
        // the hole's own cell is capped too.
        assert_eq!(second.balloons[0].capacity, 27);
        // An edit changes that: the hole opens into the bottom layer, which
        // lets out with everything below it. The gas in the shell stays.
        second.inherit(&first);
        assert_eq!(second.balloons.len(), 1);
        assert_eq!(second.balloons[0].capacity, 18);
        second.balloons[0].fill = 26.0;

        // The layer stays out: a later edit does not bring it back.
        let mut third = analysed(without(heated_shell(), [0, -2, 0]));
        third.inherit(&second);
        assert_eq!(third.balloons.len(), 1);
        assert_eq!(third.balloons[0].capacity, 18);
        assert!((third.balloons[0].fill - 26.0).abs() < 1.0e-4);

        // A corner of the floor does not touch the balloon's cells.
        let mut corner = analysed(without(heated_shell(), [-2, -2, 0]));
        corner.inherit(&first);
        assert_eq!(corner.balloons[0].capacity, 26);
    }

    #[test]
    fn a_hole_in_the_wall_vents_the_balloon_down_to_there() {
        let first = analysed(heated_shell());
        let mut second = analysed(without(heated_shell(), [2, 0, 0]));
        // Only the layer above the hole is still a balloon.
        assert_eq!(second.balloons.len(), 1);
        assert_eq!(second.balloons[0].capacity, 9);
        second.inherit(&first);
        // The controller sits in that layer, so the gas stays with it.
        second.balloons[0].fill = 26.0;
        let mut third = analysed(without(heated_shell(), [2, 0, 0]));
        third.inherit(&second);
        assert_eq!(third.balloons.len(), 1);
        assert_eq!(third.balloons[0].capacity, 9);
        assert!((third.balloons[0].fill - 26.0).abs() < 1.0e-4);
    }

    #[test]
    fn a_hole_in_the_roof_leaks_the_whole_balloon() {
        let mut first = analysed(heated_shell());
        first.balloons[0].fill = 26.0;
        let mut second = analysed(without(heated_shell(), [0, 2, 0]));
        assert!(second.balloons.is_empty());
        second.inherit(&first);
        // What is left leaks: no balloon, but the gas goes on lifting as it
        // cools, and is gone in the end.
        let mut leaking = second;
        assert_eq!(leaking.balloons.len(), 1);
        assert_eq!(leaking.balloons[0].capacity, 0);
        assert!((leaking.balloons[0].fill - 26.0).abs() < 1.0e-4);
        for _ in 0..180 {
            leaking.advance(0.05);
        }
        let fill = leaking.balloons[0].fill;
        assert!(fill < 12.0 && fill > 1.0, "{fill}");
        for _ in 0..1500 {
            leaking.advance(0.05);
        }
        assert!(leaking.balloons.is_empty());
    }

    #[test]
    fn gas_fills_in_the_fill_time_and_cools_without_heat() {
        let mut hot = analysed(heated_shell());
        assert_eq!(hot.balloons[0].capacity, 26);
        for _ in 0..180 {
            hot.advance(0.05);
        }
        let fill = hot.balloons[0].fill;
        assert!(fill > 14.0 && fill < 25.0, "{fill}");
        for _ in 0..600 {
            hot.advance(0.05);
        }
        assert!(
            (hot.balloons[0].fill - 26.0).abs() < 0.15,
            "{}",
            hot.balloons[0].fill
        );

        // Cutting the heat lets the gas out just the same, through a rescan.
        let mut cold = analysed({
            let mut voxels = shell([-2, -2, -2], [2, 2, 2]);
            voxels.push(heater(0, -1, 0, 0.0));
            voxels
        });
        cold.inherit(&hot);
        assert_eq!(cold.balloons.len(), 1);
        assert!((cold.balloons[0].fill - 26.0).abs() < 0.15);
        for _ in 0..180 {
            cold.advance(0.05);
        }
        let fill = cold.balloons[0].fill;
        assert!(fill < 12.0 && fill > 1.0, "{fill}");
    }

    #[test]
    fn air_thins_with_altitude() {
        let air = Air::new(63, 320);
        assert!((air.pressure(63.0) - 1.0).abs() < 1.0e-6);
        assert!((air.pressure(-200.0) - MAX_PRESSURE).abs() < 1.0e-6);
        assert_eq!(air.pressure(320.0), 0.0);
        assert_eq!(air.pressure(400.0), 0.0);
        let mut previous = f32::INFINITY;
        for y in -64..=320 {
            let pressure = air.pressure(y as f32);
            assert!(pressure <= previous, "pressure rose at y={y}");
            previous = pressure;
        }
        assert!(air.pressure(100.0) < 0.9 && air.pressure(100.0) > 0.8);
    }
}
