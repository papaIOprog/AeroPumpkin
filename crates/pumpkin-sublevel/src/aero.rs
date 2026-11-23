//! Aerodynamics of an assembled sub-level: its mass, the lifting gas sealed
//! inside it, the thrusters that push it and the surfaces that bite the air.
//!
//! The balloon and propeller behaviour follows Create: Aeronautics (MIT
//! code). The air model, gravity and drag match the defaults Sable ships for
//! the overworld, so builds handle the same on both.

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};

use rapier3d::math::{Point, Real, Vector};
use rapier3d::prelude::RigidBody;

/// Downward acceleration on a sub-level, in blocks per second squared.
pub const GRAVITY: f32 = 11.0;
/// The fraction of its velocity a sub-level loses to air drag per second.
pub const UNIVERSAL_DRAG: f32 = 0.09;
/// Lifting force of one cubic metre of hot air, in block masses.
pub const LIFT_PER_BLOCK: f32 = 1.5;
/// Seconds a balloon takes to fill with, or empty of, lifting gas.
pub const FILL_TIME: f32 = 9.0;
/// A balloon far from its target fill moves this much faster than the fill
/// time suggests.
const RESPONSIVENESS_BOOST: f32 = 5.0;
/// "Far" means this fraction of the balloon's capacity.
const RESPONSIVENESS_RANGE: f32 = 0.05;
/// Rated push of one thruster block at full throttle.
pub const THRUSTER_THRUST: f32 = 512.0;
/// A thruster stops adding push once the sub-level moves this fast along it.
pub const THRUSTER_AIRFLOW: f32 = 64.0;
/// How hard a lift surface resists air moving along its normal, resists air
/// moving in any direction, and lifts from air sliding across it.
const SURFACE_PARALLEL_DRAG: f32 = 0.75;
const SURFACE_DRAG: f32 = 0.068_882;
const SURFACE_LIFT: f32 = 0.475;
/// Air thins by this fraction per block above sea level.
const PRESSURE_FALLOFF: f32 = 0.004;
/// Air pressure stops growing this far below sea level.
const MAX_PRESSURE: f32 = 1.5;
/// Pressure fades to nothing over this many blocks below the build limit.
const PRESSURE_CEILING_BAND: f32 = 40.0;

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

/// What one block of a sub-level does to the physics besides its mass.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VoxelKind {
    /// Ordinary material.
    Solid,
    /// Seals lifting gas in; like [`Self::Solid`] otherwise.
    Envelope,
    /// Heats the balloon it sits in, filling it with up to `output` cubic
    /// metres of lifting gas.
    Heater { output: f32 },
    /// Pushes the sub-level along `direction` when the sub-level is
    /// throttled.
    Thruster { direction: [f32; 3] },
    /// A thin surface whose open face points along `normal`.
    LiftSurface { normal: [f32; 3] },
}

/// One block of a sub-level, `position` blocks from the body's anchor block.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Voxel {
    pub position: [i32; 3],
    pub kind: VoxelKind,
    pub mass: f32,
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

/// Lifting gas sealed into a sub-level. It fills towards what the heaters
/// can heat, capped by the space the envelope encloses.
pub struct Balloon {
    pub capacity: u32,
    /// Centre of the gas volume in the body frame.
    pub centre: [f32; 3],
    heater_output: f32,
    pub fill: f32,
}

impl Balloon {
    fn advance(&mut self, dt: f32) {
        let target = self.heater_output.min(self.capacity as f32);
        let difference = target - self.fill;
        if difference == 0.0 {
            return;
        }
        let mut nudge = difference / (FILL_TIME / dt);
        let distance = difference / (self.capacity as f32 * RESPONSIVENESS_RANGE);
        nudge *= 1.0 + RESPONSIVENESS_BOOST / (1.0 + 3.0 * distance * distance);
        self.fill += nudge;
        if (difference > 0.0) == (self.fill > target) {
            self.fill = target;
        }
    }

    /// What the gas lifts at this air pressure, in block masses.
    pub fn lift(&self, pressure: f32) -> f32 {
        self.fill * LIFT_PER_BLOCK * pressure
    }
}

pub struct Thruster {
    pub centre: [f32; 3],
    pub direction: [f32; 3],
}

pub struct LiftSurface {
    pub centre: [f32; 3],
    pub normal: [f32; 3],
}

/// A box of material in the body frame: centre, half extents and mass.
pub type MaterialBox = ([f32; 3], [f32; 3], f32);

/// A [`Content`] worked out for the physics.
pub struct Analysed {
    pub boxes: Vec<MaterialBox>,
    pub balloons: Vec<Balloon>,
    pub thrusters: Vec<Thruster>,
    pub surfaces: Vec<LiftSurface>,
}

impl Content {
    #[must_use]
    pub fn analyse(self) -> Analysed {
        let pivot_offset = self.pivot_offset;
        Analysed {
            boxes: collision_boxes(&self.voxels, pivot_offset),
            balloons: balloons(&self.voxels, pivot_offset),
            thrusters: self
                .voxels
                .iter()
                .filter_map(|voxel| match voxel.kind {
                    VoxelKind::Thruster { direction } => Some(Thruster {
                        centre: centre_of(voxel.position, pivot_offset),
                        direction,
                    }),
                    _ => None,
                })
                .collect(),
            surfaces: self
                .voxels
                .iter()
                .filter_map(|voxel| match voxel.kind {
                    VoxelKind::LiftSurface { normal } => Some(LiftSurface {
                        centre: centre_of(voxel.position, pivot_offset),
                        normal,
                    }),
                    _ => None,
                })
                .collect(),
        }
    }
}

/// The middle of block `position` in the body frame.
fn centre_of(position: [i32; 3], pivot_offset: [f32; 3]) -> [f32; 3] {
    [
        position[0] as f32 + 0.5 - pivot_offset[0],
        position[1] as f32 + 0.5 - pivot_offset[1],
        position[2] as f32 + 0.5 - pivot_offset[2],
    ]
}

/// A run of blocks still growing along x: (last column, first x, y range,
/// mass so far).
type Strip = ([i32; 2], i32, [i32; 2], f32);

/// Turns finished strips into boxes in the body frame.
fn emit_strips(strips: Vec<Strip>, pivot_offset: [f32; 3], boxes: &mut Vec<MaterialBox>) {
    for ([z, last_x], first_x, [y0, y1], mass) in strips {
        boxes.push((
            [
                (first_x + last_x + 1) as f32 / 2.0 - pivot_offset[0],
                (y0 + y1 + 1) as f32 / 2.0 - pivot_offset[1],
                z as f32 + 0.5 - pivot_offset[2],
            ],
            [
                (last_x - first_x + 1) as f32 / 2.0,
                (y1 - y0 + 1) as f32 / 2.0,
                0.5,
            ],
            mass,
        ));
    }
}

/// Boxes covering every block: one per run of blocks stacked in a column,
/// stretched along x while neighbouring columns stack the same way. Each
/// box weighs exactly the blocks in it.
fn collision_boxes(voxels: &[Voxel], pivot_offset: [f32; 3]) -> Vec<MaterialBox> {
    let mut columns: BTreeMap<[i32; 2], Vec<(i32, f32)>> = BTreeMap::new();
    for voxel in voxels {
        let [x, y, z] = voxel.position;
        columns.entry([z, x]).or_default().push((y, voxel.mass));
    }

    let mut boxes = Vec::new();
    let mut strips: Vec<Strip> = Vec::new();
    let mut previous: Option<[i32; 2]> = None;
    for ([z, x], mut column) in columns {
        if previous != Some([z, x - 1]) {
            emit_strips(std::mem::take(&mut strips), pivot_offset, &mut boxes);
        }
        column.sort_unstable_by_key(|(y, _)| *y);
        // Runs of consecutive blocks in this column: y range and mass.
        let mut runs: Vec<([i32; 2], f32)> = Vec::new();
        for (y, mass) in column {
            match runs.last_mut() {
                Some(run) if run.0[1] + 1 == y => {
                    run.0[1] = y;
                    run.1 += mass;
                }
                _ => runs.push(([y, y], mass)),
            }
        }
        let mut reached = Vec::new();
        for (column, first_x, range, mass) in std::mem::take(&mut strips) {
            if let Some(index) = runs.iter().position(|(candidate, _)| *candidate == range) {
                reached.push(([z, x], first_x, range, mass + runs[index].1));
                runs.swap_remove(index);
            } else {
                emit_strips(
                    vec![(column, first_x, range, mass)],
                    pivot_offset,
                    &mut boxes,
                );
            }
        }
        for (range, mass) in runs {
            reached.push(([z, x], x, range, mass));
        }
        strips = reached;
        previous = Some([z, x]);
    }
    emit_strips(strips, pivot_offset, &mut boxes);
    boxes
}

/// A balloon in the making: the empty cells its gas fills and what heats it.
struct Fill {
    cells: Vec<[i32; 3]>,
    heater_output: f32,
}

/// Where the blocks of a content are, for the flood fills to find their way.
struct Layout {
    grid: HashMap<[i32; 3], VoxelKind>,
    min: [i32; 3],
    max: [i32; 3],
}

impl Layout {
    fn of(voxels: &[Voxel]) -> Self {
        let Some(first) = voxels.first() else {
            return Self {
                grid: HashMap::new(),
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
        Self {
            grid: voxels
                .iter()
                .map(|voxel| (voxel.position, voxel.kind))
                .collect(),
            min,
            max,
        }
    }

    fn sealed(&self, cell: [i32; 3]) -> bool {
        matches!(self.grid.get(&cell), Some(VoxelKind::Envelope))
    }

    fn inside(&self, cell: [i32; 3]) -> bool {
        (0..3).all(|axis| cell[axis] >= self.min[axis] && cell[axis] <= self.max[axis])
    }

    fn empty(&self, cell: [i32; 3]) -> bool {
        !self.grid.contains_key(&cell)
    }
}

/// What one flood fill from above a heater reached.
struct Flood {
    visited: Vec<[i32; 3]>,
    /// The fill left the content or met cells already known to be outside.
    leaks: bool,
    /// Balloons whose cells the fill ran into.
    touched: Vec<usize>,
}

/// Every cell reachable from `start` without crossing an envelope, stopping
/// as soon as the fill gets out.
fn flood(
    layout: &Layout,
    open: &HashSet<[i32; 3]>,
    owner: &HashMap<[i32; 3], usize>,
    start: [i32; 3],
) -> Flood {
    let mut flood = Flood {
        visited: Vec::new(),
        leaks: false,
        touched: Vec::new(),
    };
    let mut queue = VecDeque::from([start]);
    let mut seen = HashSet::from([start]);
    while let Some(cell) = queue.pop_front() {
        flood.visited.push(cell);
        if !layout.inside(cell) || open.contains(&cell) {
            flood.leaks = true;
            break;
        }
        if let Some(&index) = owner.get(&cell) {
            flood.touched.push(index);
        }
        for axis in 0..3 {
            for step in [-1, 1] {
                let mut next = cell;
                next[axis] += step;
                if seen.insert(next) && !layout.sealed(next) {
                    queue.push_back(next);
                }
            }
        }
    }
    flood
}

/// The balloons of a content: every cell reachable from above a heater
/// without crossing an envelope. A fill that leaves the content is open to
/// the sky and lifts nothing.
fn balloons(voxels: &[Voxel], pivot_offset: [f32; 3]) -> Vec<Balloon> {
    let layout = Layout::of(voxels);
    // Cells known to be open to the sky, and empty cells by the fill holding them.
    let mut open: HashSet<[i32; 3]> = HashSet::new();
    let mut owner: HashMap<[i32; 3], usize> = HashMap::new();
    let mut fills: Vec<Option<Fill>> = Vec::new();

    for heater in voxels {
        let VoxelKind::Heater { output } = heater.kind else {
            continue;
        };
        let start = [
            heater.position[0],
            heater.position[1] + 1,
            heater.position[2],
        ];
        if layout.sealed(start) {
            continue;
        }
        let flood = flood(&layout, &open, &owner, start);

        if flood.leaks {
            // Everything reached shares the leak, balloons included.
            open.extend(flood.visited);
            for index in flood.touched {
                if let Some(fill) = fills[index].take() {
                    open.extend(fill.cells);
                }
            }
            owner.retain(|_, index| fills[*index].is_some());
            continue;
        }

        // Only empty cells no balloon holds yet join this one; a block
        // inside the balloon, heater included, takes its share of the space.
        let mut cells: Vec<[i32; 3]> = flood
            .visited
            .into_iter()
            .filter(|cell| layout.empty(*cell) && !owner.contains_key(cell))
            .collect();
        // Heaters sharing a space share its balloon: everything the flood
        // touched folds into the first balloon it met, or a new one.
        let survivor = flood.touched.first().copied().unwrap_or(fills.len());
        if survivor == fills.len() {
            fills.push(None);
        }
        let mut output = output;
        for other in flood.touched {
            if other == survivor {
                continue;
            }
            if let Some(fill) = fills[other].take() {
                cells.extend(fill.cells);
                output += fill.heater_output;
                for index in owner.values_mut() {
                    if *index == other {
                        *index = survivor;
                    }
                }
            }
        }
        let fill = fills[survivor].get_or_insert_with(|| Fill {
            cells: Vec::new(),
            heater_output: 0.0,
        });
        fill.heater_output += output;
        for &cell in &cells {
            owner.insert(cell, survivor);
        }
        fill.cells.extend(cells);
    }

    fills
        .into_iter()
        .flatten()
        .map(|fill| {
            let count = fill.cells.len();
            let mut centre = [0.0; 3];
            for cell in &fill.cells {
                for axis in 0..3 {
                    centre[axis] += cell[axis] as f32 + 0.5 - pivot_offset[axis];
                }
            }
            for value in &mut centre {
                *value /= count as f32;
            }
            Balloon {
                capacity: count as u32,
                centre,
                heater_output: fill.heater_output,
                fill: 0.0,
            }
        })
        .collect()
}

/// One tick of lift, thrust and air resistance on a sub-level's body.
pub fn apply_aero(body: &mut RigidBody, analysed: &mut Analysed, throttle: f32, air: Air, dt: f32) {
    let pressure = air.pressure(body.translation().y);
    let mut lift = 0.0;
    for balloon in &mut analysed.balloons {
        balloon.advance(dt);
        lift += balloon.lift(pressure);
    }
    // A resting sub-level with nothing to gain from the air stays asleep.
    if body.is_sleeping() && lift <= body.mass() && throttle == 0.0 {
        return;
    }

    let rotation = *body.rotation();
    let position = *body.position();
    if lift > 0.0 {
        for balloon in &analysed.balloons {
            let share = balloon.lift(pressure) / lift;
            let point = position.transform_point(&point_of(balloon.centre));
            body.apply_impulse_at_point(
                Vector::new(0.0, lift * share * GRAVITY * dt, 0.0),
                point,
                true,
            );
        }
    }
    for thruster in &analysed.thrusters {
        if throttle == 0.0 {
            break;
        }
        let point = position.transform_point(&point_of(thruster.centre));
        let direction = rotation.transform_vector(&vector_of(thruster.direction));
        // A propeller only accelerates air up to its airflow: past that
        // speed it pushes the sub-level nowhere.
        let speed_along = body.velocity_at_point(&point).dot(&direction);
        let remaining = ((THRUSTER_AIRFLOW - speed_along) / THRUSTER_AIRFLOW).clamp(0.0, 1.0);
        body.apply_impulse_at_point(
            direction * (THRUSTER_THRUST * throttle * pressure * remaining * dt),
            point,
            true,
        );
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
            + normal * (along * SURFACE_PARALLEL_DRAG + across * SURFACE_LIFT))
            * pressure
            * dt;
        body.apply_impulse_at_point(rotation.transform_vector(&force), point, true);
    }
}

const fn point_of(values: [f32; 3]) -> Point<Real> {
    Point::new(values[0], values[1], values[2])
}

const fn vector_of(values: [f32; 3]) -> Vector<Real> {
    Vector::new(values[0], values[1], values[2])
}

#[cfg(test)]
mod tests {
    use super::*;

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

    fn heater(x: i32, y: i32, z: i32, output: f32) -> Voxel {
        Voxel {
            position: [x, y, z],
            kind: VoxelKind::Heater { output },
            mass: 1.0,
        }
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

    #[test]
    fn boxes_merge_columns_that_stack_alike_and_keep_their_mass() {
        // Two rows of three blocks: one box per row, centred on the body.
        let mut voxels = Vec::new();
        for x in -1..=1 {
            for z in -1..=0 {
                voxels.push(solid(x, 0, z));
            }
        }
        assert_eq!(
            collision_boxes(&voxels, [0.5; 3]),
            vec![
                ([0.0, 0.0, -1.0], [1.5, 0.5, 0.5], 3.0),
                ([0.0, 0.0, 0.0], [1.5, 0.5, 0.5], 3.0)
            ]
        );

        // Two stacked blocks in one column, one in the next: nothing merges,
        // and a light block weighs less.
        let boxes = collision_boxes(&[solid(0, 0, 0), wool(0, 1, 0), solid(1, 0, 0)], [0.5; 3]);
        assert_eq!(
            boxes,
            vec![
                ([0.0, 0.5, 0.0], [0.5, 1.0, 0.5], 1.25),
                ([1.0, 0.0, 0.0], [0.5, 0.5, 0.5], 1.0)
            ]
        );
    }

    #[test]
    fn balloons_fill_the_space_heaters_can_reach() {
        // A sealed 5x5x5 shell holds a 3x3x3 space; a heater in the middle
        // of its floor takes one cell of it.
        let mut voxels = shell([-2, -2, -2], [2, 2, 2]);
        voxels.push(heater(0, -1, 0, 500.0));
        let balloons = analysed(voxels.clone()).balloons;
        assert_eq!(balloons.len(), 1);
        assert_eq!(balloons[0].capacity, 26);
        // The gas sits at the centre of the 26 empty cells: one block above
        // the middle in total, spread over all of them.
        assert!((balloons[0].centre[1] - 1.0 / 26.0).abs() < 1.0e-5);

        // A gap in the shell lets the gas out: no balloon at all.
        voxels.retain(|voxel| voxel.position != [2, 0, 0]);
        assert!(analysed(voxels.clone()).balloons.is_empty());

        // Without a heater the sealed space holds cold air and lifts nothing.
        voxels.push(wool(2, 0, 0));
        voxels.retain(|voxel| voxel.position != [0, -1, 0]);
        assert!(analysed(voxels).balloons.is_empty());
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
        let mut voxels = shell([-2, -2, -2], [2, 2, 2]);
        voxels.push(heater(0, -1, 0, 500.0));
        voxels.push(solid(0, 1, 0));
        let balloons = analysed(voxels).balloons;
        assert_eq!(balloons.len(), 1);
        assert_eq!(balloons[0].capacity, 25);
    }

    #[test]
    fn balloon_fill_converges_within_the_fill_time() {
        let balloon = Balloon {
            capacity: 100,
            centre: [0.0; 3],
            heater_output: 1000.0,
            fill: 0.0,
        };
        // Each tick moves a share of what is missing, so most of the gas is
        // in after the fill time and the rest follows quickly.
        let mut filling = balloon;
        for _ in 0..180 {
            filling.advance(0.05);
        }
        assert!(
            filling.fill > 55.0 && filling.fill < 95.0,
            "{}",
            filling.fill
        );
        for _ in 0..600 {
            filling.advance(0.05);
        }
        assert!((filling.fill - 100.0).abs() < 0.5, "{}", filling.fill);

        // Cutting the heat lets the gas out just the same.
        let mut emptying = filling;
        emptying.heater_output = 0.0;
        for _ in 0..180 {
            emptying.advance(0.05);
        }
        assert!(
            emptying.fill < 45.0 && emptying.fill > 5.0,
            "{}",
            emptying.fill
        );
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
