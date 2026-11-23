//! Collision shapes of blocks and the boxes the physics collides with.
//!
//! Sable gives every block its real collision boxes, clipped to the unit
//! cube, and never collides blocks buried behind other solid blocks. Here
//! full cubes with the same material are merged into as few boxes as
//! possible, which buries interior blocks inside a box for free; blocks
//! with other shapes keep one box per collision shape.

use std::collections::{HashMap, HashSet};

/// A block's collision shape, in block-local coordinates.
#[derive(Debug, Clone, PartialEq)]
pub enum VoxelShape {
    /// A full unit cube.
    Cube,
    /// Other collision boxes, each `[min x, min y, min z, max x, max y, max z]`
    /// inside the unit cube.
    Boxes(Box<[[f32; 6]]>),
    /// Nothing to collide with.
    None,
}

impl VoxelShape {
    /// Whether anything can bump into this block.
    #[must_use]
    pub const fn is_solid(&self) -> bool {
        !matches!(self, Self::None)
    }

    /// Whether this is a full unit cube, the only shape that merges.
    #[must_use]
    pub const fn is_cube(&self) -> bool {
        matches!(self, Self::Cube)
    }
}

/// How a block slides, bounces and breaks on impact.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Material {
    /// Sable's `friction` block property: 1.0 for most blocks, 0.0 for ice,
    /// 1.65 for soul sand and the like.
    pub friction: f32,
    pub restitution: f32,
    /// Breaks when hit at speed, like glass panes and leaves do in Sable.
    pub fragile: bool,
}

impl Material {
    pub const DEFAULT: Self = Self {
        friction: 1.0,
        restitution: 0.0,
        fragile: false,
    };
}

/// A box of blocks in the body frame, what becomes one collider.
#[derive(Debug, Clone, PartialEq)]
pub struct MaterialBox {
    pub centre: [f32; 3],
    pub half_extents: [f32; 3],
    pub mass: f32,
    pub material: Material,
    /// The blocks the box covers, in grid coordinates: inclusive min and
    /// max. A box with other shapes than the cube covers a single block.
    pub blocks: [[i32; 3]; 2],
}

/// What one block of a sub-level does besides weighing something.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum VoxelKind {
    /// Ordinary material.
    #[default]
    Solid,
    /// Seals lifting gas in; like [`Self::Solid`] otherwise.
    Envelope,
    /// Heats the balloon it sits in, filling it with up to `output` cubic
    /// metres of lifting gas.
    Heater { output: f32 },
    /// A propeller: spins at `rpm` revolutions per minute and pushes the
    /// sub-level along `direction` (against it when `rpm` is negative).
    Thruster { direction: [f32; 3], rpm: f32 },
    /// A thin surface whose open face points along `normal`: it resists air
    /// moving along the normal by `parallel_drag` and lifts along it in
    /// proportion to `lift`.
    LiftSurface {
        normal: [f32; 3],
        parallel_drag: f32,
        lift: f32,
    },
}

/// One block of a sub-level or of the world around it.
#[derive(Debug, Clone, PartialEq)]
pub struct Voxel {
    /// Position on the grid: whole blocks from the grid's corner.
    pub position: [i32; 3],
    pub kind: VoxelKind,
    pub shape: VoxelShape,
    pub material: Material,
    pub mass: f32,
    /// How much of a full block this is for displacing liquids.
    pub volume: f32,
    /// A liquid: no collider, but sub-levels float in it.
    pub liquid: bool,
}

impl Voxel {
    /// A cube of ordinary material weighing `mass`.
    #[must_use]
    pub const fn cube(position: [i32; 3], mass: f32) -> Self {
        Self {
            position,
            kind: VoxelKind::Solid,
            shape: VoxelShape::Cube,
            material: Material::DEFAULT,
            mass,
            volume: 1.0,
            liquid: false,
        }
    }

    /// A cube of the given material.
    #[must_use]
    pub const fn cube_of(position: [i32; 3], mass: f32, material: Material) -> Self {
        Self {
            position,
            kind: VoxelKind::Solid,
            shape: VoxelShape::Cube,
            material,
            mass,
            volume: 1.0,
            liquid: false,
        }
    }

    /// A block of a given shape, with a mass and volume to match.
    #[must_use]
    pub const fn shaped(
        position: [i32; 3],
        shape: VoxelShape,
        mass: f32,
        volume: f32,
        material: Material,
    ) -> Self {
        Self {
            position,
            kind: VoxelKind::Solid,
            shape,
            material,
            mass,
            volume,
            liquid: false,
        }
    }

    /// A liquid filling its block.
    #[must_use]
    pub const fn liquid(position: [i32; 3]) -> Self {
        Self {
            position,
            kind: VoxelKind::Solid,
            shape: VoxelShape::None,
            material: Material::DEFAULT,
            mass: 0.0,
            volume: 1.0,
            liquid: true,
        }
    }

    /// The same block playing an aerodynamic role.
    #[must_use]
    pub const fn with_kind(mut self, kind: VoxelKind) -> Self {
        self.kind = kind;
        self
    }
}

/// Whether `at` holds a cube of `material` that no box has taken yet.
fn free_cube(
    cubes: &HashMap<[i32; 3], &Voxel>,
    merged: &HashSet<[i32; 3]>,
    at: [i32; 3],
    material: Material,
) -> bool {
    cubes
        .get(&at)
        .is_some_and(|voxel| voxel.material == material && !merged.contains(&at))
}

/// Every collision box of a grid of voxels, in the frame whose origin sits
/// at `origin` relative to the grid's corner. Cubes of one material merge
/// into rectangles per layer, and equal rectangles stacked in y merge into
/// one box; other shapes get a box each.
#[must_use]
pub fn collision_boxes(voxels: &[Voxel], origin: [f32; 3]) -> Vec<MaterialBox> {
    let mut boxes = merge_cubes(voxels, origin);
    for voxel in voxels {
        let VoxelShape::Boxes(shapes) = &voxel.shape else {
            continue;
        };
        let shapes: &[[f32; 6]] = shapes;
        let share = voxel.mass / shapes.len() as f32;
        for shape in shapes {
            let min = [
                origin[0] + voxel.position[0] as f32 + shape[0],
                origin[1] + voxel.position[1] as f32 + shape[1],
                origin[2] + voxel.position[2] as f32 + shape[2],
            ];
            let max = [
                origin[0] + voxel.position[0] as f32 + shape[3],
                origin[1] + voxel.position[1] as f32 + shape[4],
                origin[2] + voxel.position[2] as f32 + shape[5],
            ];
            boxes.push(box_of(min, max, share, voxel.material, [voxel.position; 2]));
        }
    }
    boxes
}

/// The merged boxes of the full cubes among the voxels.
fn merge_cubes(voxels: &[Voxel], origin: [f32; 3]) -> Vec<MaterialBox> {
    let mut cubes: HashMap<[i32; 3], &Voxel> = HashMap::new();
    let mut order: Vec<[i32; 3]> = Vec::new();
    for voxel in voxels {
        if voxel.shape.is_cube() && cubes.insert(voxel.position, voxel).is_none() {
            order.push(voxel.position);
        }
    }
    // Layer by layer, so rectangles stack in y on top of equal rectangles.
    order.sort_unstable();

    let mut merged: HashSet<[i32; 3]> = HashSet::default();
    let mut boxes = Vec::new();
    for start in &order {
        if merged.contains(start) {
            continue;
        }
        let material = cubes[start].material;
        let [x, y, z] = *start;
        // Along x, then the whole row along z, then the whole rectangle in y.
        let mut x1 = x;
        while free_cube(&cubes, &merged, [x1 + 1, y, z], material) {
            x1 += 1;
        }
        let row_fits =
            |at_z: i32| (x..=x1).all(|at_x| free_cube(&cubes, &merged, [at_x, y, at_z], material));
        let mut z1 = z;
        while row_fits(z1 + 1) {
            z1 += 1;
        }
        let layer_fits = |at_y: i32| {
            (z..=z1).all(|at_z| {
                (x..=x1).all(|at_x| free_cube(&cubes, &merged, [at_x, at_y, at_z], material))
            })
        };
        let mut y1 = y;
        while layer_fits(y1 + 1) {
            y1 += 1;
        }

        let mut mass = 0.0;
        for by in y..=y1 {
            for bz in z..=z1 {
                for bx in x..=x1 {
                    merged.insert([bx, by, bz]);
                    mass += cubes[&[bx, by, bz]].mass;
                }
            }
        }
        boxes.push(box_of(
            [
                origin[0] + x as f32,
                origin[1] + y as f32,
                origin[2] + z as f32,
            ],
            [
                origin[0] + x1 as f32 + 1.0,
                origin[1] + y1 as f32 + 1.0,
                origin[2] + z1 as f32 + 1.0,
            ],
            mass,
            material,
            [[x, y, z], [x1, y1, z1]],
        ));
    }
    boxes
}

/// The box spanning `min..max` in the body frame.
fn box_of(
    min: [f32; 3],
    max: [f32; 3],
    mass: f32,
    material: Material,
    blocks: [[i32; 3]; 2],
) -> MaterialBox {
    let centre = [
        min[0].midpoint(max[0]),
        min[1].midpoint(max[1]),
        min[2].midpoint(max[2]),
    ];
    let half_extents = [
        (max[0] - min[0]) / 2.0,
        (max[1] - min[1]) / 2.0,
        (max[2] - min[2]) / 2.0,
    ];
    MaterialBox {
        centre,
        half_extents,
        mass,
        material,
        blocks,
    }
}

/// The grid bounds of the voxels that collide, inclusive; `None` when
/// nothing does.
#[must_use]
pub fn solid_bounds(voxels: &[Voxel]) -> Option<[[i32; 3]; 2]> {
    let mut bounds: Option<[[i32; 3]; 2]> = None;
    for voxel in voxels.iter().filter(|voxel| voxel.shape.is_solid()) {
        // Slabs and the like may end short of the unit cube, but the grid
        // cell is the finest resolution here.
        bounds = Some(
            bounds.map_or([voxel.position, voxel.position], |[min, max]| {
                [
                    [
                        min[0].min(voxel.position[0]),
                        min[1].min(voxel.position[1]),
                        min[2].min(voxel.position[2]),
                    ],
                    [
                        max[0].max(voxel.position[0]),
                        max[1].max(voxel.position[1]),
                        max[2].max(voxel.position[2]),
                    ],
                ]
            }),
        );
    }
    bounds
}

#[cfg(test)]
mod tests {
    use super::*;

    fn blocks(voxels: &[Voxel]) -> usize {
        collision_boxes(voxels, [0.0; 3])
            .iter()
            .map(|boxed| {
                let [[x0, y0, z0], [x1, y1, z1]] = boxed.blocks;
                ((x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1)) as usize
            })
            .sum()
    }

    #[test]
    fn cubes_merge_into_few_boxes_without_losing_blocks_or_mass() -> Result<(), &'static str> {
        // A 4x4x4 cube of stone with a 2x2x2 hole of air inside, plus a
        // slab of lighter material on top: a handful of boxes.
        let mut voxels = Vec::new();
        for x in 0..4 {
            for y in 0..4 {
                for z in 0..4 {
                    if (1..3).contains(&x) && (1..3).contains(&y) && (1..3).contains(&z) {
                        continue;
                    }
                    voxels.push(Voxel::cube([x, y, z], 2.0));
                }
            }
        }
        for x in 0..4 {
            for z in 0..4 {
                voxels.push(Voxel::cube_of(
                    [x, 4, z],
                    0.5,
                    Material {
                        friction: 0.8,
                        ..Material::DEFAULT
                    },
                ));
            }
        }
        let boxes = collision_boxes(&voxels, [0.0; 3]);
        assert_eq!(blocks(&voxels), 72);
        assert!(boxes.len() <= 12, "{} boxes", boxes.len());
        let mass: f32 = boxes.iter().map(|boxed| boxed.mass).sum();
        assert!((mass - 120.0).abs() < 1.0e-4, "mass {mass}");
        // Stone weighs 2 and planks 0.5: the box on top weighs 8, and only
        // planks of one material merge together.
        let top = boxes
            .iter()
            .find(|boxed| boxed.centre[1] == 4.5)
            .ok_or("no box on top")?;
        assert!((top.mass - 8.0).abs() < 1.0e-4);
        Ok(())
    }

    #[test]
    fn other_shapes_keep_their_own_boxes() -> Result<(), &'static str> {
        let slab = Voxel::shaped(
            [0, 0, 0],
            VoxelShape::Boxes(Box::new([[0.0, 0.0, 0.0, 1.0, 0.5, 1.0]])),
            0.5,
            0.5,
            Material::DEFAULT,
        );
        let boxes = collision_boxes(&[slab, Voxel::cube([1, 0, 0], 1.0)], [10.0, 64.0, -3.0]);
        assert_eq!(boxes.len(), 2);
        let slab = boxes
            .iter()
            .find(|boxed| boxed.blocks[0] == [0, 0, 0])
            .ok_or("no slab box")?;
        assert_eq!(slab.centre, [10.5, 64.25, -2.5]);
        assert_eq!(slab.half_extents, [0.5, 0.25, 0.5]);
        assert_eq!(slab.blocks, [[0, 0, 0]; 2]);
        Ok(())
    }

    #[test]
    fn solid_bounds_skip_liquids_and_shapeless_blocks() {
        let voxels = [
            Voxel::liquid([5, 1, 5]),
            Voxel::shaped([9, 2, 9], VoxelShape::None, 0.0, 0.0, Material::DEFAULT),
            Voxel::cube([3, 7, 4], 1.0),
            Voxel::cube([2, 5, 6], 1.0),
        ];
        assert_eq!(solid_bounds(&voxels), Some([[2, 5, 4], [3, 7, 6]]));
        assert_eq!(solid_bounds(&[Voxel::liquid([0, 0, 0])]), None);
    }
}
