//! What blocks do to a sub-level, in the physics' terms.
//!
//! Sable and Create: Aeronautics describe that with data files for their own
//! blocks. Pumpkin has none of those blocks, so ordinary ones stand in for
//! them: wool seals a balloon (airtight in Aeronautics too), a lit campfire
//! heated by a redstone signal is a hot air burner, a piston powered by
//! redstone is a propeller that pushes the way it faces (a sticky piston
//! pushes the other way), a carpet is a sail and a chain is a symmetric
//! sail. Masses, volumes, friction and fragility follow Sable's shipped
//! definitions for vanilla blocks, so builds handle alike.

use pumpkin_data::block_properties::{
    Axis, CampfireLikeProperties, ChainLikeProperties, SlabProperties, SlabType,
    StickyPistonLikeProperties,
};
use pumpkin_data::tag::{self, Taggable};
use pumpkin_data::{Block, BlockDirection, BlockState, FacingExt};
use pumpkin_sublevel::{Material, Voxel, VoxelKind, VoxelShape};
use pumpkin_util::math::position::BlockPos;

use crate::block::blocks::redstone::get_redstone_power;
use crate::world::World;

const SUPER_LIGHT_MASS: f32 = 0.25;
const LIGHT_MASS: f32 = 0.5;
const NORMAL_MASS: f32 = 1.0;
const HEAVY_MASS: f32 = 2.0;
const SUPER_HEAVY_MASS: f32 = 4.0;
/// Sable makes bedrock absurdly heavy so it can never be lifted.
const BEDROCK_MASS: f32 = 1000.0;

/// Lifting gas a hot air burner feeds its balloon at full signal, in cubic
/// metres, as in Aeronautics.
const BURNER_OUTPUT: f32 = 500.0;
/// A propeller at full signal turns at Create's top speed.
const FULL_SIGNAL_RPM: f32 = 256.0;
/// Sable's lift scalars for a sail (Create's) and a symmetric sail.
const SAIL_PARALLEL_DRAG: f32 = 0.75;
const SAIL_LIFT: f32 = 0.475;
const SYMMETRIC_SAIL_PARALLEL_DRAG: f32 = 1.75;

/// Stone in its many forms weighs twice what its size suggests.
const HEAVY: &[Block] = &[
    Block::STONE,
    Block::COBBLESTONE,
    Block::MOSSY_COBBLESTONE,
    Block::SMOOTH_STONE,
    Block::STONE_BRICKS,
    Block::MOSSY_STONE_BRICKS,
    Block::CRACKED_STONE_BRICKS,
    Block::CHISELED_STONE_BRICKS,
    Block::DEEPSLATE,
    Block::COBBLED_DEEPSLATE,
    Block::POLISHED_DEEPSLATE,
    Block::DEEPSLATE_BRICKS,
    Block::DEEPSLATE_TILES,
    Block::BLACKSTONE,
    Block::POLISHED_BLACKSTONE,
    Block::POLISHED_BLACKSTONE_BRICKS,
    Block::BASALT,
    Block::SMOOTH_BASALT,
    Block::POLISHED_BASALT,
    Block::OBSIDIAN,
    Block::CRYING_OBSIDIAN,
    Block::GRANITE,
    Block::POLISHED_GRANITE,
    Block::DIORITE,
    Block::POLISHED_DIORITE,
    Block::ANDESITE,
    Block::POLISHED_ANDESITE,
    Block::TUFF,
    Block::CALCITE,
];

/// Blocks of stored material, and anvils, weigh four times as much.
const SUPER_HEAVY: &[Block] = &[
    Block::COAL_BLOCK,
    Block::RAW_IRON_BLOCK,
    Block::RAW_GOLD_BLOCK,
    Block::RAW_COPPER_BLOCK,
    Block::IRON_BLOCK,
    Block::GOLD_BLOCK,
    Block::DIAMOND_BLOCK,
    Block::EMERALD_BLOCK,
    Block::LAPIS_BLOCK,
    Block::REDSTONE_BLOCK,
    Block::NETHERITE_BLOCK,
    Block::QUARTZ_BLOCK,
    Block::BRICKS,
    Block::ANVIL,
    Block::CHIPPED_ANVIL,
    Block::DAMAGED_ANVIL,
];

/// Blocks a quarter of a block thick: Sable's `quarter_volume` list.
const QUARTER_VOLUME: &[Block] = &[
    Block::LADDER,
    Block::IRON_BARS,
    Block::COCOA,
    Block::MOSS_CARPET,
    Block::BAMBOO,
    Block::SEA_PICKLE,
    Block::CHAIN,
    Block::LIGHTNING_ROD,
    Block::FLOWER_POT,
    Block::END_ROD,
    Block::LANTERN,
    Block::SOUL_LANTERN,
    Block::REPEATER,
    Block::COMPARATOR,
    Block::OBSERVER,
];

/// Sable's `fragile` tag: these break when a sub-level hits something.
const FRAGILE: &[Block] = &[
    Block::BAMBOO,
    Block::MELON,
    Block::PUMPKIN,
    Block::CACTUS,
    Block::ICE,
    Block::FROSTED_ICE,
    Block::LILY_PAD,
];

/// Sable's `frictive` tag, plus its grindstone and mud exceptions.
const STICKY: &[Block] = &[
    Block::SOUL_SAND,
    Block::SOUL_SOIL,
    Block::HONEY_BLOCK,
    Block::CACTUS,
];

fn fragile(block: &Block) -> bool {
    block.has_tag(&tag::Block::MINECRAFT_LEAVES) || FRAGILE.contains(block)
}

/// Describes the block at `position` for the physics: its shape, mass,
/// volume, material, and any aerodynamic role. `signal` is the redstone
/// signal reaching it, which drives burners and propellers.
#[must_use]
pub fn block_voxel(position: [i32; 3], block: &Block, state: &BlockState, signal: u8) -> Voxel {
    if state.is_liquid() {
        return Voxel::liquid(position);
    }
    let shape = collision_shape(state);
    // Sable gives blocks without a collision shape no mass: torches and
    // flowers ride for free.
    let mass = if shape.is_solid() {
        block_mass(block, state)
    } else {
        0.0
    };
    Voxel {
        position,
        kind: aerodynamic_kind(block, state, signal),
        shape,
        material: Material {
            friction: friction(block),
            restitution: if *block == Block::SLIME_BLOCK {
                0.5
            } else {
                0.0
            },
            fragile: fragile(block),
        },
        mass,
        volume: volume(block, state),
        liquid: false,
    }
}

/// The block's collision boxes, clipped to the unit cube as Sable does.
#[must_use]
pub fn collision_shape(state: &BlockState) -> VoxelShape {
    let mut boxes: Vec<[f32; 6]> = Vec::new();
    for shape in state.get_block_collision_shapes() {
        let clamp = |value: f64| value.clamp(0.0, 1.0) as f32;
        boxes.push([
            clamp(shape.min.x),
            clamp(shape.min.y),
            clamp(shape.min.z),
            clamp(shape.max.x),
            clamp(shape.max.y),
            clamp(shape.max.z),
        ]);
    }
    match boxes.as_slice() {
        [] => VoxelShape::None,
        [[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]] => VoxelShape::Cube,
        _ => VoxelShape::Boxes(boxes.into_boxed_slice()),
    }
}

/// What `block` in `state` does besides weighing something.
#[must_use]
pub fn aerodynamic_kind(block: &Block, state: &BlockState, signal: u8) -> VoxelKind {
    if block.has_tag(&tag::Block::MINECRAFT_WOOL) {
        return VoxelKind::Envelope;
    }
    if block.has_tag(&tag::Block::MINECRAFT_CAMPFIRES) {
        // A burner heats in proportion to its signal; an unlit campfire
        // is just a grill.
        return if CampfireLikeProperties::from_state_id(state.id).lit {
            VoxelKind::Heater {
                output: BURNER_OUTPUT * f32::from(signal) / 15.0,
            }
        } else {
            VoxelKind::Solid
        };
    }
    if *block == Block::PISTON || *block == Block::STICKY_PISTON {
        let facing = StickyPistonLikeProperties::from_state_id(state.id).facing;
        let offset = facing.to_block_direction().to_offset();
        let rpm = FULL_SIGNAL_RPM * f32::from(signal) / 15.0;
        return VoxelKind::Thruster {
            direction: [offset.x as f32, offset.y as f32, offset.z as f32],
            // A sticky piston is a reversed propeller.
            rpm: if *block == Block::STICKY_PISTON {
                -rpm
            } else {
                rpm
            },
        };
    }
    if block.has_tag(&tag::Block::MINECRAFT_WOOL_CARPETS) {
        return VoxelKind::LiftSurface {
            normal: [0.0, 1.0, 0.0],
            parallel_drag: SAIL_PARALLEL_DRAG,
            lift: SAIL_LIFT,
        };
    }
    if *block == Block::CHAIN {
        let axis = ChainLikeProperties::from_state_id(state.id).r#axis;
        let normal = match axis {
            Axis::X => [1.0, 0.0, 0.0],
            Axis::Y => [0.0, 1.0, 0.0],
            Axis::Z => [0.0, 0.0, 1.0],
        };
        return VoxelKind::LiftSurface {
            normal,
            parallel_drag: SYMMETRIC_SAIL_PARALLEL_DRAG,
            lift: 0.0,
        };
    }
    VoxelKind::Solid
}

/// The strongest redstone signal any neighbour feeds into `pos`, 0 to 15.
#[must_use]
pub fn signal_at(world: &World, pos: BlockPos) -> u8 {
    let mut best = 0;
    for direction in BlockDirection::all() {
        let neighbour = pos.offset(direction.to_offset());
        let (block, state) = world.get_block_and_state(&neighbour);
        best = best.max(get_redstone_power(
            block, state, world, &neighbour, direction,
        ));
    }
    best
}

/// The mass of `block` in `state`, in block masses.
#[must_use]
pub fn block_mass(block: &Block, state: &BlockState) -> f32 {
    if *block == Block::BEDROCK {
        return BEDROCK_MASS;
    }
    let double = block.has_tag(&tag::Block::MINECRAFT_SLABS)
        && SlabProperties::from_state_id(state.id).r#type == SlabType::Double;
    if *block == Block::PISTON || *block == Block::STICKY_PISTON {
        // Sable: an extended piston weighs half.
        return if StickyPistonLikeProperties::from_state_id(state.id).extended {
            0.5
        } else {
            NORMAL_MASS
        };
    }
    let super_light = block.has_tag(&tag::Block::MINECRAFT_WOOL)
        || block.has_tag(&tag::Block::MINECRAFT_WOOL_CARPETS)
        || block.has_tag(&tag::Block::MINECRAFT_LEAVES)
        || block.has_tag(&tag::Block::MINECRAFT_TRAPDOORS)
        || block.has_tag(&tag::Block::MINECRAFT_DOORS)
        || block.has_tag(&tag::Block::MINECRAFT_FENCES)
        || block.has_tag(&tag::Block::MINECRAFT_FENCE_GATES)
        || block.has_tag(&tag::Block::MINECRAFT_CANDLES)
        || block.has_tag(&tag::Block::MINECRAFT_SAPLINGS)
        || QUARTER_VOLUME.contains(block);
    let light = block.has_tag(&tag::Block::MINECRAFT_PLANKS)
        || block.has_tag(&tag::Block::MINECRAFT_LOGS)
        || block.has_tag(&tag::Block::MINECRAFT_SLABS)
        || block.has_tag(&tag::Block::MINECRAFT_STAIRS);
    let mass = if super_light {
        SUPER_LIGHT_MASS
    } else if light {
        LIGHT_MASS
    } else if HEAVY.contains(block) {
        HEAVY_MASS
    } else if SUPER_HEAVY.contains(block) {
        SUPER_HEAVY_MASS
    } else {
        NORMAL_MASS
    };
    if double { mass * 2.0 } else { mass }
}

/// How much of a full block `block` displaces in a liquid.
#[must_use]
pub fn volume(block: &Block, state: &BlockState) -> f32 {
    let double = block.has_tag(&tag::Block::MINECRAFT_SLABS)
        && SlabProperties::from_state_id(state.id).r#type == SlabType::Double;
    if QUARTER_VOLUME.contains(block)
        || block.has_tag(&tag::Block::MINECRAFT_FENCES)
        || block.has_tag(&tag::Block::MINECRAFT_FENCE_GATES)
        || block.has_tag(&tag::Block::MINECRAFT_TRAPDOORS)
        || block.has_tag(&tag::Block::MINECRAFT_DOORS)
        || block.has_tag(&tag::Block::MINECRAFT_WOOL_CARPETS)
        || block.has_tag(&tag::Block::MINECRAFT_CANDLES)
    {
        return if double { 0.5 } else { 0.25 };
    }
    if block.has_tag(&tag::Block::MINECRAFT_SLABS) || block.has_tag(&tag::Block::MINECRAFT_STAIRS) {
        return if double { 1.0 } else { 0.5 };
    }
    1.0
}

/// Sable's friction block properties.
fn friction(block: &Block) -> f32 {
    if block.has_tag(&tag::Block::MINECRAFT_ICE) {
        0.0
    } else if STICKY.contains(block) {
        1.65
    } else if *block == Block::GRINDSTONE {
        0.05
    } else if *block == Block::MUD {
        0.25
    } else {
        1.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn voxel(block: &Block, state: &BlockState, signal: u8) -> (VoxelKind, f32, f32, Material) {
        let voxel = block_voxel([0; 3], block, state, signal);
        (voxel.kind, voxel.mass, voxel.volume, voxel.material)
    }

    fn thrust(direction: [f32; 3], rpm: f32) -> VoxelKind {
        VoxelKind::Thruster { direction, rpm }
    }

    #[test]
    fn stand_in_blocks_take_their_roles() {
        assert_eq!(
            voxel(&Block::WHITE_WOOL, Block::WHITE_WOOL.default_state, 0).0,
            VoxelKind::Envelope
        );
        assert_eq!(
            voxel(&Block::OAK_PLANKS, Block::OAK_PLANKS.default_state, 0).1,
            0.5
        );
        assert_eq!(voxel(&Block::STONE, Block::STONE.default_state, 0).1, 2.0);
        assert_eq!(
            voxel(&Block::GOLD_BLOCK, Block::GOLD_BLOCK.default_state, 0).1,
            4.0
        );
        assert_eq!(voxel(&Block::DIRT, Block::DIRT.default_state, 0).1, 1.0);
        assert_eq!(
            voxel(&Block::BEDROCK, Block::BEDROCK.default_state, 0).1,
            1000.0
        );
        // A torch has no collision shape and so no mass.
        assert_eq!(voxel(&Block::TORCH, Block::TORCH.default_state, 0).1, 0.0);
        assert_eq!(
            collision_shape(Block::TORCH.default_state),
            VoxelShape::None
        );

        // A campfire heats only when lit and fed a signal, in proportion.
        assert_eq!(
            voxel(&Block::CAMPFIRE, Block::CAMPFIRE.default_state, 0).0,
            VoxelKind::Heater { output: 0.0 }
        );
        assert_eq!(
            voxel(&Block::CAMPFIRE, Block::CAMPFIRE.default_state, 7).0,
            VoxelKind::Heater {
                output: 500.0 * 7.0 / 15.0
            }
        );
        let mut campfire = CampfireLikeProperties::from_state_id(Block::CAMPFIRE.default_state.id);
        campfire.lit = false;
        let quenched = BlockState::from_id(campfire.to_state_id(&Block::CAMPFIRE));
        assert_eq!(voxel(&Block::CAMPFIRE, quenched, 15).0, VoxelKind::Solid);

        // A piston at full signal is a 256 rpm propeller along its facing
        // (north by default); a sticky one runs reversed.
        assert_eq!(
            voxel(&Block::PISTON, Block::PISTON.default_state, 15).0,
            thrust([0.0, 0.0, -1.0], 256.0)
        );
        assert_eq!(
            voxel(
                &Block::STICKY_PISTON,
                Block::STICKY_PISTON.default_state,
                15
            )
            .0,
            thrust([0.0, 0.0, -1.0], -256.0)
        );
        assert_eq!(voxel(&Block::PISTON, Block::PISTON.default_state, 0).1, 1.0);
    }

    #[test]
    fn sails_come_in_two_kinds() {
        assert_eq!(
            voxel(&Block::WHITE_CARPET, Block::WHITE_CARPET.default_state, 0).0,
            VoxelKind::LiftSurface {
                normal: [0.0, 1.0, 0.0],
                parallel_drag: 0.75,
                lift: 0.475
            }
        );
        assert_eq!(
            voxel(&Block::CHAIN, Block::CHAIN.default_state, 0).0,
            VoxelKind::LiftSurface {
                normal: [0.0, 1.0, 0.0],
                parallel_drag: 1.75,
                lift: 0.0
            }
        );
        // Chains are thin: a quarter of a block in volume.
        assert_eq!(voxel(&Block::CHAIN, Block::CHAIN.default_state, 0).2, 0.25);
    }

    #[test]
    fn slabs_weigh_and_displace_their_halves() {
        let mut slab = SlabProperties::default(&Block::OAK_SLAB);
        let single = BlockState::from_id(slab.to_state_id(&Block::OAK_SLAB));
        let (kind, mass, volume, _) = voxel(&Block::OAK_SLAB, single, 0);
        assert_eq!(kind, VoxelKind::Solid);
        assert_eq!((mass, volume), (0.5, 0.5));
        assert!(matches!(collision_shape(single), VoxelShape::Boxes(_)));
        slab.r#type = SlabType::Double;
        let double = BlockState::from_id(slab.to_state_id(&Block::OAK_SLAB));
        assert_eq!(
            (
                voxel(&Block::OAK_SLAB, double, 0).1,
                voxel(&Block::OAK_SLAB, double, 0).2
            ),
            (1.0, 1.0)
        );
        assert!(matches!(collision_shape(double), VoxelShape::Cube));
    }

    #[test]
    fn sable_materials_carry_over() {
        assert_eq!(
            voxel(&Block::ICE, Block::ICE.default_state, 0).3.friction,
            0.0
        );
        assert_eq!(
            voxel(&Block::SOUL_SAND, Block::SOUL_SAND.default_state, 0)
                .3
                .friction,
            1.65
        );
        assert_eq!(
            voxel(&Block::SLIME_BLOCK, Block::SLIME_BLOCK.default_state, 0)
                .3
                .restitution,
            0.5
        );
        assert!(
            voxel(&Block::OAK_LEAVES, Block::OAK_LEAVES.default_state, 0)
                .3
                .fragile
        );
        // Glass is not in Sable's fragile tag, nor is water.
        assert!(
            !voxel(&Block::GLASS, Block::GLASS.default_state, 0)
                .3
                .fragile
        );
        assert!(
            !voxel(&Block::WATER, Block::WATER.default_state, 0)
                .3
                .fragile
        );
        assert_eq!(
            voxel(&Block::WATER, Block::WATER.default_state, 0).0,
            VoxelKind::Solid
        );
        assert!(Block::WATER.default_state.is_liquid());
    }
}
