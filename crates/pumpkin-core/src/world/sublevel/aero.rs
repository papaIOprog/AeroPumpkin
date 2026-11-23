//! What an assembled region's blocks do to its sub-level.
//!
//! Sable and Create: Aeronautics describe that with data files for their own
//! blocks. Pumpkin has none of those blocks, so ordinary ones stand in for
//! them: wool seals a balloon, a lit campfire or a fire heats it, a piston
//! pushes along the way it faces, and carpets and slabs are lift surfaces.
//! Masses follow Sable's tiers for vanilla blocks, so builds handle alike.

use pumpkin_data::block_properties::{
    CampfireLikeProperties, SlabProperties, SlabType, StickyPistonLikeProperties,
};
use pumpkin_data::tag::{self, Taggable};
use pumpkin_data::{Block, BlockState, FacingExt};
use pumpkin_sublevel::VoxelKind;

const SUPER_LIGHT_MASS: f32 = 0.25;
const LIGHT_MASS: f32 = 0.5;
pub const NORMAL_MASS: f32 = 1.0;
const HEAVY_MASS: f32 = 2.0;
const SUPER_HEAVY_MASS: f32 = 4.0;

/// Lifting gas a heater feeds its balloon, in cubic metres. Aeronautics'
/// hot air burner tops out at 500.
const CAMPFIRE_OUTPUT: f32 = 500.0;
const FIRE_OUTPUT: f32 = 250.0;

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

/// What `block` in `state` does besides weighing something.
#[must_use]
pub fn voxel_kind(block: &Block, state: &BlockState) -> VoxelKind {
    if *block == Block::PISTON || *block == Block::STICKY_PISTON {
        let facing = StickyPistonLikeProperties::from_state_id(state.id).facing;
        let offset = facing.to_block_direction().to_offset();
        return VoxelKind::Thruster {
            direction: [offset.x as f32, offset.y as f32, offset.z as f32],
        };
    }
    if block.has_tag(&tag::Block::MINECRAFT_CAMPFIRES) {
        return if CampfireLikeProperties::from_state_id(state.id).lit {
            VoxelKind::Heater {
                output: CAMPFIRE_OUTPUT,
            }
        } else {
            VoxelKind::Solid
        };
    }
    if block.has_tag(&tag::Block::MINECRAFT_FIRE) {
        return VoxelKind::Heater {
            output: FIRE_OUTPUT,
        };
    }
    if block.has_tag(&tag::Block::MINECRAFT_WOOL) {
        return VoxelKind::Envelope;
    }
    // A thin surface lifts along its open face.
    if block.has_tag(&tag::Block::MINECRAFT_WOOL_CARPETS) {
        return VoxelKind::LiftSurface {
            normal: [0.0, 1.0, 0.0],
        };
    }
    if block.has_tag(&tag::Block::MINECRAFT_SLABS) {
        return match SlabProperties::from_state_id(state.id).r#type {
            SlabType::Bottom => VoxelKind::LiftSurface {
                normal: [0.0, 1.0, 0.0],
            },
            SlabType::Top => VoxelKind::LiftSurface {
                normal: [0.0, -1.0, 0.0],
            },
            SlabType::Double => VoxelKind::Solid,
        };
    }
    VoxelKind::Solid
}

/// The mass of `block` in `state`, in block masses.
#[must_use]
pub fn block_mass(block: &Block, state: &BlockState) -> f32 {
    let super_light = block.has_tag(&tag::Block::MINECRAFT_WOOL)
        || block.has_tag(&tag::Block::MINECRAFT_WOOL_CARPETS)
        || block.has_tag(&tag::Block::MINECRAFT_LEAVES)
        || block.has_tag(&tag::Block::MINECRAFT_TRAPDOORS)
        || block.has_tag(&tag::Block::MINECRAFT_DOORS)
        || block.has_tag(&tag::Block::MINECRAFT_FENCES)
        || block.has_tag(&tag::Block::MINECRAFT_FENCE_GATES)
        || block.has_tag(&tag::Block::MINECRAFT_CANDLES)
        || block.has_tag(&tag::Block::MINECRAFT_SAPLINGS);
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
    // Two slabs in one block are two slabs.
    if block.has_tag(&tag::Block::MINECRAFT_SLABS)
        && SlabProperties::from_state_id(state.id).r#type == SlabType::Double
    {
        return mass * 2.0;
    }
    mass
}

#[cfg(test)]
mod tests {
    use super::*;
    use pumpkin_data::block_properties::Facing;

    fn voxel(block: &Block, state: &BlockState) -> (VoxelKind, f32) {
        (voxel_kind(block, state), block_mass(block, state))
    }

    #[test]
    fn stand_in_blocks_take_their_roles() {
        assert_eq!(
            voxel(&Block::WHITE_WOOL, Block::WHITE_WOOL.default_state),
            (VoxelKind::Envelope, 0.25)
        );
        assert_eq!(
            voxel(&Block::OAK_PLANKS, Block::OAK_PLANKS.default_state).1,
            0.5
        );
        assert_eq!(voxel(&Block::STONE, Block::STONE.default_state).1, 2.0);
        assert_eq!(
            voxel(&Block::GOLD_BLOCK, Block::GOLD_BLOCK.default_state).1,
            4.0
        );
        assert_eq!(voxel(&Block::DIRT, Block::DIRT.default_state).1, 1.0);

        // A campfire heats while lit, as it is by default; quenched, it is
        // just material.
        assert_eq!(
            voxel_kind(&Block::CAMPFIRE, Block::CAMPFIRE.default_state),
            VoxelKind::Heater {
                output: CAMPFIRE_OUTPUT
            }
        );
        let mut campfire = CampfireLikeProperties::from_state_id(Block::CAMPFIRE.default_state.id);
        campfire.lit = false;
        let quenched = BlockState::from_id(campfire.to_state_id(&Block::CAMPFIRE));
        assert_eq!(voxel_kind(&Block::CAMPFIRE, quenched), VoxelKind::Solid);

        // A piston pushes the way it faces.
        let mut piston = StickyPistonLikeProperties::default(&Block::PISTON);
        piston.facing = Facing::Up;
        let raised = BlockState::from_id(piston.to_state_id(&Block::PISTON));
        assert_eq!(
            voxel_kind(&Block::PISTON, raised),
            VoxelKind::Thruster {
                direction: [0.0, 1.0, 0.0]
            }
        );
    }

    #[test]
    fn slabs_weigh_their_halves() {
        let mut slab = SlabProperties::default(&Block::OAK_SLAB);
        let single = BlockState::from_id(slab.to_state_id(&Block::OAK_SLAB));
        assert_eq!(
            voxel(&Block::OAK_SLAB, single),
            (
                VoxelKind::LiftSurface {
                    normal: [0.0, 1.0, 0.0]
                },
                0.5
            )
        );
        slab.r#type = SlabType::Double;
        let double = BlockState::from_id(slab.to_state_id(&Block::OAK_SLAB));
        assert_eq!(voxel(&Block::OAK_SLAB, double), (VoxelKind::Solid, 1.0));
    }
}
