//! What blocks do to a sub-level, in the physics' terms.
//!
//! Sable and the mods on it describe that with tags: Sable's own for mass,
//! volume and material, Aeronautics' and Simulated's for what a block does
//! to the air. Pumpkin knows vanilla's tags from its generated tables and
//! the mods' from the installed profile, `pumpkin_data::mods`; where a mod
//! names a block instead of tagging it, the block is named here.

use pumpkin_data::block_properties::{SlabProperties, SlabType, StickyPistonLikeProperties};
use pumpkin_data::mods;
use pumpkin_data::tag::{self, Taggable};
use pumpkin_data::{Block, BlockDirection, BlockState};
use pumpkin_sublevel::{FloatingMaterial, Material, Voxel, VoxelKind, VoxelShape};
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

/// Aeronautics' `hot_air_burner_max` and `steam_vent_hot_air_amount`: the
/// lifting gas a hot air burner, or a steam vent, feeds its balloon at full
/// signal, in cubic metres.
const BURNER_OUTPUT: f32 = 500.0;
const STEAM_VENT_OUTPUT: f32 = 5000.0;
/// Sable's lift scalars for a sail (Create's) and a symmetric sail.
const SAIL_PARALLEL_DRAG: f32 = 0.75;
const SAIL_LIFT: f32 = 0.475;
const SYMMETRIC_SAIL_PARALLEL_DRAG: f32 = 1.75;

/// Aeronautics' adjustable burner (`HotAirBurnerBlock`): heats the balloon
/// above it in proportion to the redstone signal reaching it.
pub const HOT_AIR_BURNER: &str = "aeronautics:adjustable_burner";
/// Aeronautics' steam vent: the same, with ten times the gas.
pub const STEAM_VENT: &str = "aeronautics:steam_vent";

/// Stone in its many forms weighs twice what its size suggests: Sable's
/// `#sable:heavy`, for vanilla.
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

/// Blocks of stored material, and anvils, weigh four times as much: Sable's
/// `#sable:super_heavy`, for vanilla.
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

/// Blocks a quarter of a block thick: Sable's `quarter_volume` list, for
/// vanilla.
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

/// Sable's `#sable:end_stones`, which carry its `end_stone` floating
/// material; the slab at half strength. For vanilla.
const END_STONE: &[Block] = &[
    Block::END_STONE,
    Block::END_STONE_BRICKS,
    Block::END_STONE_BRICK_SLAB,
    Block::END_STONE_BRICK_STAIRS,
    Block::END_STONE_BRICK_WALL,
];

/// Sable's `fragile` tag: these break when a sub-level hits something. For
/// vanilla.
const FRAGILE: &[Block] = &[
    Block::BAMBOO,
    Block::MELON,
    Block::PUMPKIN,
    Block::CACTUS,
    Block::ICE,
    Block::FROSTED_ICE,
    Block::LILY_PAD,
];

/// Sable's `frictive` tag, for vanilla.
const STICKY: &[Block] = &[
    Block::SOUL_SAND,
    Block::SOUL_SOIL,
    Block::HONEY_BLOCK,
    Block::CACTUS,
];

/// Whether the installed profile puts `block` in the tag `tag`
/// (`namespace:path`). Vanilla's own tags are the generated ones.
fn tagged(block: &Block, tag: &str) -> bool {
    mods::block_has_tag(block, tag)
}

fn fragile(block: &Block) -> bool {
    block.has_tag(&tag::Block::MINECRAFT_LEAVES)
        || FRAGILE.contains(block)
        || tagged(block, "sable:fragile")
}

/// Whether `block` in `state` is a slab, and a double one at that: Sable
/// doubles what a double slab weighs, displaces and lifts.
fn double_slab(block: &Block, state: &BlockState) -> Option<bool> {
    if block.has_tag(&tag::Block::MINECRAFT_SLABS) {
        return Some(SlabProperties::from_state_id(state.id).r#type == SlabType::Double);
    }
    // A mod's slab has the same `type` property.
    mods::state_prop(state.id, "type").map(|kind| kind == "double")
}

/// Whether a piston in `state` is extended: Sable lets it weigh half.
fn extended(block: &Block, state: &BlockState) -> bool {
    if *block == Block::PISTON || *block == Block::STICKY_PISTON {
        return StickyPistonLikeProperties::from_state_id(state.id).extended;
    }
    mods::state_prop(state.id, "extended") == Some("true")
}

/// Describes the block at `position` for the physics: its shape, mass,
/// volume, material, and any aerodynamic role. `signal` is the redstone
/// signal reaching it, which drives burners.
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
            // Sable's `#sable:bouncy`, the slime block among vanilla.
            restitution: if tagged(block, "sable:bouncy") || *block == Block::SLIME_BLOCK {
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
    // Aeronautics' `#aeronautics:airtight`: its envelopes, and wool.
    if tagged(block, "aeronautics:airtight") || block.has_tag(&tag::Block::MINECRAFT_WOOL) {
        return VoxelKind::Envelope;
    }
    // A burner heats in proportion to the signal reaching it.
    let output = match block.name {
        HOT_AIR_BURNER => BURNER_OUTPUT,
        STEAM_VENT => STEAM_VENT_OUTPUT,
        _ => 0.0,
    };
    if output > 0.0 {
        return VoxelKind::Heater {
            output: output * f32::from(signal) / 15.0,
        };
    }
    if tagged(block, "aeronautics:levitite") {
        return levitite();
    }
    if tagged(block, "sable:end_stones") || END_STONE.contains(block) {
        return VoxelKind::Floating {
            material: FloatingMaterial::END_STONE,
            scale: if double_slab(block, state) == Some(false) {
                0.5
            } else {
                1.0
            },
        };
    }
    if tagged(block, "simulated:symmetric_sails")
        && let Some(normal) = axis_normal(state)
    {
        return VoxelKind::LiftSurface {
            normal,
            parallel_drag: SYMMETRIC_SAIL_PARALLEL_DRAG,
            lift: 0.0,
        };
    }
    // Create's sails, `create:<colour>_sail`, face the way they were placed;
    // Sable gives them their lift off the back.
    if is_create_sail(block)
        && let Some(normal) = facing_opposite(state)
    {
        return VoxelKind::LiftSurface {
            normal,
            parallel_drag: SAIL_PARALLEL_DRAG,
            lift: SAIL_LIFT,
        };
    }
    VoxelKind::Solid
}

/// A block of Aeronautics' `levitite` floating material.
const fn levitite() -> VoxelKind {
    VoxelKind::Floating {
        material: FloatingMaterial::LEVITITE,
        scale: 1.0,
    }
}

/// The positive direction of a block's `axis` property: a symmetric sail's
/// normal, Simulated's `sable$getNormal`.
fn axis_normal(state: &BlockState) -> Option<[f32; 3]> {
    Some(match mods::state_prop(state.id, "axis")? {
        "x" => [1.0, 0.0, 0.0],
        "y" => [0.0, 1.0, 0.0],
        "z" => [0.0, 0.0, 1.0],
        _ => return None,
    })
}

fn is_create_sail(block: &Block) -> bool {
    block.name.starts_with("create:") && block.name.ends_with("_sail")
}

/// The direction opposite to the one a block's `facing` property names:
/// the normal Sable gives Create's sails.
fn facing_opposite(state: &BlockState) -> Option<[f32; 3]> {
    Some(match mods::state_prop(state.id, "facing")? {
        "north" => [0.0, 0.0, 1.0],
        "south" => [0.0, 0.0, -1.0],
        "west" => [1.0, 0.0, 0.0],
        "east" => [-1.0, 0.0, 0.0],
        "up" => [0.0, -1.0, 0.0],
        "down" => [0.0, 1.0, 0.0],
        _ => return None,
    })
}

/// Whether `block` heats a balloon: its output follows the redstone signal
/// reaching it.
#[must_use]
pub fn heater(block: &Block) -> bool {
    matches!(block.name, HOT_AIR_BURNER | STEAM_VENT)
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
    let double = double_slab(block, state) == Some(true);
    // Sable: an extended piston weighs half.
    if tagged(block, "sable:pistons") && extended(block, state) {
        return 0.5;
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
        || QUARTER_VOLUME.contains(block)
        || tagged(block, "sable:super_light");
    let light = block.has_tag(&tag::Block::MINECRAFT_PLANKS)
        || block.has_tag(&tag::Block::MINECRAFT_LOGS)
        || block.has_tag(&tag::Block::MINECRAFT_SLABS)
        || block.has_tag(&tag::Block::MINECRAFT_STAIRS)
        || tagged(block, "sable:light");
    // Sable's mass tags, the lighter winning.
    let mass = if tagged(block, "sable:weightless") {
        0.0
    } else if super_light {
        SUPER_LIGHT_MASS
    } else if light {
        LIGHT_MASS
    } else if HEAVY.contains(block) || tagged(block, "sable:heavy") {
        HEAVY_MASS
    } else if SUPER_HEAVY.contains(block) || tagged(block, "sable:super_heavy") {
        SUPER_HEAVY_MASS
    } else {
        NORMAL_MASS
    };
    if double { mass * 2.0 } else { mass }
}

/// How much of a full block `block` displaces in a liquid.
#[must_use]
pub fn volume(block: &Block, state: &BlockState) -> f32 {
    let double = double_slab(block, state) == Some(true);
    if QUARTER_VOLUME.contains(block)
        || block.has_tag(&tag::Block::MINECRAFT_FENCES)
        || block.has_tag(&tag::Block::MINECRAFT_FENCE_GATES)
        || block.has_tag(&tag::Block::MINECRAFT_TRAPDOORS)
        || block.has_tag(&tag::Block::MINECRAFT_DOORS)
        || block.has_tag(&tag::Block::MINECRAFT_WOOL_CARPETS)
        || block.has_tag(&tag::Block::MINECRAFT_CANDLES)
        || tagged(block, "sable:quarter_volume")
    {
        return if double { 0.5 } else { 0.25 };
    }
    if block.has_tag(&tag::Block::MINECRAFT_SLABS)
        || block.has_tag(&tag::Block::MINECRAFT_STAIRS)
        || tagged(block, "sable:half_volume")
    {
        return if double { 1.0 } else { 0.5 };
    }
    1.0
}

/// Sable's friction block properties.
fn friction(block: &Block) -> f32 {
    if block.has_tag(&tag::Block::MINECRAFT_ICE) || tagged(block, "sable:slippery") {
        0.0
    } else if STICKY.contains(block) || tagged(block, "sable:frictive") {
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

    /// A block of the test profile in the state with the properties `props`.
    fn modded(
        name: &str,
        props: &[(&str, &str)],
    ) -> Result<(&'static Block, &'static BlockState), &'static str> {
        crate::mods::profile::tests::install_test_profile();
        let block = Block::from_registry_key(name).ok_or("no such block in the profile")?;
        let state = mods::state_with(block, props).ok_or("no such state in the profile")?;
        Ok((block, state))
    }

    #[test]
    fn the_mods_blocks_take_their_roles() -> Result<(), &'static str> {
        let (envelope, state) = modded("aeronautics:white_envelope", &[])?;
        assert_eq!(aerodynamic_kind(envelope, state, 0), VoxelKind::Envelope);
        // Sable's `#sable:super_light`: a quarter of a block mass, and bouncy.
        assert_eq!(voxel(envelope, state, 0).1, 0.25);
        assert_eq!(voxel(envelope, state, 0).3.restitution, 0.5);

        // A burner heats in proportion to the signal reaching it.
        let (burner, state) = modded(HOT_AIR_BURNER, &[])?;
        assert_eq!(
            aerodynamic_kind(burner, state, 0),
            VoxelKind::Heater { output: 0.0 }
        );
        assert_eq!(
            aerodynamic_kind(burner, state, 7),
            VoxelKind::Heater {
                output: 500.0 * 7.0 / 15.0
            }
        );
        // A steam vent: ten times a burner's gas.
        let (vent, state) = modded(STEAM_VENT, &[])?;
        assert_eq!(
            aerodynamic_kind(vent, state, 15),
            VoxelKind::Heater { output: 5000.0 }
        );

        let (levitite, state) = modded("aeronautics:levitite", &[])?;
        assert_eq!(
            aerodynamic_kind(levitite, state, 0),
            VoxelKind::Floating {
                material: FloatingMaterial::LEVITITE,
                scale: 1.0
            }
        );
        Ok(())
    }

    #[test]
    fn sails_take_the_air_off_their_broad_sides() -> Result<(), &'static str> {
        // A symmetric sail's normal is the positive direction of its axis,
        // and it only drags.
        let (sail, state) = modded("simulated:symmetric_sail", &[("axis", "z")])?;
        assert_eq!(
            aerodynamic_kind(sail, state, 0),
            VoxelKind::LiftSurface {
                normal: [0.0, 0.0, 1.0],
                parallel_drag: 1.75,
                lift: 0.0
            }
        );
        // A Create sail faces the way it was placed and lifts off the back.
        let (sail, state) = modded("create:white_sail", &[("facing", "east")])?;
        assert_eq!(
            aerodynamic_kind(sail, state, 0),
            VoxelKind::LiftSurface {
                normal: [-1.0, 0.0, 0.0],
                parallel_drag: 0.75,
                lift: 0.475
            }
        );
        Ok(())
    }

    #[test]
    fn vanilla_blocks_keep_sables_properties() {
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
        // End stone keeps its floating material, a slab at half strength.
        assert_eq!(
            voxel(&Block::END_STONE, Block::END_STONE.default_state, 0).0,
            VoxelKind::Floating {
                material: FloatingMaterial::END_STONE,
                scale: 1.0
            }
        );
        let single = Block::END_STONE_BRICK_SLAB.default_state;
        assert_eq!(
            voxel(&Block::END_STONE_BRICK_SLAB, single, 0).0,
            VoxelKind::Floating {
                material: FloatingMaterial::END_STONE,
                scale: 0.5
            }
        );
        let mut slab = SlabProperties::default(&Block::END_STONE_BRICK_SLAB);
        slab.r#type = SlabType::Double;
        let double = BlockState::from_id(slab.to_state_id(&Block::END_STONE_BRICK_SLAB));
        assert_eq!(
            voxel(&Block::END_STONE_BRICK_SLAB, double, 0).0,
            VoxelKind::Floating {
                material: FloatingMaterial::END_STONE,
                scale: 1.0
            }
        );
        // An extended piston weighs half.
        assert_eq!(voxel(&Block::PISTON, Block::PISTON.default_state, 0).1, 1.0);
        assert_eq!(
            voxel(&Block::PISTON, Block::PISTON.default_state, 0).0,
            VoxelKind::Solid
        );
        // Nothing else heats, lifts or floats: those are the mods' blocks.
        for block in [
            Block::CAMPFIRE,
            Block::SOUL_CAMPFIRE,
            Block::STICKY_PISTON,
            Block::WHITE_CARPET,
            Block::CHAIN,
            Block::OCHRE_FROGLIGHT,
        ] {
            assert_eq!(
                voxel(&block, block.default_state, 15).0,
                VoxelKind::Solid,
                "{}",
                block.name
            );
        }
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
