//! Sails: Create's, which face the way they were placed, and Simulated's
//! symmetric ones, which lie across an axis.
//!
//! A dye recolours either, and Create's canvas comes off with shears. How
//! they take the air is the physics' (`world::sublevel::aero`). `C:` is
//! `com.simibubi.create.content.contraptions.bearing`, `S:` is
//! `dev.simulated_team.simulated.content.blocks.symmetric_sail`.

use std::sync::Arc;

use pumpkin_data::block_properties::{Axis, HorizontalFacing};
use pumpkin_data::item::Item;
use pumpkin_data::sound::{Sound, SoundCategory};
use pumpkin_data::{Block, BlockDirection, BlockId, BlockState, BlockStateId, FacingExt, mods};
use pumpkin_util::math::position::BlockPos;
use pumpkin_util::math::vector3::Vector3;
use pumpkin_world::world::BlockFlags;
use rand::RngExt;

use crate::block::registry::BlockActionResult;
use crate::block::{
    BlockBehaviour, BlockMetadata, OnLandedUponArgs, OnPlaceArgs, PathComputationType,
    UpdateEntityMovementAfterFallOnArgs, UseWithItemArgs, bounce_entity_after_fall,
};
use crate::entity::EntityBase;
use crate::world::World;

/// The sixteen dye colours sails come in.
const COLOURS: [&str; 16] = [
    "white",
    "orange",
    "magenta",
    "light_blue",
    "yellow",
    "lime",
    "pink",
    "gray",
    "light_gray",
    "cyan",
    "purple",
    "blue",
    "brown",
    "green",
    "red",
    "black",
];

/// `C:SailBlock.bounce`: how much of their fall sails give back.
const SAIL_BOUNCE: f64 = 0.26;

/// Create's `SailBlock`: a canvas on a frame.
pub struct SailBlock;

impl BlockMetadata for SailBlock {
    fn ids() -> Box<[BlockId]> {
        crate::mods::ported_blocks("create", &coloured("create", "_sail"))
    }
}

impl BlockBehaviour for SailBlock {
    /// `WrenchableDirectionalBlock.getStateForPlacement`, turned twice: the
    /// sail faces the way the player does.
    fn on_place(&self, args: OnPlaceArgs<'_>) -> BlockStateId {
        let facing = args.player.get_entity().get_horizontal_facing();
        mods::state_with(args.block, &[("facing", facing_name(facing))])
            .map_or(args.block.default_state.id, |state| state.id)
    }

    /// `fallOn`: a sail breaks the fall.
    fn on_landed_upon(&self, args: OnLandedUponArgs<'_>) {
        if let Some(living) = args.entity.get_living_entity() {
            living.handle_fall_damage(args.entity, args.fall_distance, 0.0);
        }
    }

    /// `updateEntityAfterFallOn`: a sail gives a little of the fall back.
    fn update_entity_movement_after_fall_on(&self, args: UpdateEntityMovementAfterFallOnArgs<'_>) {
        bounce_entity_after_fall(args.entity, SAIL_BOUNCE);
    }

    /// `useItemOn`: a dye recolours the sail, and shears take its canvas.
    fn use_with_item(&self, args: UseWithItemArgs<'_>) -> BlockActionResult {
        let colour = match dye_colour(args.item_stack) {
            Some(colour) => Some(colour),
            None if is_shears(args.item_stack) => None,
            None => return BlockActionResult::PassToDefaultBlockAction,
        };
        sound_of(args.world, args.position, colour.is_some());
        dye(
            args.world,
            args.block,
            args.world.get_block_state_id(args.position),
            *args.position,
            args.position.to_f64() + args.hit.cursor_pos.to_f64(),
            colour,
        );
        BlockActionResult::Consume
    }

    /// `isPathfindable`: nothing finds a way over a sail.
    fn is_pathfindable(&self, _state: &BlockState, _kind: PathComputationType) -> bool {
        false
    }
}

/// Create's sail frame: the sail without its canvas, which breaks no fall.
pub struct SailFrameBlock;

impl BlockMetadata for SailFrameBlock {
    fn ids() -> Box<[BlockId]> {
        crate::mods::ported_blocks("create", &["create:sail_frame"])
    }
}

impl BlockBehaviour for SailFrameBlock {
    /// `isPathfindable`: nothing finds a way over a sail frame.
    fn is_pathfindable(&self, _state: &BlockState, _kind: PathComputationType) -> bool {
        false
    }
}

/// Simulated's `SymmetricSailBlock`.
pub struct SymmetricSailBlock;

impl BlockMetadata for SymmetricSailBlock {
    fn ids() -> Box<[BlockId]> {
        crate::mods::ported_blocks("simulated", &coloured("simulated", "_symmetric_sail"))
    }
}

impl BlockBehaviour for SymmetricSailBlock {
    /// `RotatedPillarBlock.getStateForPlacement`, as the sail overrides it:
    /// it lies across the axis nearest to where the player looks.
    fn on_place(&self, args: OnPlaceArgs<'_>) -> BlockStateId {
        let nearest = args.player.get_entity().get_entity_facing_order()[0].to_block_direction();
        mods::state_with(args.block, &[("axis", axis_name(nearest.to_axis()))])
            .map_or(args.block.default_state.id, |state| state.id)
    }

    /// `fallOn`: a sail breaks the fall.
    fn on_landed_upon(&self, args: OnLandedUponArgs<'_>) {
        if let Some(living) = args.entity.get_living_entity() {
            living.handle_fall_damage(args.entity, args.fall_distance, 0.0);
        }
    }

    /// `updateEntityAfterFallOn`: a sail gives a little of the fall back.
    fn update_entity_movement_after_fall_on(&self, args: UpdateEntityMovementAfterFallOnArgs<'_>) {
        bounce_entity_after_fall(args.entity, SAIL_BOUNCE);
    }

    /// `useItemOn`: a dye recolours the sail.
    fn use_with_item(&self, args: UseWithItemArgs<'_>) -> BlockActionResult {
        let Some(colour) = dye_colour(args.item_stack) else {
            return BlockActionResult::PassToDefaultBlockAction;
        };
        sound_of(args.world, args.position, true);
        dye(
            args.world,
            args.block,
            args.world.get_block_state_id(args.position),
            *args.position,
            args.position.to_f64() + args.hit.cursor_pos.to_f64(),
            Some(colour),
        );
        BlockActionResult::Consume
    }
}

/// `S:SymmetricSailBlock.applyDye` and `C:SailBlock.applyDye`: the sail at
/// `position` becomes one of `colour`, for Create the frame for `None`.
///
/// A sail of that colour already passes the dye on: to the sail next to it
/// nearest to where the dye was used, and, if that is of the same colour
/// too, to every sail that plane holds, the sails reaching it across the
/// way it lies.
fn dye(
    world: &Arc<World>,
    block: &Block,
    state: BlockStateId,
    position: BlockPos,
    hit: Vector3<f64>,
    colour: Option<&str>,
) {
    let Some(family) = Sail::of(block) else {
        return;
    };
    let Some(dyed) = family.block(colour) else {
        return;
    };
    let Some(lies) = family.lies(state) else {
        return;
    };
    let dyed = mods::state_with(dyed, mods::state_props(state))
        .unwrap_or(dyed.default_state)
        .id;
    let flags = BlockFlags::NOTIFY_ALL;
    if dyed != state {
        world.set_block_state(&position, dyed, flags);
        return;
    }
    for direction in nearest_first(position, hit, lies.axis) {
        let offset = position.offset(direction.to_offset());
        let (other, other_state) = world.get_block_and_state(&offset);
        if family.takes(other, other_state.id, lies, false) && other_state.id != state {
            world.set_block_state(&offset, dyed, flags);
            return;
        }
    }
    // `timeout`: how many sails into the plane the dye reaches.
    let mut frontier = vec![position];
    let mut visited = vec![position];
    for _ in 0..=100 {
        let Some(current) = frontier.first().copied() else {
            break;
        };
        frontier.remove(0);
        for direction in BlockDirection::all() {
            if direction.to_axis() == lies.axis {
                continue;
            }
            let offset = current.offset(direction.to_offset());
            if visited.contains(&offset) {
                continue;
            }
            let (other, other_state) = world.get_block_and_state(&offset);
            // Create's frames take the dye only when it is shears.
            if !family.takes(other, other_state.id, lies, colour.is_none()) {
                continue;
            }
            if other_state.id != state {
                world.set_block_state(&offset, dyed, flags);
            }
            frontier.push(offset);
            visited.push(offset);
        }
    }
}

/// Which mod's sail a block is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Sail {
    Symmetric,
    Create,
}

/// How a sail lies: what its `axis` or `facing` property says, and the
/// axis that is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Bearing {
    value: &'static str,
    axis: Axis,
}

impl Sail {
    /// The family `block` belongs to.
    fn of(block: &Block) -> Option<Self> {
        let symmetric =
            block.name.starts_with("simulated:") && block.name.ends_with("_symmetric_sail");
        let create = block.name.starts_with("create:")
            && (block.name.ends_with("_sail") || block.name == "create:sail_frame");
        match (symmetric, create) {
            (true, false) => Some(Self::Symmetric),
            (false, true) => Some(Self::Create),
            _ => None,
        }
    }

    /// The family's sail of `colour`; Create's frame for `None`.
    fn block(self, colour: Option<&str>) -> Option<&'static Block> {
        let owned;
        let name = match (self, colour) {
            (Self::Create, None) => "create:sail_frame",
            (Self::Create, Some(colour)) => {
                owned = format!("create:{colour}_sail");
                &owned
            }
            (Self::Symmetric, Some(colour)) => {
                owned = format!("simulated:{colour}_symmetric_sail");
                &owned
            }
            (Self::Symmetric, None) => return None,
        };
        Block::from_registry_key(name)
    }

    /// How the sail in `state` lies.
    fn lies(self, state: BlockStateId) -> Option<Bearing> {
        let value = mods::state_prop(
            state,
            match self {
                Self::Symmetric => "axis",
                Self::Create => "facing",
            },
        )?;
        let axis = match value {
            "x" | "east" | "west" => Axis::X,
            "y" | "up" | "down" => Axis::Y,
            "z" | "north" | "south" => Axis::Z,
            _ => return None,
        };
        Some(Bearing { value, axis })
    }

    /// Whether the dye may reach `other`: a sail of the same family, lying
    /// the same way. Create's frames only take shears (`frame`).
    fn takes(self, other: &Block, other_state: BlockStateId, lies: Bearing, frame: bool) -> bool {
        Self::of(other) == Some(self)
            && self.lies(other_state) == Some(lies)
            && (frame || other.name != "create:sail_frame")
    }
}

/// The sails' neighbours across the way they lie, the one whose block the
/// dye was used nearest to first:
/// `IPlacementHelper.orderedByDistanceExceptAxis`.
fn nearest_first(position: BlockPos, hit: Vector3<f64>, except: Axis) -> Vec<BlockDirection> {
    let mut directions: Vec<BlockDirection> = BlockDirection::all()
        .into_iter()
        .filter(|direction| direction.to_axis() != except)
        .collect();
    let distance = |direction: &BlockDirection| {
        (hit - position.offset(direction.to_offset()).to_centered_f64()).length_squared()
    };
    directions.sort_by(|a, b| distance(a).total_cmp(&distance(b)));
    directions
}

/// The sound of a dye on a sail, or of shears on a canvas.
fn sound_of(world: &World, position: &BlockPos, dye: bool) {
    let at = position.to_centered_f64();
    if dye {
        world.play_sound_fine(
            Sound::ItemDyeUse,
            SoundCategory::Blocks,
            &at,
            1.0,
            1.1 - rand::rng().random::<f32>() * 0.2,
        );
    } else {
        world.play_sound(Sound::EntitySheepShear, SoundCategory::Blocks, &at);
    }
}

/// `DyeColor.getColor`: the colour of the dye item `stack` is.
fn dye_colour(stack: &pumpkin_data::item_stack::ItemStack) -> Option<&'static str> {
    COLOURS
        .iter()
        .find(|colour| is_item(stack, &format!("minecraft:{colour}_dye")))
        .copied()
}

fn is_shears(stack: &pumpkin_data::item_stack::ItemStack) -> bool {
    is_item(stack, "minecraft:shears")
}

fn is_item(stack: &pumpkin_data::item_stack::ItemStack, name: &str) -> bool {
    Item::from_registry_key(name).is_some_and(|item| item.id == stack.item.id)
}

/// The family's blocks by colour.
fn coloured(namespace: &str, suffix: &str) -> Vec<&'static str> {
    COLOURS
        .iter()
        .map(|colour| {
            Box::leak(format!("{namespace}:{colour}{suffix}").into_boxed_str()) as &'static str
        })
        .collect()
}

const fn facing_name(facing: HorizontalFacing) -> &'static str {
    match facing {
        HorizontalFacing::North => "north",
        HorizontalFacing::South => "south",
        HorizontalFacing::West => "west",
        HorizontalFacing::East => "east",
    }
}

const fn axis_name(axis: Axis) -> &'static str {
    match axis {
        Axis::X => "x",
        Axis::Y => "y",
        Axis::Z => "z",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_sail_lies_the_way_its_property_says() {
        crate::mods::profile::tests::install_test_profile();
        let create = Block::from_registry_key("create:white_sail").expect("sail");
        let state = mods::state_with(create, &[("facing", "north")]).expect("state");
        assert_eq!(
            Sail::of(create).and_then(|sail| sail.lies(state.id)),
            Some(Bearing {
                value: "north",
                axis: Axis::Z
            })
        );
        let symmetric = Block::from_registry_key("simulated:white_symmetric_sail").expect("sail");
        let state = mods::state_with(symmetric, &[("axis", "y")]).expect("state");
        assert_eq!(
            Sail::of(symmetric)
                .and_then(|sail| sail.lies(state.id))
                .map(|lies| lies.axis),
            Some(Axis::Y)
        );
        // A stone is nobody's sail, and the test profile knows no frame.
        assert_eq!(Sail::of(&Block::STONE), None);
        assert!(Sail::Create.block(None).is_none());
    }

    #[test]
    fn the_nearest_neighbour_comes_first_across_the_way_a_sail_lies() {
        let position = BlockPos(Vector3::new(10, 64, -7));
        // Used towards the block's east and south faces: east first, then
        // south, and never up or down, which lie along the sail.
        let hit = position.to_f64() + Vector3::new(1.0, 0.2, 0.9);
        assert_eq!(
            nearest_first(position, hit, Axis::Y),
            [
                BlockDirection::East,
                BlockDirection::South,
                BlockDirection::North,
                BlockDirection::West
            ]
        );
    }
}
