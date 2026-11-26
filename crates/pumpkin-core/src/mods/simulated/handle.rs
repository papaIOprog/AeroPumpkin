//! Simulated's handles: a bar to hold onto a contraption by.
//!
//! The client runs the hold and says what it wants each tick; the server
//! pulls the contraption. A player holding a handle without sneaking
//! climbs it themselves and pulls nothing. `S:` is
//! `dev.simulated_team.simulated.content.blocks.handle`.

use std::sync::Arc;

use pumpkin_data::block_properties::Axis;
use pumpkin_data::item::Item;
use pumpkin_data::tag::Taggable;
use pumpkin_data::{
    Block, BlockDirection, BlockId, BlockState, BlockStateId, HorizontalFacingExt, mods,
};
use pumpkin_protocol::codec::var_int::VarInt;
use pumpkin_protocol::ser::NetworkWriteExt;
use pumpkin_util::math::position::BlockPos;
use pumpkin_util::math::vector3::Vector3;

use crate::block::registry::BlockActionResult;
use crate::block::{
    BlockBehaviour, BlockMetadata, CanPlaceAtArgs, GetComparatorOutputArgs,
    GetStateForNeighborUpdateArgs, OnPlaceArgs, UseWithItemArgs,
};
use crate::entity::EntityBase;
use crate::entity::player::Player;
use crate::mods::packets::{read_block_pos, write_uuid};
use pumpkin_world::world::BlockAccessor;

/// Simulated's handles, `#simulated:handles`: iron, copper, and one of
/// every dye colour.
const HANDLES: [&str; 18] = [
    "simulated:iron_handle",
    "simulated:copper_handle",
    "simulated:white_handle",
    "simulated:orange_handle",
    "simulated:magenta_handle",
    "simulated:light_blue_handle",
    "simulated:yellow_handle",
    "simulated:lime_handle",
    "simulated:pink_handle",
    "simulated:gray_handle",
    "simulated:light_gray_handle",
    "simulated:cyan_handle",
    "simulated:purple_handle",
    "simulated:blue_handle",
    "simulated:brown_handle",
    "simulated:green_handle",
    "simulated:red_handle",
    "simulated:black_handle",
];

/// `S:ClientHandleHandler.inInteractionRange(player, target, 4)`: how much
/// past their reach a hold reaches.
const HOLD_REACH: f64 = 4.0;

/// Simulated's `HandleBlock`, an `AbstractDirectionalAxisBlock`.
pub struct HandleBlock;

impl BlockMetadata for HandleBlock {
    fn ids() -> Box<[BlockId]> {
        crate::mods::ported_blocks("simulated", &HANDLES)
    }
}

impl BlockBehaviour for HandleBlock {
    /// `useItemOn`: with a free hand the client starts its hold. A wrench
    /// goes on to the block, and anything else in the main hand to the
    /// item.
    fn use_with_item(&self, args: UseWithItemArgs<'_>) -> BlockActionResult {
        if is_item(args.item_stack, "create:wrench") {
            return BlockActionResult::PassToDefaultBlockAction;
        }
        if can_interact(args.player) {
            BlockActionResult::SuccessServer
        } else {
            BlockActionResult::PassToDefaultBlockAction
        }
    }

    /// `AbstractDirectionalAxisBlock.getStateForPlacement`.
    fn on_place(&self, args: OnPlaceArgs<'_>) -> BlockStateId {
        placed(args.block, args.direction, args.player)
            .map_or(args.block.default_state.id, |state| state.id)
    }

    /// `canSurvive`: the block the handle points away from has a collision
    /// shape.
    fn can_place_at(&self, args: CanPlaceAtArgs<'_>) -> bool {
        supported(
            args.block_accessor,
            args.position,
            facing(args.state.id).unwrap_or(BlockDirection::Up),
        )
    }

    /// `neighborChanged`: a handle whose support is gone is destroyed.
    fn get_state_for_neighbor_update(
        &self,
        args: GetStateForNeighborUpdateArgs<'_>,
    ) -> BlockStateId {
        let points = facing(args.state_id).unwrap_or(BlockDirection::Up);
        if args.direction == points.opposite() && !supported(args.world, args.position, points) {
            Block::AIR.default_state.id
        } else {
            args.state_id
        }
    }

    /// `getAnalogOutputSignal`: full strength while a player holds it.
    fn get_comparator_output(&self, args: GetComparatorOutputArgs<'_>) -> Option<u8> {
        Some(u8::from(args.world.handles_held_at(args.position)))
    }
}

/// Whether `block` is one of Simulated's handles.
#[must_use]
pub fn is_handle(block: &Block) -> bool {
    mods::block_has_tag(block, "simulated:handles")
}

/// The way a handle points, out of the block it is fastened to.
#[must_use]
pub fn facing(state: BlockStateId) -> Option<BlockDirection> {
    let name = mods::state_prop(state, "facing")?;
    BlockDirection::all()
        .into_iter()
        .find(|direction| direction_name(*direction) == name)
}

/// `S:HandleBlock.canInteractWithHandle`: the main hand is empty, or
/// holds Create's extendo grip.
#[must_use]
pub fn can_interact(player: &Player) -> bool {
    let held = player.inventory().held_item();
    held.is_empty() || is_item(&held, "create:extendo_grip")
}

/// `S:HandleBlockEntity.getGrabCenter`: the middle of the block the handle
/// is in, three sixteenths back towards what it is fastened to.
#[must_use]
pub fn grab_centre(handle: BlockPos, state: BlockStateId) -> Vector3<f64> {
    let back = facing(state)
        .unwrap_or(BlockDirection::Up)
        .opposite()
        .to_offset();
    Vector3::new(
        f64::from(handle.0.x) + 0.5 + f64::from(back.x) * (3.0 / 16.0),
        f64::from(handle.0.y) + 0.5 + f64::from(back.y) * (3.0 / 16.0),
        f64::from(handle.0.z) + 0.5 + f64::from(back.z) * (3.0 / 16.0),
    )
}

/// Where `player`, `scroll` blocks off the handle they hold, draws the
/// point of it that lies at `shown` in the world.
///
/// Simulated's `HandleConstraint.physicsTick`: `None` when they are in no
/// state to pull, standing on nothing or out of range.
#[must_use]
pub fn pull_goal(player: &Arc<Player>, scroll: f32, shown: Vector3<f64>) -> Option<Vector3<f64>> {
    let entity = &player.living_entity.entity;
    let feet = BlockPos(entity.pos.load().floor_to_i32());
    let (block, _) = player.world().get_block_and_state(&feet);
    let climbing = block.has_tag(&pumpkin_data::tag::Block::MINECRAFT_CLIMBABLE);
    if !entity.on_ground.load(std::sync::atomic::Ordering::Relaxed)
        && !entity.is_in_water()
        && !player.is_flying()
        && !climbing
    {
        return None;
    }
    let goal = player.eye_position() + player.get_looking_vector() * f64::from(scroll.max(2.0));
    // Their block reach, and two blocks more.
    let valid = player.block_interaction_range() + 2.0;
    let apart = goal - shown;
    (apart.length_squared() <= valid * valid).then_some(goal)
}

/// `S:UpdatePlayerUsingHandlePacket` (`simulated:update_player_handle`):
/// the player holds the handle at `position` from `range` blocks off, or
/// with `remove` lets it go.
pub fn update_holding(player: &Arc<Player>, data: &[u8]) {
    let Some(range) = data
        .get(0..4)
        .and_then(|bytes| bytes.try_into().ok())
        .map(f32::from_be_bytes)
    else {
        return;
    };
    let Some(&remove) = data.get(4) else {
        return;
    };
    let Some(position) = read_block_pos(data, 5) else {
        return;
    };
    let world = player.world();
    let (block, _) = world.get_block_and_state(&position);
    if !is_handle(block) || !world.block_within_reach(player, &position, HOLD_REACH) {
        return;
    }
    if remove != 0 {
        world.handle_let_go(player, &position);
    } else if can_interact(player) {
        world.handle_held(player, &position, scroll_distance(range));
    }
}

/// `S:HandleConstraint.setScrollDistance`: no pull below zero, and at most
/// two and a half blocks off.
const fn scroll_distance(range: f32) -> f32 {
    if range < 0.0 {
        range
    } else {
        range.clamp(1.0, 2.5)
    }
}

/// `S:ClientboundPlayersHoldingHandlePacket`
/// (`simulated:players_holding_handles`): who holds a handle, for the
/// others to see them hang by it.
#[must_use]
pub fn holding(players: &[uuid::Uuid]) -> Vec<u8> {
    let mut data = Vec::with_capacity(2 + 16 * players.len());
    let _ = data.write_var_int(&VarInt(i32::try_from(players.len()).unwrap_or(i32::MAX)));
    for player in players {
        write_uuid(&mut data, *player);
    }
    data
}

/// `AbstractDirectionalAxisBlock.getStateForPlacement`: the handle points
/// out of the clicked face. `axis_along_first` says which way its bar
/// runs, by which way the face or the player faces, and sneaking swaps
/// that.
fn placed(block: &Block, points: BlockDirection, player: &Player) -> Option<&'static BlockState> {
    let along_first = if points.is_horizontal() {
        points.to_axis() == Axis::Z
    } else {
        player
            .get_entity()
            .get_horizontal_facing()
            .to_block_direction()
            .to_axis()
            == Axis::Z
    };
    let along_first = player.get_entity().is_sneaking() != along_first;
    mods::state_with(
        block,
        &[
            ("facing", direction_name(points)),
            (
                "axis_along_first",
                if along_first { "true" } else { "false" },
            ),
        ],
    )
}

/// `canSurvive`: what lies behind the handle has a collision shape.
fn supported(world: &dyn BlockAccessor, handle: &BlockPos, points: BlockDirection) -> bool {
    let behind = handle.offset(points.opposite().to_offset());
    world
        .get_block_state(&behind)
        .get_block_collision_shapes()
        .next()
        .is_some()
}

fn is_item(stack: &pumpkin_data::item_stack::ItemStack, name: &str) -> bool {
    Item::from_registry_key(name).is_some_and(|item| item.id == stack.item.id)
}

const fn direction_name(direction: BlockDirection) -> &'static str {
    match direction {
        BlockDirection::Up => "up",
        BlockDirection::Down => "down",
        BlockDirection::North => "north",
        BlockDirection::South => "south",
        BlockDirection::West => "west",
        BlockDirection::East => "east",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A handle of the test profile with the properties `props`.
    fn handle(props: &[(&str, &str)]) -> (&'static Block, &'static BlockState) {
        crate::mods::profile::tests::install_test_profile();
        let block = Block::from_registry_key("simulated:iron_handle").expect("handle");
        let state = mods::state_with(block, props).expect("state");
        (block, state)
    }

    #[test]
    fn a_handle_points_out_of_its_support_and_is_grabbed_a_little_behind_its_middle() {
        let (block, state) = handle(&[("facing", "west"), ("axis_along_first", "true")]);
        assert!(is_handle(block));
        assert_eq!(facing(state.id), Some(BlockDirection::West));
        let centre = grab_centre(BlockPos(Vector3::new(10, 64, -7)), state.id);
        // Three sixteenths back towards the east.
        assert_eq!(centre.x, 10.5 + 3.0 / 16.0);
        assert_eq!(centre.y, 64.5);
        assert_eq!(centre.z, -6.5);
        assert_eq!(
            facing(
                handle(&[("facing", "down"), ("axis_along_first", "false")])
                    .1
                    .id
            ),
            Some(BlockDirection::Down)
        );
        assert_eq!(facing(block.default_state.id), Some(BlockDirection::North));
        // Whatever order the properties come in.
        assert_eq!(
            handle(&[("facing", "west"), ("axis_along_first", "true")])
                .1
                .id,
            mods::state_with(
                Block::from_registry_key("simulated:iron_handle").expect("handle"),
                &[("axis_along_first", "true"), ("facing", "west")]
            )
            .expect("state")
            .id
        );
    }

    #[test]
    fn a_pull_is_asked_for_from_two_blocks_off_at_least_and_capped_close_in() {
        assert_eq!(scroll_distance(-1.0), -1.0);
        assert_eq!(scroll_distance(0.5), 1.0);
        assert_eq!(scroll_distance(2.0), 2.0);
        assert_eq!(scroll_distance(3.0), 2.5);
    }

    #[test]
    fn holding_packet_lists_the_players() {
        let first = uuid::Uuid::from_u128(0x0123_4567_89AB_CDEF_FEDC_BA98_7654_3210);
        let second = uuid::Uuid::from_u128(17);
        let data = holding(&[first, second]);
        // A count, then each uuid most significant bits first.
        let mut expected = vec![2];
        expected.extend_from_slice(first.as_bytes());
        assert_eq!(&data[..17], expected.as_slice());
        assert_eq!(&data[17..], second.as_bytes());
        assert_eq!(holding(&[]), [0]);
    }
}
