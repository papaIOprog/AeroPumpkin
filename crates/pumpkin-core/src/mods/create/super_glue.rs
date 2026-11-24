//! Create's super glue: a box of blocks an assembly takes along whole.
//!
//! The client picks the two corners and shows the group; the server checks
//! the group again and puts the box in the world. `C:` is
//! `com.simibubi.create.content.contraptions.glue`.

use std::any::Any;
use std::sync::Arc;

use pumpkin_data::BlockDirection;
use pumpkin_data::data_component_impl::EquipmentSlot;
use pumpkin_data::item::Item;
use pumpkin_data::sound::SoundCategory;
use pumpkin_util::GameMode;
use pumpkin_util::math::position::BlockPos;
use rustc_hash::FxHashSet;

use crate::entity::EntityBase;
use crate::entity::player::Player;
use crate::item::{ItemBehaviour, ItemMetadata};
use crate::mods::glue::{GlueEntity, GlueKind, covers, play_named_sound, with_glue_by_id};
use crate::mods::packets::read_block_pos;
use crate::world::World;
use crate::world::sublevel::attachment::{attached_towards, movement_necessary, not_supportive};

/// `C:SuperGlueSelectionPacket`: how far past their reach the player may
/// glue, and how far apart the two corners may be.
const REACH: f64 = 2.0;
const MAX_DISTANCE: f64 = 25.0;
/// `C:SuperGlueRemovalPacket`: how far the player may be from the glue.
const REMOVAL_RANGE: f64 = 32.0;

/// Create's `SuperGlueItem`. Nothing happens on the server when it is used:
/// the client sends [`GLUE_IN_AREA`] once it picked both corners.
pub struct SuperGlueItem;

impl ItemMetadata for SuperGlueItem {
    fn ids() -> Box<[u16]> {
        crate::mods::ported_items("create", &["create:super_glue"])
    }
}

impl ItemBehaviour for SuperGlueItem {
    /// `canAttackBlock` is false: the glue in hand cannot mine.
    fn can_mine(&self, _player: &Player) -> bool {
        false
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// `C:SuperGlueSelectionPacket` (`create:glue_in_area`): the player glued
/// the blocks from `from` towards `to`.
pub fn glue_in_area(player: &Arc<Player>, data: &[u8]) {
    let (Some(from), Some(to)) = (read_block_pos(data, 0), read_block_pos(data, 8)) else {
        return;
    };
    let world = player.world();
    let apart = f64::from(from.squared_distance(&to));
    if !world.block_within_reach(player, &to, REACH) || apart >= MAX_DISTANCE * MAX_DISTANCE {
        return;
    }
    let Some(group) = search_glue_group(&world, from, to) else {
        return;
    };
    if !group.contains(&to) || !take_glue(player, 1) {
        return;
    }
    if let Some(glue) = GlueEntity::spawn(&world, GlueKind::Super, span(from, to)) {
        glue.spawn_particles(&world);
    }
}

/// `C:SuperGlueRemovalPacket` (`create:glue_removed`): the player broke the
/// glue entity `id`, hearing it from `sound_source`.
pub fn glue_removed(player: &Arc<Player>, data: &[u8]) {
    let Some(id) = data
        .get(..4)
        .and_then(|bytes| Some(i32::from_be_bytes(bytes.try_into().ok()?)))
    else {
        return;
    };
    let Some(sound_source) = read_block_pos(data, 4) else {
        return;
    };
    let world = player.world();
    let player_position = player.get_entity().pos.load();
    with_glue_by_id(&world, id, |glue| {
        let glue_position = glue.position();
        if glue.kind() != GlueKind::Super
            || glue_position.squared_distance_to_vec(&player_position)
                > REMOVAL_RANGE * REMOVAL_RANGE
        {
            return;
        }
        play_named_sound(
            &world,
            "create:slime_added",
            SoundCategory::Blocks,
            &sound_source.to_centered_f64(),
            0.5,
            0.5,
            None,
        );
        glue.spawn_particles(&world);
        glue.remove();
    });
}

/// `C:SuperGlueEntity.span`: the box over the blocks from `start` to `end`.
pub(crate) fn span(start: BlockPos, end: BlockPos) -> [f64; 6] {
    [
        f64::from(start.0.x.min(end.0.x)),
        f64::from(start.0.y.min(end.0.y)),
        f64::from(start.0.z.min(end.0.z)),
        f64::from(start.0.x.max(end.0.x) + 1),
        f64::from(start.0.y.max(end.0.y) + 1),
        f64::from(start.0.z.max(end.0.z) + 1),
    ]
}

/// `C:SuperGlueSelectionHelper.searchGlueGroup`: the blocks a glue from
/// `start` towards `end` would hold, if the group reaches them both. Glue
/// spreads through the blocks inside the box that a glue can sit on, and
/// through sticky ones touching them.
fn search_glue_group(world: &World, start: BlockPos, end: BlockPos) -> Option<FxHashSet<BlockPos>> {
    let bounds = span(start, end);
    let mut frontier = vec![start];
    let mut visited = FxHashSet::from_iter([start]);
    let mut attached = FxHashSet::default();
    while let Some(current) = frontier.pop() {
        attached.insert(current);
        let (block, state) = world.get_block_and_state(&current);
        for direction in BlockDirection::all() {
            let next = current.offset(direction.to_offset());
            let (next_block, next_state) = world.get_block_and_state(&next);
            let towards = direction.opposite();
            let joins = is_sticky(next_block) || covers(bounds, next);
            if !joins
                || !movement_necessary(next_block, next_state)
                || !is_valid_face(block, state, direction)
                || !is_valid_face(next_block, next_state, towards)
                || !visited.insert(next)
            {
                continue;
            }
            frontier.push(next);
        }
    }
    // A single block is nothing to glue.
    (!(attached.len() < 2 && attached.contains(&end))).then_some(attached)
}

/// Whether glue can sit on the `side` of a block: `C:SuperGlueEntity.isValidFace`.
fn is_valid_face(
    block: &pumpkin_data::Block,
    state: &pumpkin_data::BlockState,
    side: BlockDirection,
) -> bool {
    attached_towards(block, state, side)
        || (movement_necessary(block, state) && !not_supportive(block, state, side))
}

/// `C:SuperGlueEntity.isSideSticky` for the blocks Pumpkin runs: slime and
/// honey hold whatever is glued against them.
fn is_sticky(block: &pumpkin_data::Block) -> bool {
    *block == pumpkin_data::Block::SLIME_BLOCK || *block == pumpkin_data::Block::HONEY_BLOCK
}

/// `C:SuperGlueSelectionHelper.collectGlueFromInventory`: takes `required`
/// charges of glue from the player's stacks, the held one first; whether the
/// player had them. A creative player needs none. A glue item holds one
/// charge per point of durability it has left.
fn take_glue(player: &Player, mut required: i32) -> bool {
    if player.gamemode.load() == GameMode::Creative {
        return true;
    }
    let Some(glue) = Item::from_registry_key("create:super_glue") else {
        return false;
    };
    let inventory = player.inventory();
    let mut slots: Vec<usize> = Vec::with_capacity(37);
    slots.push(usize::from(inventory.get_selected_slot()));
    slots.extend(0..36);
    for slot in slots {
        let stack = inventory.get_slot(slot);
        if stack.is_empty() || stack.item != glue {
            continue;
        }
        let Some(charges) = stack.get_max_damage().map(|max| max - stack.get_damage()) else {
            continue;
        };
        if charges <= 0 {
            continue;
        }
        player.damage_item_in_inventory_slot(slot, charges, &EquipmentSlot::MAIN_HAND);
        required -= charges;
        if required <= 0 {
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use pumpkin_util::math::vector3::Vector3;

    #[test]
    fn the_span_covers_both_blocks_whichever_corner_came_last() {
        assert_eq!(
            span(
                BlockPos(Vector3::new(3, 4, 5)),
                BlockPos(Vector3::new(1, 2, 3))
            ),
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        );
        assert_eq!(
            span(
                BlockPos(Vector3::new(1, 2, 3)),
                BlockPos(Vector3::new(1, 2, 3))
            ),
            [1.0, 2.0, 3.0, 2.0, 3.0, 4.0]
        );
    }
}
