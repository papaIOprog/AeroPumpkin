//! Simulated's honey glue: Create's super glue that reaches further and
//! needs no path, and whose faces can be dragged once it stands. `S:` is
//! `dev.simulated_team.simulated.content.entities.honey_glue`.

use std::any::Any;
use std::sync::Arc;

use pumpkin_data::data_component_impl::EquipmentSlot;
use pumpkin_data::item::Item;
use pumpkin_data::sound::{Sound, SoundCategory};
use pumpkin_protocol::java::client::play::CCustomPayload;
use pumpkin_protocol::ser::NetworkWriteExt;
use pumpkin_util::Hand;
use pumpkin_util::math::position::BlockPos;

use crate::entity::player::Player;
use crate::item::{ItemBehaviour, ItemMetadata};
use crate::mods::glue::{GlueEntity, GlueKind, play_named_sound, with_glue_by_uuid};
use crate::mods::packets::{read_block_pos, read_uuid, write_uuid};
use crate::mods::simulated::HONEY_GLUE_SYNC;
use crate::world::World;

/// `S:SimAssembly.honeyGlueRange`: how far a honey glue reaches on each axis.
const HONEY_GLUE_RANGE: f64 = 48.0;

/// Simulated's `HoneyGlueItem`. The client picks both blocks itself and
/// sends [`super::HONEY_GLUE_SPAWN`].
pub struct HoneyGlueItem;

impl ItemMetadata for HoneyGlueItem {
    fn ids() -> Box<[u16]> {
        crate::mods::ported_items("simulated", &["simulated:honey_glue"])
    }
}

impl ItemBehaviour for HoneyGlueItem {
    /// `canAttackBlock` is false: the glue in hand cannot mine.
    fn can_mine(&self, _player: &Player) -> bool {
        false
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// `S:HoneyGlueSpawnPacket` (`simulated:honey_glue_spawn`): the player
/// glued from `from` to `to`.
pub fn spawn(player: &Arc<Player>, data: &[u8]) {
    let (Some(from), Some(to)) = (read_block_pos(data, 0), read_block_pos(data, 8)) else {
        return;
    };
    let Some(hand) = glue_hand(player) else {
        return;
    };
    let bounds = encapsulating_full_blocks(from, to);
    if !check_bounds(bounds) {
        return;
    }
    let world = player.world();
    let at = to.to_f64();
    play_named_sound(
        &world,
        "simulated:item.honey_glue.use",
        SoundCategory::Players,
        &at,
        0.5,
        0.95,
        Some(player.gameprofile.id),
    );
    world.play_sound_raw_expect(
        player,
        Sound::EntityItemFrameAddItem as u16,
        SoundCategory::Blocks,
        &at,
        0.75,
        1.0,
    );
    player.damage_item_in_slot(
        match hand {
            Hand::Right => &EquipmentSlot::MAIN_HAND,
            Hand::Left => &EquipmentSlot::OFF_HAND,
        },
        1,
    );
    if let Some(glue) = GlueEntity::spawn(&world, GlueKind::Honey, bounds) {
        glue.spawn_particles(&world);
    }
}

/// `S:HoneyGlueChangeBoundsPacket` (`simulated:honey_glue_change_bounds`):
/// the player dragged a face of the glue `uuid` to `bounds`. A box too
/// small or too large takes the glue away instead.
pub fn change_bounds(player: &Arc<Player>, data: &[u8]) {
    let Some(bounds) = read_box(data) else {
        return;
    };
    let Some(uuid) = read_uuid(data, 48) else {
        return;
    };
    let world = player.world();
    with_glue_by_uuid(&world, uuid, |glue| {
        if glue.kind() != GlueKind::Honey {
            return;
        }
        if check_bounds(bounds) {
            glue.set_bounds(bounds);
            sync_bounds(&world, glue, bounds, player.gameprofile.id);
        } else {
            play_named_sound(
                &world,
                "simulated:item.honey_glue.use",
                SoundCategory::Players,
                &glue.centre(),
                0.5,
                0.5,
                None,
            );
            glue.spawn_particles(&world);
            glue.remove();
        }
    });
}

/// `S:HoneyGlueSyncBoundsPacket` (`simulated:honey_glue_sync`): where the
/// glue entity `id` is now, for the players tracking it. The player `who`
/// moved it applied the change themselves and skips the packet.
fn sync_bounds(world: &World, glue: &GlueEntity, bounds: [f64; 6], who: uuid::Uuid) {
    let mut buf = Vec::with_capacity(53 + 16);
    for value in bounds {
        if buf.write_f64(value).is_err() {
            return;
        }
    }
    if buf.write_i32(glue.id()).is_err() || buf.write_bool(true).is_err() {
        return;
    }
    write_uuid(&mut buf, who);
    let packet = CCustomPayload::new(HONEY_GLUE_SYNC, &buf);
    if let Some(tracked) = world.entity_tracker.get_tracked_entity(glue.id()) {
        tracked.send_to_tracking_players(&packet, world);
    }
}

/// The six doubles of a box at the start of `data`.
fn read_box(data: &[u8]) -> Option<[f64; 6]> {
    let mut bounds = [0.0; 6];
    for (index, value) in bounds.iter_mut().enumerate() {
        let at = index * 8;
        *value = f64::from_be_bytes(data.get(at..at + 8)?.try_into().ok()?);
    }
    Some(bounds)
}

/// `AABB.encapsulatingFullBlocks`: the box over the blocks from `from` to `to`.
fn encapsulating_full_blocks(from: BlockPos, to: BlockPos) -> [f64; 6] {
    [
        f64::from(from.0.x.min(to.0.x)),
        f64::from(from.0.y.min(to.0.y)),
        f64::from(from.0.z.min(to.0.z)),
        f64::from(from.0.x.max(to.0.x) + 1),
        f64::from(from.0.y.max(to.0.y) + 1),
        f64::from(from.0.z.max(to.0.z) + 1),
    ]
}

/// `S:HoneyGlueMaxSizing.checkBounds`: each side at least one block and at
/// most the range long.
fn check_bounds(bounds: [f64; 6]) -> bool {
    (0..3).all(|axis| {
        let size = bounds[axis + 3] - bounds[axis];
        (1.0..=HONEY_GLUE_RANGE).contains(&size)
    })
}

/// `S:HoneyGlueSpawnPacket.getHoneyGlueHand`: the hand holding the honey
/// glue, the main hand first.
fn glue_hand(player: &Player) -> Option<Hand> {
    let honey = Item::from_registry_key("simulated:honey_glue")?;
    let inventory = player.inventory();
    [
        (Hand::Right, inventory.held_item()),
        (Hand::Left, inventory.off_hand_item()),
    ]
    .into_iter()
    .find(|(_, stack)| stack.item == honey)
    .map(|(hand, _)| hand)
}

#[cfg(test)]
mod tests {
    use super::*;
    use pumpkin_util::math::vector3::Vector3;

    #[test]
    fn a_honey_glue_needs_a_full_block_and_stays_within_its_range() {
        assert!(check_bounds([0.0, 0.0, 0.0, 1.0, 1.0, 1.0]));
        assert!(check_bounds([0.0, 0.0, 0.0, 48.0, 48.0, 48.0]));
        assert!(!check_bounds([0.0, 0.0, 0.0, 0.9, 1.0, 1.0]));
        assert!(!check_bounds([0.0, 0.0, 0.0, 49.0, 1.0, 1.0]));
        // The box a pull leaves behind.
        assert!(!check_bounds([0.0; 6]));
    }

    #[test]
    fn the_change_bounds_packet_carries_the_box_then_the_glue() {
        let id = uuid::Uuid::from_u128(0x0123_4567_89AB_CDEF_FEDC_BA98_7654_3210);
        let mut data = Vec::new();
        for value in [1.5f64, 2.5, 3.5, 4.5, 5.5, 6.5] {
            data.extend_from_slice(&value.to_be_bytes());
        }
        write_uuid(&mut data, id);
        assert_eq!(read_box(&data), Some([1.5, 2.5, 3.5, 4.5, 5.5, 6.5]));
        assert_eq!(read_uuid(&data, 48), Some(id));
        assert_eq!(
            encapsulating_full_blocks(
                BlockPos(Vector3::new(4, 1, 9)),
                BlockPos(Vector3::new(2, 3, 7))
            ),
            [2.0, 1.0, 7.0, 5.0, 4.0, 10.0]
        );
    }
}
