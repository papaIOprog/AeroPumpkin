//! Simulated's creative physics staff, the server's half.
//!
//! The client aims and says where a sub-level should go; the server keeps
//! one drag per player and pulls the sub-level there, and holds the locked
//! ones where they stand. `S:` is
//! `dev.simulated_team.simulated.content.physics_staff`.

use std::any::Any;
use std::sync::Arc;

use pumpkin_data::item::Item;
use pumpkin_protocol::codec::var_int::VarInt;
use pumpkin_util::math::vector3::Vector3;
use uuid::Uuid;

use crate::entity::player::Player;
use crate::item::{ItemBehaviour, ItemMetadata};
use crate::mods::packets::{
    read_quaternion, read_uuid, read_vector, write_count, write_location, write_uuid, write_vector,
};

/// `S:PhysicsStaffItem.RANGE`: how far the staff reaches.
pub const RANGE: f64 = 128.0;

/// Simulated's `PhysicsStaffAction`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// The player let go: `STOP_DRAG`.
    StopDrag,
    /// The player struck a sub-level: `LOCK`.
    Lock,
    /// `START_DRAG`, which the client keeps to itself.
    StartDrag,
}

impl Action {
    /// `CatnipStreamCodecBuilders.ofEnum`: the ordinal, and where it ends.
    fn read(data: &[u8]) -> Option<(Self, usize)> {
        let mut buf: &[u8] = data;
        let ordinal = VarInt::decode(&mut buf).ok()?;
        let read = data.len() - buf.len();
        let action = match ordinal.0 {
            0 => Self::StopDrag,
            1 => Self::Lock,
            2 => Self::StartDrag,
            _ => return None,
        };
        Some((action, read))
    }
}

/// What the staff's drag packet says: `S:PhysicsStaffDragPacket`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StaffDrag {
    /// The sub-level dragged.
    pub sub_level: Uuid,
    /// The goal, counted from the player's eyes.
    pub goal: Vector3<f64>,
    /// The point of the sub-level held, in the plot's coordinates.
    pub anchor: Vector3<f64>,
    /// The orientation the sub-level is turned to, `(x, y, z, w)`.
    pub orientation: [f32; 4],
}

/// Simulated's `PhysicsStaffItem`.
pub struct PhysicsStaffItem;

impl ItemMetadata for PhysicsStaffItem {
    fn ids() -> Box<[u16]> {
        crate::mods::ported_items("simulated", &["simulated:creative_physics_staff"])
    }
}

impl ItemBehaviour for PhysicsStaffItem {
    /// `canAttackBlock` is false: the staff in hand cannot mine.
    fn can_mine(&self, _player: &Player) -> bool {
        false
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// `S:PhysicsStaffItem.isHolding`: whether the player holds the staff in
/// either hand.
#[must_use]
pub fn holding(player: &Player) -> bool {
    let Some(staff) = Item::from_registry_key("simulated:creative_physics_staff") else {
        return false;
    };
    let inventory = player.inventory();
    inventory.held_item().item == staff || inventory.off_hand_item().item == staff
}

/// Reads a `simulated:physics_staff_drag` payload, sent every client tick
/// while a drag lasts.
#[must_use]
pub fn read_drag(data: &[u8]) -> Option<StaffDrag> {
    let sub_level = read_uuid(data, 0)?;
    let (goal, at) = read_vector(data, 16)?;
    let (anchor, at) = read_vector(data, at)?;
    Some(StaffDrag {
        sub_level,
        goal,
        anchor,
        orientation: read_quaternion(data, at)?,
    })
}

/// Reads a `simulated:physics_staff_action` payload: what the player did,
/// to which sub-level, and where they struck.
#[must_use]
pub fn read_action(data: &[u8]) -> Option<(Action, Uuid, Vector3<f64>)> {
    let (action, at) = Action::read(data)?;
    let sub_level = read_uuid(data, at)?;
    let (location, _) = read_vector(data, at + 16)?;
    Some((action, sub_level, location))
}

/// `S:PhysicsStaffDragPacket.handle`: the player drags, as `drag` says. A
/// player not holding the staff, or a goal no staff could name, is ignored.
pub fn drag(player: &Arc<Player>, drag: StaffDrag) {
    let reachable = drag.goal.length() <= RANGE;
    if !holding(player) || !reachable || !all_finite(drag.goal) || !all_finite(drag.anchor) {
        return;
    }
    let StaffDrag {
        sub_level,
        goal,
        anchor,
        orientation,
    } = drag;
    player
        .world()
        .staff_drag(player, sub_level, goal, anchor, orientation);
}

/// `S:PhysicsStaffLocksPacket` (`simulated:physics_staff_locks`): the
/// locked sub-levels of a dimension.
#[must_use]
pub fn encode_locks(dimension: &str, locked: &[Uuid]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(32 + 16 * locked.len());
    let _ = write_location(&mut buf, dimension);
    let _ = write_count(&mut buf, locked.len());
    for id in locked {
        write_uuid(&mut buf, *id);
    }
    buf
}

/// `S:PhysicsStaffDragSessionsPacket` (`simulated:physics_staff_drag_sessions`):
/// who drags which point, in a dimension.
#[must_use]
pub fn encode_sessions(dimension: &str, sessions: &[(Uuid, Vector3<f64>)]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(48 + 40 * sessions.len());
    let _ = write_location(&mut buf, dimension);
    let _ = write_count(&mut buf, sessions.len());
    for (player, anchor) in sessions {
        write_uuid(&mut buf, *player);
        let _ = write_vector(&mut buf, *anchor);
    }
    buf
}

/// `S:PhysicsStaffBeamPacket` (`simulated:physics_staff_beam`): the beam
/// the player's staff casts from `start` to `end`.
#[must_use]
pub fn encode_beam(player: Uuid, start: Vector3<f64>, end: Vector3<f64>) -> Vec<u8> {
    let mut buf = Vec::with_capacity(64);
    write_uuid(&mut buf, player);
    let _ = write_vector(&mut buf, start);
    let _ = write_vector(&mut buf, end);
    buf
}

const fn all_finite(vector: Vector3<f64>) -> bool {
    vector.x.is_finite() && vector.y.is_finite() && vector.z.is_finite()
}

#[cfg(test)]
mod tests {
    use super::*;
    use pumpkin_protocol::ser::NetworkWriteExt;

    const ID: Uuid = Uuid::from_u128(0x0123_4567_89AB_CDEF_FEDC_BA98_7654_3210);

    #[test]
    fn the_staffs_packets_read_as_their_codecs_write_them() {
        // VarInt ordinal 1 (LOCK), the sub-level, then the strike.
        let mut action = vec![1];
        write_uuid(&mut action, ID);
        let _ = write_vector(&mut action, Vector3::new(1.0, 65.5, -2.0));
        let (what, sub_level, location) = read_action(&action).ok_or("action").unwrap();
        assert_eq!((what, sub_level), (Action::Lock, ID));
        assert_eq!(location, Vector3::new(1.0, 65.5, -2.0));
        assert_eq!(read_action(&[7, 0, 0]), None);

        let mut drag = Vec::new();
        write_uuid(&mut drag, ID);
        let _ = write_vector(&mut drag, Vector3::new(3.0, 4.0, 0.0));
        let _ = write_vector(&mut drag, Vector3::new(20_481_033.5, 128.5, 20_481_032.5));
        for value in [0.0, 0.707_106_77, 0.0, 0.707_106_77] {
            drag.write_f32(value).unwrap();
        }
        assert_eq!(drag.len(), 16 + 24 + 24 + 16);
        let drag = read_drag(&drag).ok_or("drag").unwrap();
        assert_eq!(drag.sub_level, ID);
        assert_eq!(drag.goal, Vector3::new(3.0, 4.0, 0.0));
        assert_eq!(drag.anchor.x, 20_481_033.5);
        assert_eq!(drag.orientation, [0.0, 0.707_106_77, 0.0, 0.707_106_77]);
    }

    #[test]
    fn locks_and_sessions_start_with_the_dimension() {
        // The location as a varint string, then the list's count.
        let locks = encode_locks("minecraft:overworld", &[ID]);
        assert_eq!(&locks[..2], &[19, b'm']);
        assert_eq!(locks[20], 1);
        assert_eq!(&locks[21..37], &ID.as_bytes()[..]);
        assert_eq!(
            encode_locks("minecraft:the_nether", &[]).len(),
            "minecraft:the_nether".len() + 2
        );
        let sessions = encode_sessions("minecraft:overworld", &[(ID, Vector3::new(1.0, 2.0, 3.0))]);
        assert_eq!(sessions.len(), 20 + 1 + 16 + 24);
        // A beam: the player, then two vectors.
        let zero = Vector3::new(0.0, 0.0, 0.0);
        assert_eq!(encode_beam(ID, zero, zero).len(), 16 + 48);
    }
}
