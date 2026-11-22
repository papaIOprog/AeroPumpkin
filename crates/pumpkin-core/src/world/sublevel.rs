//! Server-side glue for native sub-levels: ticking, rider carrying and the
//! `pumpkin:sublevel` custom-payload protocol consumed by the Pumpkin
//! Sublevels client mod.
//!
//! Sub-levels simulate on the world tick thread (see `pumpkin-sublevel` for
//! why that can never contend with the generation pool). Riders are carried
//! before the entity tracker runs, so clients receive the post-carry
//! positions in the same tick.

use bytes::BufMut;
use pumpkin_protocol::java::client::play::CCustomPayload;
use pumpkin_sublevel::{SubLevelSnapshot, SubLevelSystem};
use pumpkin_util::math::vector3::Vector3;

use super::World;
use crate::net::java::JavaClient;

/// Custom payload channel for sub-level sync. Vanilla clients ignore it;
/// the Pumpkin Sublevels client mod renders from it.
pub const CHANNEL: &str = "pumpkin:sublevel";

/// Widths are bounded by the command argument range; `i16` keeps the
/// integer-to-float conversion exact for clippy.
const DEFAULT_WIDTH_I16: i16 = 16;

const MSG_CREATE: u8 = 1;
const MSG_TRANSFORM: u8 = 2;
const MSG_REMOVE: u8 = 3;

fn encode_create(snapshot: &SubLevelSnapshot) -> Vec<u8> {
    let mut buf = Vec::with_capacity(57);
    buf.put_u8(MSG_CREATE);
    buf.put_u32_le(snapshot.id);
    put_vec3(&mut buf, &snapshot.position);
    put_quat(&mut buf, &snapshot.rotation);
    put_vec3(&mut buf, &snapshot.half_extents);
    put_vec3(&mut buf, &snapshot.linear_velocity);
    buf
}

fn encode_transform(snapshot: &SubLevelSnapshot) -> Vec<u8> {
    let mut buf = Vec::with_capacity(45);
    buf.put_u8(MSG_TRANSFORM);
    buf.put_u32_le(snapshot.id);
    put_vec3(&mut buf, &snapshot.position);
    put_quat(&mut buf, &snapshot.rotation);
    put_vec3(&mut buf, &snapshot.linear_velocity);
    buf
}

fn encode_remove(id: u32) -> Vec<u8> {
    let mut buf = Vec::with_capacity(5);
    buf.put_u8(MSG_REMOVE);
    buf.put_u32_le(id);
    buf
}

fn put_vec3(buf: &mut Vec<u8>, values: &[f32; 3]) {
    buf.put_f32_le(values[0]);
    buf.put_f32_le(values[1]);
    buf.put_f32_le(values[2]);
}

fn put_quat(buf: &mut Vec<u8>, values: &[f32; 4]) {
    buf.put_f32_le(values[0]);
    buf.put_f32_le(values[1]);
    buf.put_f32_le(values[2]);
    buf.put_f32_le(values[3]);
}

impl World {
    /// Spawns a hovering test platform centred on `center` and mounts every
    /// non-player entity standing within its bounds.
    ///
    /// Returns the new sub-level id and the number of riders mounted.
    pub fn spawn_sublevel_platform(&self, center: Vector3<f64>, width: i32) -> (u32, usize) {
        let half = f32::from(i16::try_from(width).unwrap_or(DEFAULT_WIDTH_I16)) / 2.0;
        let center_f32 = center.to_f32_lossy();
        let position = [center_f32.x, center_f32.y, center_f32.z];

        let mut system = self.lock_sublevels();
        let id = system.spawn_platform(position, [half, 0.5, half]);
        drop(system);

        let mounted = self.mount_entities_on_sublevel(id, half);
        (id, mounted)
    }

    /// Mounts every non-player entity within `half + 1` blocks of the
    /// sub-level's horizontal bounds, so armor stands and dropped items on
    /// the platform ride it. Players are skipped: their movement is
    /// client-authoritative, so carrying them needs the client mod.
    ///
    /// Returns how many entities were newly mounted.
    pub fn mount_entities_on_sublevel(&self, id: u32, half: f32) -> usize {
        let (center, half_extents) = {
            let system = self.lock_sublevels();
            let Some(snapshot) = system.snapshots().into_iter().find(|s| s.id == id) else {
                return 0;
            };
            (snapshot.position, snapshot.half_extents)
        };

        let reach_y = half_extents[1] + 2.0;
        let mut candidates = Vec::new();
        for entity in self.entities.load().iter() {
            if entity.get_player().is_some() {
                continue;
            }
            let pos = entity.get_entity().pos.load();
            let (dx, dy, dz) = (
                pos.x - f64::from(center[0]),
                pos.y - f64::from(center[1]),
                pos.z - f64::from(center[2]),
            );
            if dx.abs() > f64::from(half + 1.0)
                || dz.abs() > f64::from(half + 1.0)
                || dy.abs() > f64::from(reach_y)
            {
                continue;
            }
            let anchor = pos.to_f32_lossy();
            candidates.push((
                entity.get_entity().entity_id,
                [anchor.x, anchor.y, anchor.z],
            ));
        }

        let mut system = self.lock_sublevels();
        candidates
            .iter()
            .filter(|(rider, anchor)| system.mount(id, *rider, *anchor))
            .count()
    }

    /// Steps the simulation, carries riders to their new positions and
    /// broadcasts the sync payloads. Runs from the world tick, before the
    /// entity tracker so clients see carried positions without lag.
    pub fn tick_sublevels(&self) {
        let mut system = self.lock_sublevels();
        let snapshots = system.step();

        for snapshot in &snapshots {
            carry_riders(self, &mut system, snapshot);
        }

        let messages: Vec<Vec<u8>> = if system.should_refresh_clients() {
            snapshots.iter().map(encode_create).collect()
        } else if system.should_send_transforms() {
            snapshots.iter().map(encode_transform).collect()
        } else {
            Vec::new()
        };
        drop(system);

        if messages.is_empty() {
            return;
        }
        broadcast_sublevel_payloads(self, &messages);
    }

    /// Removes a sub-level, notifying clients.
    pub fn remove_sublevel(&self, id: u32) -> bool {
        let removed = self.lock_sublevels().remove(id);
        if removed {
            broadcast_sublevel_payloads(self, &[encode_remove(id)]);
        }
        removed
    }

    /// Unlocks the sub-level state. Every helper goes through this so a
    /// poisoned lock takes the same path as the rest of the code base.
    fn lock_sublevels(&self) -> std::sync::MutexGuard<'_, SubLevelSystem> {
        self.sublevels
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

fn carry_riders(world: &World, system: &mut SubLevelSystem, snapshot: &SubLevelSnapshot) {
    let entities = world.entities.load();
    for (rider, world_pos) in snapshot.riders.iter().take(snapshot.rider_count) {
        let Some(entity) = entities
            .iter()
            .find(|entity| entity.get_entity().entity_id == *rider)
        else {
            // The entity is gone; stop dragging a stale anchor around.
            system.unmount(*rider);
            continue;
        };
        entity.get_entity().set_pos(Vector3::new(
            f64::from(world_pos[0]),
            f64::from(world_pos[1]),
            f64::from(world_pos[2]),
        ));
    }
}

fn broadcast_sublevel_payloads(world: &World, messages: &[Vec<u8>]) {
    let players = world.players.load();
    for player in players.iter() {
        let Some(java) = player.client.java() else {
            continue;
        };
        let version = player.client.java_version();
        for data in messages {
            let packet = CCustomPayload::new(CHANNEL, data.as_slice());
            if let Ok(bytes) = JavaClient::serialize_packet_for_version(&packet, version) {
                java.try_enqueue_packet(bytes);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_snapshot() -> SubLevelSnapshot {
        SubLevelSnapshot {
            id: 7,
            position: [1.0, 64.0, -2.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            linear_velocity: [0.5, 0.0, 0.0],
            half_extents: [8.0, 0.5, 8.0],
            riders: [(0, [0.0; 3]); 8],
            rider_count: 0,
        }
    }

    fn read_vec3(buf: &mut &[u8]) -> [f32; 3] {
        let mut values = [0.0f32; 3];
        for value in &mut values {
            *value = read_f32(buf);
        }
        values
    }

    fn read_f32(buf: &mut &[u8]) -> f32 {
        let value = f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        *buf = &buf[4..];
        value
    }

    #[test]
    fn create_message_layout_matches_the_client_decoder() {
        let bytes = encode_create(&sample_snapshot());
        assert_eq!(bytes.len(), 57);
        let mut buf: &[u8] = &bytes;
        assert_eq!(buf[0], MSG_CREATE);
        buf = &buf[1..];
        let id = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        buf = &buf[4..];
        assert_eq!(id, 7);
        assert_eq!(read_vec3(&mut buf), [1.0, 64.0, -2.0]);
        assert_eq!(read_vec3(&mut buf), [0.0, 0.0, 0.0]);
        assert_eq!(read_f32(&mut buf), 1.0);
        assert_eq!(read_vec3(&mut buf), [8.0, 0.5, 8.0]);
        assert_eq!(read_vec3(&mut buf), [0.5, 0.0, 0.0]);
        assert!(buf.is_empty());
    }

    #[test]
    fn transform_and_remove_layouts() {
        let transform = encode_transform(&sample_snapshot());
        assert_eq!(transform.len(), 45);
        assert_eq!(transform[0], MSG_TRANSFORM);

        let remove = encode_remove(9);
        assert_eq!(remove, [MSG_REMOVE, 9, 0, 0, 0]);
    }
}
