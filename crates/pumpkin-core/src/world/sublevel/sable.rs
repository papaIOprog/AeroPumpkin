//! Encoders for Sable's sub-level protocol (clientbound, PLAY phase).
//!
//! Layouts follow Sable 2.0.6's wire format (see the protocol notes in
//! `D:\Projects\aeronautics\ARCHITECTURE-NOTES.md`), re-implemented from
//! scratch. Every payload rides a plain custom payload on the channel named
//! by its constant.

use pumpkin_protocol::codec::var_int::VarInt;
use pumpkin_protocol::ser::{NetworkWriteExt, WritingError};
use uuid::Uuid;

pub const START_TRACKING: &str = "sable:start_tracking_sub_level";
pub const FINALIZE: &str = "sable:finalize_sub_level";
pub const STOP_TRACKING: &str = "sable:stop_tracking_sub_level";
pub const STOP_MOVING: &str = "sable:stop_moving_sub_level";
pub const SNAPSHOT: &str = "sable:snapshot_packet";
pub const SNAPSHOT_INFO: &str = "sable:snapshot_info_packet";

/// Sable clients register this channel, so its presence identifies them
/// during `NeoForge` network negotiation.
pub const DETECT_CHANNEL: &str = SNAPSHOT;

/// Sable networks at most this many moving sub-levels per snapshot.
pub const SNAPSHOT_BATCH: usize = 16;

/// Position and angle change below which Sable does not network a move.
const POSITION_TOLERANCE: f64 = 0.015 / 16.0;
const ANGLE_TOLERANCE: f64 = 0.015f64.to_radians();

/// A sub-level pose: where the plot's origin lands in the world, and how the
/// plot is turned around its rotation point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub position: [f64; 3],
    /// Quaternion `(x, y, z, w)`.
    pub orientation: [f32; 4],
    pub rotation_point: [f64; 3],
}

impl Default for Pose {
    fn default() -> Self {
        Self {
            position: [0.0; 3],
            orientation: [0.0, 0.0, 0.0, 1.0],
            rotation_point: [0.0; 3],
        }
    }
}

impl Pose {
    fn write(&self, buf: &mut Vec<u8>) -> Result<(), WritingError> {
        for value in self.position {
            buf.write_f64(value)?;
        }
        for value in self.orientation {
            buf.write_f32(value)?;
        }
        for value in self.rotation_point {
            buf.write_f64(value)?;
        }
        Ok(())
    }

    /// Whether this pose differs from `other` by more than Sable's network
    /// tolerance: 0.015 / 16 blocks or 0.015 degrees.
    #[must_use]
    pub fn moved_beyond(&self, other: &Self) -> bool {
        let distance_squared: f64 = self
            .position
            .iter()
            .zip(other.position.iter())
            .map(|(a, b)| (a - b) * (a - b))
            .sum();
        distance_squared > POSITION_TOLERANCE * POSITION_TOLERANCE
            || quaternion_angle(self.orientation, other.orientation) > ANGLE_TOLERANCE
    }
}

/// Angle between two unit quaternions, in radians.
fn quaternion_angle(a: [f32; 4], b: [f32; 4]) -> f64 {
    let dot: f32 = a.iter().zip(b.iter()).map(|(x, y)| x * y).sum();
    2.0 * f64::from(dot.abs().clamp(0.0, 1.0)).acos()
}

/// One moving sub-level in a snapshot.
#[derive(Debug, Clone, Copy)]
pub struct SnapshotEntry {
    pub plot: i64,
    pub pose: Pose,
    pub linear_velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
}

/// `ChunkPos#toLong`, how Sable packs a plot index.
#[must_use]
pub fn chunk_pos_to_long(x: i32, z: i32) -> i64 {
    (i64::from(x) & 0xFFFF_FFFF) | (i64::from(z) << 32)
}

/// `sable:start_tracking_sub_level`. Sub-levels are unnamed for now, so no
/// name field is written.
pub fn encode_start_tracking(
    plot: i64,
    id: Uuid,
    last_pose: &Pose,
    pose: &Pose,
    bounds: [i32; 6],
    game_tick: i32,
) -> Result<Vec<u8>, WritingError> {
    let mut buf = Vec::with_capacity(133);
    buf.write_i64(plot)?;
    let bits = id.as_u128();
    buf.extend_from_slice(&u64::try_from(bits >> 64).unwrap_or(0).to_be_bytes());
    buf.extend_from_slice(
        &u64::try_from(bits & 0xFFFF_FFFF_FFFF_FFFF)
            .unwrap_or(0)
            .to_be_bytes(),
    );
    last_pose.write(&mut buf)?;
    pose.write(&mut buf)?;
    for value in bounds {
        buf.write_i32(value)?;
    }
    buf.write_bool(false)?;
    buf.write_i32(game_tick)?;
    Ok(buf)
}

/// `sable:finalize_sub_level`, `sable:stop_tracking_sub_level` and
/// `sable:stop_moving_sub_level` all carry just the plot.
pub fn encode_plot_ref(plot: i64) -> Result<Vec<u8>, WritingError> {
    let mut buf = Vec::with_capacity(8);
    buf.write_i64(plot)?;
    Ok(buf)
}

/// `sable:snapshot_info_packet`.
pub fn encode_snapshot_info(
    ms_since_last: i32,
    game_tick: i32,
    stopped: bool,
) -> Result<Vec<u8>, WritingError> {
    let mut buf = Vec::with_capacity(9);
    buf.write_i32(ms_since_last)?;
    buf.write_i32(game_tick)?;
    buf.write_bool(stopped)?;
    Ok(buf)
}

/// `sable:snapshot_packet` with up to [`SNAPSHOT_BATCH`] entries.
pub fn encode_snapshot(game_tick: i32, entries: &[SnapshotEntry]) -> Result<Vec<u8>, WritingError> {
    debug_assert!(entries.len() <= SNAPSHOT_BATCH);
    let mut buf = Vec::with_capacity(5 + entries.len() * 73);
    buf.write_i32(game_tick)?;
    buf.write_var_int(&VarInt(entries.len() as i32))?;
    for entry in entries {
        buf.write_i64(entry.plot)?;
        entry.pose.write(&mut buf)?;
        for value in entry.linear_velocity {
            buf.write_f32(value)?;
        }
        for value in entry.angular_velocity {
            buf.write_f32(value)?;
        }
    }
    Ok(buf)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plot_refs_are_a_big_endian_long() {
        assert_eq!(encode_plot_ref(7).unwrap(), [0, 0, 0, 0, 0, 0, 0, 7]);
        assert_eq!(chunk_pos_to_long(3, 5), 3 | (5 << 32));
        assert_eq!(chunk_pos_to_long(-1, 0), 0xFFFF_FFFF);
    }

    #[test]
    fn snapshot_info_layout() {
        assert_eq!(
            encode_snapshot_info(50, 3, false).unwrap(),
            [0, 0, 0, 50, 0, 0, 0, 3, 0]
        );
        assert_eq!(encode_snapshot_info(50, 3, true).unwrap()[8], 1);
    }

    #[test]
    fn snapshot_layout() {
        let entry = SnapshotEntry {
            plot: chunk_pos_to_long(1, 2),
            pose: Pose::default(),
            linear_velocity: [1.0, 0.0, 0.0],
            angular_velocity: [0.0; 3],
        };
        let bytes = encode_snapshot(9, &[entry]).unwrap();
        // tick + count + plot + pose (24 + 16 + 24) + two velocities
        assert_eq!(bytes.len(), 4 + 1 + 8 + 64 + 12 + 12);
        assert_eq!(&bytes[..5], [0, 0, 0, 9, 1]);
        assert_eq!(&bytes[5..13], &[0, 0, 0, 2, 0, 0, 0, 1]);
        // Pose position (0, 0, 0) then the identity quaternion.
        assert!(bytes[13..37].iter().all(|byte| *byte == 0));
        let identity: Vec<u8> = [0.0f32, 0.0, 0.0, 1.0]
            .iter()
            .flat_map(|value| value.to_be_bytes())
            .collect();
        assert_eq!(&bytes[37..53], &identity[..]);
    }

    #[test]
    fn start_tracking_layout() {
        let id = Uuid::from_u128(0x0123_4567_89AB_CDEF_FEDC_BA98_7654_3210);
        let bytes = encode_start_tracking(
            chunk_pos_to_long(0, 0),
            id,
            &Pose::default(),
            &Pose::default(),
            [-8, -1, -8, 7, -1, 7],
            4,
        )
        .unwrap();
        assert_eq!(bytes.len(), 8 + 16 + 64 + 64 + 24 + 1 + 4);
        assert_eq!(
            &bytes[8..24],
            &[
                0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54,
                0x32, 0x10
            ]
        );
        assert_eq!(
            &bytes[152..176],
            &[-8i32, -1, -8, 7, -1, 7]
                .iter()
                .flat_map(|value| value.to_be_bytes())
                .collect::<Vec<u8>>()
        );
        assert_eq!(bytes[176], 0);
        assert_eq!(&bytes[177..181], &[0, 0, 0, 4]);
    }

    #[test]
    fn moves_below_tolerance_are_not_networked() {
        let base = Pose::default();
        let tiny = Pose {
            position: [0.0005, 0.0, 0.0],
            ..base
        };
        let visible = Pose {
            position: [0.01, 0.0, 0.0],
            ..base
        };
        let turned = Pose {
            orientation: [
                0.0,
                (1.0f64.to_radians() / 2.0) as f32,
                0.0,
                (1.0f64.to_radians() / 2.0).cos() as f32,
            ],
            ..base
        };
        assert!(!tiny.moved_beyond(&base));
        assert!(visible.moved_beyond(&base));
        assert!(turned.moved_beyond(&base));
    }
}
