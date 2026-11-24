//! The primitives the mods' packets are made of, as their stream codecs
//! write them.
//!
//! A block position as a packed long, a uuid, a vector as three doubles, a
//! quaternion as four floats, a resource location, a box as six doubles.
//! All big-endian.

use pumpkin_protocol::codec::var_int::VarInt;
use pumpkin_protocol::ser::{NetworkWriteExt, WritingError};
use pumpkin_util::math::position::BlockPos;
use pumpkin_util::math::vector3::Vector3;
use uuid::Uuid;

/// `BlockPos.STREAM_CODEC`: `BlockPos.asLong`.
pub(crate) fn write_block_pos(buf: &mut Vec<u8>, pos: BlockPos) -> Result<(), WritingError> {
    buf.write_i64(pos.as_long())
}

/// A block position read at `at`.
pub(crate) fn read_block_pos(data: &[u8], at: usize) -> Option<BlockPos> {
    Some(BlockPos::from_i64(i64::from_be_bytes(
        data.get(at..at + 8)?.try_into().ok()?,
    )))
}

/// `UUIDUtil.STREAM_CODEC`: the most significant bits first.
pub(crate) fn write_uuid(buf: &mut Vec<u8>, id: Uuid) {
    buf.extend_from_slice(id.as_bytes());
}

/// A uuid read at `at`.
pub(crate) fn read_uuid(data: &[u8], at: usize) -> Option<Uuid> {
    Some(Uuid::from_bytes(data.get(at..at + 16)?.try_into().ok()?))
}

/// Sable's `SableBufferUtils.write(ByteBuf, Vector3dc)`.
pub(crate) fn write_vector(buf: &mut Vec<u8>, vector: Vector3<f64>) -> Result<(), WritingError> {
    buf.write_f64(vector.x)?;
    buf.write_f64(vector.y)?;
    buf.write_f64(vector.z)
}

/// A vector read at `at`, with where it ends.
pub(crate) fn read_vector(data: &[u8], at: usize) -> Option<(Vector3<f64>, usize)> {
    let mut next = at;
    let mut values = [0.0; 3];
    for value in &mut values {
        *value = f64::from_be_bytes(data.get(next..next + 8)?.try_into().ok()?);
        next += 8;
    }
    Some((Vector3::new(values[0], values[1], values[2]), next))
}

/// Sable's `SableBufferUtils.write(ByteBuf, Quaterniondc)`: `(x, y, z, w)`.
pub(crate) fn write_quaternion(
    buf: &mut Vec<u8>,
    orientation: [f32; 4],
) -> Result<(), WritingError> {
    for value in orientation {
        buf.write_f32(value)?;
    }
    Ok(())
}

/// A quaternion `(x, y, z, w)` read at `at`.
pub(crate) fn read_quaternion(data: &[u8], at: usize) -> Option<[f32; 4]> {
    let mut values = [0.0; 4];
    for (index, value) in values.iter_mut().enumerate() {
        let start = at + index * 4;
        *value = f32::from_be_bytes(data.get(start..start + 4)?.try_into().ok()?);
    }
    Some(values)
}

/// `ResourceKey.streamCodec`: the key's location as a resource location,
/// a length-prefixed string.
pub(crate) fn write_location(buf: &mut Vec<u8>, location: &str) -> Result<(), WritingError> {
    buf.write_string(location)
}

/// `ByteBufCodecs.list`: a count, then the elements.
pub(crate) fn write_count(buf: &mut Vec<u8>, count: usize) -> Result<(), WritingError> {
    buf.write_var_int(&VarInt(i32::try_from(count).unwrap_or(i32::MAX)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn primitives_round_trip_in_the_codecs_order() -> Result<(), &'static str> {
        let mut buf = Vec::new();
        let position = BlockPos(Vector3::new(-3, 70, 1_000_003));
        let id = Uuid::from_u128(0x0123_4567_89AB_CDEF_FEDC_BA98_7654_3210);
        write_block_pos(&mut buf, position).map_err(|_| "pos")?;
        write_uuid(&mut buf, id);
        write_vector(&mut buf, Vector3::new(1.5, -2.25, 3.0)).map_err(|_| "vector")?;
        write_quaternion(&mut buf, [0.0, 0.5, 0.25, 0.75]).map_err(|_| "quaternion")?;
        write_location(&mut buf, "minecraft:overworld").map_err(|_| "location")?;
        write_count(&mut buf, 300).map_err(|_| "count")?;

        // BlockPos.asLong packs x over 26 bits, z over 26 and y over 12.
        assert_eq!(
            &buf[..8],
            &(((i64::from(-3) & 0x03FF_FFFF) << 38) | (1_000_003i64 << 12) | 0x46).to_be_bytes()
        );
        assert_eq!(read_block_pos(&buf, 0), Some(position));
        assert_eq!(read_uuid(&buf, 8), Some(id));
        let (vector, next) = read_vector(&buf, 24).ok_or("vector")?;
        assert_eq!(vector, Vector3::new(1.5, -2.25, 3.0));
        assert_eq!(next, 48);
        assert_eq!(read_quaternion(&buf, 48), Some([0.0, 0.5, 0.25, 0.75]));
        // A varint string, then a two-byte varint.
        assert_eq!(&buf[64..66], &[19, b'm']);
        assert_eq!(buf[64 + 1 + 19..], [0xAC, 0x02]);
        Ok(())
    }
}
