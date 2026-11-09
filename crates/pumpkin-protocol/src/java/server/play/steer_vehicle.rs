use pumpkin_data::packet::serverbound::play::STEER_VEHICLE;
use pumpkin_macros::java_packet;

use crate::{
    ServerPacket,
    ser::{NetworkReadExt, ReadingError},
};
use pumpkin_util::version::JavaMinecraftVersion;

/// Player Input (Steer Vehicle in the registry), sent by the client every
/// tick while riding; the sneak flag is how it asks to dismount.
#[java_packet(STEER_VEHICLE)]
pub struct SSteerVehicle {
    pub sideways: f32,
    pub forward: f32,
    pub jump: bool,
    pub sneak: bool,
}

impl<'a> ServerPacket<'a> for SSteerVehicle {
    fn read(bytebuf: &mut &'a [u8], _version: &JavaMinecraftVersion) -> Result<Self, ReadingError> {
        let sideways = bytebuf.get_f32_be()?;
        let forward = bytebuf.get_f32_be()?;
        let flags = bytebuf.get_u8()?;
        Ok(Self {
            sideways,
            forward,
            jump: flags & 0b1 != 0,
            sneak: flags & 0b10 != 0,
        })
    }
}

impl crate::ClientPacket for SSteerVehicle {
    fn write_packet_data(
        &self,
        mut write: impl std::io::Write,
        _version: &JavaMinecraftVersion,
    ) -> Result<(), crate::ser::WritingError> {
        use crate::ser::NetworkWriteExt;
        write.write_f32_be(self.sideways)?;
        write.write_f32_be(self.forward)?;
        let mut flags = 0u8;
        if self.jump {
            flags |= 0b1;
        }
        if self.sneak {
            flags |= 0b10;
        }
        write.write_u8(flags)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use pumpkin_util::version::JavaMinecraftVersion;

    use super::SSteerVehicle;
    use crate::{ClientPacket, ServerPacket};

    #[test]
    fn steer_vehicle_round_trips_with_flag_byte() -> Result<(), Box<dyn std::error::Error>> {
        let packet = SSteerVehicle {
            sideways: 0.5,
            forward: -0.98,
            jump: false,
            sneak: true,
        };
        let mut encoded = Vec::new();
        packet.write_packet_data(&mut encoded, &JavaMinecraftVersion::V_1_21)?;

        // Vanilla writes two big-endian floats and one flag byte (0x2 = sneak).
        assert_eq!(
            encoded,
            [0x3F, 0x00, 0x00, 0x00, 0xBF, 0x7A, 0xE1, 0x48, 0x02]
        );

        let mut bytes = encoded.as_slice();
        let decoded = SSteerVehicle::read(&mut bytes, &JavaMinecraftVersion::V_1_21)?;
        assert_eq!(
            (decoded.sideways, decoded.forward),
            (packet.sideways, packet.forward)
        );
        assert_eq!((decoded.jump, decoded.sneak), (false, true));
        Ok(())
    }
}
