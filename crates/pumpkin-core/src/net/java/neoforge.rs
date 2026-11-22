//! `NeoForge` network negotiation.
//!
//! A `NeoForge` client that carries any mod with mandatory network channels
//! refuses servers that look vanilla. During the configuration phase the server
//! announces its channels, asks the client for its own (`neoforge:register`) and
//! answers with the agreed set (`neoforge:network`). Agreeing to everything the
//! client announced lets such clients join; Pumpkin then only ever sends on
//! channels it knows, like Sable's sub-level protocol.
//!
//! Byte layouts mirror `NeoForge` 21.1's `ModdedNetworkQueryPayload` and
//! `ModdedNetworkPayload`.

use pumpkin_protocol::codec::var_int::VarInt;
use pumpkin_protocol::java::client::config::CPluginMessage;
use pumpkin_protocol::ser::{NetworkReadSliceExt, NetworkWriteExt, ReadingError, WritingError};

pub const REGISTER_CHANNEL: &str = "minecraft:register";
pub const UNREGISTER_CHANNEL: &str = "minecraft:unregister";
const QUERY_CHANNEL: &str = "neoforge:register";
const SETUP_CHANNEL: &str = "neoforge:network";

/// `ConnectionProtocol` ordinals on the wire.
const PROTOCOL_PLAY: i32 = 1;

/// Bound on the channel list a client may announce; a real modpack stays far
/// below this.
const MAX_CHANNELS: usize = 4096;
/// Vanilla's string bound for resource locations.
const MAX_IDENTIFIER_LEN: usize = 32767;

/// The channels a `NeoForge` client announced, as (protocol ordinal, channel,
/// version).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct NeoforgeChannels {
    pub channels: Vec<(i32, String, String)>,
}

impl NeoforgeChannels {
    /// Parses the client's answer to the network query.
    pub fn parse_query(data: &[u8]) -> Result<Self, ReadingError> {
        let mut buf: &[u8] = data;
        let protocols = read_var_int(&mut buf)?;
        if !(0..=8).contains(&protocols) {
            return Err(ReadingError::Message(format!(
                "client announced {protocols} protocols"
            )));
        }

        let mut channels = Vec::new();
        for _ in 0..protocols {
            let protocol = read_var_int(&mut buf)?;
            let count = read_var_int(&mut buf)?;
            if count < 0 || channels.len() + count as usize > MAX_CHANNELS {
                return Err(ReadingError::Message(format!(
                    "client announced {count} channels for protocol {protocol}"
                )));
            }
            for _ in 0..count {
                let id = buf.get_str_bounded_borrowed(MAX_IDENTIFIER_LEN)?;
                let version = buf.get_str_bounded_borrowed(MAX_IDENTIFIER_LEN)?;
                let has_flow = buf.read_slice_borrowed(1)?[0] != 0;
                if has_flow {
                    read_var_int(&mut buf)?;
                }
                // Whether the client's registration is optional; the server
                // agrees to everything either way.
                buf.read_slice_borrowed(1)?;
                channels.push((protocol, id.to_string(), version.to_string()));
            }
        }
        Ok(Self { channels })
    }

    /// Encodes `ModdedNetworkPayload`: every announced channel, agreed.
    pub fn encode_setup(&self) -> Result<Vec<u8>, WritingError> {
        let mut protocols: Vec<i32> = self
            .channels
            .iter()
            .map(|(protocol, _, _)| *protocol)
            .collect();
        protocols.sort_unstable();
        protocols.dedup();

        let mut out = Vec::new();
        out.write_var_int(&VarInt(i32::try_from(protocols.len()).unwrap_or(i32::MAX)))?;
        for protocol in protocols {
            out.write_var_int(&VarInt(protocol))?;
            let members: Vec<_> = self
                .channels
                .iter()
                .filter(|(entry, _, _)| *entry == protocol)
                .collect();
            out.write_var_int(&VarInt(i32::try_from(members.len()).unwrap_or(i32::MAX)))?;
            for (_, id, version) in members {
                out.write_string(id)?;
                out.write_string(version)?;
            }
        }
        Ok(out)
    }

    /// Whether the client listens on a PLAY channel, e.g. Sable's snapshot
    /// channel.
    #[must_use]
    pub fn has_play_channel(&self, channel: &str) -> bool {
        self.channels
            .iter()
            .any(|(protocol, id, _)| *protocol == PROTOCOL_PLAY && id == channel)
    }
}

/// Encodes a vanilla `minecraft:register` / `minecraft:unregister` payload.
#[must_use]
pub fn encode_channel_list(identifiers: &[&str]) -> Vec<u8> {
    let mut out = Vec::new();
    let _ = out.write_var_int(&VarInt(
        i32::try_from(identifiers.len()).unwrap_or(i32::MAX),
    ));
    for identifier in identifiers {
        // Identifiers are static ASCII; writing one cannot fail.
        let _ = out.write_string(identifier);
    }
    out
}

/// The custom payload that asks the client for its channels: an empty
/// `ModdedNetworkQueryPayload`.
#[must_use]
pub const fn query_payload() -> CPluginMessage<'static> {
    CPluginMessage::new(QUERY_CHANNEL, &[0])
}

/// The custom payload that agrees to the client's channels.
#[must_use]
pub const fn setup_payload(data: &[u8]) -> CPluginMessage<'_> {
    CPluginMessage::new(SETUP_CHANNEL, data)
}

/// Whether a serverbound configuration payload is the client's channel list.
#[must_use]
pub fn is_query_answer(channel: &str) -> bool {
    channel == QUERY_CHANNEL
}

fn read_var_int(buf: &mut &[u8]) -> Result<i32, ReadingError> {
    Ok(VarInt::decode(buf)?.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use pumpkin_protocol::ser::NetworkWriteExt;

    /// A client with one mandatory PLAY channel (`sable:snapshot_packet` v1,
    /// clientbound flow = 1) and one optional bidirectional CONFIGURATION
    /// channel, written the way `NeoForge`'s codecs do.
    fn sample_query() -> Vec<u8> {
        let mut out = Vec::new();
        out.write_var_int(&VarInt(2)).unwrap();
        // PLAY
        out.write_var_int(&VarInt(1)).unwrap();
        out.write_var_int(&VarInt(1)).unwrap();
        out.write_string("sable:snapshot_packet").unwrap();
        out.write_string("1").unwrap();
        out.write_bool(true).unwrap();
        out.write_var_int(&VarInt(1)).unwrap();
        out.write_bool(false).unwrap();
        // CONFIGURATION
        out.write_var_int(&VarInt(4)).unwrap();
        out.write_var_int(&VarInt(1)).unwrap();
        out.write_string("example:config").unwrap();
        out.write_string("2").unwrap();
        out.write_bool(false).unwrap();
        out.write_bool(true).unwrap();
        out
    }

    #[test]
    fn parses_the_client_channel_list() {
        let channels = NeoforgeChannels::parse_query(&sample_query()).unwrap();
        assert_eq!(channels.channels.len(), 2);
        assert!(channels.has_play_channel("sable:snapshot_packet"));
        assert!(!channels.has_play_channel("sable:start_tracking_sub_level"));
        assert_eq!(
            channels.channels[0],
            (1, "sable:snapshot_packet".into(), "1".into())
        );
    }

    #[test]
    fn setup_echoes_every_channel_grouped_by_protocol() {
        let channels = NeoforgeChannels::parse_query(&sample_query()).unwrap();
        let setup = channels.encode_setup().unwrap();

        let mut buf: &[u8] = &setup;
        assert_eq!(read_var_int(&mut buf).unwrap(), 2);
        assert_eq!(read_var_int(&mut buf).unwrap(), 1);
        assert_eq!(read_var_int(&mut buf).unwrap(), 1);
        assert_eq!(
            buf.get_str_bounded_borrowed(32767).unwrap(),
            "sable:snapshot_packet"
        );
        assert_eq!(buf.get_str_bounded_borrowed(32767).unwrap(), "1");
        assert_eq!(read_var_int(&mut buf).unwrap(), 4);
        assert_eq!(read_var_int(&mut buf).unwrap(), 1);
        assert_eq!(
            buf.get_str_bounded_borrowed(32767).unwrap(),
            "example:config"
        );
        assert_eq!(buf.get_str_bounded_borrowed(32767).unwrap(), "2");
        assert!(buf.is_empty());
    }

    #[test]
    fn rejects_runaway_channel_counts() {
        let mut hostile = Vec::new();
        hostile.write_var_int(&VarInt(1)).unwrap();
        hostile.write_var_int(&VarInt(1)).unwrap();
        hostile.write_var_int(&VarInt(i32::MAX)).unwrap();
        assert!(NeoforgeChannels::parse_query(&hostile).is_err());
    }

    #[test]
    fn channel_lists_are_varint_prefixed_identifiers() {
        let list = encode_channel_list(&["minecraft:register", "neoforge:register"]);
        let mut buf: &[u8] = &list;
        assert_eq!(read_var_int(&mut buf).unwrap(), 2);
        assert_eq!(
            buf.get_str_bounded_borrowed(32767).unwrap(),
            "minecraft:register"
        );
        assert_eq!(
            buf.get_str_bounded_borrowed(32767).unwrap(),
            "neoforge:register"
        );
        assert!(buf.is_empty());
    }
}
