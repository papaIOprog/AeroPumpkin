#[allow(clippy::wildcard_imports)]
use super::*;

impl JavaClient {
    pub fn handle_steer_vehicle(
        &self,
        player: &Arc<Player>,
        packet: &SSteerVehicle,
        server: &Arc<Server>,
    ) {
        if !player.has_client_loaded() {
            return;
        }
        player.update_last_action_time();

        // Vanilla `handlePlayerInput` only applies the flags to a riding
        // player. While riding, sneak arrives here and never through the
        // player command packet, so the toggle event is fired from this path.
        let entity = player.get_entity();
        if !entity.has_vehicle() {
            return;
        }

        if entity.is_sneaking() != packet.sneak {
            send_cancellable_blocking! {{
                server;
                PlayerToggleSneakEvent::new(player.clone(), packet.sneak);
                'after: {
                    entity.set_sneaking(packet.sneak);
                }
            }}
        }
    }
}
