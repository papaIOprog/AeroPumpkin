use std::sync::Arc;

use pumpkin_macros::pumpkin_block;

use crate::block::entities::end_gateway::EndGatewayBlockEntity;
use crate::block::{BlockBehaviour, OnEntityCollisionArgs, OnSyncedBlockEventArgs, PlacedArgs};

#[pumpkin_block("minecraft:end_gateway")]
pub struct EndGatewayBlock;

impl BlockBehaviour for EndGatewayBlock {
    fn on_entity_collision(&self, args: OnEntityCollisionArgs<'_>) {
        let entity = args.entity.get_entity();
        if entity
            .portal_cooldown
            .load(std::sync::atomic::Ordering::Relaxed)
            > 0
        {
            return;
        }

        let Some(block_entity) = args.world.get_block_entity(args.position) else {
            return;
        };
        let Some(end_gateway) = block_entity
            .as_any()
            .downcast_ref::<EndGatewayBlockEntity>()
        else {
            return;
        };

        if end_gateway.is_cooling_down() {
            return;
        }

        end_gateway.trigger_cooldown(args.world, *args.position);

        // The exit can be a thousand blocks out in ungenerated chunks; the
        // teleport runs once those exist (vanilla loads them synchronously).
        let entity_id = entity.entity_id;
        let yaw = entity.yaw.load();
        let pitch = entity.pitch.load();
        let world = args.world.clone();
        let entry = *args.position;
        tokio::spawn(async move {
            let Some(entity) = world.get_entity_by_id(entity_id) else {
                return;
            };
            let entity_base = entity.get_entity();
            entity_base.yaw.store(yaw);
            entity_base.pitch.store(pitch);
            EndGatewayBlockEntity::teleport_when_ready(&world, &entity, entry).await;
        });
    }

    fn on_synced_block_event(&self, args: OnSyncedBlockEventArgs<'_>) -> bool {
        if let Some(block_entity) = args.world.get_block_entity(args.position)
            && let Some(end_gateway) = block_entity
                .as_any()
                .downcast_ref::<EndGatewayBlockEntity>()
        {
            end_gateway.trigger_event(args.r#type, args.data)
        } else {
            args.r#type == EndGatewayBlockEntity::EVENT_COOLDOWN
        }
    }

    fn placed(&self, args: PlacedArgs<'_>) {
        let block_entity = EndGatewayBlockEntity::new(*args.position);
        args.world.add_block_entity(Arc::new(block_entity));
    }
}
