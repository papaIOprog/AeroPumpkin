//! Simulated's physics assembler: a lever that freezes a contraption.
//!
//! The contraption is the one its sticky side touches; once the assembler
//! rides in a sub-level, the lever melts that back into the world instead.
//! The client pulls the lever and says so; the server does the rest. `S:`
//! is `dev.simulated_team.simulated.content.blocks.physics_assembler`.

use std::sync::Arc;

use pumpkin_data::HorizontalFacingExt;
use pumpkin_data::block_properties::{AttachFace, HorizontalFacing};
use pumpkin_data::{Block, BlockDirection, BlockId, BlockState, BlockStateId, mods};

use crate::block::blocks::abstract_wall_mounting::WallMountedBlock;
use crate::block::registry::BlockActionResult;
use crate::block::{
    BlockBehaviour, BlockMetadata, CanPlaceAtArgs, GetStateForNeighborUpdateArgs, NormalUseArgs,
    OnPlaceArgs,
};
use crate::entity::player::Player;
use crate::mods::packets::read_block_pos;

/// The block, as the clients name it.
const BLOCK: &str = "simulated:physics_assembler";
/// `S:AssemblePacket`: how far past their reach the player may pull the lever.
const REACH: f64 = 4.0;

/// Simulated's `PhysicsAssemblerBlock`, a `FaceAttachedHorizontalDirectionalBlock`.
pub struct PhysicsAssemblerBlock;

impl BlockMetadata for PhysicsAssemblerBlock {
    fn ids() -> Box<[BlockId]> {
        crate::mods::ported_blocks("simulated", &[BLOCK])
    }
}

impl BlockBehaviour for PhysicsAssemblerBlock {
    /// `useWithoutItem`: the client runs the lever and sends
    /// [`super::ASSEMBLE`] once it is down.
    fn normal_use(&self, _args: NormalUseArgs<'_>) -> BlockActionResult {
        BlockActionResult::Consume
    }

    /// `FaceAttachedHorizontalDirectionalBlock.getStateForPlacement`.
    fn on_place(&self, args: OnPlaceArgs<'_>) -> BlockStateId {
        let (face, facing) = self.get_placement_face(args.player, args.direction);
        placed(args.block, face, facing).map_or(args.block.default_state.id, |state| state.id)
    }

    fn can_place_at(&self, args: CanPlaceAtArgs<'_>) -> bool {
        let direction = args
            .direction
            .unwrap_or_else(|| self.get_direction(args.state.id, args.block));
        WallMountedBlock::can_place_at(self, args.block_accessor, args.position, direction)
    }

    fn get_state_for_neighbor_update(
        &self,
        args: GetStateForNeighborUpdateArgs<'_>,
    ) -> BlockStateId {
        WallMountedBlock::get_state_for_neighbor_update(self, args)
    }
}

impl WallMountedBlock for PhysicsAssemblerBlock {
    /// `getConnectedDirection`: where the block it hangs on lies.
    fn get_direction(&self, state_id: BlockStateId, _block: &Block) -> BlockDirection {
        match mods::state_prop(state_id, "face") {
            Some("ceiling") => BlockDirection::Down,
            Some("wall") => {
                facing(state_id).map_or(BlockDirection::North, |facing| facing.to_block_direction())
            }
            _ => BlockDirection::Up,
        }
    }
}

/// Simulated's `getStickyFacing`: the side whose block's contraption the
/// assembler takes.
#[must_use]
pub fn sticky_facing(state_id: BlockStateId) -> BlockDirection {
    match mods::state_prop(state_id, "face") {
        Some("floor") => BlockDirection::Down,
        Some("ceiling") => BlockDirection::Up,
        _ => facing(state_id).map_or(BlockDirection::North, |facing| {
            facing.opposite().to_block_direction()
        }),
    }
}

/// The state of the block hung on `face`, looking `facing`.
fn placed(
    block: &Block,
    face: AttachFace,
    facing: HorizontalFacing,
) -> Option<&'static BlockState> {
    let face = match face {
        AttachFace::Floor => "floor",
        AttachFace::Wall => "wall",
        AttachFace::Ceiling => "ceiling",
    };
    mods::state_with(block, &[("face", face), ("facing", facing_name(facing))])
}

/// The direction the block looks, from its `facing` property.
fn facing(state_id: BlockStateId) -> Option<HorizontalFacing> {
    match mods::state_prop(state_id, "facing") {
        Some("north") => Some(HorizontalFacing::North),
        Some("south") => Some(HorizontalFacing::South),
        Some("west") => Some(HorizontalFacing::West),
        Some("east") => Some(HorizontalFacing::East),
        _ => None,
    }
}

const fn facing_name(facing: HorizontalFacing) -> &'static str {
    match facing {
        HorizontalFacing::North => "north",
        HorizontalFacing::South => "south",
        HorizontalFacing::West => "west",
        HorizontalFacing::East => "east",
    }
}

/// `S:AssemblePacket` (`simulated:assemble`): the player pulled the lever
/// of the assembler at a block all the way.
pub fn assemble(player: &Arc<Player>, data: &[u8]) {
    let Some(position) = read_block_pos(data, 0) else {
        return;
    };
    let world = player.world();
    if !world.block_within_reach(player, &position, REACH) {
        return;
    }
    let (block, _) = world.get_block_and_state(&position);
    if block.name == BLOCK {
        world.assembler_used(&position);
    }
}
