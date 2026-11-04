use pumpkin_data::block_properties::Facing;
use pumpkin_data::{Block, BlockState, FacingExt};
use pumpkin_macros::pumpkin_block;
use pumpkin_world::world::BlockFlags;

use pumpkin_data::BlockDirection;
use pumpkin_data::block_properties::PistonType;

use crate::block::blocks::piston::piston::try_move;
use crate::block::{
    BlockBehaviour, BlockStateId, BrokenArgs, GetStateForNeighborUpdateArgs, OnNeighborUpdateArgs,
    PathComputationType,
};

use super::piston::PistonProps;

pub(crate) type PistonHeadProperties = pumpkin_data::block_properties::PistonHeadLikeProperties;

#[pumpkin_block("minecraft:piston_head")]
pub struct PistonHeadBlock;

impl PistonHeadBlock {
    /// Vanilla `PistonHeadBlock.canSurvive`: the block behind the head is a
    /// fitting extended piston (matching type and facing) or a matching
    /// moving piston.
    fn can_survive(
        world: &crate::world::World,
        position: &pumpkin_util::math::position::BlockPos,
        state_id: BlockStateId,
    ) -> bool {
        let props = PistonHeadProperties::from_state_id(state_id);
        let base_pos = position.offset(props.facing.opposite().to_block_direction().to_offset());
        let (base_block, base_state) = world.get_block_and_state(&base_pos);
        let expected_base = match props.r#type {
            PistonType::Normal => &Block::PISTON,
            PistonType::Sticky => &Block::STICKY_PISTON,
        };
        if base_block == expected_base {
            let base_props = PistonProps::from_state_id(base_state.id);
            return base_props.extended && base_props.facing == props.facing;
        }
        if base_block == &Block::MOVING_PISTON {
            let moving_props =
                super::piston_extension::MovingPistonProps::from_state_id(base_state.id);
            return moving_props.facing == props.facing;
        }
        false
    }
}

impl BlockBehaviour for PistonHeadBlock {
    fn broken(&self, args: BrokenArgs<'_>) {
        let props = PistonHeadProperties::from_state_id(args.state.id);
        let pos = args
            .position
            .offset(props.facing.opposite().to_block_direction().to_offset());
        let (new_block, new_state) = args.world.get_block_and_state_id(&pos);
        if &Block::PISTON == new_block || &Block::STICKY_PISTON == new_block {
            let props = PistonProps::from_state_id(new_state);
            if props.extended {
                // TODO: use player
                args.world
                    .break_block(&pos, None, BlockFlags::NOTIFY_ALL | BlockFlags::SKIP_DROPS);
            }
        }
    }
    fn on_neighbor_update(&self, args: OnNeighborUpdateArgs<'_>) {
        let head_state_id = args.world.get_block_state_id(args.position);
        let head_props = PistonHeadProperties::from_state_id(head_state_id);
        if head_props.facing != Facing::Up {
            return;
        }
        let piston_pos = args.position.offset(
            head_props
                .facing
                .opposite()
                .to_block_direction()
                .to_offset(),
        );
        let piston_block = args.world.get_block(&piston_pos);
        if &Block::PISTON == piston_block || &Block::STICKY_PISTON == piston_block {
            let up_pos = args
                .position
                .offset(head_props.facing.to_block_direction().to_offset());
            let upper_block = args.world.get_block(&up_pos);
            if upper_block != &Block::REDSTONE_BLOCK {
                //Then somebody probably broke the redstone block, try to check if piston should still be extended.
                try_move(args.world, piston_block, &piston_pos);
            }
        }
    }

    /// Vanilla `PistonHeadBlock.updateShape`: when the neighbour toward the
    /// base changes and the head can no longer survive, it turns to air --
    /// this is what removes the rod when the piston base is broken by any
    /// means (dig, command, explosion).
    fn get_state_for_neighbor_update(
        &self,
        args: GetStateForNeighborUpdateArgs<'_>,
    ) -> BlockStateId {
        let props = PistonHeadProperties::from_state_id(args.state_id);
        let toward_base = props.facing.opposite().to_block_direction();
        if args.direction == toward_base
            && !Self::can_survive(args.world, args.position, args.state_id)
        {
            return Block::AIR.default_state.id;
        }
        args.state_id
    }

    fn is_pathfindable(&self, _state: &BlockState, _computation_type: PathComputationType) -> bool {
        false
    }
}
