//! Which blocks a physics contraption takes along: Simulated's
//! `SimAssemblyContraption`, which is Create's `Contraption.moveBlock` over
//! Create's `BlockMovementChecks`, with Aeronautics' envelope rule on top.
//!
//! Touching is not enough. A block joins through glue, through slime or
//! honey, by being fastened to its support, or, as Aeronautics adds, by
//! being an envelope next to another.

use pumpkin_data::block_properties::{
    AttachFace, BedPart, BellAttachment, BellLikeProperties, ChestLikeProperties, ChestType,
    DoubleBlockHalf, GrindstoneLikeProperties, HorizontalFacing, LadderLikeProperties,
    LanternLikeProperties, LeverLikeProperties, MangrovePropaguleLikeProperties,
    OakDoorLikeProperties, WallTorchLikeProperties, WhiteBedLikeProperties,
};
use pumpkin_data::block_state::PistonBehavior;
use pumpkin_data::mods;
use pumpkin_data::tag::{self, Taggable};
use pumpkin_data::{Block, BlockDirection, BlockId, BlockState, BlockStateId};
use pumpkin_util::math::vector3::Vector3;

const STANDING_TORCHES: &[Block] = &[Block::TORCH, Block::SOUL_TORCH, Block::REDSTONE_TORCH];
const WALL_TORCHES: &[Block] = &[
    Block::WALL_TORCH,
    Block::SOUL_WALL_TORCH,
    Block::REDSTONE_WALL_TORCH,
];

const fn horizontal(facing: HorizontalFacing) -> BlockDirection {
    match facing {
        HorizontalFacing::North => BlockDirection::North,
        HorizontalFacing::South => BlockDirection::South,
        HorizontalFacing::West => BlockDirection::West,
        HorizontalFacing::East => BlockDirection::East,
    }
}

/// Simulated's `movementAllowed`: whether a block may be moved at all.
/// What cannot be destroyed cannot, nor what Simulated's `non_movable` tag
/// names.
pub(crate) fn movement_allowed(block: &Block, state: &BlockState) -> bool {
    state.hardness != -1.0 && !mods::block_has_tag(block, "simulated:non_movable")
}

/// Create's `isMovementNecessary`: whether a block is worth moving at all.
/// Brittle blocks always are; what has no collision and can simply be
/// replaced, like flowers and grass, is left behind.
pub(crate) fn movement_necessary(block: &Block, state: &BlockState) -> bool {
    brittle(block, state)
        || (!state.replaceable() && state.get_block_collision_shapes().next().is_some())
}

/// Create's `isBrittle`: blocks that come apart when something pulls on
/// them, so slime never drags them along.
pub(crate) fn brittle(block: &Block, state: &BlockState) -> bool {
    hanging(block, state).is_some()
        || *block == Block::LADDER
        || STANDING_TORCHES.contains(block)
        || WALL_TORCHES.contains(block)
        || block.has_tag(&tag::Block::MINECRAFT_STANDING_SIGNS)
        || block.has_tag(&tag::Block::MINECRAFT_WALL_SIGNS)
        || block.has_tag(&tag::Block::MINECRAFT_PRESSURE_PLATES)
        || *block == Block::LEVER
        || block.has_tag(&tag::Block::MINECRAFT_BUTTONS)
        || block.has_tag(&tag::Block::MINECRAFT_RAILS)
        || *block == Block::REPEATER
        || *block == Block::COMPARATOR
        || *block == Block::REDSTONE_WIRE
        || block.has_tag(&tag::Block::MINECRAFT_WOOL_CARPETS)
        || mods::block_has_tag(block, "create:brittle")
        || mounted_face(state.id).is_some()
        || mods::state_prop(state.id, "hanging").is_some()
}

/// Create's `isNotSupportive`: the side of a block that nothing can be glued
/// or stuck to. A carpet has nothing on its top, and a brittle block nowhere.
pub(crate) fn not_supportive(block: &Block, state: &BlockState, side: BlockDirection) -> bool {
    (block.has_tag(&tag::Block::MINECRAFT_WOOL_CARPETS) && side == BlockDirection::Up)
        || brittle(block, state)
}

fn hanging(block: &Block, state: &BlockState) -> Option<bool> {
    match block.id {
        BlockId::LANTERN | BlockId::SOUL_LANTERN => {
            Some(LanternLikeProperties::from_state_id(state.id).r#hanging)
        }
        BlockId::MANGROVE_PROPAGULE => {
            Some(MangrovePropaguleLikeProperties::from_state_id(state.id).r#hanging)
        }
        _ => None,
    }
}

/// Create's `isBlockAttachedTowards`: whether the block is fastened to the
/// block next to it towards `direction`, as a torch stands on what is below.
pub(crate) fn attached_towards(
    block: &Block,
    state: &BlockState,
    direction: BlockDirection,
) -> bool {
    // The mods' blocks, by the properties vanilla's classes are known for.
    if let Some((face, facing)) = mounted_face(state.id) {
        return attached_by_face(face, facing, direction);
    }
    if let Some(hanging) = mods::state_prop(state.id, "hanging") {
        return direction
            == if hanging == "true" {
                BlockDirection::Up
            } else {
                BlockDirection::Down
            };
    }
    // Simulated's symmetric sails hold onto what lies beside them, across
    // their normal, and a spring onto the block it is fastened to.
    if mods::block_has_tag(block, "simulated:symmetric_sails") {
        return mods::state_prop(state.id, "axis") != Some(axis_of(direction));
    }
    if block.name == "simulated:spring" {
        return mods::state_prop(state.id, "facing") == Some(direction_name(direction.opposite()));
    }
    let facing = || LadderLikeProperties::from_state_id(state.id).r#facing;
    let hangs_from = |towards: HorizontalFacing| opposite_horizontal(direction) == Some(towards);
    match block.id {
        BlockId::LADDER => hangs_from(facing()),
        BlockId::WALL_TORCH | BlockId::SOUL_WALL_TORCH | BlockId::REDSTONE_WALL_TORCH => {
            hangs_from(WallTorchLikeProperties::from_state_id(state.id).r#facing)
        }
        _ if block.has_tag(&tag::Block::MINECRAFT_WALL_SIGNS) => hangs_from(facing()),
        _ if block.has_tag(&tag::Block::MINECRAFT_STANDING_SIGNS)
            || STANDING_TORCHES.contains(block)
            || block.has_tag(&tag::Block::MINECRAFT_PRESSURE_PLATES)
            || block.has_tag(&tag::Block::MINECRAFT_FLOWER_POTS)
            || block.has_tag(&tag::Block::MINECRAFT_RAILS)
            || matches!(
                block.id,
                BlockId::REPEATER | BlockId::COMPARATOR | BlockId::REDSTONE_WIRE
            )
            || block.has_tag(&tag::Block::MINECRAFT_WOOL_CARPETS) =>
        {
            direction == BlockDirection::Down
        }
        _ if block.has_tag(&tag::Block::MINECRAFT_DOORS) => {
            let door = OakDoorLikeProperties::from_state_id(state.id);
            (door.half == DoubleBlockHalf::Lower && direction == BlockDirection::Up)
                || direction == BlockDirection::Down
        }
        _ if block.has_tag(&tag::Block::MINECRAFT_BEDS) => {
            let bed = WhiteBedLikeProperties::from_state_id(state.id);
            let towards = match bed.part {
                BedPart::Head => horizontal(bed.facing).opposite(),
                BedPart::Foot => horizontal(bed.facing),
            };
            direction == towards
        }
        BlockId::BELL => {
            let bell = BellLikeProperties::from_state_id(state.id);
            match bell.attachment {
                BellAttachment::Floor => direction == BlockDirection::Down,
                BellAttachment::Ceiling => direction == BlockDirection::Up,
                BellAttachment::SingleWall | BellAttachment::DoubleWall => {
                    direction == horizontal(bell.facing)
                }
            }
        }
        BlockId::GRINDSTONE => {
            let mount = GrindstoneLikeProperties::from_state_id(state.id);
            attached_by_face(mount.face, mount.facing, direction)
        }
        _ if block.id == BlockId::LEVER || block.has_tag(&tag::Block::MINECRAFT_BUTTONS) => {
            let mount = LeverLikeProperties::from_state_id(state.id);
            attached_by_face(mount.face, mount.facing, direction)
        }
        _ if let Some(hanging) = hanging(block, state) => {
            direction
                == if hanging {
                    BlockDirection::Up
                } else {
                    BlockDirection::Down
                }
        }
        _ => false,
    }
}

/// A mod's block hung like a lever, as `FaceAttachedHorizontalDirectionalBlock`
/// hangs vanilla's: told by its `face` and `facing` properties. Simulated's
/// physics assembler is one.
fn mounted_face(state: BlockStateId) -> Option<(AttachFace, HorizontalFacing)> {
    let face = match mods::state_prop(state, "face")? {
        "floor" => AttachFace::Floor,
        "wall" => AttachFace::Wall,
        "ceiling" => AttachFace::Ceiling,
        _ => return None,
    };
    let facing = match mods::state_prop(state, "facing")? {
        "north" => HorizontalFacing::North,
        "south" => HorizontalFacing::South,
        "west" => HorizontalFacing::West,
        "east" => HorizontalFacing::East,
        _ => return None,
    };
    Some((face, facing))
}

/// A direction as a `facing` property names it.
const fn direction_name(direction: BlockDirection) -> &'static str {
    match direction {
        BlockDirection::Up => "up",
        BlockDirection::Down => "down",
        BlockDirection::North => "north",
        BlockDirection::South => "south",
        BlockDirection::West => "west",
        BlockDirection::East => "east",
    }
}

/// The `axis` property a direction lies along.
const fn axis_of(direction: BlockDirection) -> &'static str {
    match direction {
        BlockDirection::Up | BlockDirection::Down => "y",
        BlockDirection::North | BlockDirection::South => "z",
        BlockDirection::West | BlockDirection::East => "x",
    }
}

fn attached_by_face(face: AttachFace, facing: HorizontalFacing, direction: BlockDirection) -> bool {
    match face {
        AttachFace::Ceiling => direction == BlockDirection::Up,
        AttachFace::Floor => direction == BlockDirection::Down,
        AttachFace::Wall => opposite_horizontal(direction) == Some(facing),
    }
}

const fn opposite_horizontal(direction: BlockDirection) -> Option<HorizontalFacing> {
    match direction.opposite() {
        BlockDirection::North => Some(HorizontalFacing::North),
        BlockDirection::South => Some(HorizontalFacing::South),
        BlockDirection::West => Some(HorizontalFacing::West),
        BlockDirection::East => Some(HorizontalFacing::East),
        _ => None,
    }
}

/// Vanilla's `ChestBlock.getConnectedDirection`: where the other half of a
/// double chest is. `None` for a single chest.
pub(crate) fn chest_half(block: &Block, state: &BlockState) -> Option<BlockDirection> {
    if !matches!(block.id, BlockId::CHEST | BlockId::TRAPPED_CHEST) {
        return None;
    }
    let chest = ChestLikeProperties::from_state_id(state.id);
    let facing = horizontal(chest.facing);
    match chest.r#type {
        ChestType::Single => None,
        // Clockwise seen from above, which is a right-handed turn the other way.
        ChestType::Left => match facing {
            BlockDirection::North => Some(BlockDirection::East),
            BlockDirection::South => Some(BlockDirection::West),
            BlockDirection::West => Some(BlockDirection::North),
            BlockDirection::East => Some(BlockDirection::South),
            _ => None,
        },
        ChestType::Right => match facing {
            BlockDirection::North => Some(BlockDirection::West),
            BlockDirection::South => Some(BlockDirection::East),
            BlockDirection::West => Some(BlockDirection::South),
            BlockDirection::East => Some(BlockDirection::North),
            _ => None,
        },
    }
}

/// Whether two blocks are stuck together, as pistons know it: slime and
/// honey stick to anything but each other.
pub(crate) fn stuck(a: &Block, b: &Block) -> bool {
    let slime = |block: &Block| *block == Block::SLIME_BLOCK;
    let honey = |block: &Block| *block == Block::HONEY_BLOCK;
    if (slime(a) && honey(b)) || (honey(a) && slime(b)) {
        return false;
    }
    slime(a) || honey(a) || slime(b) || honey(b)
}

/// Whether the block a contraption reaches from takes the block `to` that
/// touches it at `offset`, glue aside: through slime or honey, by being
/// fastened to the contraption, or, Aeronautics' rule, by being an envelope
/// next to one.
pub(crate) fn takes(
    from: &Block,
    from_state: &BlockState,
    to: &Block,
    to_state: &BlockState,
    offset: Vector3<i32>,
) -> bool {
    if let Some(direction) = face_direction(offset) {
        if attached_towards(to, to_state, direction.opposite()) {
            return true;
        }
        // Aeronautics' `AeroBlockMovementChecks`: envelopes hold each other.
        if mods::block_has_tag(from, "aeronautics:envelope")
            && mods::block_has_tag(to, "aeronautics:envelope")
        {
            return true;
        }
        if not_supportive(from, from_state, direction)
            || not_supportive(to, to_state, direction.opposite())
        {
            return false;
        }
    }
    !brittle(to, to_state)
        && stuck(from, to)
        && from_state.piston_behavior != PistonBehavior::PushOnly
        && to_state.piston_behavior != PistonBehavior::PushOnly
}

/// The direction a face neighbour lies in; `None` for an edge.
fn face_direction(offset: Vector3<i32>) -> Option<BlockDirection> {
    BlockDirection::all()
        .into_iter()
        .find(|direction| direction.to_offset() == offset)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state(block: &Block) -> &BlockState {
        block.default_state
    }

    #[test]
    fn plain_blocks_do_not_hold_together() {
        let stone = state(&Block::STONE);
        assert!(!takes(
            &Block::STONE,
            stone,
            &Block::STONE,
            stone,
            Vector3::new(1, 0, 0)
        ));
        assert!(!takes(
            &Block::OAK_PLANKS,
            state(&Block::OAK_PLANKS),
            &Block::STONE,
            stone,
            Vector3::new(1, 1, 0)
        ));
    }

    #[test]
    fn slime_holds_anything_but_honey_along_faces_and_edges() {
        let slime = state(&Block::SLIME_BLOCK);
        let stone = state(&Block::STONE);
        assert!(takes(
            &Block::SLIME_BLOCK,
            slime,
            &Block::STONE,
            stone,
            Vector3::new(0, 1, 1)
        ));
        assert!(takes(
            &Block::STONE,
            stone,
            &Block::SLIME_BLOCK,
            slime,
            Vector3::new(0, -1, 0)
        ));
        assert!(!takes(
            &Block::SLIME_BLOCK,
            slime,
            &Block::HONEY_BLOCK,
            state(&Block::HONEY_BLOCK),
            Vector3::new(1, 0, 0)
        ));
        // Brittle blocks and glazed terracotta are left out.
        assert!(!takes(
            &Block::SLIME_BLOCK,
            slime,
            &Block::OAK_SIGN,
            state(&Block::OAK_SIGN),
            Vector3::new(1, 0, 0)
        ));
        assert!(!takes(
            &Block::SLIME_BLOCK,
            slime,
            &Block::WHITE_GLAZED_TERRACOTTA,
            state(&Block::WHITE_GLAZED_TERRACOTTA),
            Vector3::new(1, 0, 0)
        ));
    }

    #[test]
    fn fastened_blocks_follow_their_support() {
        let up = Vector3::new(0, 1, 0);
        // A torch stands on what is below it.
        assert!(takes(
            &Block::STONE,
            state(&Block::STONE),
            &Block::TORCH,
            state(&Block::TORCH),
            up
        ));
        assert!(!takes(
            &Block::STONE,
            state(&Block::STONE),
            &Block::TORCH,
            state(&Block::TORCH),
            Vector3::new(1, 0, 0)
        ));
        // A standing sign and a carpet too; a carpet's top holds nothing.
        assert!(takes(
            &Block::STONE,
            state(&Block::STONE),
            &Block::OAK_SIGN,
            state(&Block::OAK_SIGN),
            up
        ));
        assert!(!takes(
            &Block::WHITE_CARPET,
            state(&Block::WHITE_CARPET),
            &Block::SLIME_BLOCK,
            state(&Block::SLIME_BLOCK),
            up
        ));
        // A wall torch hangs on the wall it faces away from.
        let torch = &Block::WALL_TORCH.states[1];
        let facing = WallTorchLikeProperties::from_state_id(torch.id).r#facing;
        assert!(takes(
            &Block::STONE,
            state(&Block::STONE),
            &Block::WALL_TORCH,
            torch,
            horizontal(facing).to_offset()
        ));
    }

    #[test]
    fn what_cannot_be_destroyed_cannot_be_moved() {
        assert!(movement_allowed(&Block::STONE, state(&Block::STONE)));
        assert!(!movement_allowed(&Block::BEDROCK, state(&Block::BEDROCK)));
    }

    #[test]
    fn a_double_chest_points_at_its_other_half() {
        let mut chest = ChestLikeProperties::default(&Block::CHEST);
        chest.facing = HorizontalFacing::North;
        chest.r#type = ChestType::Left;
        let left = chest.to_state_id(&Block::CHEST);
        assert_eq!(
            chest_half(&Block::CHEST, pumpkin_data::BlockState::from_id(left)),
            Some(BlockDirection::East)
        );
        chest.r#type = ChestType::Single;
        let single = chest.to_state_id(&Block::CHEST);
        assert_eq!(
            chest_half(&Block::CHEST, pumpkin_data::BlockState::from_id(single)),
            None
        );
    }

    #[test]
    fn sails_hold_what_lies_beside_them_and_springs_what_they_are_fastened_to()
    -> Result<(), &'static str> {
        crate::mods::profile::tests::install_test_profile();
        let sail = Block::from_registry_key("simulated:symmetric_sail").ok_or("sail")?;
        let sail = mods::state_with(sail, &[("axis", "y")]).ok_or("sail state")?;
        let stone = state(&Block::STONE);
        // A sail with an upright normal takes the block beside it, not the
        // one in front of or behind it.
        assert!(takes(
            &Block::STONE,
            stone,
            Block::from_registry_key("simulated:symmetric_sail").ok_or("sail")?,
            sail,
            Vector3::new(1, 0, 0)
        ));
        assert!(!takes(
            &Block::STONE,
            stone,
            Block::from_registry_key("simulated:symmetric_sail").ok_or("sail")?,
            sail,
            Vector3::new(0, 1, 0)
        ));
        Ok(())
    }

    #[test]
    fn what_has_no_collision_stays_behind() {
        assert!(!movement_necessary(
            &Block::SHORT_GRASS,
            state(&Block::SHORT_GRASS)
        ));
        assert!(movement_necessary(&Block::STONE, state(&Block::STONE)));
        // Brittle blocks come along, torch first.
        assert!(movement_necessary(&Block::TORCH, state(&Block::TORCH)));
    }
}
