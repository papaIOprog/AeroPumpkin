//! What a client's mods add to the vanilla registries. A server that
//! mirrors such a client installs a pack profile once at startup, and from
//! then on every lookup by id or by name that falls past the vanilla tables
//! ends here.
//!
//! A block id the profile does not know still resolves, to an inert
//! stand-in that keeps the id: a block from a mod the server has no profile
//! for is neither lost nor taken for another block.

use std::collections::HashMap;
use std::sync::OnceLock;

#[cfg(feature = "block")]
use crate::block_properties::COLLISION_SHAPES;
#[cfg(feature = "block")]
use crate::block_rotation::{Mirror, Rotation, transform_block_properties};
#[cfg(feature = "block")]
use crate::block_state::PistonBehavior;
#[cfg(feature = "entity_type")]
use crate::entity::EntityType;
#[cfg(feature = "item")]
use crate::item::Item;
#[cfg(feature = "block")]
use crate::{Block, BlockId, BlockState, BlockStateId};
#[cfg(feature = "block")]
use pumpkin_util::math::boundingbox::BoundingBox;
use pumpkin_util::math::vector3::Vector3;

/// The modded registries, every table indexed by id past the vanilla one.
#[derive(Default)]
pub struct Modded {
    #[cfg(feature = "block")]
    pub blocks: Blocks,
    #[cfg(feature = "item")]
    pub items: Items,
    #[cfg(feature = "block")]
    pub block_entity_types: BlockEntityTypes,
    #[cfg(feature = "entity_type")]
    pub entities: Entities,
    /// The tags the mods put blocks in, by tag id; the blocks in each.
    #[cfg(feature = "block")]
    pub block_tags: HashMap<&'static str, Box<[BlockId]>>,
    /// The tags the mods put entity types in, by tag id; the types in each.
    #[cfg(feature = "entity_type")]
    pub entity_tags: HashMap<&'static str, Box<[u16]>>,
}

/// Blocks and block states, by ids past the vanilla ones.
#[cfg(feature = "block")]
#[derive(Default)]
pub struct Blocks {
    /// Blocks by block id.
    pub by_id: Vec<&'static Block>,
    /// Block states by state id.
    pub states: Vec<&'static BlockState>,
    /// The block each of those states belongs to, by the same index.
    pub state_blocks: Vec<BlockId>,
    pub by_name: HashMap<&'static str, &'static Block>,
    /// The block an item places, by item id.
    pub by_item: HashMap<u16, &'static Block>,
    /// The collision boxes of the mods' blocks, by shape id past the
    /// vanilla ones.
    pub shapes: Vec<BoundingBox>,
    /// The properties of each state, by the same index as `states`, as
    /// `(name, value)` in the client's order.
    pub props: Vec<&'static [(&'static str, &'static str)]>,
}

/// Items by ids past the vanilla ones; `None` where the client has none.
#[cfg(feature = "item")]
#[derive(Default)]
pub struct Items {
    pub by_id: Vec<Option<&'static Item>>,
    pub by_name: HashMap<&'static str, &'static Item>,
}

/// Entity types by ids past the vanilla ones; `None` where the client has
/// none.
/// The block entity types the mods add, by id past the vanilla ones.
#[cfg(feature = "block")]
#[derive(Default)]
pub struct BlockEntityTypes {
    pub by_id: Vec<&'static str>,
}

#[cfg(feature = "entity_type")]
#[derive(Default)]
pub struct Entities {
    pub by_id: Vec<Option<&'static EntityType>>,
    pub by_name: HashMap<&'static str, &'static EntityType>,
}

static MODDED: OnceLock<Modded> = OnceLock::new();

/// Installs the modded registries. There is one profile per server: a
/// second call is refused and hands the tables back.
pub fn install(modded: Modded) -> Result<(), Modded> {
    MODDED.set(modded)
}

/// Whether a profile is installed.
#[must_use]
pub fn is_installed() -> bool {
    MODDED.get().is_some()
}

/// Where an id lies in the tables of what the mods add: `None` for a
/// vanilla id, or one in a gap the vanilla registry left.
#[must_use]
fn past(id: u16, vanilla: usize) -> Option<usize> {
    usize::from(id).checked_sub(vanilla)
}

/// How many block ids the server knows: the vanilla ones and those of the
/// installed mods.
#[cfg(feature = "block")]
#[must_use]
pub fn block_count() -> usize {
    BlockId::COUNT as usize + MODDED.get().map_or(0, |modded| modded.blocks.by_id.len())
}

/// The collision box a state's shape id names: a vanilla one, or one of
/// the mods' for an id past those.
#[cfg(feature = "block")]
#[must_use]
pub(crate) fn shape(id: u16) -> BoundingBox {
    if let Some(shape) = COLLISION_SHAPES.get(usize::from(id)).copied() {
        return shape;
    }
    let known = MODDED.get().and_then(|modded| {
        modded
            .blocks
            .shapes
            .get(past(id, COLLISION_SHAPES.len())?)
            .copied()
    });
    // An id nobody names: a full block, so nothing falls through it.
    known.unwrap_or_else(|| {
        BoundingBox::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0))
    })
}

#[cfg(feature = "block")]
pub(crate) fn block_state(id: u16) -> &'static BlockState {
    let known = MODDED.get().and_then(|modded| {
        modded
            .blocks
            .states
            .get(past(id, BlockStateId::COUNT as usize)?)
            .copied()
    });
    known.unwrap_or_else(|| unknown_state(id))
}

#[cfg(feature = "block")]
pub(crate) fn block_from_state(id: u16) -> BlockId {
    let known = MODDED.get().and_then(|modded| {
        modded
            .blocks
            .state_blocks
            .get(past(id, BlockStateId::COUNT as usize)?)
            .copied()
    });
    known.unwrap_or_else(|| unknown_block_id(id))
}

#[cfg(feature = "block")]
pub(crate) fn block_from_id(id: u16) -> &'static Block {
    let known = MODDED.get().and_then(|modded| {
        modded
            .blocks
            .by_id
            .get(past(id, BlockId::COUNT as usize)?)
            .copied()
    });
    known.unwrap_or_else(|| unknown_block(id))
}

#[cfg(feature = "block")]
pub(crate) fn block_by_name(name: &str) -> Option<&'static Block> {
    MODDED.get()?.blocks.by_name.get(name).copied()
}

#[cfg(feature = "block")]
pub(crate) fn block_by_item(item: u16) -> Option<&'static Block> {
    MODDED.get()?.blocks.by_item.get(&item).copied()
}

#[cfg(feature = "item")]
pub(crate) fn item(id: u16) -> Option<&'static Item> {
    let index = past(id, Item::COUNT as usize)?;
    MODDED.get()?.items.by_id.get(index).copied().flatten()
}

#[cfg(feature = "item")]
pub(crate) fn item_by_name(name: &str) -> Option<&'static Item> {
    MODDED.get()?.items.by_name.get(name).copied()
}

#[cfg(feature = "entity_type")]
pub(crate) fn entity(id: u16) -> Option<&'static EntityType> {
    let index = past(id, EntityType::ALL.len())?;
    MODDED.get()?.entities.by_id.get(index).copied().flatten()
}

/// The name of the block entity type an id names: a vanilla one, or a
/// mod's for an id past those; `None` where there is none.
#[cfg(feature = "block")]
#[must_use]
pub fn block_entity_type(id: u16) -> Option<&'static str> {
    MODDED
        .get()
        .and_then(|modded| {
            modded
                .block_entity_types
                .by_id
                .get(past(id, crate::block_properties::BLOCK_ENTITY_TYPES.len())?)
                .copied()
        })
        .or_else(|| {
            crate::block_properties::BLOCK_ENTITY_TYPES
                .get(usize::from(id))
                .copied()
        })
}

/// The id the clients' registry gives the block entity type `name`
/// (`namespace:path`): a vanilla one by its path, or a mod's.
#[cfg(feature = "block")]
#[must_use]
pub fn block_entity_type_id(name: &str) -> Option<u16> {
    let path = name.rsplit(':').next().unwrap_or(name);
    crate::block_properties::BLOCK_ENTITY_TYPES
        .iter()
        .position(|known| *known == path)
        .and_then(|index| u16::try_from(index).ok())
        .or_else(|| {
            let vanilla = crate::block_properties::BLOCK_ENTITY_TYPES.len();
            MODDED
                .get()?
                .block_entity_types
                .by_id
                .iter()
                .position(|known| *known == name)
                .and_then(|index| u16::try_from(vanilla + index).ok())
        })
}

#[cfg(feature = "entity_type")]
pub(crate) fn entity_by_name(name: &str) -> Option<&'static EntityType> {
    MODDED.get()?.entities.by_name.get(name).copied()
}

/// The value of the property `name` of the modded block state `state`;
/// `None` where it has no such property or is not one of the mods'.
#[cfg(feature = "block")]
#[must_use]
pub fn state_prop(state: BlockStateId, name: &str) -> Option<&'static str> {
    state_props(state)
        .iter()
        .find(|(key, _)| *key == name)
        .map(|(_, value)| *value)
}

/// The properties of a modded block state; empty for any other.
#[cfg(feature = "block")]
#[must_use]
pub fn state_props(state: BlockStateId) -> &'static [(&'static str, &'static str)] {
    MODDED
        .get()
        .and_then(|modded| {
            modded
                .blocks
                .props
                .get(past(state.as_u16(), BlockStateId::COUNT as usize)?)
                .copied()
        })
        .unwrap_or(&[])
}

/// The state of the modded block `block` whose properties are exactly `props`, whatever their order.
#[cfg(feature = "block")]
#[must_use]
pub fn state_with(block: &Block, props: &[(&str, &str)]) -> Option<&'static BlockState> {
    block.states.iter().find(|state| {
        let own = state_props(state.id);
        own.len() == props.len()
            && props
                .iter()
                .all(|(key, value)| own.contains(&(*key, *value)))
    })
}

/// The state a modded block's state `id` becomes under `rotation` and
/// `mirror`: vanilla's turning of the properties any block may have. `None`
/// when no state of the block has the turned properties.
#[cfg(feature = "block")]
pub(crate) fn state_rotated(
    block: &Block,
    id: BlockStateId,
    rotation: Rotation,
    mirror: Mirror,
) -> Option<&'static BlockState> {
    let props = state_props(id);
    if props.is_empty() {
        return None;
    }
    let turned = transform_block_properties(block.name, props, rotation, mirror);
    if turned.is_empty() {
        return None;
    }
    block.states.iter().find(|state| {
        let own = state_props(state.id);
        own.len() == turned.len()
            && own
                .iter()
                .zip(&turned)
                .all(|((key, value), (turned_key, turned_value))| {
                    *key == turned_key && *value == turned_value
                })
    })
}

/// Whether the installed profile puts `block` in the tag `tag`
/// (`namespace:path`). Vanilla's own tags are the generated ones.
#[cfg(feature = "block")]
#[must_use]
pub fn block_has_tag(block: &Block, tag: &str) -> bool {
    MODDED.get().is_some_and(|modded| {
        modded
            .block_tags
            .get(tag)
            .is_some_and(|blocks| blocks.contains(&block.id))
    })
}

/// Whether the installed profile puts the entity type in the tag `tag`.
#[cfg(feature = "entity_type")]
#[must_use]
pub fn entity_has_tag(entity_type: &EntityType, tag: &str) -> bool {
    MODDED.get().is_some_and(|modded| {
        modded
            .entity_tags
            .get(tag)
            .is_some_and(|types| types.contains(&entity_type.id))
    })
}

// A block nobody has a profile for: one stand-in state per state id, and
// one stand-in block per state, so that ids stay distinct and come back
// out unchanged. They are made on first use and never freed.
#[cfg(feature = "block")]
const LAST_ID: usize = u16::MAX as usize;

#[cfg(feature = "block")]
static UNKNOWN_STATES: OnceLock<Box<[OnceLock<&'static BlockState>]>> = OnceLock::new();

#[cfg(feature = "block")]
static UNKNOWN_BLOCKS: OnceLock<Box<[OnceLock<&'static Block>]>> = OnceLock::new();

#[cfg(feature = "block")]
fn profile_blocks() -> usize {
    MODDED.get().map_or(0, |modded| modded.blocks.by_id.len())
}

#[cfg(feature = "block")]
fn unknown_state(id: u16) -> &'static BlockState {
    let vanilla = BlockStateId::COUNT as usize;
    let slots = UNKNOWN_STATES
        .get_or_init(|| vec![OnceLock::new(); LAST_ID + 1 - vanilla].into_boxed_slice());
    let index = past(id, vanilla).unwrap_or(0);
    slots[index].get_or_init(|| {
        Box::leak(Box::new(BlockState {
            id: BlockStateId::from_raw(id),
            state_flags: 0,
            side_flags: 0,
            instrument: crate::block_properties::NoteblockInstrument::Harp,
            luminance: 0,
            piston_behavior: PistonBehavior::Normal,
            hardness: 0.0,
            collision_shapes: &[],
            outline_shapes: &[],
            opacity: 0,
            block_entity_type: u16::MAX,
        }))
    })
}

/// The block of an unknown state: one of its own past the profile's blocks,
/// so that two such blocks never pass for each other.
#[cfg(feature = "block")]
fn unknown_block_id(state: u16) -> BlockId {
    let vanilla = BlockStateId::COUNT as usize;
    let id =
        BlockId::COUNT as usize + profile_blocks() + usize::from(state).saturating_sub(vanilla);
    BlockId::from_raw(u16::try_from(id).unwrap_or(u16::MAX))
}

#[cfg(feature = "block")]
fn unknown_block(id: u16) -> &'static Block {
    let vanilla = BlockId::COUNT as usize;
    // The state this block stands in for: the one its id was derived from.
    let state = u16::try_from(
        BlockStateId::COUNT as usize + usize::from(id).saturating_sub(vanilla + profile_blocks()),
    )
    .unwrap_or(u16::MAX);
    let slots = UNKNOWN_BLOCKS
        .get_or_init(|| vec![OnceLock::new(); LAST_ID + 1 - vanilla].into_boxed_slice());
    let index = past(id, vanilla).unwrap_or(0);
    slots[index].get_or_init(|| {
        let first = unknown_state(state);
        Box::leak(Box::new(Block {
            id: BlockId::from_raw(id),
            name: "pumpkin:unknown_block",
            hardness: 0.0,
            blast_resistance: 0.0,
            map_color: 0,
            slipperiness: 0.6,
            velocity_multiplier: 1.0,
            jump_velocity_multiplier: 1.0,
            item_id: u16::MAX,
            default_state: first,
            states: &[],
            flammable: None,
            experience: None,
        }))
    })
}

#[cfg(all(test, feature = "block"))]
mod tests {
    use super::*;
    use crate::block_state::REPLACEABLE;

    #[test]
    fn ids_past_the_vanilla_tables_resolve() {
        let first = BlockStateId::COUNT;
        let state = |offset: u16| BlockState {
            id: BlockStateId::from_raw(first + offset),
            state_flags: REPLACEABLE,
            side_flags: 0,
            instrument: crate::block_properties::NoteblockInstrument::Harp,
            luminance: offset as u8,
            piston_behavior: PistonBehavior::Normal,
            hardness: 0.5,
            collision_shapes: &[],
            outline_shapes: &[],
            opacity: 0,
            block_entity_type: u16::MAX,
        };
        let states: &'static [BlockState] = Box::leak(vec![state(0), state(1)].into_boxed_slice());
        let id = BlockId::from_raw(BlockId::COUNT);
        let slab: &'static Block = &*Box::leak(Box::new(Block {
            id,
            name: "test:slab",
            hardness: 0.5,
            blast_resistance: 0.5,
            map_color: 0,
            slipperiness: 0.6,
            velocity_multiplier: 1.0,
            jump_velocity_multiplier: 1.0,
            item_id: 1400,
            default_state: &states[0],
            states,
            flammable: None,
            experience: None,
        }));
        install(Modded {
            blocks: Blocks {
                by_id: vec![slab],
                states: states.iter().collect(),
                state_blocks: vec![id; 2],
                by_name: HashMap::from([("test:slab", slab)]),
                by_item: HashMap::from([(1400, slab)]),
                shapes: Vec::new(),
                props: vec![&[], &[("half", "bottom")]],
            },
            block_tags: HashMap::from([("test:sticky".into(), vec![id].into_boxed_slice())]),
            ..Modded::default()
        })
        .ok();

        // By state id, by block id, by name and by item.
        assert_eq!(BlockStateId::from_raw(first + 1).to_state().luminance, 1);
        assert_eq!(BlockStateId::from_raw(first).to_block().name, "test:slab");
        assert_eq!(Block::from_id(id).name, "test:slab");
        assert_eq!(
            Block::from_registry_key("test:slab").map(|block| block.name),
            Some("test:slab")
        );
        assert_eq!(
            Block::from_item_id(1400).map(|block| block.name),
            Some("test:slab")
        );
        assert_eq!(block_count(), BlockId::COUNT as usize + 1);
        assert!(Block::from_registry_key("test:none").is_none());

        // An id the profile does not know keeps its id and is nobody's block.
        let unknown = BlockStateId::from_raw(first + 40);
        assert_eq!(unknown.to_state().id, unknown);
        assert_eq!(unknown.to_block().name, "pumpkin:unknown_block");
        assert_eq!(unknown.to_state().id, unknown);
        assert_ne!(
            unknown.to_block().id,
            BlockStateId::from_raw(first + 41).to_block().id
        );
        // Vanilla ids still come from the vanilla tables.
        assert_eq!(
            Block::STONE.default_state.id.to_state().id,
            Block::STONE.default_state.id
        );

        // Properties and tags of the mods' states.
        assert_eq!(state_prop(BlockStateId::from_raw(first), "half"), None);
        assert_eq!(
            state_prop(BlockStateId::from_raw(first + 1), "half"),
            Some("bottom")
        );
        assert_eq!(
            state_with(slab, &[("half", "bottom")]).map(|state| state.id),
            Some(BlockStateId::from_raw(first + 1))
        );
        assert!(state_with(slab, &[("half", "top")]).is_none());
        assert!(block_has_tag(slab, "test:sticky"));
        assert!(!block_has_tag(slab, "test:none"));
        assert!(!block_has_tag(&Block::STONE, "test:sticky"));
    }
}
