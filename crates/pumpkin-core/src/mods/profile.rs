//! The pack profile: a registry dump of the clients that join, turned into
//! the tables `pumpkin_data::mods` resolves modded ids by.

use std::collections::HashMap;
use std::path::Path;

use pumpkin_data::block_properties::{BLOCK_ENTITY_TYPES, COLLISION_SHAPES, NoteblockInstrument};
use pumpkin_data::block_state::{
    BURNABLE, CAN_OCCLUDE, DOWN_CENTER_SOLID, DOWN_SIDE_SOLID, EAST_SIDE_SOLID,
    HAS_ANALOG_OUTPUT_SIGNAL, HAS_RANDOM_TICKS, IS_AIR, IS_FULL_CUBE, IS_LIQUID, IS_SOLID,
    IS_SOLID_BLOCK, IS_SOLID_RENDER, NORTH_SIDE_SOLID, PistonBehavior, REPLACEABLE,
    SIDED_TRANSPARENCY, SOUTH_SIDE_SOLID, TOOL_REQUIRED, UP_CENTER_SOLID, UP_SIDE_SOLID,
    WEST_SIDE_SOLID,
};
use pumpkin_data::data_component::DataComponent;
use pumpkin_data::data_component_impl::{DataComponentImpl, MaxDamageImpl, MaxStackSizeImpl};
use pumpkin_data::entity::{EntityType, MobCategory, SpawnLocation, SpawnRestriction};
use pumpkin_data::item::Item;
use pumpkin_data::mods::{BlockEntityTypes, Blocks, Entities, Items, Modded};
use pumpkin_data::{Block, BlockId, BlockState, BlockStateId, Flammable};
use pumpkin_util::HeightMap;
use pumpkin_util::math::boundingbox::BoundingBox;
use pumpkin_util::math::vector3::Vector3;
use serde::Deserialize;

/// The dump format `tools/pumpkin_registry_dump.js` writes.
const FORMAT: u32 = 1;

#[derive(Deserialize)]
struct Dump {
    format: u32,
    #[allow(dead_code)]
    game: Game,
    registries: HashMap<String, Registry>,
    blocks: HashMap<String, DumpBlock>,
    #[serde(default)]
    items: HashMap<String, DumpItem>,
    #[serde(default)]
    entities: HashMap<String, DumpEntity>,
}

#[derive(Deserialize)]
struct Game {
    #[allow(dead_code)]
    version: String,
}

#[derive(Deserialize)]
struct Registry {
    /// Every entry as `[id, "namespace:path"]`.
    entries: Vec<(u16, String)>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct DumpBlock {
    /// The first of the block's state ids; the rest follow.
    base: u16,
    /// The state a fresh placement has.
    #[serde(default)]
    default: Option<u16>,
    /// The block entity type the block holds, if any.
    #[serde(default)]
    block_entity: Option<String>,
    #[serde(default)]
    resistance: f32,
    states: Vec<DumpState>,
}

#[derive(Deserialize, Default)]
#[serde(rename_all = "camelCase")]
struct DumpState {
    #[serde(default)]
    shape: Vec<[f64; 6]>,
    #[serde(default)]
    outline: Vec<[f64; 6]>,
    #[serde(default)]
    light: u8,
    #[serde(default)]
    opacity: u8,
    #[serde(default)]
    solid_render: bool,
    #[serde(default)]
    solid: bool,
    #[serde(default)]
    full_cube: bool,
    #[serde(default)]
    can_occlude: bool,
    #[serde(default)]
    tool_required: bool,
    #[serde(default)]
    analog: bool,
    /// Down and up.
    #[serde(default)]
    center_sturdy: Vec<bool>,
    /// Down, up, north, south, west, east.
    #[serde(default)]
    sturdy: Vec<bool>,
    #[serde(default)]
    skip: Vec<bool>,
    #[serde(default)]
    liquid: bool,
    #[serde(default)]
    air: bool,
    #[serde(default)]
    replaceable: bool,
    #[serde(default)]
    random_tick: bool,
    #[serde(default)]
    push: Option<String>,
    #[serde(default)]
    hardness: f32,
    #[serde(default)]
    map_color: u8,
    #[serde(default = "slippery")]
    friction: f32,
    #[serde(default)]
    ignite: u16,
    #[serde(default)]
    burn: u16,
}

const fn slippery() -> f32 {
    0.6
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct DumpItem {
    #[serde(default = "stack_of")]
    max_stack: u8,
    #[serde(default)]
    max_damage: Option<i32>,
}

const fn stack_of() -> u8 {
    64
}

#[derive(Deserialize, Default, Clone)]
#[serde(rename_all = "camelCase")]
struct DumpEntity {
    width: f32,
    height: f32,
    eye: f32,
    #[serde(default)]
    tracking: u32,
    #[serde(default)]
    interval: u32,
    #[serde(default)]
    fire_immune: bool,
    #[serde(default)]
    summon: bool,
    category: Option<String>,
}

/// Reads the dump at `path` and builds the modded registries from it, with
/// the namespaces the dump shows. The dump's vanilla ids must agree with
/// the server's, or the dump is from another game version and is refused.
pub(crate) fn load(path: &Path) -> Result<(Modded, Box<[String]>), String> {
    let bytes = std::fs::read(path)
        .map_err(|error| format!("could not read {}: {error}", path.display()))?;
    let dump: Dump = serde_json::from_slice(&bytes)
        .map_err(|error| format!("{} is not a registry dump: {error}", path.display()))?;
    if dump.format != FORMAT {
        return Err(format!(
            "dump format {} is not the {FORMAT} this server reads",
            dump.format
        ));
    }
    check_vanilla(&dump)?;
    let block_ids = ids_of(&dump, "minecraft:block")?;
    let types = ids_of(&dump, "minecraft:block_entity_type")?;
    let namespaces = namespaces(&dump);
    Ok((
        Modded {
            blocks: blocks(&dump, &block_ids, &types)?,
            block_entity_types: BlockEntityTypes {
                by_id: block_entity_types(&dump, &block_ids, &types)?,
            },
            items: items(&dump)?,
            entities: entities(&dump)?,
        },
        namespaces,
    ))
}

/// The dump's registry `name`, as id and name pairs.
fn entries<'a>(dump: &'a Dump, name: &str) -> Result<&'a [(u16, String)], String> {
    dump.registries
        .get(name)
        .map(|registry| registry.entries.as_slice())
        .ok_or_else(|| format!("dump has no {name} registry"))
}

fn ids_of<'a>(dump: &'a Dump, name: &str) -> Result<HashMap<&'a str, u16>, String> {
    Ok(entries(dump, name)?
        .iter()
        .map(|(id, name)| (name.as_str(), *id))
        .collect())
}

/// The mods the dump shows content of, in the order their first entries
/// appear: the namespace of every block, item and entity type.
fn namespaces(dump: &Dump) -> Box<[String]> {
    let mut found: Vec<String> = Vec::new();
    for name in ["minecraft:block", "minecraft:item", "minecraft:entity_type"]
        .into_iter()
        .filter_map(|registry| dump.registries.get(registry))
        .flat_map(|registry| registry.entries.iter().map(|(_, name)| name))
    {
        if let Some((namespace, _)) = name.split_once(':')
            && namespace != "minecraft"
            && !found.iter().any(|known: &String| known == namespace)
        {
            found.push(namespace.to_owned());
        }
    }
    found.into_boxed_slice()
}

/// The text, kept for as long as the server runs.
fn leaked(text: &str) -> &'static str {
    Box::leak(text.to_owned().into_boxed_str())
}

/// `namespace:path` of the vanilla namespace without its namespace, as the
/// server's tables name it.
fn bare(name: &str) -> &str {
    name.strip_prefix("minecraft:").unwrap_or(name)
}

/// Vanilla ids are fixed: a dump that disagrees with the server's tables
/// was made with another version of the game.
fn check_vanilla(dump: &Dump) -> Result<(), String> {
    let mut wrong: Vec<String> = Vec::new();
    for (id, name) in entries(dump, "minecraft:block")? {
        if usize::from(*id) < BlockId::COUNT as usize {
            let known = Block::from_id(BlockId::new_or_air(*id));
            if known.name != bare(name) {
                wrong.push(format!(
                    "block {id} is {name} there and {} here",
                    known.name
                ));
            }
        }
    }
    for (id, name) in entries(dump, "minecraft:item")? {
        if let Some(known) = Item::from_id(*id)
            && known.registry_key != bare(name)
        {
            wrong.push(format!(
                "item {id} is {name} there and {} here",
                known.registry_key
            ));
        }
    }
    for (id, name) in entries(dump, "minecraft:entity_type")? {
        if let Some(known) = EntityType::from_raw(*id)
            && known.resource_name != bare(name)
        {
            wrong.push(format!(
                "entity type {id} is {name} there and {} here",
                known.resource_name
            ));
        }
    }
    if wrong.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "{} vanilla ids disagree with this server's, among them {}; \
             the dump is from another version of the game",
            wrong.len(),
            wrong[0]
        ))
    }
}

/// The dump's blocks past the vanilla ones, by their first state id.
fn modded_blocks<'a>(
    dump: &'a Dump,
    block_ids: &HashMap<&str, u16>,
) -> Result<Vec<(&'a str, &'a DumpBlock)>, String> {
    let mut found: Vec<(&str, &DumpBlock)> = dump
        .blocks
        .iter()
        .filter(|(name, _)| {
            block_ids
                .get(name.as_str())
                .is_none_or(|id| usize::from(*id) >= BlockId::COUNT as usize)
        })
        .map(|(name, block)| (name.as_str(), block))
        .collect();
    found.sort_by_key(|(_, block)| block.base);
    // The mods' ids follow the vanilla ones without a gap, in the dump's
    // tables as in the client's registries.
    let vanilla_blocks = BlockId::COUNT as usize;
    let mut next_state = usize::from(BlockStateId::COUNT);
    for (index, (name, block)) in found.iter().enumerate() {
        if usize::from(block.base) != next_state {
            return Err(format!(
                "the states of {name} start at {} where {next_state} was expected",
                block.base
            ));
        }
        next_state += block.states.len();
        let id = block_ids[name];
        let next_block = vanilla_blocks + index;
        if usize::from(id) != next_block {
            return Err(format!(
                "block {name} has id {id} where {next_block} was expected"
            ));
        }
    }
    Ok(found)
}

fn blocks(
    dump: &Dump,
    block_ids: &HashMap<&str, u16>,
    types: &HashMap<&str, u16>,
) -> Result<Blocks, String> {
    let found = modded_blocks(dump, block_ids)?;
    let item_ids = ids_of(dump, "minecraft:item")?;
    let mut shapes = Shapes::default();
    let mut built: Vec<&'static Block> = Vec::with_capacity(found.len());
    let mut states = Vec::new();
    let mut state_blocks = Vec::new();
    for (name, dumped) in &found {
        let id = block_ids[name];
        if dumped.states.is_empty() {
            return Err(format!("block {name} has no states"));
        }
        let kind = dumped
            .block_entity
            .as_deref()
            .and_then(|kind| types.get(kind))
            .copied()
            .unwrap_or(u16::MAX);
        let values: Vec<BlockState> = dumped
            .states
            .iter()
            .enumerate()
            .map(|(offset, state)| {
                let id = BlockStateId::new_or_air(
                    dumped.base + u16::try_from(offset).unwrap_or(u16::MAX),
                );
                state_of(id, state, kind, &mut shapes)
            })
            .collect();
        let states_of: &'static [BlockState] = Box::leak(values.into_boxed_slice());
        let owned: Vec<&'static BlockState> = states_of.iter().collect();
        let default = dumped
            .default
            .and_then(|default| {
                owned
                    .get(usize::from(default).saturating_sub(usize::from(dumped.base)))
                    .copied()
            })
            .unwrap_or(owned[0]);
        let first = &dumped.states[0];
        built.push(&*Box::leak(Box::new(Block {
            id: BlockId::new_or_air(id),
            name: leaked(name),
            hardness: first.hardness,
            blast_resistance: dumped.resistance,
            map_color: first.map_color,
            slipperiness: first.friction,
            velocity_multiplier: 1.0,
            jump_velocity_multiplier: 1.0,
            item_id: item_ids.get(*name).copied().unwrap_or(u16::MAX),
            default_state: default,
            states: states_of,
            flammable: (first.ignite > 0 || first.burn > 0).then_some(Flammable {
                spread_chance: first.ignite.min(u8::MAX as u16) as u8,
                burn_chance: first.burn.min(u8::MAX as u16) as u8,
            }),
            experience: None,
        })));
        states.extend(owned.iter().copied());
        state_blocks.extend(std::iter::repeat_n(BlockId::new_or_air(id), owned.len()));
    }
    let by_item = built
        .iter()
        .filter(|block| block.item_id != u16::MAX)
        .map(|block| (block.item_id, *block))
        .collect();
    Ok(Blocks {
        by_name: built.iter().map(|block| (block.name, *block)).collect(),
        by_id: built,
        states,
        state_blocks,
        by_item,
        shapes: shapes.owned,
    })
}

/// One state of a modded block, from what the game reported of it.
fn state_of(
    id: BlockStateId,
    dumped: &DumpState,
    entity_type: u16,
    shapes: &mut Shapes,
) -> BlockState {
    /// The side flags in dump order: down, up, north, south, west, east.
    const SIDES: [u8; 6] = [
        DOWN_SIDE_SOLID,
        UP_SIDE_SOLID,
        NORTH_SIDE_SOLID,
        SOUTH_SIDE_SOLID,
        WEST_SIDE_SOLID,
        EAST_SIDE_SOLID,
    ];
    let mut flags = 0;
    for (on, flag) in [
        (dumped.air, IS_AIR),
        (dumped.burn > 0, BURNABLE),
        (dumped.tool_required, TOOL_REQUIRED),
        (dumped.skip.iter().any(|skip| *skip), SIDED_TRANSPARENCY),
        (dumped.replaceable, REPLACEABLE),
        (dumped.liquid, IS_LIQUID),
        (dumped.solid, IS_SOLID),
        (dumped.solid, IS_SOLID_BLOCK),
        (dumped.full_cube, IS_FULL_CUBE),
        (dumped.random_tick, HAS_RANDOM_TICKS),
        (dumped.solid_render, IS_SOLID_RENDER),
        (dumped.can_occlude, CAN_OCCLUDE),
        (dumped.analog, HAS_ANALOG_OUTPUT_SIGNAL),
    ] {
        if on {
            flags |= flag;
        }
    }
    let mut sides = 0;
    for (index, side) in SIDES.iter().enumerate() {
        if dumped
            .sturdy
            .get(index)
            .copied()
            .unwrap_or(dumped.full_cube)
        {
            sides |= side;
        }
    }
    for (index, side) in [DOWN_CENTER_SOLID, UP_CENTER_SOLID].into_iter().enumerate() {
        if dumped
            .center_sturdy
            .get(index)
            .copied()
            .unwrap_or(dumped.full_cube)
        {
            sides |= side;
        }
    }
    BlockState {
        id,
        state_flags: flags,
        side_flags: sides,
        instrument: NoteblockInstrument::Harp,
        luminance: dumped.light,
        piston_behavior: match dumped.push.as_deref() {
            Some("destroy") => PistonBehavior::Destroy,
            Some("block") => PistonBehavior::Block,
            Some("ignore") => PistonBehavior::Ignore,
            Some("push_only") => PistonBehavior::PushOnly,
            _ => PistonBehavior::Normal,
        },
        hardness: dumped.hardness,
        collision_shapes: shapes.ids(&dumped.shape),
        outline_shapes: shapes.ids(&dumped.outline),
        opacity: dumped.opacity,
        block_entity_type: entity_type,
    }
}

/// The block entity types the mods add, named by their id past the vanilla
/// ones; empty where the dump has no type for an id.
fn block_entity_types<'a>(
    dump: &'a Dump,
    block_ids: &HashMap<&str, u16>,
    types: &HashMap<&'a str, u16>,
) -> Result<Vec<&'static str>, String> {
    let vanilla = BLOCK_ENTITY_TYPES.len();
    let mut named: Vec<&'static str> = Vec::new();
    for (_, block) in modded_blocks(dump, block_ids)? {
        let Some(kind) = block.block_entity.as_deref() else {
            continue;
        };
        let id = types
            .get(kind)
            .copied()
            .ok_or_else(|| format!("dump has no block entity type {kind}"))?;
        let index = usize::from(id)
            .checked_sub(vanilla)
            .ok_or_else(|| format!("block entity type {kind} is a vanilla one here"))?;
        while named.len() <= index {
            named.push("");
        }
        if named[index].is_empty() {
            named[index] = leaked(kind);
        }
    }
    Ok(named)
}

fn items(dump: &Dump) -> Result<Items, String> {
    let vanilla = usize::from(Item::COUNT);
    let mut by_id: Vec<Option<&'static Item>> = Vec::new();
    let mut by_name = HashMap::new();
    for (id, name) in entries(dump, "minecraft:item")? {
        if usize::from(*id) < vanilla {
            continue;
        }
        let Some(dumped) = dump.items.get(name) else {
            return Err(format!("dump has no data for item {name}"));
        };
        let mut components: Vec<(DataComponent, &'static dyn DataComponentImpl)> = Vec::new();
        if dumped.max_stack != 64 {
            components.push((
                DataComponent::MaxStackSize,
                Box::leak(
                    MaxStackSizeImpl {
                        size: dumped.max_stack,
                    }
                    .to_dyn(),
                ),
            ));
        }
        if let Some(max_damage) = dumped.max_damage {
            components.push((
                DataComponent::MaxDamage,
                Box::leak(MaxDamageImpl { max_damage }.to_dyn()),
            ));
        }
        let item: &'static Item = &*Box::leak(Box::new(Item {
            id: *id,
            registry_key: leaked(name),
            components: Box::leak(components.into_boxed_slice()),
        }));
        while by_id.len() < usize::from(*id) - vanilla + 1 {
            by_id.push(None);
        }
        by_id[usize::from(*id) - vanilla] = Some(item);
        by_name.insert(item.registry_key, item);
    }
    Ok(Items { by_id, by_name })
}

fn entities(dump: &Dump) -> Result<Entities, String> {
    let vanilla = EntityType::ALL.len();
    let mut by_id: Vec<Option<&'static EntityType>> = Vec::new();
    let mut by_name = HashMap::new();
    for (id, name) in entries(dump, "minecraft:entity_type")? {
        if usize::from(*id) < vanilla {
            continue;
        }
        let dumped = dump.entities.get(name).cloned().unwrap_or_default();
        let entity: &'static EntityType = &*Box::leak(Box::new(EntityType {
            id: *id,
            attributes: &[],
            experience_reward: 0,
            hurt_sound: None,
            death_sound: None,
            attackable: None,
            // What the mod's mobs are and drop is the mod's to say; none
            // of them spawn from Pumpkin.
            mob: false,
            saveable: true,
            limit_per_chunk: 0,
            summonable: dumped.summon,
            fire_immune: dumped.fire_immune,
            category: match dumped.category.as_deref() {
                Some("monster") => &MobCategory::MONSTER,
                Some("creature") => &MobCategory::CREATURE,
                Some("ambient") => &MobCategory::AMBIENT,
                Some("axolotls") => &MobCategory::AXOLOTLS,
                Some("underground_water_creature") => &MobCategory::UNDERGROUND_WATER_CREATURE,
                Some("water_creature") => &MobCategory::WATER_CREATURE,
                Some("water_ambient") => &MobCategory::WATER_AMBIENT,
                _ => &MobCategory::MISC,
            },
            can_spawn_far_from_player: false,
            client_tracking_range: dumped.tracking,
            update_interval: dumped.interval,
            track_deltas: true,
            dimension: [dumped.width, dumped.height],
            eye_height: dumped.eye,
            spawn_dimensions_scale: 1.0,
            spawn_restriction: SpawnRestriction {
                location: SpawnLocation::Unrestricted,
                heightmap: HeightMap::MotionBlockingNoLeaves,
            },
            resource_name: leaked(name),
        }));
        while by_id.len() < usize::from(*id) - vanilla + 1 {
            by_id.push(None);
        }
        by_id[usize::from(*id) - vanilla] = Some(entity);
        by_name.insert(entity.resource_name, entity);
    }
    Ok(Entities { by_id, by_name })
}

/// The collision boxes of the mods' states: vanilla boxes keep their ids,
/// new ones follow past them.
#[derive(Default)]
struct Shapes {
    known: Option<HashMap<[u64; 6], u16>>,
    owned: Vec<BoundingBox>,
}

impl Shapes {
    fn ids(&mut self, boxes: &[[f64; 6]]) -> &'static [u16] {
        let known = self.known.get_or_insert_with(|| {
            COLLISION_SHAPES
                .iter()
                .enumerate()
                .map(|(id, shape)| (key(*shape), id as u16))
                .collect()
        });
        let ids = boxes
            .iter()
            .map(|boxed| {
                let shape = BoundingBox::new(
                    Vector3::new(boxed[0], boxed[1], boxed[2]),
                    Vector3::new(boxed[3], boxed[4], boxed[5]),
                );
                if let Some(id) = known.get(&key(shape)) {
                    return *id;
                }
                let id = (COLLISION_SHAPES.len() + self.owned.len()) as u16;
                self.owned.push(shape);
                known.insert(key(shape), id);
                id
            })
            .collect::<Vec<_>>();
        Box::leak(ids.into_boxed_slice())
    }
}

/// A box as bits, so equal boxes hash equal whatever their origin.
const fn key(shape: BoundingBox) -> [u64; 6] {
    [
        shape.min.x.to_bits(),
        shape.min.y.to_bits(),
        shape.min.z.to_bits(),
        shape.max.x.to_bits(),
        shape.max.y.to_bits(),
        shape.max.z.to_bits(),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A dump of one modded block, item and entity type over a stub of
    /// vanilla, with `state_offset` applied to the block's first state id.
    fn dump(state_offset: u16, vanilla_first_block: &str) -> serde_json::Value {
        let block = BlockId::COUNT;
        let state = BlockStateId::COUNT + state_offset;
        let item = Item::COUNT;
        let entity = EntityType::ALL.len() as u16;
        serde_json::json!({
            "format": 1,
            "game": { "version": "test" },
            "registries": {
                "minecraft:block": { "sync": true, "entries": [[0, vanilla_first_block], [block, "test:slab"]] },
                "minecraft:item": { "sync": true, "entries": [[0, "minecraft:air"], [item, "test:slab"]] },
                "minecraft:entity_type": { "sync": true, "entries": [[0, "minecraft:allay"], [entity, "test:glue"]] },
                "minecraft:block_entity_type": { "sync": true, "entries": [[BLOCK_ENTITY_TYPES.len() as u16 + 3, "test:holder"]] },
            },
            "blocks": {
                "minecraft:air": { "base": 0, "default": 0, "resistance": 0.0, "states": [ { "air": true } ] },
                "test:slab": {
                    "base": state,
                    "default": state,
                    "blockEntity": "test:holder",
                    "resistance": 3.5,
                    "states": [
                        { "shape": [[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]], "solid": true, "solidRender": true,
                          "fullCube": true, "canOcclude": true, "sturdy": [true, true, true, true, true, true],
                          "light": 3, "opacity": 15, "push": "normal", "hardness": 1.5, "mapColor": 8,
                          "friction": 0.6, "burn": 20, "ignite": 5 },
                        { "shape": [[0.0, 0.0, 0.0, 1.0, 0.5, 1.0]], "outline": [[0.1, 0.2, 0.3, 0.9, 0.7, 0.9]],
                          "solid": true, "fullCube": false, "sturdy": [true, false, true, true, true, true],
                          "opacity": 0, "push": "destroy", "hardness": 0.5, "toolRequired": true }
                    ]
                }
            },
            "items": { "test:slab": { "maxStack": 16, "maxDamage": 250 } },
            "entities": { "test:glue": { "width": 0.5, "height": 0.5, "eye": 0.25, "tracking": 10, "interval": 2147483647u32, "summon": true } }
        })
    }

    fn write(dump: &serde_json::Value, name: &str) -> std::path::PathBuf {
        let path = std::env::temp_dir().join(name);
        std::fs::write(&path, dump.to_string()).ok();
        path
    }

    #[test]
    fn a_dump_becomes_the_modded_registries() -> Result<(), &'static str> {
        let path = write(&dump(0, "minecraft:air"), "pumpkin-profile-ok.json");
        let (modded, namespaces) = load(&path).map_err(|_| "dump refused")?;
        let _ = std::fs::remove_file(&path);
        assert_eq!(
            namespaces.iter().map(String::as_str).collect::<Vec<_>>(),
            ["test"]
        );
        assert_eq!(modded.block_entity_types.by_id, ["", "", "", "test:holder"]);
        // The collision boxes are vanilla's; the odd outline is a new one.
        assert_eq!(modded.blocks.shapes.len(), 1);
        let entity_type = BLOCK_ENTITY_TYPES.len() as u16 + 3;
        let [first, half] = [
            modded.blocks.states[0],
            modded.blocks.states[BlockStateId::COUNT as usize + 1 - BlockStateId::COUNT as usize],
        ];
        assert!(first.is_solid_render() && first.is_full_cube() && first.can_occlude());
        assert!(first.is_side_solid(pumpkin_data::BlockDirection::Up));
        assert_eq!(
            (first.luminance, first.opacity, first.block_entity_type),
            (3, 15, entity_type)
        );
        assert!(!half.is_full_cube() && half.is_solid());
        assert!(!half.is_side_solid(pumpkin_data::BlockDirection::Up));
        assert!(half.is_side_solid(pumpkin_data::BlockDirection::Down));
        assert!(half.tool_required() && !half.replaceable());
        assert_eq!(half.outline_shapes, [COLLISION_SHAPES.len() as u16]);
        let slab = modded.blocks.by_name["test:slab"];
        assert_eq!(
            (slab.hardness, slab.blast_resistance, slab.slipperiness),
            (1.5, 3.5, 0.6)
        );
        assert_eq!(slab.item_id, Item::COUNT);
        assert_eq!(slab.default_state.id, first.id);
        assert_eq!(slab.states.len(), 2);
        let flammable = slab.flammable.as_ref().ok_or("flammable")?;
        assert_eq!((flammable.spread_chance, flammable.burn_chance), (5, 20));
        let item = modded.items.by_id[0].ok_or("item")?;
        assert_eq!(item.registry_key, "test:slab");
        assert_eq!(item.components.len(), 2);
        let glue = modded.entities.by_id[0].ok_or("entity")?;
        assert_eq!(
            (glue.width(), glue.height(), glue.eye_height),
            (0.5, 0.5, 0.25)
        );
        assert!(glue.summonable && glue.client_tracking_range == 10);

        pumpkin_data::mods::install(modded).map_err(|_| "installed twice")?;
        // Every way in resolves to the same entries.
        let entity = EntityType::ALL.len() as u16;
        assert_eq!(
            BlockStateId::new_or_air(first.id.as_u16() + 1)
                .to_block()
                .name,
            "test:slab"
        );
        assert_eq!(Block::from_registry_key("test:slab"), Some(slab));
        assert_eq!(Block::from_item_id(Item::COUNT), Some(slab));
        assert_eq!(Item::from_id(Item::COUNT), Some(item));
        assert_eq!(
            pumpkin_data::mods::block_entity_type(entity_type),
            Some("test:holder")
        );
        assert_eq!(
            pumpkin_data::mods::block_entity_type_id("test:holder"),
            Some(entity_type)
        );
        assert_eq!(EntityType::from_raw(entity), Some(glue));
        assert_eq!(EntityType::from_name("test:glue"), Some(glue));
        // The shape ids past the vanilla boxes reach the new boxes.
        let boxed: Vec<_> = half.get_block_outline_shapes().collect();
        assert_eq!(boxed.len(), 1);
        assert_eq!((boxed[0].min.x, boxed[0].max.y), (0.1, 0.7));
        let collision: Vec<_> = half.get_block_collision_shapes().collect();
        assert_eq!(collision[0].max.y, 0.5);
        Ok(())
    }

    #[test]
    fn a_dump_of_another_game_version_is_refused() {
        let path = write(&dump(0, "minecraft:stone"), "pumpkin-profile-wrong.json");
        let error = load(&path).err().unwrap_or_default();
        let _ = std::fs::remove_file(&path);
        assert!(error.contains("another version"), "{error}");
    }

    #[test]
    fn states_that_leave_a_gap_are_refused() {
        let path = write(&dump(1, "minecraft:air"), "pumpkin-profile-gap.json");
        let error = load(&path).err().unwrap_or_default();
        let _ = std::fs::remove_file(&path);
        assert!(error.contains("was expected"), "{error}");
    }
}
