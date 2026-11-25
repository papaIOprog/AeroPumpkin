//! Glue: a box of blocks an assembly takes along whole. Create's super glue
//! and Simulated's honey glue are such boxes, each an entity whose box the
//! clients read from its spawn data.

use std::sync::{Arc, Mutex};

use pumpkin_data::damage::DamageType;
use pumpkin_data::entity::EntityType;
use pumpkin_data::item::Item;
use pumpkin_data::item_stack::ItemStack;
use pumpkin_data::particle::Particle;
use pumpkin_data::sound::SoundCategory;
use pumpkin_nbt::compound::NbtCompound;
use pumpkin_nbt::tag::NbtTag;
use pumpkin_protocol::codec::item_stack_seralizer::ItemStackSerializer;
use pumpkin_protocol::codec::var_int::VarInt;
use pumpkin_protocol::java::client::play::{CParticle, CSoundEffect};
use pumpkin_protocol::ser::NetworkWriteExt;
use pumpkin_protocol::{IdOr, SoundEvent};
use pumpkin_util::math::position::BlockPos;
use pumpkin_util::math::vector3::Vector3;

use crate::entity::player::Player;
use crate::entity::{Entity, EntityBase, living::LivingEntity};
use crate::net::bedrock::BedrockClient;
use crate::server::Server;
use crate::world::World;

/// The two glues, by the entity type that carries each.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GlueKind {
    /// Create's `create:super_glue`.
    Super,
    /// Simulated's `simulated:honey_glue`.
    Honey,
}

impl GlueKind {
    /// The entity type that carries this glue, if the profile knows it.
    #[must_use]
    pub fn entity_type(self) -> Option<&'static EntityType> {
        EntityType::from_name(match self {
            Self::Super => "create:super_glue",
            Self::Honey => "simulated:honey_glue",
        })
    }
}

/// A glue box in the world: an entity at the middle of the box's floor,
/// of the box's size.
pub struct GlueEntity {
    entity: Entity,
    kind: GlueKind,
    size: Mutex<Vector3<f64>>,
}

impl GlueEntity {
    /// Puts a box of `kind` over `bounds` (`min_x, min_y, min_z, max_x,
    /// max_y, max_z`) into the world, if the profile knows the glue.
    pub fn spawn(world: &Arc<World>, kind: GlueKind, bounds: [f64; 6]) -> Option<Arc<Self>> {
        let entity_type = kind.entity_type()?;
        let entity = Entity::new(Arc::clone(world), Self::centre_of(bounds), entity_type);
        entity
            .no_physics
            .store(true, std::sync::atomic::Ordering::Relaxed);
        let glue = Arc::new(Self {
            entity,
            kind,
            size: Mutex::new(Self::size_of(bounds)),
        });
        world.spawn_entity(glue.clone());
        Some(glue)
    }

    /// The middle of a box's floor, where its entity stands.
    const fn centre_of(bounds: [f64; 6]) -> Vector3<f64> {
        Vector3::new(
            f64::midpoint(bounds[0], bounds[3]),
            bounds[1],
            f64::midpoint(bounds[2], bounds[5]),
        )
    }

    fn size_of(bounds: [f64; 6]) -> Vector3<f64> {
        Vector3::new(
            bounds[3] - bounds[0],
            bounds[4] - bounds[1],
            bounds[5] - bounds[2],
        )
    }

    /// The box, in world coordinates.
    #[must_use]
    pub fn bounds(&self) -> [f64; 6] {
        let position = self.entity.pos.load();
        let size = *self
            .size
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        [
            position.x - size.x * 0.5,
            position.y,
            position.z - size.z * 0.5,
            position.x + size.x * 0.5,
            position.y + size.y,
            position.z + size.z * 0.5,
        ]
    }

    /// Moves the glue to another box: Simulated's `setBounds`.
    pub fn set_bounds(&self, bounds: [f64; 6]) {
        *self
            .size
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner) = Self::size_of(bounds);
        self.entity.set_pos(Self::centre_of(bounds));
    }

    /// Which of the two glues this is.
    #[must_use]
    pub const fn kind(&self) -> GlueKind {
        self.kind
    }

    /// Where the entity stands: the middle of the box's floor.
    #[must_use]
    pub fn position(&self) -> Vector3<f64> {
        self.entity.pos.load()
    }

    /// The middle of the box: where a sound about the glue belongs.
    #[must_use]
    pub fn centre(&self) -> Vector3<f64> {
        let [min_x, min_y, min_z, max_x, max_y, max_z] = self.bounds();
        Vector3::new(
            f64::midpoint(min_x, max_x),
            f64::midpoint(min_y, max_y),
            f64::midpoint(min_z, max_z),
        )
    }

    /// The entity's id, as the clients name it.
    #[must_use]
    pub const fn id(&self) -> i32 {
        self.entity.entity_id
    }

    /// The entity's uuid, as the clients name it.
    #[must_use]
    pub const fn uuid(&self) -> uuid::Uuid {
        self.entity.entity_uuid
    }

    /// Takes the glue out of the world.
    pub fn remove(&self) {
        self.entity.remove();
    }

    /// The mods' `spawnParticles`: the glue's chips along the edges of its
    /// box, every half block.
    pub fn spawn_particles(&self, world: &World) {
        let bounds = self.bounds();
        let origin = [bounds[0], bounds[1], bounds[2]];
        let extents = [
            bounds[3] - bounds[0],
            bounds[4] - bounds[1],
            bounds[5] - bounds[2],
        ];
        let (particle, data): (Particle, Vec<u8>) = match self.kind {
            GlueKind::Super => (Particle::ItemSlime, Vec::new()),
            GlueKind::Honey => (Particle::Item, honey_block_stack()),
        };
        for axis in 0..3 {
            let (second, third) = ((axis + 1) % 3, (axis + 2) % 3);
            for step in 0..=(extents[axis] * 2.0) as u32 {
                for (along_second, along_third) in [(0.0, 0.0), (0.0, 1.0), (1.0, 0.0), (1.0, 1.0)]
                {
                    let mut at = origin;
                    at[axis] += f64::from(step) * 0.5;
                    at[second] += extents[second] * along_second;
                    at[third] += extents[third] * along_third;
                    let packet = CParticle::new(
                        false,
                        false,
                        Vector3::new(at[0], at[1], at[2]),
                        Vector3::new(0.0, 0.0, 0.0),
                        0.0,
                        1,
                        VarInt(i32::from(particle.to_id())),
                        &data,
                    );
                    world.broadcast_packet_all(&packet);
                }
            }
        }
    }

    /// The box's corners counted from the entity, as the entity saves them:
    /// Create's and Simulated's `addAdditionalSaveData`.
    fn save(&self, nbt: &mut NbtCompound) {
        let position = self.entity.pos.load();
        let [min_x, min_y, min_z, max_x, max_y, max_z] = self.bounds();
        nbt.put(
            "From",
            doubles([min_x - position.x, min_y - position.y, min_z - position.z]),
        );
        nbt.put(
            "To",
            doubles([max_x - position.x, max_y - position.y, max_z - position.z]),
        );
    }
}

/// The `item` particle's data: a single honey block, as Simulated's
/// `spawnParticles` shows.
fn honey_block_stack() -> Vec<u8> {
    let mut data = Vec::new();
    ItemStackSerializer::from(ItemStack::new(1, &Item::HONEY_BLOCK))
        .write(&mut data)
        .ok();
    data
}

/// Three numbers as the NBT list vanilla writes vectors as.
fn doubles(values: [f64; 3]) -> NbtTag {
    NbtTag::List(values.iter().map(|value| NbtTag::Double(*value)).collect())
}

/// Whether the box `bounds` covers the block `pos`, its centre lying in it.
#[must_use]
pub(crate) fn covers(bounds: [f64; 6], pos: BlockPos) -> bool {
    let [min_x, min_y, min_z, max_x, max_y, max_z] = bounds;
    let centre = pos.to_centered_f64();
    centre.x >= min_x
        && centre.x < max_x
        && centre.y >= min_y
        && centre.y < max_y
        && centre.z >= min_z
        && centre.z < max_z
}

/// Plays the sound event `name` (`namespace:path`) for the players around
/// `position`, all but `except`. A sound by name reaches the clients'
/// registries, the mods' included.
pub(crate) fn play_named_sound(
    world: &World,
    name: &str,
    category: SoundCategory,
    position: &Vector3<f64>,
    volume: f32,
    pitch: f32,
    except: Option<uuid::Uuid>,
) {
    let packet = CSoundEffect::new(
        IdOr::Value(SoundEvent {
            sound_name: name.into(),
            range: None,
        }),
        category,
        position,
        volume,
        pitch,
        rand::random::<i64>(),
    );
    match except {
        Some(except) => world.broadcast_packet_except(&[except], &packet),
        None => world.broadcast_packet_all(&packet),
    }
}

/// A glue box as an assembly finds them: what holds two blocks together.
#[derive(Debug, Clone, Copy)]
pub struct GlueBox {
    /// The entity carrying the glue, by id.
    pub entity_id: i32,
    pub kind: GlueKind,
    /// The box in world coordinates, as [`GlueEntity::bounds`] gives them.
    pub bounds: [f64; 6],
}

impl GlueBox {
    /// Whether the box covers the block `pos`: the mods' `contains`, which
    /// tries the block's centre.
    #[must_use]
    pub fn contains(self, pos: BlockPos) -> bool {
        covers(self.bounds, pos)
    }

    /// Whether the glue holds the two blocks together: both lie in it.
    #[must_use]
    pub fn holds(self, from: BlockPos, to: BlockPos) -> bool {
        self.contains(from) && self.contains(to)
    }
}

/// Every glue box in the world.
#[must_use]
pub fn glue_boxes(world: &World) -> Vec<GlueBox> {
    world
        .entities
        .load()
        .iter()
        .filter_map(|entity| {
            let glue = entity.cast_any().downcast_ref::<GlueEntity>()?;
            Some(GlueBox {
                entity_id: glue.id(),
                kind: glue.kind,
                bounds: glue.bounds(),
            })
        })
        .collect()
}

/// What `with` makes of the glue entity with the id `id`, if there is one.
pub fn with_glue_by_id<R>(
    world: &World,
    id: i32,
    with: impl FnOnce(&GlueEntity) -> R,
) -> Option<R> {
    with_glue(world, |glue| glue.id() == id, with)
}

/// What `with` makes of the glue entity with the uuid `uuid`, if there is one.
pub fn with_glue_by_uuid<R>(
    world: &World,
    uuid: uuid::Uuid,
    with: impl FnOnce(&GlueEntity) -> R,
) -> Option<R> {
    with_glue(world, |glue| glue.uuid() == uuid, with)
}

fn with_glue<R>(
    world: &World,
    found: impl Fn(&GlueEntity) -> bool,
    with: impl FnOnce(&GlueEntity) -> R,
) -> Option<R> {
    let mut with = Some(with);
    world.entities.load().iter().find_map(|entity| {
        let glue = entity.cast_any().downcast_ref::<GlueEntity>()?;
        if !found(glue) {
            return None;
        }
        with.take().map(|with| with(glue))
    })
}

impl EntityBase for GlueEntity {
    fn get_entity(&self) -> &Entity {
        &self.entity
    }

    fn get_living_entity(&self) -> Option<&LivingEntity> {
        None
    }

    fn cast_any(&self) -> &dyn std::any::Any {
        self
    }

    fn tick(&self, _caller: &dyn EntityBase, _server: &Server) {}

    fn is_pushable(&self) -> bool {
        false
    }

    fn is_pushed_by_fluids(&self) -> bool {
        false
    }

    fn is_immune_to_explosion(&self) -> bool {
        true
    }

    fn damage_with_context(
        &self,
        _caller: &dyn EntityBase,
        _amount: f32,
        _damage_type: DamageType,
        _position: Option<Vector3<f64>>,
        _source: Option<&dyn EntityBase>,
        _cause: Option<&dyn EntityBase>,
    ) -> bool {
        false
    }

    fn interact(
        &self,
        _player: &Arc<Player>,
        _item_stack: &mut pumpkin_data::item_stack::ItemStack,
    ) -> bool {
        false
    }

    fn write_custom_nbt(&self, nbt: &mut NbtCompound) {
        self.save(nbt);
    }

    /// The box, which the clients read right after the spawn packet.
    fn neoforge_spawn_data(&self) -> Option<Vec<u8>> {
        let mut nbt = NbtCompound::new();
        self.save(&mut nbt);
        if self.kind == GlueKind::Honey {
            // Simulated's honey glue reads its box relative to a saved
            // position of its own.
            let position = self.entity.pos.load();
            nbt.put("Pos", doubles([position.x, position.y, position.z]));
        }
        let mut data = Vec::new();
        data.write_compound_nbt_with_version(
            Some(&nbt),
            &pumpkin_util::version::JavaMinecraftVersion::V_1_21,
        )
        .ok()?;
        let mut out = Vec::new();
        out.write_var_int(&VarInt(self.entity.entity_id)).ok()?;
        out.write_var_int(&VarInt(i32::try_from(data.len()).ok()?))
            .ok()?;
        out.extend_from_slice(&data);
        Some(out)
    }

    fn send_bedrock_spawn_packet(&self, _client: &BedrockClient) {
        // Bedrock has no mods to carry the glue.
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn box_of(min: [f64; 3], max: [f64; 3]) -> [f64; 6] {
        [min[0], min[1], min[2], max[0], max[1], max[2]]
    }

    #[test]
    fn a_glue_box_covers_the_blocks_whose_centres_lie_in_it() {
        let glue = GlueBox {
            entity_id: 7,
            kind: GlueKind::Super,
            bounds: box_of([1.0, 2.0, 3.0], [3.0, 4.0, 5.0]),
        };
        assert!(glue.contains(BlockPos(Vector3::new(1, 2, 3))));
        assert!(glue.contains(BlockPos(Vector3::new(2, 3, 4))));
        // The box ends where the last block does.
        assert!(!glue.contains(BlockPos(Vector3::new(3, 3, 3))));
        assert!(!glue.contains(BlockPos(Vector3::new(2, 4, 3))));
        assert!(glue.holds(
            BlockPos(Vector3::new(1, 2, 3)),
            BlockPos(Vector3::new(2, 3, 4))
        ));
        assert!(!glue.holds(
            BlockPos(Vector3::new(2, 3, 4)),
            BlockPos(Vector3::new(3, 3, 4))
        ));
    }

    #[test]
    fn the_honey_glue_shows_one_honey_block_as_vanilla_writes_a_stack() {
        // Count, the honey block's id 665, then no component set and none
        // removed: `DataComponentPatch` sends two counts, also when empty.
        assert_eq!(honey_block_stack(), [1, 0x99, 0x05, 0, 0]);
    }
}
