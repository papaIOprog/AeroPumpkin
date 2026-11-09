use std::sync::{
    Arc, Weak,
    atomic::{AtomicI32, AtomicU8, Ordering},
};

use crossbeam::atomic::AtomicCell;
use pumpkin_data::entity::EntityType;
use pumpkin_data::item::Item;
use pumpkin_data::item_stack::ItemStack;
use pumpkin_data::sound::{Sound, SoundCategory};
use pumpkin_data::tag::{self, Taggable};
use pumpkin_inventory::mount_screen_handler::HorseScreenHandler;
use pumpkin_inventory::screen_handler::{
    InventoryPlayer, ScreenHandlerFactory, SharedScreenHandler,
};
use pumpkin_inventory::{Inventory, SimpleInventory};
use pumpkin_nbt::compound::NbtCompound;
use pumpkin_protocol::codec::var_int::VarInt;
use pumpkin_util::text::TextComponent;
use rand::RngExt;
use uuid::Uuid;

use crate::entity::{
    Entity, EntityBase,
    ageable::{AgeableData, AgeableMob},
    ai::goal::{
        breed::BreedGoal, escape_danger::EscapeDangerGoal, follow_parent::FollowParentGoal,
        look_around::RandomLookAroundGoal, look_at_entity::LookAtEntityGoal,
        run_around_like_crazy::RunAroundLikeCrazyGoal, swim::SwimGoal, tempt::TemptGoal,
        wander_around::WanderAroundGoal,
    },
    mob::{Mob, MobEntity},
    passive::animal::Animal,
    player::Player,
};

const TEMPT_ITEMS: &[&Item] = &[
    &Item::GOLDEN_APPLE,
    &Item::ENCHANTED_GOLDEN_APPLE,
    &Item::GOLDEN_CARROT,
];

pub const FLAG_TAME: u8 = 2;
pub const FLAG_SADDLE: u8 = 4;
pub const FLAG_BRED: u8 = 8;
pub const FLAG_EATING: u8 = 16;
pub const FLAG_STANDING: u8 = 32;
pub const FLAG_OPEN_MOUTH: u8 = 64;

pub struct HorseEntity {
    pub mob_entity: MobEntity,
    pub ageable_data: AgeableData,
    pub variant: AtomicI32,
    pub flags: AtomicU8,
    pub temper: AtomicI32,
    pub owner: AtomicCell<Option<Uuid>>,
    /// Vanilla keeps the saddle in the first slot of the horse inventory and
    /// body armor in the body equipment slot; Pumpkin keeps both as one-slot
    /// inventories backing the mount screen.
    pub saddle_inventory: Arc<SimpleInventory>,
    pub armor_inventory: Arc<SimpleInventory>,
}

impl HorseEntity {
    pub fn new(entity: Entity) -> Arc<Self> {
        let mob_entity = MobEntity::new(entity);
        let mut rng = rand::rng();
        let color = rng.random_range(0..7);
        let style = rng.random_range(0..5);
        let variant = color | (style << 8);

        let horse = Self {
            mob_entity,
            ageable_data: AgeableData::default(),
            variant: AtomicI32::new(variant),
            flags: AtomicU8::new(0),
            temper: AtomicI32::new(0),
            owner: AtomicCell::new(None),
            saddle_inventory: Arc::new(SimpleInventory::new(1)),
            armor_inventory: Arc::new(SimpleInventory::new(1)),
        };
        let mob_arc = Arc::new(horse);
        let mob_weak: Weak<dyn Mob> = {
            let mob_arc: Arc<dyn Mob> = mob_arc.clone();
            Arc::downgrade(&mob_arc)
        };

        {
            let mut goal_selector = mob_arc
                .mob_entity
                .goals_selector
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);

            goal_selector.add_goal(0, Box::new(SwimGoal::default()));
            goal_selector.add_goal(
                1,
                Box::new(RunAroundLikeCrazyGoal::new(mob_arc.clone(), 1.2)),
            );
            goal_selector.add_goal(1, EscapeDangerGoal::new(1.2));
            goal_selector.add_goal(2, BreedGoal::new(1.0));
            goal_selector.add_goal(3, Box::new(TemptGoal::new(1.25, TEMPT_ITEMS, false)));
            goal_selector.add_goal(4, Box::new(FollowParentGoal::new(1.0)));
            goal_selector.add_goal(6, Box::new(WanderAroundGoal::new(0.7)));
            goal_selector.add_goal(
                7,
                LookAtEntityGoal::with_default(mob_weak, &EntityType::PLAYER, 6.0),
            );
            goal_selector.add_goal(8, Box::new(RandomLookAroundGoal::default()));
        };

        mob_arc
    }

    #[must_use]
    pub fn get_variant(&self) -> i32 {
        self.variant.load(Ordering::Relaxed)
    }

    pub fn set_variant(&self, variant: i32) {
        self.variant.store(variant, Ordering::Relaxed);
        let entity = self.get_entity();
        entity.set_synced_data(pumpkin_data::tracked_data::horse::VARIANT, VarInt(variant));
    }

    #[must_use]
    pub fn has_flag(&self, flag: u8) -> bool {
        (self.flags.load(Ordering::Relaxed) & flag) != 0
    }

    pub fn set_flag(&self, flag: u8, val: bool) {
        let current = self.flags.load(Ordering::Relaxed);
        let new_flags = if val { current | flag } else { current & !flag };
        self.flags.store(new_flags, Ordering::Relaxed);
        let entity = self.get_entity();
        entity.set_synced_data(
            pumpkin_data::tracked_data::horse::DATA_ID_FLAGS,
            new_flags as i8,
        );
    }

    #[must_use]
    pub fn is_tame(&self) -> bool {
        self.has_flag(FLAG_TAME)
    }

    pub fn set_tame(&self, val: bool) {
        self.set_flag(FLAG_TAME, val);
    }

    #[must_use]
    pub fn is_saddled(&self) -> bool {
        self.has_flag(FLAG_SADDLE)
    }

    pub fn set_saddled(&self, val: bool) {
        self.set_flag(FLAG_SADDLE, val);
    }

    /// Vanilla `AbstractHorse.syncSaddleToClients`: the saddle flag mirrors
    /// the saddle slot. Called every tick because Pumpkin inventories have no
    /// change listener; `set_synced_data` only marks dirty on change.
    pub fn sync_saddle_to_clients(&self) {
        let saddled = !self.saddle_inventory.get_stack(0).is_empty();
        self.set_saddled(saddled);
    }

    /// Vanilla `AbstractHorse.makeMad`: rear up with an angry sound. The
    /// standing pose needs `aiStep` counters we do not keep, so only the
    /// sound plays.
    pub fn make_mad(&self) {
        let entity = self.get_entity();
        entity.world.load().play_sound(
            Sound::EntityHorseAngry,
            SoundCategory::Neutral,
            &entity.pos.load(),
        );
    }
}

impl ScreenHandlerFactory for HorseEntity {
    fn create_screen_handler(
        &self,
        sync_id: u8,
        player_inventory: &Arc<pumpkin_inventory::player::player_inventory::PlayerInventory>,
        _player: &dyn InventoryPlayer,
    ) -> Option<SharedScreenHandler> {
        let handler = HorseScreenHandler::new(
            sync_id,
            player_inventory,
            self.saddle_inventory.clone(),
            self.armor_inventory.clone(),
        );
        Some(Arc::new(std::sync::Mutex::new(handler)) as SharedScreenHandler)
    }

    fn get_display_name(&self) -> TextComponent {
        TextComponent::translate("container.horse", vec![])
    }
}

impl AgeableMob for HorseEntity {
    fn get_ageable_data(&self) -> &AgeableData {
        &self.ageable_data
    }
}

impl Animal for HorseEntity {
    fn is_food(&self, item_stack: &ItemStack) -> bool {
        item_stack.item.has_tag(&tag::Item::MINECRAFT_HORSE_FOOD)
            || item_stack.item == &Item::WHEAT
            || item_stack.item == &Item::SUGAR
            || item_stack.item == &Item::HAY_BLOCK
            || item_stack.item == &Item::APPLE
            || item_stack.item == &Item::GOLDEN_CARROT
            || item_stack.item == &Item::GOLDEN_APPLE
            || item_stack.item == &Item::ENCHANTED_GOLDEN_APPLE
    }
}

impl Mob for HorseEntity {
    fn as_ageable(&self) -> Option<&dyn AgeableMob> {
        Some(self)
    }

    fn as_animal(&self) -> Option<&dyn Animal> {
        Some(self)
    }

    /// Vanilla `AbstractHorse.openCustomInventoryScreen`: only a tamed horse
    /// that is unridden or ridden by the opening player shows its inventory.
    fn open_custom_inventory_screen(&self, player: &Arc<Player>) {
        let entity = self.get_entity();
        let opened_by_rider = entity
            .passengers
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .iter()
            .any(|passenger| passenger.get_entity().entity_uuid == player.get_entity().entity_uuid);
        if (!entity.has_passengers() || opened_by_rider) && self.is_tame() {
            player.open_handled_screen(self, None);
        }
    }

    fn mob_write_nbt(&self, nbt: &mut NbtCompound) {
        self.write_ageable_nbt(nbt);
        nbt.put_int("Variant", self.get_variant());
        nbt.put_bool("Tame", self.is_tame());
        nbt.put_int("Temper", self.temper.load(Ordering::Relaxed));
        if let Some(owner) = self.owner.load() {
            nbt.put_uuid("Owner", owner);
        }
        let saddle = self.saddle_inventory.get_stack(0);
        if !saddle.is_empty() {
            let mut saddle_nbt = NbtCompound::new();
            saddle.write_item_stack(&mut saddle_nbt);
            nbt.put_compound("SaddleItem", saddle_nbt);
        }
        let armor = self.armor_inventory.get_stack(0);
        if !armor.is_empty() {
            let mut armor_nbt = NbtCompound::new();
            armor.write_item_stack(&mut armor_nbt);
            nbt.put_compound("ArmorItem", armor_nbt);
        }
    }

    fn mob_read_nbt(&self, nbt: &NbtCompound) {
        self.read_ageable_nbt(nbt);
        if let Some(variant) = nbt.get_int("Variant") {
            self.set_variant(variant);
        }
        if let Some(tame) = nbt.get_bool("Tame") {
            self.set_tame(tame);
        }
        if let Some(temper) = nbt.get_int("Temper") {
            self.temper.store(temper, Ordering::Relaxed);
        }
        if let Some(owner) = nbt.get_uuid("Owner") {
            self.owner.store(Some(owner));
        }
        if let Some(saddle_nbt) = nbt.get_compound("SaddleItem")
            && let Some(saddle) = ItemStack::read_item_stack(saddle_nbt)
            && saddle.item == &Item::SADDLE
        {
            self.saddle_inventory.set_stack(0, saddle);
        }
        if let Some(armor_nbt) = nbt.get_compound("ArmorItem")
            && let Some(armor) = ItemStack::read_item_stack(armor_nbt)
            && armor.item.registry_key.ends_with("_horse_armor")
        {
            self.armor_inventory.set_stack(0, armor);
        }
        self.sync_saddle_to_clients();
    }

    fn get_mob_entity(&self) -> &MobEntity {
        &self.mob_entity
    }

    fn mob_tick(&self, _caller: &dyn EntityBase) {
        self.sync_saddle_to_clients();
        self.ageable_ai_step();
    }

    fn mob_init_data_tracker(&self) {
        let entity = self.get_entity();
        let is_baby = entity.age.load(Ordering::Relaxed) < 0;
        if is_baby {
            entity.set_synced_data(pumpkin_data::tracked_data::horse::DATA_BABY_ID, true);
        }
        entity.set_synced_data(
            pumpkin_data::tracked_data::horse::VARIANT,
            VarInt(self.get_variant()),
        );
        entity.set_synced_data(
            pumpkin_data::tracked_data::horse::DATA_ID_FLAGS,
            self.flags.load(Ordering::Relaxed) as i8,
        );
    }

    fn mob_interact(&self, player: &Arc<Player>, item_stack: &mut ItemStack) -> bool {
        // Vanilla `Horse.mobInteract` on top of `AbstractHorse.mobInteract`.
        let entity = self.get_entity();

        // A ridden or baby horse only responds to feeding.
        if entity.has_passengers() || self.is_baby() {
            return self.animal_interact(player, item_stack, Sound::EntityHorseAmbient);
        }

        // Secondary use (shift) on a tamed horse opens its inventory.
        if self.is_tame() && player.get_entity().is_sneaking() {
            self.open_custom_inventory_screen(player);
            return true;
        }

        let item = item_stack.get_item();
        if !item_stack.is_empty() {
            if self.is_food(item_stack) {
                return self.animal_interact(player, item_stack, Sound::EntityHorseAmbient);
            }
            // Any other item on an untamed horse just angers it.
            if !self.is_tame() {
                self.make_mad();
                return true;
            }
        }

        // Vanilla equips a held saddle (tamed horses only reach this point)
        // or equestrian body armor; anything else falls through to riding.
        if item == &Item::SADDLE && !self.is_saddled() {
            let mut saddle = item_stack.clone();
            saddle.item_count = 1;
            self.saddle_inventory.set_stack(0, saddle);
            item_stack.decrement_unless_creative(player.gamemode.load(), 1);
            self.sync_saddle_to_clients();
            let world = entity.world.load();
            world.play_sound(
                Sound::EntityHorseSaddle,
                SoundCategory::Neutral,
                &entity.pos.load(),
            );
            return true;
        }
        if item.registry_key.ends_with("_horse_armor")
            && self.armor_inventory.get_stack(0).is_empty()
        {
            let mut armor = item_stack.clone();
            armor.item_count = 1;
            self.armor_inventory.set_stack(0, armor);
            item_stack.decrement_unless_creative(player.gamemode.load(), 1);
            let world = entity.world.load();
            world.play_sound(
                Sound::EntityHorseArmor,
                SoundCategory::Neutral,
                &entity.pos.load(),
            );
            return true;
        }

        // Vanilla `doPlayerRide`: any non-baby horse can be ridden; an
        // untamed one throws the rider off via RunAroundLikeCrazyGoal.
        let world = player.world();
        if let Some(vehicle) = world.get_entity_by_id(entity.entity_id)
            && let Some(passenger) = world.get_player_by_id(player.entity_id())
        {
            entity.add_passenger(vehicle, passenger as Arc<dyn EntityBase>);
            return true;
        }

        false
    }
}
