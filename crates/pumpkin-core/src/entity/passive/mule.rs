use std::sync::{
    Arc, Weak,
    atomic::{AtomicBool, AtomicI32, AtomicU8, Ordering},
};

use crossbeam::atomic::AtomicCell;
use pumpkin_data::entity::EntityType;
use pumpkin_data::item::Item;
use pumpkin_data::item_stack::ItemStack;
use pumpkin_data::sound::{Sound, SoundCategory};
use pumpkin_data::tag::{self, Taggable};
use pumpkin_inventory::mount_screen_handler::{MountEquipFilter, MountEquipFilters, MountScreenHandler};
use pumpkin_inventory::screen_handler::{
    InventoryPlayer, ScreenHandlerFactory, SharedScreenHandler,
};
use pumpkin_inventory::{Inventory, SimpleInventory};
use pumpkin_nbt::compound::NbtCompound;
use pumpkin_nbt::tag::NbtTag;
use pumpkin_util::text::TextComponent;
use uuid::Uuid;

use crate::entity::{
    Entity, EntityBase,
    ageable::{AgeableData, AgeableMob},
    ai::goal::{
        escape_danger::EscapeDangerGoal,
        follow_parent::FollowParentGoal,
        look_around::RandomLookAroundGoal,
        look_at_entity::LookAtEntityGoal,
        run_around_like_crazy::{RunAroundLikeCrazyGoal, TemperamentalMount},
        swim::SwimGoal,
        tempt::TemptGoal,
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

pub struct MuleEntity {
    pub mob_entity: MobEntity,
    pub ageable_data: AgeableData,
    pub flags: AtomicU8,
    pub has_chest: AtomicBool,
    pub temper: AtomicI32,
    pub owner: AtomicCell<Option<Uuid>>,
    /// The saddle lives in the first slot of the mount screen; mules carry a
    /// chest of 15 slots when one is attached and never wear body armor.
    pub saddle_inventory: Arc<SimpleInventory>,
    pub chest_inventory: Arc<SimpleInventory>,
}

impl MuleEntity {
    pub fn new(entity: Entity) -> Arc<Self> {
        let mob_entity = MobEntity::new(entity);
        let mule = Self {
            mob_entity,
            ageable_data: AgeableData::default(),
            flags: AtomicU8::new(0),
            has_chest: AtomicBool::new(false),
            temper: AtomicI32::new(0),
            owner: AtomicCell::new(None),
            saddle_inventory: Arc::new(SimpleInventory::new(1)),
            chest_inventory: Arc::new(SimpleInventory::new(15)),
        };
        let mob_arc = Arc::new(mule);
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
    pub fn has_flag(&self, flag: u8) -> bool {
        (self.flags.load(Ordering::Relaxed) & flag) != 0
    }

    pub fn set_flag(&self, flag: u8, val: bool) {
        let current = self.flags.load(Ordering::Relaxed);
        let new_flags = if val { current | flag } else { current & !flag };
        self.flags.store(new_flags, Ordering::Relaxed);
        let entity = self.get_entity();
        entity.set_synced_data(
            pumpkin_data::tracked_data::mule::DATA_ID_FLAGS,
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

    #[must_use]
    pub fn has_chest(&self) -> bool {
        self.has_chest.load(Ordering::Relaxed)
    }

    pub fn set_has_chest(&self, val: bool) {
        self.has_chest.store(val, Ordering::Relaxed);
        let entity = self.get_entity();
        entity.set_synced_data(pumpkin_data::tracked_data::mule::CHEST, val);
    }

    /// Vanilla `AbstractHorse.syncSaddleToClients`: the saddle flag mirrors
    /// the saddle slot. Called every tick because Pumpkin inventories have no
    /// change listener; `set_synced_data` only marks dirty on change.
    pub fn sync_saddle_to_clients(&self) {
        let saddled = !self.saddle_inventory.get_stack(0).is_empty();
        self.set_saddled(saddled);
    }
}

impl ScreenHandlerFactory for MuleEntity {
    fn create_screen_handler(
        &self,
        sync_id: u8,
        player_inventory: &Arc<pumpkin_inventory::player::player_inventory::PlayerInventory>,
        _player: &dyn InventoryPlayer,
    ) -> Option<SharedScreenHandler> {
        // The slot list must match the client's menu, which draws chest slots
        // only for a chested mule.
        let chest: Arc<dyn Inventory> = if self.has_chest() {
            self.chest_inventory.clone()
        } else {
            Arc::new(SimpleInventory::new(0))
        };
        let columns = if self.has_chest() { 5 } else { 0 };
        let handler = MountScreenHandler::new(
            sync_id,
            player_inventory,
            chest,
            self.saddle_inventory.clone(),
            Arc::new(SimpleInventory::new(1)),
            MountEquipFilters {
                saddle: MountEquipFilter::Saddle,
                armor: MountEquipFilter::None,
            },
            columns,
        );
        Some(Arc::new(std::sync::Mutex::new(handler)) as SharedScreenHandler)
    }

    fn get_display_name(&self) -> TextComponent {
        TextComponent::translate("container.horse", vec![])
    }
}

impl crate::entity::ai::goal::run_around_like_crazy::TemperamentalMount for MuleEntity {
    fn is_tame(&self) -> bool {
        self.has_flag(FLAG_TAME)
    }

    fn set_tame(&self, tame: bool) {
        self.set_flag(FLAG_TAME, tame);
    }

    fn max_temper(&self) -> i32 {
        // Vanilla `AbstractHorse.getMaxTemper` base.
        100
    }

    fn temper(&self) -> i32 {
        self.temper.load(Ordering::Relaxed)
    }

    fn set_temper(&self, temper: i32) {
        self.temper.store(temper, Ordering::Relaxed);
    }

    fn set_owner(&self, owner: Uuid) {
        self.owner.store(Some(owner));
    }

    fn make_mad(&self) {
        let entity = self.get_entity();
        entity.world.load().play_sound(
            pumpkin_data::sound::Sound::EntityMuleAngry,
            pumpkin_data::sound::SoundCategory::Neutral,
            &entity.pos.load(),
        );
    }
}

impl AgeableMob for MuleEntity {
    fn get_ageable_data(&self) -> &AgeableData {
        &self.ageable_data
    }
}

impl Animal for MuleEntity {
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

impl Mob for MuleEntity {
    fn as_ageable(&self) -> Option<&dyn AgeableMob> {
        Some(self)
    }

    fn as_animal(&self) -> Option<&dyn Animal> {
        Some(self)
    }

    /// Vanilla `AbstractHorse.openCustomInventoryScreen`.
    fn open_custom_inventory_screen(&self, player: &Arc<Player>) {
        let entity = self.get_entity();
        let opened_by_rider = entity
            .passengers
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .iter()
            .any(|passenger| passenger.get_entity().entity_uuid == player.get_entity().entity_uuid);
        if (!entity.has_passengers() || opened_by_rider) && self.is_tame() {
            let columns = i32::from(self.has_chest()) * 5;
            player.open_horse_inventory(self, entity.entity_id, columns);
        }
    }

    fn mob_write_nbt(&self, nbt: &mut NbtCompound) {
        self.write_ageable_nbt(nbt);
        nbt.put_bool("ChestedHorse", self.has_chest());
        nbt.put_bool("Tame", self.is_tame());
        nbt.put_int("Temper", self.temper.load(Ordering::Relaxed));
        if let Some(owner) = self.owner.load() {
            nbt.put_uuid("Owner", owner);
        }
        let saddle = self.saddle_inventory.get_stack(0);
        let mut items = Vec::new();
        if !saddle.is_empty() {
            let mut saddle_nbt = NbtCompound::new();
            saddle_nbt.put_byte("Slot", 0);
            saddle.write_item_stack(&mut saddle_nbt);
            items.push(NbtTag::Compound(saddle_nbt));
        }
        if self.has_chest() {
            for (slot, stack) in (1..=self.chest_inventory.size())
                .map(|i| (i, self.chest_inventory.get_stack(i - 1)))
                .filter(|(_, stack)| !stack.is_empty())
            {
                let mut item_nbt = NbtCompound::new();
                item_nbt.put_byte("Slot", slot as i8);
                stack.write_item_stack(&mut item_nbt);
                items.push(NbtTag::Compound(item_nbt));
            }
        }
        if !items.is_empty() {
            nbt.put_list("Items", items);
        }
    }

    fn mob_read_nbt(&self, nbt: &NbtCompound) {
        self.read_ageable_nbt(nbt);
        if let Some(chested) = nbt.get_bool("ChestedHorse") {
            self.set_has_chest(chested);
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
        // Vanilla stores the saddle and chest in one Items list, saddle at
        // slot 0 and the chest contents behind it.
        if let Some(items) = nbt.get_list("Items") {
            for tag in items {
                if let NbtTag::Compound(item_nbt) = tag
                    && let Some(stack) = ItemStack::read_item_stack(item_nbt)
                {
                    match item_nbt.get_byte("Slot") {
                        Some(0) if stack.item == &Item::SADDLE => {
                            self.saddle_inventory.set_stack(0, stack);
                        }
                        Some(slot) if slot >= 1 => {
                            let index = (slot - 1) as usize;
                            if index < self.chest_inventory.size() {
                                self.chest_inventory.set_stack(index, stack);
                            }
                        }
                        _ => {}
                    }
                }
            }
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
            entity.set_synced_data(pumpkin_data::tracked_data::mule::DATA_BABY_ID, true);
        }
        entity.set_synced_data(pumpkin_data::tracked_data::mule::CHEST, self.has_chest());
        entity.set_synced_data(
            pumpkin_data::tracked_data::mule::DATA_ID_FLAGS,
            self.flags.load(Ordering::Relaxed) as i8,
        );
    }

    fn mob_interact(&self, player: &Arc<Player>, item_stack: &mut ItemStack) -> bool {
        // Vanilla `AbstractChestedHorse.mobInteract` on top of the
        // `AbstractHorse.mobInteract` order.
        let entity = self.get_entity();

        // A ridden or baby donkey only responds to feeding.
        if entity.has_passengers() || self.is_baby() {
            return self.animal_interact(player, item_stack, Sound::EntityMuleAmbient);
        }

        // Secondary use (shift) on a tamed donkey opens its inventory.
        if self.is_tame() && player.get_entity().is_sneaking() {
            self.open_custom_inventory_screen(player);
            return true;
        }

        let item = item_stack.get_item();
        if !item_stack.is_empty() {
            if self.is_food(item_stack) {
                return self.animal_interact(player, item_stack, Sound::EntityMuleAmbient);
            }
            // Any other item on an untamed donkey just angers it.
            if !self.is_tame() {
                self.make_mad();
                return true;
            }

            // Vanilla `equipChest`: attaching a chest grows the inventory.
            if item == &Item::CHEST && !self.has_chest() {
                self.set_has_chest(true);
                item_stack.decrement_unless_creative(player.gamemode.load(), 1);
                let world = entity.world.load();
                world.play_sound(
                    Sound::EntityDonkeyChest,
                    SoundCategory::Neutral,
                    &entity.pos.load(),
                );
                return true;
            }
        }

        // Vanilla saddles through the horse inventory's first slot; donkeys
        // never wear body armor.
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

        // Vanilla `doPlayerRide`.
        let world = player.world();
        if player.get_entity().can_ride()
            && let Some(vehicle) = world.get_entity_by_id(entity.entity_id)
            && let Some(passenger) = world.get_player_by_id(player.entity_id())
        {
            entity.add_passenger(vehicle, passenger as Arc<dyn EntityBase>);
            return true;
        }

        false
    }
}
