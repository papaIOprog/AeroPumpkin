use std::any::Any;
use std::sync::Arc;

use crate::inventory::Inventory;
use crate::inventory::SimpleInventory;
use crate::slot::{NormalSlot, Slot};
use pumpkin_data::item::Item;
use pumpkin_data::item_stack::ItemStack;

use crate::player::player_inventory::PlayerInventory;
use crate::screen_handler::{InventoryPlayer, ScreenHandler, ScreenHandlerBehaviour};

/// What a mount's saddle or body-armor slot accepts.
///
/// Mirrors the vanilla per-entity overrides (`isSaddleable`,
/// `isBodyArmorItem`, `canUseSlot`). The client keeps both slot indices in
/// every mount menu even when it hides them, so the handler always has the
/// two slots.
/// The saddle and armor slot filters for one mount type.
#[derive(Clone, Copy)]
pub struct MountEquipFilters {
    pub saddle: MountEquipFilter,
    pub armor: MountEquipFilter,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum MountEquipFilter {
    /// `Items.SADDLE` on saddleable mounts.
    Saddle,
    /// Equestrian animal armor (horses).
    HorseArmor,
    /// Wool carpets, worn as decor (`Llama.isBodyArmorItem`).
    Carpet,
    /// The client hides this slot for this mount; accept nothing.
    None,
}

struct MountEquipSlot(NormalSlot, MountEquipFilter);

impl MountEquipSlot {
    fn new(inventory: Arc<dyn Inventory>, filter: MountEquipFilter) -> Self {
        Self(NormalSlot::new(inventory, 0), filter)
    }
}

impl Slot for MountEquipSlot {
    fn get_inventory(&self) -> Arc<dyn Inventory> {
        self.0.get_inventory()
    }

    fn get_index(&self) -> usize {
        self.0.get_index()
    }

    fn set_id(&self, id: usize) {
        self.0.set_id(id);
    }

    fn can_insert(&self, stack: &ItemStack) -> bool {
        match self.1 {
            MountEquipFilter::Saddle => stack.item == &Item::SADDLE,
            MountEquipFilter::HorseArmor => stack.item.registry_key.ends_with("_horse_armor"),
            MountEquipFilter::Carpet => stack.item.registry_key.ends_with("_carpet"),
            MountEquipFilter::None => false,
        }
    }

    fn mark_dirty(&self) {
        self.0.mark_dirty();
    }

    /// Mount equipment slots hold a single item.
    fn get_max_item_count(&self) -> u8 {
        1
    }
}

pub struct MountScreenHandler {
    behaviour: ScreenHandlerBehaviour,
    pub mount_inventory: Arc<dyn Inventory>,
    pub saddle_inventory: Arc<dyn Inventory>,
    pub armor_inventory: Arc<dyn Inventory>,
    pub inventory_columns: usize,
}

impl MountScreenHandler {
    pub const SLOT_SADDLE: usize = 0;
    pub const SLOT_BODY_ARMOR: usize = 1;
    pub const SLOT_INVENTORY_START: usize = 2;
    pub const INVENTORY_ROWS: usize = 3;

    #[must_use]
    pub const fn get_inventory_size(inventory_columns: usize) -> usize {
        inventory_columns * Self::INVENTORY_ROWS
    }

    pub fn new(
        sync_id: u8,
        player_inventory: &Arc<PlayerInventory>,
        mount_inventory: Arc<dyn Inventory>,
        saddle_inventory: Arc<dyn Inventory>,
        armor_inventory: Arc<dyn Inventory>,
        filters: MountEquipFilters,
        inventory_columns: usize,
    ) -> Self {
        let mut handler = Self {
            behaviour: ScreenHandlerBehaviour::new(sync_id, None),
            mount_inventory,
            saddle_inventory: saddle_inventory.clone(),
            armor_inventory: armor_inventory.clone(),
            inventory_columns,
        };

        handler.add_slot(Arc::new(MountEquipSlot::new(
            saddle_inventory,
            filters.saddle,
        )));
        handler.add_slot(Arc::new(MountEquipSlot::new(
            armor_inventory,
            filters.armor,
        )));

        let mount_size = handler.mount_inventory.size();
        for i in 0..mount_size {
            handler.add_slot(Arc::new(NormalSlot::new(
                handler.mount_inventory.clone(),
                i,
            )));
        }

        let pi: Arc<dyn Inventory> = player_inventory.clone();
        handler.add_player_slots(&pi);

        handler
    }
}

impl ScreenHandler for MountScreenHandler {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn get_behaviour(&self) -> &ScreenHandlerBehaviour {
        &self.behaviour
    }

    fn get_behaviour_mut(&mut self) -> &mut ScreenHandlerBehaviour {
        &mut self.behaviour
    }

    fn on_closed(&mut self, player: &dyn InventoryPlayer) {
        self.default_on_closed(player);
    }

    fn quick_move(&mut self, _player: &dyn InventoryPlayer, slot_index: i32) -> ItemStack {
        let mut clicked = ItemStack::EMPTY.clone();
        let slot = self.get_behaviour().slots.get(slot_index as usize).cloned();

        if let Some(slot) = slot {
            if !slot.has_stack() {
                return clicked;
            }

            let mut stack = slot.get_stack();
            clicked = stack.clone();

            let mount_container_size = self.mount_inventory.size();
            let player_container_start = 2 + mount_container_size as i32;
            let total_slots = self.get_behaviour().slots.len() as i32;

            if slot_index < player_container_start {
                if !self.insert_item(&mut stack, player_container_start, total_slots, true) {
                    return ItemStack::EMPTY.clone();
                }
            } else if self.get_behaviour().slots[1].can_insert(&stack)
                && !self.get_behaviour().slots[1].has_stack()
            {
                if !self.insert_item(&mut stack, 1, 2, false) {
                    return ItemStack::EMPTY.clone();
                }
            } else if self.get_behaviour().slots[0].can_insert(&stack)
                && !self.get_behaviour().slots[0].has_stack()
            {
                if !self.insert_item(&mut stack, 0, 1, false) {
                    return ItemStack::EMPTY.clone();
                }
            } else if mount_container_size == 0
                || !self.insert_item(&mut stack, 2, player_container_start, false)
            {
                let player_container_end = player_container_start + 27;
                let player_hotbar_start = player_container_end;
                let player_hotbar_end = player_hotbar_start + 9;

                if (player_hotbar_start..player_hotbar_end).contains(&slot_index) {
                    if !self.insert_item(
                        &mut stack,
                        player_container_start,
                        player_container_end,
                        false,
                    ) {
                        return ItemStack::EMPTY.clone();
                    }
                } else if (player_container_start..player_container_end).contains(&slot_index) {
                    if !self.insert_item(&mut stack, player_hotbar_start, player_hotbar_end, false)
                    {
                        return ItemStack::EMPTY.clone();
                    }
                } else if !self.insert_item(
                    &mut stack,
                    player_hotbar_start,
                    player_container_end,
                    false,
                ) {
                    return ItemStack::EMPTY.clone();
                }

                return ItemStack::EMPTY.clone();
            }

            if stack.is_empty() {
                slot.set_stack(ItemStack::EMPTY.clone());
            } else {
                slot.set_stack(stack);
            }
        }

        clicked
    }
}

/// Vanilla `HorseInventoryMenu` for a horse (no chest rows): a saddle slot,
/// a body armor slot and the player inventory. Opened through the mount
/// screen open packet, not a window type.
pub struct HorseScreenHandler {
    pub mount_handler: MountScreenHandler,
}

impl HorseScreenHandler {
    pub fn new(
        sync_id: u8,
        player_inventory: &Arc<PlayerInventory>,
        saddle_inventory: Arc<dyn Inventory>,
        armor_inventory: Arc<dyn Inventory>,
    ) -> Self {
        Self {
            mount_handler: MountScreenHandler::new(
                sync_id,
                player_inventory,
                // Horses carry no chest, so the mount inventory stays empty.
                Arc::new(SimpleInventory::new(0)),
                saddle_inventory,
                armor_inventory,
                MountEquipFilters {
                    saddle: MountEquipFilter::Saddle,
                    armor: MountEquipFilter::HorseArmor,
                },
                0,
            ),
        }
    }
}

impl ScreenHandler for HorseScreenHandler {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn get_behaviour(&self) -> &ScreenHandlerBehaviour {
        self.mount_handler.get_behaviour()
    }

    fn get_behaviour_mut(&mut self) -> &mut ScreenHandlerBehaviour {
        self.mount_handler.get_behaviour_mut()
    }

    fn on_closed(&mut self, player: &dyn InventoryPlayer) {
        self.mount_handler.on_closed(player);
    }

    fn quick_move(&mut self, player: &dyn InventoryPlayer, slot_index: i32) -> ItemStack {
        self.mount_handler.quick_move(player, slot_index)
    }
}

pub struct NautilusInventoryScreenHandler {
    pub mount_handler: MountScreenHandler,
}

impl NautilusInventoryScreenHandler {
    pub fn new(
        sync_id: u8,
        player_inventory: &Arc<PlayerInventory>,
        nautilus_inventory: Arc<dyn Inventory>,
        saddle_inventory: Arc<dyn Inventory>,
        armor_inventory: Arc<dyn Inventory>,
        inventory_columns: usize,
    ) -> Self {
        Self {
            mount_handler: MountScreenHandler::new(
                sync_id,
                player_inventory,
                nautilus_inventory,
                saddle_inventory,
                armor_inventory,
                MountEquipFilters {
                    saddle: MountEquipFilter::Saddle,
                    armor: MountEquipFilter::Carpet,
                },
                inventory_columns,
            ),
        }
    }
}

impl ScreenHandler for NautilusInventoryScreenHandler {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn get_behaviour(&self) -> &ScreenHandlerBehaviour {
        self.mount_handler.get_behaviour()
    }

    fn get_behaviour_mut(&mut self) -> &mut ScreenHandlerBehaviour {
        self.mount_handler.get_behaviour_mut()
    }

    fn on_closed(&mut self, player: &dyn InventoryPlayer) {
        self.mount_handler.on_closed(player);
    }

    fn quick_move(&mut self, player: &dyn InventoryPlayer, slot_index: i32) -> ItemStack {
        self.mount_handler.quick_move(player, slot_index)
    }
}
