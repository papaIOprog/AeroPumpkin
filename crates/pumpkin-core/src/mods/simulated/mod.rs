//! Pumpkin's port of Simulated, the physics half of Create: Aeronautics.
//!
//! Its physics assembler, its two ways of holding blocks together, its
//! handles and its creative physics staff. Simulated's other content is
//! what the profile makes of it: real to the clients, inert here.

pub mod handle;
pub mod honey_glue;
pub mod physics_assembler;
pub mod physics_staff;

use crate::item::registry::ItemRegistry;

/// The channel ids of Simulated's packets, `VeilPacketManager.create("simulated", "0.1")`.
/// `S:AssemblePacket`: the player pulled the assembler at a block all the way.
pub const ASSEMBLE: &str = "simulated:assemble";
/// `S:PhysicsAssemblerFlickAndHoldLeverPacket`: how the assembler's lever stands now.
pub const FLICK_ASSEMBLER_LEVER: &str = "simulated:flick_assembler_lever";
/// `S:PhysicsAssemblerFailedPacket`: the assembler could not do what was asked.
pub const ASSEMBLER_FAILED: &str = "simulated:assembler_failed";
/// `S:PhysicsStaffActionPacket`: a lock or the end of a drag.
pub const PHYSICS_STAFF_ACTION: &str = "simulated:physics_staff_action";
/// `S:PhysicsStaffDragPacket`: where the player drags a sub-level.
pub const PHYSICS_STAFF_DRAG: &str = "simulated:physics_staff_drag";
/// `S:PhysicsStaffBeamPacket`: the flash of a lock, for the others watching.
pub const PHYSICS_STAFF_BEAM: &str = "simulated:physics_staff_beam";
/// `S:PhysicsStaffLocksPacket`: the locked sub-levels of a dimension.
pub const PHYSICS_STAFF_LOCKS: &str = "simulated:physics_staff_locks";
/// `S:PhysicsStaffDragSessionsPacket`: who drags which sub-level, and where.
pub const PHYSICS_STAFF_DRAG_SESSIONS: &str = "simulated:physics_staff_drag_sessions";
/// `S:HoneyGlueSpawnPacket`: the player glued from one block to another.
pub const HONEY_GLUE_SPAWN: &str = "simulated:honey_glue_spawn";
/// `S:HoneyGlueChangeBoundsPacket`: a honey glue's box changed.
pub const HONEY_GLUE_CHANGE_BOUNDS: &str = "simulated:honey_glue_change_bounds";
/// `S:HoneyGlueSyncBoundsPacket`: where a honey glue is now.
pub const HONEY_GLUE_SYNC: &str = "simulated:honey_glue_sync";
/// `S:UpdatePlayerUsingHandlePacket`: a player holds a handle, or lets it go.
pub const UPDATE_PLAYER_HANDLE: &str = "simulated:update_player_handle";
/// `S:ClientboundPlayersHoldingHandlePacket`: who holds a handle.
pub const PLAYERS_HOLDING_HANDLES: &str = "simulated:players_holding_handles";

/// Registers the behaviours of Simulated's blocks.
pub fn register_blocks(blocks: &mut crate::block::registry::BlockRegistry) {
    blocks.register(physics_assembler::PhysicsAssemblerBlock);
    blocks.register(handle::HandleBlock);
}

/// Registers the behaviours of Simulated's items.
pub fn register_items(items: &mut ItemRegistry) {
    items.register(honey_glue::HoneyGlueItem);
    items.register(physics_staff::PhysicsStaffItem);
}
