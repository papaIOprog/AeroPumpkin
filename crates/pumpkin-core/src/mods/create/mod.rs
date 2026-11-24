//! Pumpkin's port of Create's pieces the sub-levels use. Create's other
//! content is what the profile makes of it: real to the clients, inert
//! here.

pub mod super_glue;

use crate::item::registry::ItemRegistry;

/// Create's `AllPackets.GLUE_IN_AREA`: where the player glued.
pub const GLUE_IN_AREA: &str = "create:glue_in_area";
/// Create's `AllPackets.GLUE_REMOVED`: the glue the player broke.
pub const GLUE_REMOVED: &str = "create:glue_removed";

/// Registers the behaviours of Create's items.
pub fn register_items(items: &mut ItemRegistry) {
    items.register(super_glue::SuperGlueItem);
}
