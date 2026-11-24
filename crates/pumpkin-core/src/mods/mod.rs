//! Modded content. A pack profile, built once at startup from a registry
//! dump of the clients that join, tells the registries what those clients
//! add to vanilla.
//!
//! The switches in the configuration say which of Pumpkin's ports of those
//! mods run; each port lives in its own module here.

pub mod create;
pub mod glue;
pub mod packets;
pub mod profile;
pub mod simulated;

use std::path::Path;
use std::sync::OnceLock;

use pumpkin_config::ModsConfig;
use pumpkin_data::{Block, BlockId, item::Item};
use tracing::{error, info, warn};

/// The installed profile, with what was installed with it.
struct Installed {
    /// The mods the profile shows content of, by namespace.
    namespaces: Box<[String]>,
    config: ModsConfig,
    /// The server config files the pack's clients expect, by file name.
    server_configs: Box<[String]>,
}

static INSTALLED: OnceLock<Installed> = OnceLock::new();

/// Installs the pack profile the configuration names, if any. It runs
/// before the first registry is built, so that everything after it sees
/// the mods' content.
pub fn install(config: &ModsConfig) {
    let Some(name) = config.profile.as_deref() else {
        info!("Mods: no profile is configured, so the server knows vanilla content only");
        return;
    };
    let path = Path::new(name);
    let profile = match profile::load(path) {
        Ok(profile) => profile,
        Err(error) => {
            error!(
                "Mods: the profile {} was refused: {error}. The server knows vanilla content only, so modded clients will not find their blocks here",
                path.display()
            );
            return;
        }
    };
    let blocks = profile.modded.blocks.by_id.len();
    let states = profile.modded.blocks.states.len();
    if profile.unfitted > 0 {
        warn!(
            "Mods: {} blocks of the profile have more states than a world can hold ({} in all) and stay unknown to the server",
            profile.unfitted, states
        );
    }
    let items = profile.modded.items.by_id.iter().flatten().count();
    let entities = profile.modded.entities.by_id.iter().flatten().count();
    let switched: Vec<&str> = profile
        .namespaces
        .iter()
        .map(String::as_str)
        .filter(|namespace| config.enabled(namespace))
        .collect();
    if pumpkin_data::mods::install(profile.modded).is_err() {
        warn!(
            "Mods: a profile is installed already; {} was not read",
            path.display()
        );
        return;
    }
    let switched = if switched.is_empty() {
        "none of their ports".to_owned()
    } else {
        switched.join(", ")
    };
    info!(
        "Mods: {blocks} blocks ({states} states), {items} items and {entities} entity types from {}; on: {switched}",
        path.display()
    );
    let _ = INSTALLED.set(Installed {
        namespaces: profile.namespaces,
        config: config.clone(),
        server_configs: profile.server_configs,
    });
}

/// Registers the behaviours of the mods' blocks whose ports are
/// switched on; called as the block registry is built.
pub fn register_blocks(blocks: &mut crate::block::registry::BlockRegistry) {
    simulated::register_blocks(blocks);
}

/// Registers the behaviours of the mods' items whose ports are switched on;
/// called as the item registry is built.
pub fn register_items(items: &mut crate::item::registry::ItemRegistry) {
    create::register_items(items);
    simulated::register_items(items);
}

/// Whether the installed profile shows content of the mod `namespace`.
#[must_use]
pub fn present(namespace: &str) -> bool {
    INSTALLED
        .get()
        .is_some_and(|installed| installed.namespaces.iter().any(|known| known == namespace))
}

/// Whether Pumpkin's port of the mod `namespace` runs: the profile shows
/// its content, and the configuration switches it on.
#[must_use]
pub fn active(namespace: &str) -> bool {
    INSTALLED
        .get()
        .is_some_and(|installed| installed.config.enabled(namespace))
        && present(namespace)
}

/// The ids of the blocks `names` (`namespace:path`) for a port of the mod
/// `namespace` to be registered under; none when the port is switched off,
/// so that the blocks stay inert.
#[must_use]
pub fn ported_blocks(namespace: &str, names: &[&str]) -> Box<[BlockId]> {
    if !active(namespace) {
        return Vec::new().into_boxed_slice();
    }
    names
        .iter()
        .filter_map(|name| Block::from_registry_key(name))
        .map(|block| block.id)
        .collect()
}

/// The ids of the items `names` for a port of the mod `namespace`; none
/// when the port is switched off.
#[must_use]
pub fn ported_items(namespace: &str, names: &[&str]) -> Box<[u16]> {
    if !active(namespace) {
        return Vec::new().into_boxed_slice();
    }
    names
        .iter()
        .filter_map(|name| Item::from_registry_key(name))
        .map(|item| item.id)
        .collect()
}

/// The server config files to send a modded client: those of the loader and
/// of Sable,
/// then those of the installed profile's pack. A client only loads a mod's
/// server config once the server sends its file.
#[must_use]
pub fn server_config_files() -> Vec<&'static str> {
    let mut files: Vec<&'static str> = crate::net::java::neoforge::SERVER_CONFIG_FILES.to_vec();
    if let Some(installed) = INSTALLED.get() {
        let known: Box<[&str]> = installed
            .server_configs
            .iter()
            .map(String::as_str)
            .filter(|file| !files.contains(file))
            .collect();
        files.extend(known);
    }
    files
}
