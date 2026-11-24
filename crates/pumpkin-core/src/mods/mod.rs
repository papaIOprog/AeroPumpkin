//! Modded content. A pack profile, built once at startup from a registry
//! dump of the clients that join, tells the registries what those clients
//! add to vanilla.
//!
//! The switches in the configuration say which of Pumpkin's ports of those
//! mods run.

pub mod profile;

use std::path::Path;
use std::sync::OnceLock;

use pumpkin_config::ModsConfig;
use tracing::{error, info, warn};

/// The mods the installed profile shows content of, by namespace.
static NAMESPACES: OnceLock<Box<[String]>> = OnceLock::new();

/// Installs the pack profile the configuration names, if any. It runs
/// before the first registry is built, so that everything after it sees
/// the mods' content.
pub fn install(config: &ModsConfig) {
    let Some(name) = config.profile.as_deref() else {
        info!("Mods: no profile is configured, so the server knows vanilla content only");
        return;
    };
    let path = Path::new(name);
    let (modded, namespaces) = match profile::load(path) {
        Ok(loaded) => loaded,
        Err(error) => {
            error!(
                "Mods: the profile {} was refused: {error}. The server knows vanilla content only, so modded clients will not find their blocks here",
                path.display()
            );
            return;
        }
    };
    let blocks = modded.blocks.by_id.len();
    let states = modded.blocks.states.len();
    let items = modded.items.by_id.iter().flatten().count();
    let entities = modded.entities.by_id.iter().flatten().count();
    let switched: Vec<&str> = namespaces
        .iter()
        .map(String::as_str)
        .filter(|namespace| config.enabled(namespace))
        .collect();
    if pumpkin_data::mods::install(modded).is_err() {
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
    let _ = NAMESPACES.set(namespaces);
}

/// Whether the installed profile shows content of the mod `namespace`.
#[must_use]
pub fn present(namespace: &str) -> bool {
    NAMESPACES
        .get()
        .is_some_and(|namespaces| namespaces.iter().any(|known| known == namespace))
}

/// Whether Pumpkin's port of the mod `namespace` runs: the profile shows
/// its content, and the configuration switches it on.
#[must_use]
pub fn active(config: &ModsConfig, namespace: &str) -> bool {
    present(namespace) && config.enabled(namespace)
}
