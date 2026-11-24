use std::collections::HashMap;

use serde::{Deserialize, Serialize};

/// Modded content: which mods the server mirrors and what it knows of them.
#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[serde(default)]
pub struct ModsConfig {
    /// A registry dump of the clients that join, written by Pumpkin's dump
    /// script (`tools` in the Pumpkin repository), relative to the server
    /// directory. Without one the server knows vanilla content only.
    ///
    /// Example:
    ///
    /// ```toml
    /// [mods]
    /// profile = "mods/slim-1.21.1.json"
    ///
    /// [mods.simulated]
    /// enabled = true
    ///
    /// [mods.create]
    /// enabled = false
    /// ```
    pub profile: Option<String>,
    /// The mods whose behaviour Pumpkin implements, each switched on or
    /// off by mod id. A mod nobody mentions is on; its blocks and items are
    /// still known from the profile when it is off, only without behaviour.
    pub mods: HashMap<String, ModConfig>,
}

impl ModsConfig {
    /// Whether Pumpkin's port of the mod `id` is switched on.
    #[must_use]
    pub fn enabled(&self, id: &str) -> bool {
        self.mods.get(id).is_none_or(|switch| switch.enabled)
    }
}

/// The switch of one mod.
#[derive(Deserialize, Serialize, Clone, Debug)]
#[serde(default)]
pub struct ModConfig {
    /// Whether Pumpkin's port of the mod runs.
    pub enabled: bool,
}

impl Default for ModConfig {
    fn default() -> Self {
        Self { enabled: true }
    }
}
