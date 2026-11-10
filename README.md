<div align="center">

# AeroPumpkin

[![License: GPL](https://img.shields.io/badge/License-GPLv3-yellow.svg)](https://opensource.org/licenses/gpl-3-0)
![Minecraft](https://img.shields.io/badge/Minecraft-1.21.1-62b47a)

**A Minecraft server written in Rust, for Java Edition 1.21.1 and the Create mod ecosystem.**
A fork of [Pumpkin](https://github.com/Pumpkin-MC/Pumpkin).

</div>

<div align="center">

![Pumpkin Chunk Loading](./assets/pumpkin-chunk-loading.webp)

</div>

AeroPumpkin inherits everything that makes [Pumpkin](https://pumpkinmc.org/) fast: the multi-threaded tick, async networking, Rayon-backed chunk generation and lighting, and the plugin API. It points all of that at a different target. Upstream Pumpkin tracks the newest Minecraft version, while AeroPumpkin stays on **1.21.1 (protocol 767)**, the version the Create mod ecosystem settled on, and aims to support Create and its addons on top of a fully working vanilla base.

## Goals

- **Vanilla parity on 1.21.1**: behave like the official 1.21.1 server, with the same registries, wire formats and tick behaviour. Ports are checked against the decompiled vanilla source, and world saves load cleanly in the official server.
- **Create ecosystem support**: let clients running Create and its addon ecosystem join and play. This starts with modded-protocol compatibility and grows into native server-side implementations of Create's gameplay.
- **Keep Pumpkin's edge**: performance, configurability, and the Wasm and native plugin APIs.

> [!IMPORTANT]
> AeroPumpkin is under heavy development. The vanilla 1.21.1 base is playable end to end today; Create support is the next mountain.

## Status

What works today, verified with real 1.21.1 clients and headless bots:

- [x] Full 1.21.1 Java protocol: status/ping, encryption, compression
- [x] World generation, loading and saving in all three dimensions (Vanilla, Linear and Pump region formats)
- [x] World saves that the official 1.21.1 server loads cleanly
- [x] Lighting, liquid physics, scheduled ticks and redstone basics that survive chunk unloads and reloads
- [x] Survival gameplay: movement, inventories, containers, combat, hunger, experience, eating, off-hand, enchanting
- [x] Recipe book sync and recipe-unlock advancements
- [x] Villagers and wandering traders with working trades
- [x] Mobs and animals (W.I.P towards full parity), including taming and mount inventories for horses, donkeys and llamas
- [x] Bosses (W.I.P), entity effects, entity saving
- [x] Advancements (W.I.P)
- [x] Tab-list, scoreboard, bossbar
- [x] Plugins (Wasm and native), commands, permissions, translations
- [x] Query and [Minecraft Server Management Protocol (MSMP)](https://minecraft.wiki/w/Minecraft_Server_Management_Protocol)
- [x] [BungeeCord](https://github.com/SpigotMC/BungeeCord), [BungeeGuard](https://github.com/lucko/BungeeGuard) and [Velocity](https://github.com/PaperMC/Velocity) proxies
- [ ] Create mod ecosystem support

What is left, roughly in order: finishing mob parity (natural spawning in the End, the dragon fight), a few command-parser edge cases, and the Create compatibility layer itself. See the issue tracker for progress.

## Running

AeroPumpkin is built from source. You need a recent stable Rust toolchain. The server writes its configuration and world files into the working directory, so start it from its own folder:

```bash
mkdir server && cd server
cargo run --manifest-path /path/to/AeroPumpkin/Cargo.toml --release -p pumpkin
```

The binary and crate names are still `pumpkin`. Once the log shows the "done" line, connect with a 1.21.1 client on port 25565.

## Upstream

AeroPumpkin is a fork of [Pumpkin](https://github.com/Pumpkin-MC/Pumpkin) and follows it closely. Fixes that are not specific to this fork's goals are intended to go upstream.

- **Upstream source**: <https://github.com/Pumpkin-MC/Pumpkin>
- **Documentation**: <https://pumpkinmc.org/> (written for upstream, which tracks newer Minecraft versions, so details may differ here)
- **Community**: [Pumpkin's Discord](https://discord.gg/wT8XjrjKkf) for everything Pumpkin; this fork's issue tracker for AeroPumpkin-specific bugs and progress

Contributions are welcome! For code contributions, read [CONTRIBUTING.md](CONTRIBUTING.md) and [AGENTS.md](AGENTS.md) first.

## License & Attribution

* **AeroPumpkin** (like the Pumpkin server it forks): Licensed under the [GNU General Public License v3.0 (GPLv3)](LICENSE).
* **Plugin API (`pumpkin-plugin-api` & `pumpkin-plugin-wit`)**: Dual-licensed under [MIT](crates/pumpkin-plugin-api/LICENSE-MIT) OR [Apache-2.0](crates/pumpkin-plugin-api/LICENSE-APACHE) for maximum flexibility when writing plugins.
* **Third-Party Assets & Data**: Bedrock mappings, protocol conversion data, and Minecraft assets are subject to their respective licenses and attribution terms. See [assets/NOTICE.md](assets/NOTICE.md) for full details.
