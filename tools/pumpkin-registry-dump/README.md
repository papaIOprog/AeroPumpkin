# Pumpkin registry dump

A profile of a modded client for the Pumpkin server: every built-in registry
with its numeric ids, every block state with its properties, shapes and
light, the items, the entity types, the block and entity tags, and the
server config files of the pack. Pumpkin reads it as its picture of the
clients that join (`[mods] profile = "..."` in `pumpkin.toml`), so it can
name the mods' blocks and items by the ids the clients use.

Block tags only exist once a world's data packs are loaded, so both ways
run inside a world. Dump with exactly the mods the players join with; a
client with other mods has different ids.

## The mod

`mod/` is a small NeoForge 1.21.1 mod and needs no other mods.

1. Build it with `gradle build` in `mod/` (JDK 21). The jar lands in
   `mod/build/libs/pumpkin_registry_dump-<version>.jar`.
2. Drop the jar into `<instance>/mods/`. It registers no network channels,
   so the client still joins any server it joined before.
3. Start the game and open (or create) any singleplayer world once. The log
   says `wrote <instance>/pumpkin-registry.json`.
4. Copy the file next to Pumpkin's `pumpkin.toml` and point `[mods] profile`
   at it.

## The KubeJS script

`pumpkin_registry_dump.js` writes the same file for a pack that already has
KubeJS with Rhino: copy it to `<instance>/kubejs/client_scripts/`, join any
singleplayer world once, and find the file in
`<instance>/local/kubejs/export/pumpkin-registry.json`.
