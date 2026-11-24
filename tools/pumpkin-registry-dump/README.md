# Pumpkin registry dump

A KubeJS 7 client script for a NeoForge 1.21.1 client. It writes
`<game directory>/local/kubejs/export/pumpkin-registry.json`: every built-in
registry with its numeric ids, every block state with its properties, shapes
and light, the items, the entity types, the block and entity tags, and the
server config files of the pack.

The Pumpkin server reads that file as its profile of the clients that join
(`[mods] profile = "..."` in `pumpkin.toml`), so it can name the mods' blocks
and items by the ids the clients use.

## Use

1. Install KubeJS and Rhino in the instance (or use a pack that has them).
2. Copy `pumpkin_registry_dump.js` to `<instance>/kubejs/client_scripts/`.
3. Launch the game with exactly the mods the players will join with, join any
   singleplayer world once and quit. The log says
   `wrote local/kubejs/export/pumpkin-registry.json`. Tags only exist once a
   world's data packs are loaded, which is why the script waits for one.
4. Copy the file next to Pumpkin's `pumpkin.toml` and point `[mods] profile` at it.

A client with other mods than the dump has different ids; dump again for it.
