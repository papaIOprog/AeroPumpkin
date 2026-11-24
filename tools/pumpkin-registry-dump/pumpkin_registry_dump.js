// Pumpkin registry dump, a KubeJS 7 client script (NeoForge 1.21.1).
//
// Writes <game directory>/local/kubejs/export/pumpkin-registry.json with every
// built-in registry's entries and numeric ids, every block state with its
// properties, shapes and light, every item's basics, every entity type's
// dimensions, the block and entity tags, and the server config files the mods
// expect. The Pumpkin server loads it as its picture of this client: dump it
// with exactly the mods the players join with.
//
// Where: <instance>/kubejs/client_scripts/pumpkin_registry_dump.js
// How:   launch the game, join any singleplayer world once (tags only exist
//        once a world's data packs are loaded) and quit.

const safe = fn => {
  try {
    return fn()
  } catch (e) {
    return null
  }
}

const aabbs = shape => {
  const out = []
  for (const box of Java.from(shape.toAabbs())) {
    out.push([box.minX, box.minY, box.minZ, box.maxX, box.maxY, box.maxZ])
  }
  return out
}

ClientEvents.loggedIn(event => {
  if (global.pumpkinRegistryDumped) {
    return
  }
  global.pumpkinRegistryDumped = true
  const started = Date.now()
  try {
    const BuiltInRegistries = Java.loadClass('net.minecraft.core.registries.BuiltInRegistries')
    const Block = Java.loadClass('net.minecraft.world.level.block.Block')
    const EmptyBlockGetter = Java.loadClass('net.minecraft.world.level.EmptyBlockGetter')
    const BlockPos = Java.loadClass('net.minecraft.core.BlockPos')
    const Direction = Java.loadClass('net.minecraft.core.Direction')
    const Blocks = Java.loadClass('net.minecraft.world.level.block.Blocks')
    const SupportType = Java.loadClass('net.minecraft.world.level.block.SupportType')
    const DataComponents = Java.loadClass('net.minecraft.core.component.DataComponents')
    const SharedConstants = Java.loadClass('net.minecraft.SharedConstants')
    const ConfigSync = safe(() => Java.loadClass('net.neoforged.neoforge.network.ConfigSync'))
    const VersionInfo = safe(() => Java.loadClass('net.neoforged.neoforge.VersionInfo'))
    const JsonIO = Java.loadClass('dev.latvian.mods.kubejs.util.JsonIO')

    const dump = {
      format: 2,
      game: {
        version: SharedConstants.getCurrentVersion().getId(),
        neoforge: VersionInfo === null ? null : safe(() => String(VersionInfo.getVersion())),
      },
      registries: {},
      blocks: {},
      items: {},
      entities: {},
      tags: {},
      serverConfigs: [],
    }

    // Every built-in registry, its entries and ids.
    for (const entry of Java.from(BuiltInRegistries.REGISTRY.entrySet())) {
      const name = String(entry.getKey().location())
      const registry = entry.getValue()
      const entries = []
      for (const held of Java.from(registry.entrySet())) {
        entries.push([registry.getId(held.getValue()), String(held.getKey().location())])
      }
      entries.sort((a, b) => a[0] - b[0])
      dump.registries[name] = {
        sync: safe(() => registry.doesSync()),
        entries: entries,
      }
    }
    console.info('[pumpkin-dump] registries listed')

    // Every block state, in state id order.
    const nothing = EmptyBlockGetter.INSTANCE
    const origin = BlockPos.ZERO
    const sides = [Direction.DOWN, Direction.UP, Direction.NORTH, Direction.SOUTH, Direction.WEST, Direction.EAST]
    const states = Block.BLOCK_STATE_REGISTRY
    const stateCount = states.size()
    const blockOf = state => String(BuiltInRegistries.BLOCK.getKey(state.getBlock()).location())
    for (let id = 0; id < stateCount; id++) {
      const state = states.byId(id)
      const name = blockOf(state)
      let block = dump.blocks[name]
      if (!block) {
        const owner = state.getBlock()
        block = dump.blocks[name] = {
          base: id,
          // The state a fresh placement has, and the block entity it holds.
          default: states.getId(owner.defaultBlockState()),
          blockEntity: safe(() => String(BuiltInRegistries.BLOCK_ENTITY_TYPE.getKey(owner.newBlockEntity(origin, state).getType()).location())),
          resistance: owner.getExplosionResistance(),
          states: [],
        }
      }
      const props = {}
      for (const value of Java.from(state.getValues().entrySet())) {
        const key = String(value.getKey().getName())
        let name = String(value.getKey().getName(value.getValue()))
        if (name.startsWith(key + '=')) {
          name = name.substring(key.length + 1)
        }
        props[key] = name
      }
      const sound = state.getSoundType()
      const sturdy = []
      const skip = []
      for (const side of sides) {
        sturdy.push(state.isFaceSturdy(nothing, origin, side))
        skip.push(state.skipRendering(nothing, origin, side))
      }
      block.states.push({
        props: props,
        shape: aabbs(state.getCollisionShape(nothing, origin)),
        outline: aabbs(state.getShape(nothing, origin)),
        light: state.getLightEmission(),
        opacity: state.getLightBlock(nothing, origin),
        sky: state.propagatesSkylightDown(nothing, origin),
        solidRender: state.isSolidRender(nothing, origin),
        solid: state.isSolid(),
        fullCube: state.isCollisionShapeFullBlock(nothing, origin),
        canOcclude: state.canOcclude(),
        toolRequired: state.requiresCorrectToolForDrops(),
        analog: state.hasAnalogOutputSignal(),
        centerSturdy: [
          state.isFaceSturdy(nothing, origin, Direction.DOWN, SupportType.CENTER),
          state.isFaceSturdy(nothing, origin, Direction.UP, SupportType.CENTER),
        ],
        skip: skip,
        liquid: state.liquid(),
        air: state.isAir(),
        replaceable: state.canBeReplaced(),
        sturdy: sturdy,
        large: state.hasLargeCollisionShape(),
        suffocate: state.isSuffocating(nothing, origin),
        viewBlock: state.isViewBlocking(nothing, origin),
        randomTick: state.isRandomlyTicking(),
        push: String(state.getPistonPushReaction()),
        hardness: state.getDestroySpeed(nothing, origin),
        mapColor: safe(() => state.getMapColor(nothing, origin).id),
        friction: state.getBlock().getFriction(),
        ignite: Blocks.FIRE.getIgniteOdds(state),
        burn: Blocks.FIRE.getBurnOdds(state),
        sound: {
          volume: sound.getVolume(),
          pitch: sound.getPitch(),
          break: String(sound.getBreakSound().getLocation()),
          step: String(sound.getStepSound().getLocation()),
          place: String(sound.getPlaceSound().getLocation()),
          hit: String(sound.getHitSound().getLocation()),
          fall: String(sound.getFallSound().getLocation()),
        },
      })
    }
    console.info('[pumpkin-dump] ' + stateCount + ' block states listed')

    // Items.
    for (const held of Java.from(BuiltInRegistries.ITEM.entrySet())) {
      const item = held.getValue()
      const components = {}
      safe(() => {
        for (const component of Java.from(item.components().entrySet())) {
          components[String(BuiltInRegistries.DATA_COMPONENT_TYPE.getKey(component.getKey()).location())] =
            String(component.getValue())
        }
        return true
      })
      dump.items[String(held.getKey().location())] = {
        maxStack: item.getDefaultMaxStackSize(),
        maxDamage: safe(() => item.components().get(DataComponents.MAX_DAMAGE)),
        components: components,
      }
    }
    console.info('[pumpkin-dump] ' + Object.keys(dump.items).length + ' items listed')

    // Entity types.
    for (const held of Java.from(BuiltInRegistries.ENTITY_TYPE.entrySet())) {
      const type = held.getValue()
      const size = type.getDimensions()
      dump.entities[String(held.getKey().location())] = {
        width: size.width(),
        height: size.height(),
        eye: size.eyeHeight(),
        tracking: type.clientTrackingRange(),
        interval: type.updateInterval(),
        fireImmune: safe(() => type.fireImmune()),
        summon: type.canSummon(),
        category: safe(() => String(type.getCategory())),
      }
    }
    console.info('[pumpkin-dump] ' + Object.keys(dump.entities).length + ' entity types listed')

    // The tags of blocks and entity types, without vanilla's own (the server
    // has those): the tag's id and the ids of its entries.
    const tagsOf = registry => {
      const out = []
      for (const pair of Java.from(registry.getTags())) {
        // 1.21.1 pairs the key with the set; later versions hand over the set alone.
        const key = safe(() => String(pair.getFirst().location())) || safe(() => String(pair.key().location()))
        const set = typeof pair.getSecond === 'function' ? pair.getSecond() : pair
        if (key === null || key.startsWith('minecraft:')) {
          continue
        }
        const ids = Java.from(set.stream()).map(held => safe(() => registry.getId(held.value()))).filter(id => id !== null)
        out.push([key, ids])
      }
      return out
    }
    dump.tags.block = tagsOf(BuiltInRegistries.BLOCK)
    dump.tags.entity_type = tagsOf(BuiltInRegistries.ENTITY_TYPE)
    console.info('[pumpkin-dump] ' + (dump.tags.block.length + dump.tags.entity_type.length) + ' tags listed')

    // The SERVER config files of this pack, by file name: a modded client
    // keeps a server config unloaded until the server sends its file.
    if (ConfigSync !== null) {
      // The list reads every file; with one missing, fall back to their names.
      safe(() => {
        try {
          dump.serverConfigs = Java.from(ConfigSync.syncConfigs()).map(file => String(file.fileName()))
        } catch (e) {
          const walk = dir => {
            for (const file of Java.from(dir.listFiles())) {
              if (file.isDirectory()) {
                walk(file)
              } else if (String(file.getName()).endsWith('-server.toml')) {
                dump.serverConfigs.push(String(file.getName()))
              }
            }
          }
          walk(Java.loadClass('java.io.File')('config'))
        }
        return true
      })
    }
    console.info('[pumpkin-dump] ' + dump.serverConfigs.length + ' server config files listed')

    JsonIO.write('local/kubejs/export/pumpkin-registry.json', dump)
    console.info('[pumpkin-dump] wrote local/kubejs/export/pumpkin-registry.json in ' + (Date.now() - started) + ' ms')
  } catch (e) {
    console.error('[pumpkin-dump] failed: ' + e)
  }
})
