package dev.pumpkinmc.registrydump;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;
import net.minecraft.SharedConstants;
import net.minecraft.core.BlockPos;
import net.minecraft.core.Direction;
import net.minecraft.core.Holder;
import net.minecraft.core.IdMap;
import net.minecraft.core.Registry;
import net.minecraft.core.component.DataComponents;
import net.minecraft.core.registries.BuiltInRegistries;
import net.minecraft.resources.ResourceKey;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.sounds.SoundEvent;
import net.minecraft.world.entity.EntityDimensions;
import net.minecraft.world.entity.EntityType;
import net.minecraft.world.item.Item;
import net.minecraft.world.level.EmptyBlockGetter;
import net.minecraft.world.level.block.Block;
import net.minecraft.world.level.block.Blocks;
import net.minecraft.world.level.block.FireBlock;
import net.minecraft.world.level.block.EntityBlock;
import net.minecraft.world.level.block.SoundType;
import net.minecraft.world.level.block.SupportType;
import net.minecraft.world.level.block.entity.BlockEntity;
import net.minecraft.world.level.block.entity.BlockEntityType;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.block.state.properties.Property;
import net.minecraft.world.level.material.PushReaction;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.shapes.VoxelShape;
import net.neoforged.bus.api.IEventBus;
import net.neoforged.fml.common.Mod;
import net.neoforged.fml.config.ModConfig;
import net.neoforged.fml.config.ModConfigs;
import net.neoforged.fml.loading.FMLPaths;
import net.neoforged.neoforge.event.server.ServerStartedEvent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

/**
 * Writes the registry dump a Pumpkin server reads as its picture of this
 * client's mods: {@code pumpkin-registry.json} next to the game directory,
 * once the first world has started (block tags only exist once a world's
 * data packs are loaded). Registers nothing on the network, so it changes
 * nothing about which servers the client can join.
 */
@Mod(PumpkinRegistryDump.MODID)
public final class PumpkinRegistryDump {
    public static final String MODID = "pumpkin_registry_dump";
    /** The layout the Pumpkin server reads; bump it with the file's shape. */
    private static final int FORMAT = 2;
    private static final String FILE_NAME = "pumpkin-registry.json";
    private static final Logger LOG = LoggerFactory.getLogger("PumpkinRegistryDump");
    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().create();

    private static boolean written;

    public PumpkinRegistryDump(IEventBus modBus) {
        modBus.addListener(PumpkinRegistryDump::serverStarted);
    }

    private static void serverStarted(ServerStartedEvent event) {
        if (written) {
            return;
        }
        written = true;
        long started = System.currentTimeMillis();
        try {
            Path path = FMLPaths.GAMEDIR.get().resolve(FILE_NAME);
            Files.writeString(path, GSON.toJson(dump()), StandardCharsets.UTF_8);
            LOG.info("wrote {} in {} ms", path, System.currentTimeMillis() - started);
        } catch (Exception e) {
            LOG.error("could not write the registry dump", e);
        }
    }

    private static JsonElement dump() {
        JsonObject dump = new JsonObject();
        dump.addProperty("format", FORMAT);
        JsonObject game = new JsonObject();
        game.addProperty("version", SharedConstants.getCurrentVersion().getId());
        dump.add("game", game);

        // Every built-in registry, its entries and ids.
        JsonObject registries = new JsonObject();
        for (var entry : BuiltInRegistries.REGISTRY.entrySet()) {
            JsonObject registry = new JsonObject();
            registry.addProperty("sync", entry.getValue().doesSync());
            registry.add("entries", entries(entry.getValue()));
            registries.add(entry.getKey().location().toString(), registry);
        }
        dump.add("registries", registries);

        JsonObject blocks = new JsonObject();
        JsonObject items = new JsonObject();
        JsonObject entities = new JsonObject();
        for (Map.Entry<ResourceKey<Item>, Item> held : BuiltInRegistries.ITEM.entrySet()) {
            JsonObject item = new JsonObject();
            item.addProperty("maxStack", held.getValue().getDefaultMaxStackSize());
            Integer maxDamage = held.getValue().components().get(DataComponents.MAX_DAMAGE);
            item.add("maxDamage", maxDamage == null ? JsonNull.INSTANCE : new JsonPrimitive(maxDamage));
            items.add(held.getKey().location().toString(), item);
        }
        for (Map.Entry<ResourceKey<EntityType<?>>, EntityType<?>> held :
            BuiltInRegistries.ENTITY_TYPE.entrySet()) {
            EntityType<?> type = held.getValue();
            EntityDimensions size = type.getDimensions();
            JsonObject entity = new JsonObject();
            entity.addProperty("width", size.width());
            entity.addProperty("height", size.height());
            entity.addProperty("eye", size.eyeHeight());
            entity.addProperty("tracking", type.clientTrackingRange());
            entity.addProperty("interval", type.updateInterval());
            entity.addProperty("fireImmune", type.fireImmune());
            entity.addProperty("summon", type.canSummon());
            entity.addProperty("category", type.getCategory().getName());
            entities.add(held.getKey().location().toString(), entity);
        }

        // Every block state, in state id order, under its block.
        IdMap<BlockState> states = Block.BLOCK_STATE_REGISTRY;
        EmptyBlockGetter nothing = EmptyBlockGetter.INSTANCE;
        BlockPos origin = BlockPos.ZERO;
        Direction[] sides = {
            Direction.DOWN, Direction.UP, Direction.NORTH, Direction.SOUTH, Direction.WEST, Direction.EAST
        };
        for (int id = 0; id < states.size(); id++) {
            BlockState state = states.byId(id);
            Block block = state.getBlock();
            String name = BuiltInRegistries.BLOCK.getKey(block).toString();
            JsonObject blockFacts = blocks.has(name) ? blocks.getAsJsonObject(name) : null;
            if (blockFacts == null) {
                blockFacts = new JsonObject();
                blockFacts.addProperty("base", id);
                blockFacts.addProperty("default", states.getId(block.defaultBlockState()));
                blockFacts.add("blockEntity", blockEntity(block, state));
                blockFacts.addProperty("resistance", block.getExplosionResistance());
                blockFacts.add("states", new JsonArray());
                blocks.add(name, blockFacts);
            }
            JsonObject facts = new JsonObject();
            JsonObject props = new JsonObject();
            for (Map.Entry<Property<?>, Comparable<?>> value : state.getValues().entrySet()) {
                props.addProperty(value.getKey().getName(), nameOf(value.getKey(), value.getValue()));
            }
            facts.add("props", props);
            facts.add("shape", boxes(state.getCollisionShape(nothing, origin)));
            facts.add("outline", boxes(state.getShape(nothing, origin)));
            facts.addProperty("light", state.getLightEmission());
            facts.addProperty("opacity", state.getLightBlock(nothing, origin));
            facts.addProperty("sky", state.propagatesSkylightDown(nothing, origin));
            facts.addProperty("solidRender", state.isSolidRender(nothing, origin));
            facts.addProperty("solid", state.isSolid());
            facts.addProperty("fullCube", state.isCollisionShapeFullBlock(nothing, origin));
            facts.addProperty("canOcclude", state.canOcclude());
            facts.addProperty("toolRequired", state.requiresCorrectToolForDrops());
            facts.addProperty("analog", state.hasAnalogOutputSignal());
            facts.add("centerSturdy", array(
                state.isFaceSturdy(nothing, origin, Direction.DOWN, SupportType.CENTER),
                state.isFaceSturdy(nothing, origin, Direction.UP, SupportType.CENTER)));
            JsonArray sturdy = new JsonArray();
            JsonArray skip = new JsonArray();
            for (Direction side : sides) {
                sturdy.add(state.isFaceSturdy(nothing, origin, side));
                skip.add(state.skipRendering(state, side));
            }
            facts.add("sturdy", sturdy);
            facts.add("skip", skip);
            facts.addProperty("liquid", state.liquid());
            facts.addProperty("air", state.isAir());
            facts.addProperty("replaceable", state.canBeReplaced());
            facts.addProperty("randomTick", state.isRandomlyTicking());
            facts.addProperty("push", nameOf(state.getPistonPushReaction()));
            facts.addProperty("hardness", state.getDestroySpeed(nothing, origin));
            facts.addProperty("mapColor", state.getMapColor(nothing, origin).id);
            facts.addProperty("friction", block.getFriction());
            facts.addProperty("ignite", ((FireBlock) Blocks.FIRE).getIgniteOdds(state));
            facts.addProperty("burn", ((FireBlock) Blocks.FIRE).getBurnOdds(state));
            SoundType sound = state.getSoundType();
            JsonObject soundFacts = new JsonObject();
            soundFacts.addProperty("volume", sound.getVolume());
            soundFacts.addProperty("pitch", sound.getPitch());
            soundFacts.add("break", location(sound.getBreakSound()));
            soundFacts.add("step", location(sound.getStepSound()));
            soundFacts.add("place", location(sound.getPlaceSound()));
            soundFacts.add("hit", location(sound.getHitSound()));
            soundFacts.add("fall", location(sound.getFallSound()));
            facts.add("sound", soundFacts);
            blockFacts.getAsJsonArray("states").add(facts);
        }
        dump.add("blocks", blocks);
        dump.add("items", items);
        dump.add("entities", entities);

        // The tags of blocks and entity types, without vanilla's own (the
        // server has those): the tag's id and the ids of its entries.
        JsonObject tags = new JsonObject();
        tags.add("block", tags(BuiltInRegistries.BLOCK));
        tags.add("entity_type", tags(BuiltInRegistries.ENTITY_TYPE));
        dump.add("tags", tags);

        // The SERVER config files of this pack: a modded client keeps a
        // server config unloaded until the server sends its file.
        JsonArray serverConfigs = new JsonArray();
        for (ModConfig config : ModConfigs.getConfigSet(ModConfig.Type.SERVER)) {
            serverConfigs.add(config.getFileName());
        }
        dump.add("serverConfigs", serverConfigs);
        return dump;
    }

    /** Every entry of the registry as `[id, "namespace:path"]`, by id. */
    private static <T> JsonArray entries(Registry<T> registry) {
        List<JsonArray> entries = new ArrayList<>();
        for (Map.Entry<ResourceKey<T>, T> held : registry.entrySet()) {
            entries.add(array(registry.getId(held.getValue()), held.getKey().location().toString()));
        }
        entries.sort(Comparator.comparingInt(e -> e.get(0).getAsInt()));
        return list(entries);
    }

    private static JsonElement blockEntity(Block block, BlockState state) {
        if (!(block instanceof EntityBlock entity)) {
            return JsonNull.INSTANCE;
        }
        BlockEntity blockEntity = entity.newBlockEntity(BlockPos.ZERO, state);
        if (blockEntity == null) {
            return JsonNull.INSTANCE;
        }
        BlockEntityType<?> type = blockEntity.getType();
        ResourceLocation key = BuiltInRegistries.BLOCK_ENTITY_TYPE.getKey(type);
        return location(key);
    }

    private static <T> JsonArray tags(Registry<T> registry) {
        JsonArray tags = new JsonArray();
        registry.getTags().forEach(pair -> {
            if (pair.getFirst().location().getNamespace().equals("minecraft")) {
                return;
            }
            JsonArray ids = new JsonArray();
            pair.getSecond().stream().map(Holder::value).mapToInt(registry::getId).forEach(ids::add);
            tags.add(array(pair.getFirst().location().toString(), ids));
        });
        return tags;
    }

    private static JsonArray boxes(VoxelShape shape) {
        JsonArray boxes = new JsonArray();
        for (AABB box : shape.toAabbs()) {
            boxes.add(array(box.minX, box.minY, box.minZ, box.maxX, box.maxY, box.maxZ));
        }
        return boxes;
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    private static String nameOf(Property property, Comparable value) {
        return property.getName(value);
    }

    private static String nameOf(PushReaction reaction) {
        return switch (reaction) {
            case NORMAL -> "normal";
            case DESTROY -> "destroy";
            case BLOCK -> "block";
            case IGNORE -> "ignore";
            case PUSH_ONLY -> "push_only";
        };
    }

    private static JsonElement location(SoundEvent sound) {
        return sound == null ? JsonNull.INSTANCE : new JsonPrimitive(sound.getLocation().toString());
    }

    private static JsonElement location(ResourceLocation location) {
        return location == null ? JsonNull.INSTANCE : new JsonPrimitive(location.toString());
    }

    private static JsonArray list(List<JsonArray> elements) {
        JsonArray list = new JsonArray();
        for (JsonArray element : elements) {
            list.add(element);
        }
        return list;
    }

    private static JsonArray array(Object... values) {
        JsonArray array = new JsonArray();
        for (Object value : values) {
            switch (value) {
                case Integer i -> array.add(i);
                case Double d -> array.add(d);
                case Boolean b -> array.add(b);
                case String s -> array.add(s);
                case JsonElement e -> array.add(e);
                case null -> array.add(JsonNull.INSTANCE);
                default -> throw new IllegalArgumentException("cannot write " + value.getClass());
            }
        }
        return array;
    }
}
