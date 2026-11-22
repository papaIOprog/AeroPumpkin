use crate::command::argument_builder::{ArgumentBuilder, argument, command, literal};
use crate::command::argument_types::core::integer::IntegerArgumentType;
use crate::command::context::command_context::CommandContext;
use crate::command::node::dispatcher::CommandDispatcher;
use crate::command::node::{CommandExecutor, CommandExecutorResult};
use crate::world::World;
use pumpkin_util::PermissionLvl;
use pumpkin_util::math::vector3::Vector3;
use pumpkin_util::permission::{Permission, PermissionDefault, PermissionRegistry};
use pumpkin_util::text::TextComponent;
use pumpkin_util::text::color::{Color, NamedColor};

const DESCRIPTION: &str = "Manage native sub-levels: physics bodies carrying blocks and entities.";
const PERMISSION: &str = "minecraft:command.sublevel";

/// Default platform width in blocks when no size is given.
const DEFAULT_WIDTH: i32 = 16;

fn green(text: String) -> TextComponent {
    TextComponent::text(text).color(Color::Named(NamedColor::Green))
}

fn world_of(context: &CommandContext) -> Option<std::sync::Arc<World>> {
    context
        .source
        .world
        .clone()
        .or_else(|| context.source.server().worlds.load().first().cloned())
}

/// Where a console `/sublevel spawn` places the sub-level when nobody
/// executed the command in the world.
fn command_center(context: &CommandContext, world: &World) -> Vector3<f64> {
    if context.source.executed_by_player() {
        let pos = context.source.position;
        Vector3::new(pos.x, pos.y, pos.z)
    } else {
        let spawn = world.get_spawn_location().0;
        Vector3::new(
            f64::from(spawn.0.x) + 0.5,
            f64::from(spawn.0.y),
            f64::from(spawn.0.z) + 0.5,
        )
    }
}

struct SpawnExecutor {
    has_width: bool,
}

impl CommandExecutor for SpawnExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let width = if self.has_width {
            IntegerArgumentType::get(context, "width")?
        } else {
            DEFAULT_WIDTH
        };
        let Some(world) = world_of(context) else {
            context
                .source
                .send_error(TextComponent::text("No worlds are loaded"));
            return Ok(0);
        };

        let center = command_center(context, &world);
        // Widths are bounded by the argument range; i16 keeps the conversion exact.
        let half =
            f32::from(i16::try_from(width).unwrap_or(i16::try_from(DEFAULT_WIDTH).unwrap_or(16)))
                / 2.0;
        let id = world.spawn_sublevel(center, [half, 0.5, half]);
        context.source.send_feedback(
            green(format!(
                "Sub-level {id} spawned at ({:.1}, {:.1}, {:.1}); its plot chunk is generating",
                center.x, center.y, center.z,
            )),
            true,
        );
        Ok(1)
    }
}

struct ImpulseExecutor;

impl CommandExecutor for ImpulseExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let id = u32::try_from(IntegerArgumentType::get(context, "id")?).unwrap_or(0);
        let x = IntegerArgumentType::get(context, "x")?;
        let y = IntegerArgumentType::get(context, "y")?;
        let z = IntegerArgumentType::get(context, "z")?;
        let Some(world) = world_of(context) else {
            context
                .source
                .send_error(TextComponent::text("No worlds are loaded"));
            return Ok(0);
        };

        // Arguments are bounded to ±100, exactly representable in f32.
        let delta = [
            f32::from(i16::try_from(x).unwrap_or(0)),
            f32::from(i16::try_from(y).unwrap_or(0)),
            f32::from(i16::try_from(z).unwrap_or(0)),
        ];
        let shoved = world
            .sublevels
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .apply_velocity_change(id, delta);
        if shoved {
            context.source.send_feedback(
                green(format!("Shoved sub-level {id} by ({x}, {y}, {z}) blocks/s")),
                true,
            );
            Ok(1)
        } else {
            context
                .source
                .send_error(TextComponent::text(format!("No sub-level {id}")));
            Ok(0)
        }
    }
}

struct RiderExecutor;

impl CommandExecutor for RiderExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let id = u32::try_from(IntegerArgumentType::get(context, "id")?).unwrap_or(0);
        let Some(world) = world_of(context) else {
            context
                .source
                .send_error(TextComponent::text("No worlds are loaded"));
            return Ok(0);
        };

        if world.summon_sublevel_rider(id).is_some() {
            context.source.send_feedback(
                green(format!(
                    "Armor stand placed on sub-level {id}; Sable clients see it ride along"
                )),
                true,
            );
            Ok(1)
        } else {
            context
                .source
                .send_error(TextComponent::text(format!("No sub-level {id}")));
            Ok(0)
        }
    }
}

struct RemoveExecutor;

impl CommandExecutor for RemoveExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let id = u32::try_from(IntegerArgumentType::get(context, "id")?).unwrap_or(0);
        let Some(world) = world_of(context) else {
            context
                .source
                .send_error(TextComponent::text("No worlds are loaded"));
            return Ok(0);
        };

        if world.remove_sublevel(id) {
            context
                .source
                .send_feedback(green(format!("Removed sub-level {id}")), true);
            Ok(1)
        } else {
            context
                .source
                .send_error(TextComponent::text(format!("No sub-level {id}")));
            Ok(0)
        }
    }
}

struct ListExecutor;

impl CommandExecutor for ListExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let Some(world) = world_of(context) else {
            context
                .source
                .send_error(TextComponent::text("No worlds are loaded"));
            return Ok(0);
        };

        let levels = world
            .sublevels
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .info();
        if levels.is_empty() {
            context
                .source
                .send_error(TextComponent::text("No sub-levels exist"));
            return Ok(0);
        }

        let count = levels.len();
        for (body, chunk, ready, tracking) in levels {
            context.source.send_feedback(
                green(format!(
                    "Sub-level {} at ({:.1}, {:.1}, {:.1}) vel ({:.1}, {:.1}, {:.1}), plot chunk ({}, {}), {}, {} tracker(s)",
                    body.id,
                    body.position[0],
                    body.position[1],
                    body.position[2],
                    body.linear_velocity[0],
                    body.linear_velocity[1],
                    body.linear_velocity[2],
                    chunk.x,
                    chunk.y,
                    if ready { "ready" } else { "generating" },
                    tracking,
                )),
                false,
            );
        }
        Ok(i32::try_from(count).unwrap_or(0))
    }
}

pub fn register(dispatcher: &mut CommandDispatcher, registry: &PermissionRegistry) {
    registry.register_permission_or_panic(Permission::new(
        PERMISSION,
        DESCRIPTION,
        PermissionDefault::Op(PermissionLvl::Two),
    ));

    let id_argument = || argument("id", IntegerArgumentType::new(1, 100_000));

    let builder = command("sublevel", DESCRIPTION)
        .requires(PERMISSION)
        .then(
            literal("spawn")
                .executes(SpawnExecutor { has_width: false })
                .then(
                    argument("width", IntegerArgumentType::new(1, 64))
                        .executes(SpawnExecutor { has_width: true }),
                ),
        )
        .then(literal("impulse").then(id_argument().then(
            argument("x", IntegerArgumentType::new(-100, 100)).then(
                argument("y", IntegerArgumentType::new(-100, 100)).then(
                    argument("z", IntegerArgumentType::new(-100, 100)).executes(ImpulseExecutor),
                ),
            ),
        )))
        .then(literal("stand").then(id_argument().executes(RiderExecutor)))
        .then(literal("remove").then(id_argument().executes(RemoveExecutor)))
        .then(literal("list").executes(ListExecutor));

    dispatcher.register(builder);
}
