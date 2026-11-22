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

/// Where a console `/sublevel spawn` places the platform when nobody
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

        let Some(world) = context
            .source
            .world
            .clone()
            .or_else(|| context.source.server().worlds.load().first().cloned())
        else {
            context
                .source
                .send_error(TextComponent::text("No worlds are loaded"));
            return Ok(0);
        };

        let center = command_center(context, &world);
        let (id, mounted) = world.spawn_sublevel_platform(center, width);
        context.source.send_feedback(
            green(format!(
                "Spawned sub-level {id} at ({:.1}, {:.1}, {:.1}), width {width}, carrying {mounted} entit{}",
                center.x,
                center.y,
                center.z,
                if mounted == 1 { "y" } else { "ies" },
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

        let Some(world) = context.source.world.clone() else {
            context
                .source
                .send_error(TextComponent::text("No world to operate in"));
            return Ok(0);
        };

        // Arguments are bounded to ±100, exactly representable in f32.
        let delta = [
            f32::from(i16::try_from(x).unwrap_or(0)),
            f32::from(i16::try_from(y).unwrap_or(0)),
            f32::from(i16::try_from(z).unwrap_or(0)),
        ];
        let mut system = world
            .sublevels
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if system.apply_velocity_change(id, delta) {
            drop(system);
            context.source.send_feedback(
                green(format!("Shoved sub-level {id} by ({x}, {y}, {z}) blocks/s")),
                true,
            );
            Ok(1)
        } else {
            drop(system);
            context
                .source
                .send_error(TextComponent::text(format!("No sub-level {id}")));
            Ok(0)
        }
    }
}

struct MountExecutor;

impl CommandExecutor for MountExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let id = u32::try_from(IntegerArgumentType::get(context, "id")?).unwrap_or(0);

        let Some(world) = context.source.world.clone() else {
            context
                .source
                .send_error(TextComponent::text("No world to operate in"));
            return Ok(0);
        };

        let half = {
            let system = world
                .sublevels
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            system
                .snapshots()
                .into_iter()
                .find(|snapshot| snapshot.id == id)
                .map(|snapshot| snapshot.half_extents[0])
        };
        let Some(half) = half else {
            context
                .source
                .send_error(TextComponent::text(format!("No sub-level {id}")));
            return Ok(0);
        };

        let mounted = world.mount_entities_on_sublevel(id, half);
        context.source.send_feedback(
            green(format!(
                "Mounted {mounted} entit{} onto sub-level {id}",
                if mounted == 1 { "y" } else { "ies" }
            )),
            false,
        );
        Ok(mounted as i32)
    }
}

struct RemoveExecutor;

impl CommandExecutor for RemoveExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let id = u32::try_from(IntegerArgumentType::get(context, "id")?).unwrap_or(0);

        let Some(world) = context.source.world.clone() else {
            context
                .source
                .send_error(TextComponent::text("No world to operate in"));
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
        let Some(world) = context.source.world.clone() else {
            context
                .source
                .send_error(TextComponent::text("No world to operate in"));
            return Ok(0);
        };

        let snapshots = world
            .sublevels
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .snapshots();

        if snapshots.is_empty() {
            context
                .source
                .send_error(TextComponent::text("No sub-levels exist"));
            return Ok(0);
        }

        for snapshot in &snapshots {
            let mut line = format!(
                "Sub-level {} at ({:.1}, {:.1}, {:.1}) vel ({:.1}, {:.1}, {:.1}), {} rider(s)",
                snapshot.id,
                snapshot.position[0],
                snapshot.position[1],
                snapshot.position[2],
                snapshot.linear_velocity[0],
                snapshot.linear_velocity[1],
                snapshot.linear_velocity[2],
                snapshot.rider_count,
            );
            for (rider, pos) in snapshot.riders.iter().take(snapshot.rider_count) {
                use std::fmt::Write as _;
                let _ = write!(
                    line,
                    " [#{rider} at ({:.1}, {:.1}, {:.1})]",
                    pos[0], pos[1], pos[2],
                );
            }
            context.source.send_feedback(green(line), false);
        }
        Ok(snapshots.len() as i32)
    }
}

pub fn register(dispatcher: &mut CommandDispatcher, registry: &PermissionRegistry) {
    registry.register_permission_or_panic(Permission::new(
        PERMISSION,
        DESCRIPTION,
        PermissionDefault::Op(PermissionLvl::Two),
    ));

    let builder = command("sublevel", DESCRIPTION)
        .requires(PERMISSION)
        .then(
            literal("spawn")
                .executes(SpawnExecutor { has_width: false })
                .then(
                    argument("width", IntegerArgumentType::new(1, 16))
                        .executes(SpawnExecutor { has_width: true }),
                ),
        )
        .then(
            literal("impulse").then(
                argument("id", IntegerArgumentType::new(1, 100_000)).then(
                    argument("x", IntegerArgumentType::new(-100, 100)).then(
                        argument("y", IntegerArgumentType::new(-100, 100)).then(
                            argument("z", IntegerArgumentType::new(-100, 100))
                                .executes(ImpulseExecutor),
                        ),
                    ),
                ),
            ),
        )
        .then(
            literal("mount")
                .then(argument("id", IntegerArgumentType::new(1, 100_000)).executes(MountExecutor)),
        )
        .then(
            literal("remove").then(
                argument("id", IntegerArgumentType::new(1, 100_000)).executes(RemoveExecutor),
            ),
        )
        .then(literal("list").executes(ListExecutor));

    dispatcher.register(builder);
}
