use crate::command::argument_builder::{ArgumentBuilder, argument, command, literal};
use crate::command::argument_types::core::float::FloatArgumentType;
use crate::command::argument_types::core::integer::IntegerArgumentType;
use crate::command::context::command_context::CommandContext;
use crate::command::node::dispatcher::CommandDispatcher;
use crate::command::node::{CommandExecutor, CommandExecutorResult};
use crate::world::World;
use crate::world::sublevel;
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
        let id = world.spawn_sublevel(center, width);
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

struct AssembleExecutor;

impl CommandExecutor for AssembleExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let x1 = IntegerArgumentType::get(context, "x1")?;
        let y1 = IntegerArgumentType::get(context, "y1")?;
        let z1 = IntegerArgumentType::get(context, "z1")?;
        let x2 = IntegerArgumentType::get(context, "x2")?;
        let y2 = IntegerArgumentType::get(context, "y2")?;
        let z2 = IntegerArgumentType::get(context, "z2")?;
        let Some(world) = world_of(context) else {
            context
                .source
                .send_error(TextComponent::text("No worlds are loaded"));
            return Ok(0);
        };

        match world.assemble_sublevel(Vector3::new(x1, y1, z1), Vector3::new(x2, y2, z2)) {
            Ok(id) => {
                context.source.send_feedback(
                    green(format!(
                        "Assembling sub-level {id} from ({x1}, {y1}, {z1})..({x2}, {y2}, {z2}); its plot chunks are generating"
                    )),
                    true,
                );
                Ok(1)
            }
            Err(error) => {
                context.source.send_error(TextComponent::text(error));
                Ok(0)
            }
        }
    }
}

/// `/sublevel assemble <x> <y> <z>`: the contraption connected to a block.
struct AssembleAtExecutor;

impl CommandExecutor for AssembleAtExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let x = IntegerArgumentType::get(context, "x1")?;
        let y = IntegerArgumentType::get(context, "y1")?;
        let z = IntegerArgumentType::get(context, "z1")?;
        let Some(world) = world_of(context) else {
            context
                .source
                .send_error(TextComponent::text("No worlds are loaded"));
            return Ok(0);
        };

        match world.assemble_connected(Vector3::new(x, y, z)) {
            Ok(id) => {
                context.source.send_feedback(
                    green(format!(
                        "Assembling sub-level {id} from the contraption at ({x}, {y}, {z}); its plot chunks are generating"
                    )),
                    true,
                );
                Ok(1)
            }
            Err(error) => {
                context.source.send_error(TextComponent::text(error));
                Ok(0)
            }
        }
    }
}

struct DisassembleExecutor;

impl CommandExecutor for DisassembleExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let id = u32::try_from(IntegerArgumentType::get(context, "id")?).unwrap_or(0);
        let Some(world) = world_of(context) else {
            context
                .source
                .send_error(TextComponent::text("No worlds are loaded"));
            return Ok(0);
        };

        match world.disassemble_sublevel(id, None) {
            Ok(sublevel::DisassemblyStart::Started) => {
                context.source.send_feedback(
                    green(format!(
                        "Sub-level {id} is turning to the nearest quarter turn to melt back into the world"
                    )),
                    true,
                );
                Ok(1)
            }
            Ok(sublevel::DisassemblyStart::Waiting(held)) => {
                let waiting = held.len();
                world.disassemble_after_loading(id, None, held, || {});
                context.source.send_feedback(
                    green(format!(
                        "Sub-level {id} waits for {waiting} chunk(s) around it to load, then melts back into the world"
                    )),
                    true,
                );
                Ok(1)
            }
            Err(error) => {
                context.source.send_error(TextComponent::text(error));
                Ok(0)
            }
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

struct ThrottleExecutor;

impl CommandExecutor for ThrottleExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let id = u32::try_from(IntegerArgumentType::get(context, "id")?).unwrap_or(0);
        let throttle = FloatArgumentType::get(context, "throttle")?;
        let Some(world) = world_of(context) else {
            context
                .source
                .send_error(TextComponent::text("No worlds are loaded"));
            return Ok(0);
        };

        let set = world
            .sublevels
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .set_throttle(id, throttle);
        if set {
            context.source.send_feedback(
                green(format!(
                    "Throttled sub-level {id} to {}%",
                    (throttle * 100.0) as u32
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

struct InfoExecutor;

impl CommandExecutor for InfoExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let id = u32::try_from(IntegerArgumentType::get(context, "id")?).unwrap_or(0);
        let Some(world) = world_of(context) else {
            context
                .source
                .send_error(TextComponent::text("No worlds are loaded"));
            return Ok(0);
        };

        let Some(info) = world
            .sublevels
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .aero_info(id)
        else {
            context
                .source
                .send_error(TextComponent::text(format!("No assembled sub-level {id}")));
            return Ok(0);
        };
        context.source.send_feedback(
            green(format!(
                "Sub-level {id} weighs {:.1} and {}; throttle {}%",
                info.mass,
                if info.active {
                    "flies"
                } else {
                    "waits for its ground"
                },
                (info.throttle * 100.0) as u32
            )),
            false,
        );
        for (index, balloon) in info.balloons.iter().enumerate() {
            context.source.send_feedback(
                green(format!(
                    "  Balloon {}: {:.0}/{:.0} m³ of lifting gas, lifts {:.1}",
                    index + 1,
                    balloon.fill,
                    balloon.capacity,
                    balloon.lift
                )),
                false,
            );
        }
        context.source.send_feedback(
            green(format!(
                "  {} thruster(s), {} lift surface(s)",
                info.thrusters, info.surfaces
            )),
            false,
        );
        Ok(1)
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
                    "Sub-level {} at ({:.2}, {:.2}, {:.2}) vel ({:.2}, {:.2}, {:.2}) turned ({:.3}, {:.3}, {:.3}, {:.3}), plot chunk ({}, {}), {}, {} tracker(s)",
                    body.id,
                    body.position[0],
                    body.position[1],
                    body.position[2],
                    body.linear_velocity[0],
                    body.linear_velocity[1],
                    body.linear_velocity[2],
                    body.rotation[0],
                    body.rotation[1],
                    body.rotation[2],
                    body.rotation[3],
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
    let coordinate =
        |name: &'static str| argument(name, IntegerArgumentType::new(-30_000_000, 30_000_000));

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
        .then(literal("assemble").then(
            coordinate("x1").then(
                coordinate("y1").then(
                    coordinate("z1").executes(AssembleAtExecutor).then(
                        coordinate("x2").then(
                            coordinate("y2").then(coordinate("z2").executes(AssembleExecutor)),
                        ),
                    ),
                ),
            ),
        ))
        .then(literal("disassemble").then(id_argument().executes(DisassembleExecutor)))
        .then(literal("impulse").then(id_argument().then(
            argument("x", IntegerArgumentType::new(-100, 100)).then(
                argument("y", IntegerArgumentType::new(-100, 100)).then(
                    argument("z", IntegerArgumentType::new(-100, 100)).executes(ImpulseExecutor),
                ),
            ),
        )))
        .then(literal("throttle").then(id_argument().then(
            argument("throttle", FloatArgumentType::new(0.0, 1.0)).executes(ThrottleExecutor),
        )))
        .then(literal("info").then(id_argument().executes(InfoExecutor)))
        .then(literal("stand").then(id_argument().executes(RiderExecutor)))
        .then(literal("remove").then(id_argument().executes(RemoveExecutor)))
        .then(literal("list").executes(ListExecutor));

    dispatcher.register(builder);
}
