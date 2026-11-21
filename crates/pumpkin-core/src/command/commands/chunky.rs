use crate::command::argument_builder::{ArgumentBuilder, argument, command, literal};
use crate::command::argument_types::core::integer::IntegerArgumentType;
use crate::command::argument_types::core::string::StringArgumentType;
use crate::command::context::command_context::CommandContext;
use crate::command::node::dispatcher::CommandDispatcher;
use crate::command::node::{CommandExecutor, CommandExecutorResult};
use crate::world::World;
use crate::world::pregen::{self, PregenShape};
use pumpkin_util::PermissionLvl;
use pumpkin_util::permission::{Permission, PermissionDefault, PermissionRegistry};
use pumpkin_util::text::TextComponent;
use pumpkin_util::text::color::{Color, NamedColor};
use std::sync::Arc;

const DESCRIPTION: &str = "Pregenerate chunks in a selected area with progress reports.";
const PERMISSION: &str = "minecraft:command.chunky";

fn green(text: String) -> TextComponent {
    TextComponent::text(text).color(Color::Named(NamedColor::Green))
}

/// Resolves a world by its dimension name (`the_end`, `minecraft:the_end`) or
/// its world folder name.
fn find_world(context: &CommandContext, name: &str) -> Option<Arc<World>> {
    let clean = name.strip_prefix("minecraft:").unwrap_or(name);
    context
        .source
        .server()
        .worlds
        .load()
        .iter()
        .find(|world| {
            world.get_world_name() == clean
                || world
                    .dimension
                    .minecraft_name
                    .strip_prefix("minecraft:")
                    .is_some_and(|stripped| stripped == clean)
        })
        .cloned()
}

struct StartExecutor {
    has_radius: bool,
}

impl CommandExecutor for StartExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let radius_arg = if self.has_radius {
            IntegerArgumentType::get(context, "radius")?
        } else {
            0
        };

        let (radius, center, shape, world_name) = pregen::with_selection(|selection| {
            if radius_arg > 0 {
                selection.radius = radius_arg;
            }
            (
                selection.radius,
                selection.center,
                selection.shape,
                selection.world.clone(),
            )
        });
        let shape = shape.unwrap_or(PregenShape::Square);

        if radius <= 0 {
            context.source.send_error(TextComponent::text(
                "No radius set. Use /chunky radius <blocks> or /chunky start <radius>",
            ));
            return Ok(0);
        }

        let world = world_name
            .as_deref()
            .and_then(|name| find_world(context, name))
            .or_else(|| context.source.world.clone())
            .or_else(|| context.source.server().worlds.load().first().cloned());
        let Some(world) = world else {
            context
                .source
                .send_error(TextComponent::text("No worlds are loaded"));
            return Ok(0);
        };

        let spawn = world.get_spawn_location().0;
        let (center_x, center_z) = center.unwrap_or_else(|| {
            let position = context.source.position;
            if context.source.executed_by_player() {
                (position.x, position.z)
            } else {
                (f64::from(spawn.0.x) + 0.5, f64::from(spawn.0.z) + 0.5)
            }
        });

        let reporter = context
            .source
            .as_player()
            .map(|player| Arc::downgrade(&player));

        match pregen::start(&world, center_x, center_z, radius, shape, reporter) {
            Ok(task) => {
                let progress = task.progress();
                context.source.send_feedback(
                    green(format!(
                        "Task started for world '{}' ({}): {} chunks around ({}, {}), shape {}, radius {} blocks",
                        progress.world_name,
                        world.dimension.minecraft_name,
                        progress.total,
                        center_x as i32,
                        center_z as i32,
                        shape.as_str(),
                        radius,
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

struct RadiusExecutor;

impl CommandExecutor for RadiusExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let radius = IntegerArgumentType::get(context, "radius")?;
        pregen::with_selection(|selection| selection.radius = radius);
        context.source.send_feedback(
            green(format!("Set pregeneration radius to {radius} blocks")),
            false,
        );
        Ok(radius)
    }
}

struct CenterExecutor;

impl CommandExecutor for CenterExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let x = IntegerArgumentType::get(context, "x")?;
        let z = IntegerArgumentType::get(context, "z")?;
        pregen::with_selection(|selection| selection.center = Some((f64::from(x), f64::from(z))));
        context.source.send_feedback(
            green(format!("Set pregeneration center to ({x}, {z})")),
            false,
        );
        Ok(1)
    }
}

struct ShapeExecutor {
    shape: PregenShape,
}

impl CommandExecutor for ShapeExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        pregen::with_selection(|selection| selection.shape = Some(self.shape));
        context.source.send_feedback(
            green(format!(
                "Set pregeneration shape to {}",
                self.shape.as_str()
            )),
            false,
        );
        Ok(1)
    }
}

struct WorldExecutor;

impl CommandExecutor for WorldExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let name = StringArgumentType::get(context, "world")?.to_string();
        let Some(world) = find_world(context, &name) else {
            let names: Vec<String> = context
                .source
                .server()
                .worlds
                .load()
                .iter()
                .map(|world| {
                    world
                        .dimension
                        .minecraft_name
                        .strip_prefix("minecraft:")
                        .unwrap_or(world.dimension.minecraft_name)
                        .to_string()
                })
                .collect();
            context.source.send_error(TextComponent::text(format!(
                "World '{name}' not found (loaded: {})",
                names.join(", ")
            )));
            return Ok(0);
        };
        let name = world
            .dimension
            .minecraft_name
            .strip_prefix("minecraft:")
            .unwrap_or(world.dimension.minecraft_name)
            .to_string();
        pregen::with_selection(|selection| selection.world = Some(name.clone()));
        context
            .source
            .send_feedback(green(format!("Set pregeneration world to '{name}'")), false);
        Ok(1)
    }
}

struct PauseExecutor;

impl CommandExecutor for PauseExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let paused = pregen::pause_all();
        if paused == 0 {
            context
                .source
                .send_error(TextComponent::text("No tasks are running"));
            return Ok(0);
        }
        context.source.send_feedback(
            green(format!("Paused {paused} pregeneration task(s)")),
            true,
        );
        Ok(1)
    }
}

struct ContinueExecutor;

impl CommandExecutor for ContinueExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let resumed = pregen::resume_all();
        if resumed == 0 {
            context
                .source
                .send_error(TextComponent::text("No tasks are paused"));
            return Ok(0);
        }
        context.source.send_feedback(
            green(format!("Continued {resumed} pregeneration task(s)")),
            true,
        );
        Ok(1)
    }
}

struct CancelExecutor;

impl CommandExecutor for CancelExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let cancelled = pregen::cancel_all();
        if cancelled == 0 {
            context
                .source
                .send_error(TextComponent::text("No tasks are running"));
            return Ok(0);
        }
        context.source.send_feedback(
            green(format!("Cancelled {cancelled} pregeneration task(s)")),
            true,
        );
        Ok(1)
    }
}

struct ProgressExecutor;

impl CommandExecutor for ProgressExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let tasks = pregen::active_tasks();
        if tasks.is_empty() {
            context
                .source
                .send_error(TextComponent::text("No tasks are running"));
            return Ok(0);
        }
        let done_total = tasks.iter().map(|task| task.progress().done).sum::<usize>();
        for task in &tasks {
            let progress = task.progress();
            let percent = (progress.done as f64 / progress.total.max(1) as f64) * 100.0;
            let state = if progress.paused { "paused" } else { "running" };
            let skipped = if progress.skipped > 0 {
                format!(", skipped {}", progress.skipped)
            } else {
                String::new()
            };
            context.source.send_feedback(
                TextComponent::text(format!(
                    "Task {state} for '{}': {}/{} chunks ({percent:.1}%), elapsed {}s{skipped}",
                    progress.world_name,
                    progress.done,
                    progress.total,
                    progress.elapsed.as_secs(),
                )),
                false,
            );
        }
        Ok(done_total as i32)
    }
}

struct QuietExecutor;

impl CommandExecutor for QuietExecutor {
    fn execute(&self, context: &CommandContext) -> CommandExecutorResult {
        let seconds = IntegerArgumentType::get(context, "seconds")?;
        let tasks = pregen::active_tasks();
        if tasks.is_empty() {
            context
                .source
                .send_error(TextComponent::text("No tasks are running"));
            return Ok(0);
        }
        for task in &tasks {
            task.set_quiet(seconds as usize);
        }
        context.source.send_feedback(
            green(format!(
                "Reporting progress every {seconds}s for {} task(s)",
                tasks.len()
            )),
            false,
        );
        Ok(1)
    }
}

pub fn register(dispatcher: &mut CommandDispatcher, registry: &PermissionRegistry) {
    registry.register_permission_or_panic(Permission::new(
        PERMISSION,
        DESCRIPTION,
        PermissionDefault::Op(PermissionLvl::Two),
    ));

    let builder = command("chunky", DESCRIPTION)
        .requires(PERMISSION)
        .then(
            literal("start")
                .executes(StartExecutor { has_radius: false })
                .then(
                    argument("radius", IntegerArgumentType::new(1, 100_000))
                        .executes(StartExecutor { has_radius: true }),
                ),
        )
        .then(literal("radius").then(
            argument("radius", IntegerArgumentType::new(1, 100_000)).executes(RadiusExecutor),
        ))
        .then(
            literal("center").then(
                argument("x", IntegerArgumentType::any())
                    .then(argument("z", IntegerArgumentType::any()).executes(CenterExecutor)),
            ),
        )
        .then(
            literal("shape")
                .then(literal("square").executes(ShapeExecutor {
                    shape: PregenShape::Square,
                }))
                .then(literal("circle").executes(ShapeExecutor {
                    shape: PregenShape::Circle,
                })),
        )
        .then(
            literal("world")
                .then(argument("world", StringArgumentType::SingleWord).executes(WorldExecutor)),
        )
        .then(literal("pause").executes(PauseExecutor))
        .then(literal("continue").executes(ContinueExecutor))
        .then(literal("cancel").executes(CancelExecutor))
        .then(literal("progress").executes(ProgressExecutor))
        .then(
            literal("quiet").then(
                argument("seconds", IntegerArgumentType::new(1, 3600)).executes(QuietExecutor),
            ),
        );

    dispatcher.register(builder);
}
