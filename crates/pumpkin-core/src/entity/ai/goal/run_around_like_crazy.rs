use std::sync::Arc;

use super::{Controls, Goal, to_goal_ticks};
use crate::entity::EntityBase;
use crate::entity::ai::pathfinder::NavigatorGoal;
use crate::entity::ai::util::default_random_pos;
use crate::entity::mob::Mob;
use pumpkin_data::entity::EntityStatus;
use pumpkin_util::math::vector3::Vector3;
use uuid::Uuid;

/// The `AbstractHorse` pieces `RunAroundLikeCrazyGoal` drives: an untamed
/// mount that gallops with a rider and rolls its temper until tamed.
pub trait TemperamentalMount: Send + Sync {
    fn is_tame(&self) -> bool;
    fn set_tame(&self, tame: bool);
    /// Vanilla `getMaxTemper`: 100 for the horse family, 30 for llamas.
    fn max_temper(&self) -> i32;
    fn temper(&self) -> i32;
    fn set_temper(&self, temper: i32);
    fn set_owner(&self, owner: Uuid);
    /// Vanilla `makeMad`: rear up with an angry sound.
    fn make_mad(&self);
}

/// Vanilla `RunAroundLikeCrazyGoal`: an untamed mount with a rider gallops
/// around, and every so often rolls its temper -- taming the mount on a win,
/// throwing the rider off on a loss.
pub struct RunAroundLikeCrazyGoal<M: TemperamentalMount + EntityBase + 'static> {
    mount: Arc<M>,
    goal_control: Controls,
    speed_modifier: f64,
    pos: Vector3<f64>,
}

impl<M: TemperamentalMount + EntityBase> RunAroundLikeCrazyGoal<M> {
    #[expect(
        clippy::missing_const_for_fn,
        reason = "Vector3::new is not const on this toolchain"
    )]
    #[must_use]
    pub fn new(mount: Arc<M>, speed_modifier: f64) -> Self {
        Self {
            mount,
            goal_control: Controls::MOVE,
            speed_modifier,
            pos: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    /// Vanilla `modifyTemper(5)` per failed attempt.
    const TEMPER_GAIN: i32 = 5;
}

impl<M: TemperamentalMount + EntityBase> Goal for RunAroundLikeCrazyGoal<M> {
    fn can_start(&mut self, mob: &dyn Mob) -> bool {
        if self.mount.is_tame() || !mob.get_entity().has_passengers() {
            return false;
        }

        let Some(pos) = default_random_pos::get_pos(mob, 5, 4) else {
            return false;
        };

        self.pos = pos;
        true
    }

    fn should_continue(&mut self, mob: &dyn Mob) -> bool {
        !self.mount.is_tame() && !mob.is_navigator_idle() && mob.get_entity().has_passengers()
    }

    fn start(&mut self, mob: &dyn Mob) {
        let mob_pos = mob.get_entity().pos.load();
        let mut navigator = mob
            .get_mob_entity()
            .navigator
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        navigator.set_progress(NavigatorGoal::new(mob_pos, self.pos, self.speed_modifier));
    }

    fn tick(&mut self, _mob: &dyn Mob) {
        // Vanilla: every ~50 ticks roll the temper once.
        if !rand::random::<u32>().is_multiple_of(to_goal_ticks(50).unsigned_abs()) {
            return;
        }

        let passengers = self
            .mount
            .get_entity()
            .passengers
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let Some(player_uuid) = passengers
            .first()
            .and_then(|passenger| passenger.get_player())
            .map(|player| player.gameprofile.id)
        else {
            return;
        };
        drop(passengers);

        let max_temper = self.mount.max_temper();
        let temper = self.mount.temper().max(0) as u32;
        let roll = rand::random::<u32>() % max_temper.max(1) as u32;
        if max_temper > 0 && roll < temper {
            self.mount.set_owner(player_uuid);
            self.mount.set_tame(true);
            let entity = self.mount.get_entity();
            entity
                .world
                .load()
                .send_entity_status(entity, EntityStatus::TamingSucceeded, None);
            return;
        }
        self.mount
            .set_temper((temper as i32 + Self::TEMPER_GAIN).min(max_temper));

        self.mount.make_mad();
        let mount: Arc<dyn EntityBase> = self.mount.clone();
        self.mount.get_entity().eject_passengers(&mount);
        let entity = self.mount.get_entity();
        entity
            .world
            .load()
            .send_entity_status(entity, EntityStatus::TamingFailed, None);
    }

    fn controls(&self) -> Controls {
        self.goal_control
    }
}
