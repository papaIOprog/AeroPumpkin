use std::sync::Arc;
use std::sync::atomic::Ordering;

use super::{Controls, Goal, to_goal_ticks};
use crate::entity::EntityBase;
use crate::entity::ai::pathfinder::NavigatorGoal;
use crate::entity::ai::util::default_random_pos;
use crate::entity::mob::Mob;
use crate::entity::passive::horse::HorseEntity;
use pumpkin_util::math::vector3::Vector3;

/// Vanilla `RunAroundLikeCrazyGoal`: an untamed horse with a rider gallops
/// around, and every so often rolls its temper -- taming the horse on a win,
/// throwing the rider off on a loss.
pub struct RunAroundLikeCrazyGoal {
    horse: Arc<HorseEntity>,
    goal_control: Controls,
    speed_modifier: f64,
    pos: Vector3<f64>,
}

impl RunAroundLikeCrazyGoal {
    #[expect(
        clippy::missing_const_for_fn,
        reason = "Vector3::new is not const on this toolchain"
    )]
    #[must_use]
    pub fn new(horse: Arc<HorseEntity>, speed_modifier: f64) -> Self {
        Self {
            horse,
            goal_control: Controls::MOVE,
            speed_modifier,
            pos: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    /// Vanilla `AbstractHorse.getMaxTemper` (the Horse override returns the
    /// base 100).
    const MAX_TEMPER: i32 = 100;
    /// Vanilla `modifyTemper(5)` per failed attempt.
    const TEMPER_GAIN: i32 = 5;

    /// Vanilla `makeMad`: rear up with an angry sound before throwing the
    /// rider. The standing pose needs `aiStep` counters we do not keep, so
    /// only the sound plays.
    fn make_mad(&self) {
        let entity = self.horse.get_entity();
        let world = entity.world.load();
        world.play_sound(
            pumpkin_data::sound::Sound::EntityHorseAngry,
            pumpkin_data::sound::SoundCategory::Neutral,
            &entity.pos.load(),
        );
    }
}

impl Goal for RunAroundLikeCrazyGoal {
    fn can_start(&mut self, mob: &dyn Mob) -> bool {
        if self.horse.is_tame() || !mob.get_entity().has_passengers() {
            return false;
        }

        let Some(pos) = default_random_pos::get_pos(mob, 5, 4) else {
            return false;
        };

        self.pos = pos;
        true
    }

    fn should_continue(&mut self, mob: &dyn Mob) -> bool {
        !self.horse.is_tame() && !mob.is_navigator_idle() && mob.get_entity().has_passengers()
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
        if rand::random::<i32>() % to_goal_ticks(50) != 0 {
            return;
        }

        let passengers = self
            .horse
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

        let temper = self.horse.temper.load(Ordering::Relaxed);
        if Self::MAX_TEMPER > 0 && rand::random::<i32>() % Self::MAX_TEMPER < temper {
            self.horse.owner.store(Some(player_uuid));
            self.horse.set_tame(true);
            let entity = self.horse.get_entity();
            entity.world.load().send_entity_status(
                entity,
                pumpkin_data::entity::EntityStatus::TamingSucceeded,
                None,
            );
            return;
        }
        self.horse.temper.store(
            (temper + Self::TEMPER_GAIN).min(Self::MAX_TEMPER),
            Ordering::Relaxed,
        );

        self.make_mad();
        let horse: Arc<dyn EntityBase> = self.horse.clone();
        self.horse.get_entity().eject_passengers(&horse);
        let entity = self.horse.get_entity();
        entity.world.load().send_entity_status(
            entity,
            pumpkin_data::entity::EntityStatus::TamingFailed,
            None,
        );
    }

    fn controls(&self) -> Controls {
        self.goal_control
    }
}
