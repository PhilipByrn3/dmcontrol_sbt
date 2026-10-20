from sbtenv.models.treadmill import Treadmill
from dm_control.suite.base import Task

class SBTTask(Task):
    """
    Base dm_control Task for the split-belt treadmill.

    Before every physics step, applies the action to the model's actuators
    (if it has any) and drives both belts at the speeds given to
    set_belt_speeds(). Belt speeds are part of the environment, not the
    action: they stay fixed until set_belt_speeds() is called again.
    Counts steps and ends the episode (as a truncation, discount 1.0) once
    time_limit is reached. Subclasses add observations, reward and their
    own termination conditions.

    :param time_limit: Episode length in seconds. Default: inf (no limit).
    """
    def __init__(self, time_limit=float('inf'), random=None):
        super().__init__(random=random)
        self.slow_belt_speed = 0.0
        self.fast_belt_speed = 0.0
        self.time_limit = time_limit
        self.step_count = 0

    def set_belt_speeds(self, slow_belt_speed, fast_belt_speed):
        self.slow_belt_speed = slow_belt_speed
        self.fast_belt_speed = fast_belt_speed

    def max_steps(self, physics):
        """
        Physics steps per episode: time_limit / timestep.
        """
        if self.time_limit == float('inf'):
            return float('inf')
        return int(round(self.time_limit / physics.timestep()))

    def initialize_episode(self, physics):
        self.step_count = 0

    def before_step(self, action, physics):
        if action is not None:
            physics.set_control(action)     # walker actuators; empty for a passive walker
        physics.named.data.qvel[Treadmill.SLOW_JOINT] = self.slow_belt_speed
        physics.named.data.qvel[Treadmill.FAST_JOINT] = self.fast_belt_speed

    def after_step(self, physics):
        self.step_count += 1

    def get_termination(self, physics):
        if self.step_count >= self.max_steps(physics):
            return 1.0      # time limit: truncation, not a terminal state
        return None

    def get_observation(self, physics):
        return {}

    def get_reward(self, physics):
        return 0.0
