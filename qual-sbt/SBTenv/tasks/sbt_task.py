from SBTenv.models.treadmill import Treadmill
from dm_control.suite.base import Task

class SBTTask(Task):
    """
    Minimal dm_control Task stub for the split-belt treadmill.
    """
    def __init__(self):
        super().__init__()
        self.slow_belt_speed = 0.0
        self.fast_belt_speed = 0.0

    def set_belt_speeds(self, slow_belt_speed, fast_belt_speed):
        self.slow_belt_speed = slow_belt_speed
        self.fast_belt_speed = fast_belt_speed

    def before_step(self, action, physics):
        physics.named.data.qvel[Treadmill.SLOW_JOINT] = self.slow_belt_speed
        physics.named.data.qvel[Treadmill.FAST_JOINT] = self.fast_belt_speed

    def get_observation(self, physics):
        pass

    def get_reward(self, physics):
        pass

    def initialize_episode(self, physics):
        pass