from dm_control.suite.base import Task

class SBTTask(Task):
    """
    Minimal dm_control Task stub for the split-belt treadmill.
    """

    def before_step(self, action, physics):
        pass

    def get_observation(self, physics):
        pass

    def get_reward(self, physics):
        pass

    def initialize_episode(self, physics):
        pass