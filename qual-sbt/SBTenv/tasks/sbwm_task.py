from SBTenv.models.treadmill import Treadmill
from SBTenv.models.wheel import Wheel
from SBTenv.tasks.sbt_task import SBTTask
from dm_control.suite.base import Task


class SBWMTask(SBTTask):
    """
    dm_control Task for SBWM env.
    """
    def __init__(self):
            super().__init__()

    def get_observation(self, physics):
            return {
                'axle_y_pos':    physics.named.data.sensordata['axlepos'][1].copy(),
                'axle_z_pos':    physics.named.data.sensordata['axlepos'][2].copy(),
                'axle_velocity': physics.named.data.qvel[Wheel.JOINT_Y].copy(),
                'axle_rotation': physics.named.data.qpos[Wheel.JOINT_HINGE].copy(),
            }
    
    def get_reward(self, physics):
            return float(physics.named.data.qvel[Wheel.JOINT_Y])

    def initialize_episode(self, physics):
        pass

