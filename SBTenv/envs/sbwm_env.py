from SBTenv.models.treadmill import Treadmill
from SBTenv.models.wheel import Wheel
from dm_control.suite.base import Task

class SBWMtask(Task):
    def __init__(self):
        super().__init__()
        self.slow_belt_speed = 0.0
        self.fast_belt_speed = 0.0
        self.give_boost = False
        
    def before_step(self, action, physics):
        physics.named.data.qvel[Treadmill.SLOW_JOINT] = self.slow_belt_speed
        physics.named.data.qvel[Treadmill.FAST_JOINT] = self.fast_belt_speed
        
    def get_observation(self, physics):
        return {
            'axle_y_pos':    physics.named.data.sensordata['axlepos'][1].copy(),
            'axle_z_pos':    physics.named.data.sensordata['axlepos'][2].copy(),
            'axle_velocity': physics.named.data.qvel[Wheel.JOINT_Y].copy(),
            'axle_rotation': physics.named.data.qpos[Wheel.JOINT_HINGE].copy(),
        }
    
    def get_reward(self, physics):
        return 0.0
    
    def initialize_episode(self, physics):
        if self.give_boost:
            physics.named.data.qvel[Wheel.JOINT_Y] = self.fast_belt_speed
