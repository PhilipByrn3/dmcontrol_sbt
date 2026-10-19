import math
import time
import numpy as np
import numpy.typing as npt
from SBTenv.envs.sbwm_env import SBWMenv
from SBTenv.models.treadmill import Treadmill
from SBTenv.models.wheel import Wheel


class SBWMSimulation(SBWMenv):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._sweep_step_times = []

    def run_episode(self, slow_speed: float, fast_speed: float) -> float:
        self.sbt_env.reset()
        self.sbt_physics.named.data.qvel[Wheel.JOINT_Y] = fast_speed
        n_steps = int(round(self.time_limit/self.timestep))
        n_settle = int(round(self.settle_time / self.timestep))
        target    = self.max_rotations
        for i in range(n_steps):
            self.sbt_physics.named.data.qvel[Treadmill.SLOW_JOINT] = slow_speed
            self.sbt_physics.named.data.qvel[Treadmill.FAST_JOINT] = fast_speed
            if i == n_settle:
                y0 = float(self.sbt_physics.named.data.sensordata['axlepos'][1])
                hinge0 = float(self.sbt_physics.named.data.qpos[Wheel.JOINT_HINGE][0])
                t0 = self.sbt_physics.data.time
            self.sbt_physics.step()
        
        y1 = float(self.sbt_physics.named.data.sensordata['axlepos'][1])
        hinge1 = float(self.sbt_physics.named.data.qpos[Wheel.JOINT_HINGE][0])
        rotations = -(hinge1 - hinge0) / (2 * np.pi)
        velocity = (y1 - y0) / (self.sbt_physics.data.time - t0)
        print('BSD: ', round(fast_speed, 4), ' \tVelocity: ', round(velocity,4))
        return velocity

    def sweep_episode(self, slow_belt_speed: float, start_bsd: float, end_bsd: float, bsd_increment: float, custom_bsd_array = 0):
        bsd_array = np.arange(start_bsd, end_bsd + bsd_increment, bsd_increment)
        if custom_bsd_array.any():
            bsd_array = custom_bsd_array
        velocities_array = np.array(bsd_array)
        for idx in range(0,bsd_array.shape[0]):
            velocities_array[idx] = self.run_episode(slow_belt_speed, slow_belt_speed+bsd_array[idx])
        return velocities_array
        
        
        
