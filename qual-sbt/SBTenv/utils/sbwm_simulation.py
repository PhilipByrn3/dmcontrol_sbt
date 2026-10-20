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

    def run_episode(self, slow_speed: float, fast_speed: float) -> float:
        """
        Run one trial through the standard dm_control loop and return the
        average forward velocity. The push, settle window, termination and
        measurement are all handled by SBWMTask.
        """
        self._sbt_task.set_belt_speeds(slow_speed, fast_speed)
        timestep = self.sbt_env.reset()
        action = np.zeros(self.sbt_env.action_spec().shape)
        while not timestep.last():
            timestep = self.sbt_env.step(action)

        result = self._sbt_task.episode_result(self.sbt_physics)
        velocity = result['avg_velocity']
        print('BSD: ', round(fast_speed, 4), ' \tVelocity: ', round(velocity,4), '\tRotations: ' , round(result['rotations'],4), '\tTimesteps', result['steps'])
        return velocity

    def sweep_episode(self, slow_belt_speed: float, start_bsd: float, end_bsd: float, bsd_increment: float, custom_bsd_array = 0):
        bsd_array = np.arange(start_bsd, end_bsd + bsd_increment, bsd_increment)
        if custom_bsd_array.any():
            bsd_array = custom_bsd_array
        velocities_array = np.array(bsd_array)
        for idx in range(0,bsd_array.shape[0]):
            velocities_array[idx] = self.run_episode(slow_belt_speed, slow_belt_speed+bsd_array[idx])
        return velocities_array
        
        
        
