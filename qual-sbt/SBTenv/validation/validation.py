from SBTenv.models.treadmill import Treadmill
from SBTenv.models.wheel import Wheel
from SBTenv.tasks.sbwm_task import SBWMtask
from dm_control import mujoco, mjcf, viewer
from dm_control.rl.control import Environment
from dm_control.suite.base import Task
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
import shutil
import subprocess

class SBWMsim:
    """
    Split-belt rimless wheel validation against Butterfield et al. (2022).

    Belt speeds follow the Treadmill convention: positive = backward, so the
    wheel walks forward (+Y) by harvesting energy from the fast belt.

    :param timestep: Physics timestep in seconds. Default: 0.001.
    :param integrator: MuJoCo integrator. Default: 'implicitfast'.
    :param wheel_kwargs: Overrides passed to Wheel.
    :param treadmill_kwargs: Overrides passed to Treadmill.
    """
    def __init__(self, timestep=0.001, integrator='implicitfast',
                    wheel_kwargs=None, treadmill_kwargs=None):
            self._model = mjcf.RootElement(model='sbt_rimless_wheel')

            self._model.compiler.autolimits = True
            self._model.compiler.angle = 'degree'
            self._model.option.gravity = [0, 0, -9.81]
            self._model.option.timestep = timestep
            self._model.option.integrator = integrator
            self._model.option.solver = 'PGS'
            self._model.option.cone = 'Elliptic'
            self._model.option.iterations = 50

            Wheel(self._model.worldbody, **(wheel_kwargs or {}))
            Treadmill(self._model.worldbody,
                    **{'belt_size': [0.05, 10, 0.1], **(treadmill_kwargs or {})})
            self._model.sensor.add('framepos',
                                name='axlepos',
                                objtype='xbody',
                                objname=Wheel.BODY_NAME
                                )
            # Side view from the fast-belt side that follows the wheel (walking
            # direction +Y points right in the image).
            self._model.find('body', Wheel.BODY_NAME).add(
                'camera', name='side', mode='trackcom',
                pos=[1.2, 0, 0], xyaxes=[0, 1, 0, 0, 0, 1])

            self.timestep = timestep
            self.time_limit = 20      # s, total trial duration
            self.settle_time = 0.05      # s, discarded before measuring
            self.min_rotations = 1    # fewer rotations while measuring = failed trial
            self._sbt_task = SBWMtask()
            self.sbt_physics = mujoco.Physics.from_xml_string(self._model.to_xml_string())
            self.sbt_env = Environment(physics=self.sbt_physics, task=self._sbt_task)
            
    def _run_episode(self, slow_belt_speed: float, fast_belt_speed: float,
                     callback=None) -> float:
        """
        Run one trial and return the average forward velocity over the
        measurement window (after settle_time), or NaN if the wheel did not
        walk (e.g. it stalled and was carried by a belt instead).

        :param callback: Optional function called as callback(step_index)
            after every physics step (used for recording video).
        """
        self._sbt_task.set_belt_speeds(slow_belt_speed, fast_belt_speed)
        self.sbt_env.reset()
        self.sbt_physics.named.data.qvel[Wheel.JOINT_Y] = fast_belt_speed

        n_steps = int(round(self.time_limit / self.timestep))
        n_settle = int(round(self.settle_time / self.timestep))
        for i in range(n_steps):
            if i == n_settle:
                y0 = float(self.sbt_physics.named.data.sensordata['axlepos'][1])
                hinge0 = float(self.sbt_physics.named.data.qpos[Wheel.JOINT_HINGE][0])
                t0 = self.sbt_physics.data.time
            self._sbt_task.before_step(None, self.sbt_physics)
            self.sbt_physics.step()
            if callback is not None:
                callback(i)

        y1 = float(self.sbt_physics.named.data.sensordata['axlepos'][1])
        hinge1 = float(self.sbt_physics.named.data.qpos[Wheel.JOINT_HINGE][0])
        rotations = -(hinge1 - hinge0) / (2 * np.pi)
        return (y1 - y0) / (self.sbt_physics.data.time - t0)
    
if __name__ == '__main__':
    s=SBWMsim()
    print(s._run_episode(0,1))