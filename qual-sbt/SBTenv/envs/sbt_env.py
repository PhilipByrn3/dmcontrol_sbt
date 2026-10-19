from SBTenv.models.treadmill import Treadmill
from SBTenv.tasks.sbt_task import SBTTask
from dm_control import mujoco, mjcf, viewer
from dm_control.rl.control import Environment


class SBTenv:
    """
    Split-belt rimless wheel validation against Butterfield et al. (2022).

    Belt speeds follow the Treadmill convention: positive = backward, so the
    wheel walks forward (+Y) by harvesting energy from the fast belt.

    :param timestep: Physics timestep in seconds. Default: 0.001.
    :param integrator: MuJoCo integrator. Default: 'implicitfast'.
    :param wheel_kwargs: Overrides passed to Wheel.
    :param treadmill_kwargs: Overrides passed to Treadmill.
    """
    def __init__(self, timestep=0.01, integrator='RK4',
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
            Treadmill(self._model.worldbody,**{'belt_size': [0.05, 100, 0.1], **(treadmill_kwargs or {})})
            self.timestep = timestep
            self.time_limit = 20      # s, total trial duration
            self.settle_time = 0.05      # s, discarded before measuring
            self.min_rotations = 1    # fewer rotations while measuring = failed trial
            self.max_rotations = 12
            
            self._add_models(wheel_kwargs)        # hook for subclasses
            self._sbt_task = self._make_task()
            self.sbt_physics = mujoco.Physics.from_xml_string(self._model.to_xml_string())
            self.sbt_env = Environment(physics=self.sbt_physics, task=self._sbt_task)

    def _add_models(self, wheel_kwargs):
        pass

    def _make_task(self):
        return SBTTask()