from SBTenv.models.treadmill import Treadmill
from SBTenv.tasks.sbt_task import SBTTask
from dm_control import mujoco, mjcf, viewer
from dm_control.rl.control import Environment


class SBTenv:
    """
    Split-belt treadmill environment. Builds the solver settings and the
    treadmill; subclasses add whatever walks on it via _add_models() and
    choose the task via _make_task(). Both hooks run before the model is
    compiled into physics.

    Belt speeds follow the Treadmill convention: positive = backward.

    :param timestep: Physics timestep in seconds. Default: 0.01.
    :param integrator: MuJoCo integrator. Default: 'RK4'.
    :param treadmill_kwargs: Overrides passed to Treadmill.
    """
    def __init__(self, timestep=0.01, integrator='RK4', treadmill_kwargs=None):
            self._model = mjcf.RootElement(model='split_belt_treadmill')

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
            self._add_models()                    # hook for subclasses
            self._sbt_task = self._make_task()
            self.sbt_physics = mujoco.Physics.from_xml_string(self._model.to_xml_string())
            self.sbt_env = Environment(physics=self.sbt_physics, task=self._sbt_task)

    def _add_models(self):
        pass

    def _make_task(self):
        return SBTTask()
