from SBTenv.defaults.treadmill import SIMULATION
from SBTenv.models.treadmill import Treadmill
from SBTenv.tasks.sbt_task import SBTTask
from dm_control import mujoco, mjcf
from dm_control.rl.control import Environment


class SBTenv:
    """
    Split-belt treadmill environment. Builds the solver settings and the
    treadmill; subclasses add whatever walks on it via _add_models() and
    choose the task via _make_task(). Both hooks run before the model is
    compiled into physics.

    Belt speeds (m/s) are written to the belts' conveyor joints as their
    velocity along +Y before every physics step; see set_belt_speeds().

    Public attributes:
        dm_env   the dm_control Environment (reset / step)
        physics  the dm_control mujoco.Physics
        task     the dm_control Task driving the episode

    :param timestep: Physics timestep in seconds. Default: 0.01.
    :param integrator: MuJoCo integrator. Default: 'RK4'.
    :param treadmill_kwargs: Overrides passed to Treadmill.

    Defaults come from SBTenv.defaults.treadmill (the validated configuration).
    """
    def __init__(self, timestep=SIMULATION['timestep'], integrator=SIMULATION['integrator'],
                 treadmill_kwargs=None):
            self._model = mjcf.RootElement(model='split_belt_treadmill')

            self._model.compiler.autolimits = True
            self._model.compiler.angle = 'degree'
            self._model.option.gravity = SIMULATION['gravity']
            self._model.option.timestep = timestep
            self._model.option.integrator = integrator
            self._model.option.solver = SIMULATION['solver']
            self._model.option.cone = SIMULATION['cone']
            self._model.option.iterations = SIMULATION['iterations']
            Treadmill(self._model.worldbody, **(treadmill_kwargs or {}))
            self.timestep = timestep
            self._add_models()                    # hook for subclasses
            self._sbt_task = self._make_task()
            self.sbt_physics = mujoco.Physics.from_xml_string(self._model.to_xml_string())
            self.sbt_env = Environment(physics=self.sbt_physics, task=self._sbt_task)

    @property
    def dm_env(self):
        return self.sbt_env

    @property
    def physics(self):
        return self.sbt_physics

    @property
    def task(self):
        return self._sbt_task

    def set_belt_speeds(self, slow_belt_speed, fast_belt_speed):
        """Set the belt speeds (m/s) used from the next reset or step onward."""
        self._sbt_task.set_belt_speeds(slow_belt_speed, fast_belt_speed)

    def _add_models(self):
        pass

    def _make_task(self):
        return SBTTask()
