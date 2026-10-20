from sbtenv.defaults.treadmill import SIMULATION
from sbtenv.defaults.wheel import TRIAL
from sbtenv.models.wheel import Wheel
from sbtenv.envs.sbt_env import SBTenv
from sbtenv.tasks.sbwm_task import SBWMTask


def _task_attr(name):
    """Expose a task setting as an env attribute (reads and writes the task)."""
    return property(lambda self: getattr(self._sbt_task, name),
                    lambda self, value: setattr(self._sbt_task, name, value))


class SBWMenv(SBTenv):
    """
    Split-belt rimless wheel environment: the SBTenv treadmill plus a
    rimless wheel and its axlepos sensor. The wheel walks forward (+Y) by
    harvesting energy from the fast belt.

    The trial settings are held by the SBWMTask; the attributes below
    read and write them there.

    :param timestep: Physics timestep in seconds. Default: 0.01.
    :param integrator: MuJoCo integrator. Default: 'RK4'.
    :param wheel_kwargs: Overrides passed to Wheel.
    :param treadmill_kwargs: Overrides passed to Treadmill.
    :param time_limit: Total trial duration in seconds. Default: 20.
    :param settle_time: Elapsed time before data is recorded, in seconds. Default: 0.05.
    :param min_rotations: Minimum wheel rotations for a valid episode. Default: 1.
    :param max_rotations: Wheel rotations before the episode terminates. Default: 12.
    """
    time_limit = _task_attr('time_limit')
    settle_time = _task_attr('settle_time')
    min_rotations = _task_attr('min_rotations')
    max_rotations = _task_attr('max_rotations')

    def __init__(self, timestep=SIMULATION['timestep'], integrator=SIMULATION['integrator'],
                 wheel_kwargs=None, treadmill_kwargs=None,
                 time_limit=TRIAL['time_limit'], settle_time=TRIAL['settle_time'],
                 min_rotations=TRIAL['min_rotations'], max_rotations=TRIAL['max_rotations']):
        self._wheel_kwargs = wheel_kwargs or {}   # read by _add_models during super().__init__
        self._trial = dict(time_limit=time_limit, settle_time=settle_time,
                           min_rotations=min_rotations, max_rotations=max_rotations)
        super().__init__(timestep, integrator, treadmill_kwargs)

    def _add_models(self):
        Wheel(self._model.worldbody, **self._wheel_kwargs)
        self._model.sensor.add('framepos',
                                        name='axlepos',
                                        objtype='xbody',
                                        objname=Wheel.BODY_NAME
                                        )

    def _make_task(self):
        return SBWMTask(**self._trial)
