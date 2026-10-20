from SBTenv.models.wheel import Wheel
from SBTenv.envs.sbt_env import SBTenv
from SBTenv.tasks.sbwm_task import SBWMTask

class SBWMenv(SBTenv):
    """
    Split-belt rimless wheel environment: the SBTenv treadmill plus a
    rimless wheel and its axlepos sensor. The wheel walks forward (+Y) by
    harvesting energy from the fast belt.

    :param timestep: Physics timestep in seconds. Default: 0.01.
    :param integrator: MuJoCo integrator. Default: 'RK4'.
    :param wheel_kwargs: Overrides passed to Wheel.
    :param treadmill_kwargs: Overrides passed to Treadmill.
    """
    def __init__(self, timestep=0.01, integrator='RK4', wheel_kwargs=None, treadmill_kwargs=None):
        self._wheel_kwargs = wheel_kwargs or {}   # read by _add_models during super().__init__
        super().__init__(timestep, integrator, treadmill_kwargs)
        self.time_limit = 20      #s, total trial duration
        self.settle_time = 0.05      # s, elapsed time before simulation data is recorded
        self.min_rotations = 1    # minimum number of rotations made by wheel for episode to be valid.
        self.max_rotations = 12   # maximum number of rotations made by wheel before episode termination

    def _add_models(self):
        Wheel(self._model.worldbody, **self._wheel_kwargs)
        self._model.sensor.add('framepos',
                                        name='axlepos',
                                        objtype='xbody',
                                        objname=Wheel.BODY_NAME
                                        )

    def _make_task(self):
        return SBWMTask()
