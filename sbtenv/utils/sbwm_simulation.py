import numpy as np
from sbtenv.envs.sbwm_env import SBWMenv


class SBWMSimulation(SBWMenv):
    """
    SBWMenv with helpers for running validation trials and belt speed
    difference (BSD) sweeps. Takes the same arguments as SBWMenv, plus:

    :param verbose: Print one line per trial. Default: True.
    """

    def __init__(self, *args, verbose=True, **kwargs):
        super().__init__(*args, **kwargs)
        self.verbose = verbose

    def run_episode(self, slow_speed: float, fast_speed: float) -> float:
        """
        Run one trial through the standard dm_control loop and return the
        average forward velocity. The push, settle window, termination and
        measurement are all handled by SBWMTask.

        Returns NaN if the trial is invalid: fewer than min_rotations during
        the measurement window (e.g. the wheel stalled).
        """
        self.set_belt_speeds(slow_speed, fast_speed)
        timestep = self.dm_env.reset()
        action = np.zeros(self.dm_env.action_spec().shape)
        while not timestep.last():
            timestep = self.dm_env.step(action)

        result = self.task.episode_result(self.physics)
        velocity = result['avg_velocity'] if result['valid'] else float('nan')
        if self.verbose:
            print('BSD: ', round(fast_speed, 4), ' \tVelocity: ', round(velocity,4), '\tRotations: ' , round(result['rotations'],4), '\tTimesteps', result['steps'],
                  '' if result['valid'] else '\t(invalid: fewer than min_rotations)')
        return velocity

    def sweep_episode(self, slow_belt_speed: float, start_bsd: float = None, end_bsd: float = None,
                      bsd_increment: float = None, custom_bsd_array=None):
        """
        Run one trial per belt speed difference and return (bsd_array, velocities).

        Either pass custom_bsd_array (any sequence of BSDs), or start_bsd,
        end_bsd and bsd_increment for an evenly spaced sweep that includes
        both ends. The fast belt runs at slow_belt_speed + BSD.
        """
        if custom_bsd_array is not None:
            bsd_array = np.asarray(custom_bsd_array, dtype=float)
        elif None in (start_bsd, end_bsd, bsd_increment):
            raise ValueError('Pass custom_bsd_array, or all of start_bsd, end_bsd and bsd_increment')
        else:
            # Count the points by rounding so float error can't add or drop one,
            # then let arange produce the values (stop sits half a step past the end).
            n = int(round((end_bsd - start_bsd) / bsd_increment)) + 1
            bsd_array = np.arange(start_bsd, start_bsd + (n - 0.5) * bsd_increment, bsd_increment)

        velocities = np.array([self.run_episode(slow_belt_speed, slow_belt_speed + bsd)
                               for bsd in bsd_array])
        return bsd_array, velocities
