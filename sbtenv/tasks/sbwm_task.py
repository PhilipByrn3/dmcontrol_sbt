import math

from sbtenv.defaults.wheel import TRIAL
from sbtenv.models.wheel import Wheel
from sbtenv.tasks.sbt_task import SBTTask


class SBWMTask(SBTTask):
    """
    dm_control Task for the split-belt rimless wheel (SBWM) env.

    One episode is one validation trial:
      - reset gives the wheel an initial forward push at fast_belt_speed
      - the first settle_time seconds are discarded before measuring
      - the episode terminates once the wheel has turned max_rotations,
        or is truncated at time_limit
      - episode_result() returns the average forward velocity over the
        measurement window

    :param time_limit: Episode length in seconds. Default: 20.
    :param settle_time: Time discarded before measuring, in seconds. Default: 0.05.
    :param min_rotations: Rotations during measurement for a valid trial. Default: 1.
    :param max_rotations: Rotations (from reset) that end the episode. Default: 12.
    """
    def __init__(self, time_limit=TRIAL['time_limit'], settle_time=TRIAL['settle_time'],
                 min_rotations=TRIAL['min_rotations'], max_rotations=TRIAL['max_rotations'],
                 random=None):
        super().__init__(time_limit=time_limit, random=random)
        self.settle_time = settle_time
        self.min_rotations = min_rotations
        self.max_rotations = max_rotations
        self._n_settle = 0
        self._start = None

    @staticmethod
    def _rotations(physics):
        """Full wheel rotations from reset; positive = walking forward (+Y)."""
        return -float(physics.named.data.qpos[Wheel.JOINT_HINGE][0]) / (2 * math.pi)

    def initialize_episode(self, physics):
        super().initialize_episode(physics)
        physics.named.data.qvel[Wheel.JOINT_Y] = self.fast_belt_speed
        self._n_settle = int(round(self.settle_time / physics.timestep()))
        self._start = None

    def before_step(self, action, physics):
        super().before_step(action, physics)
        if self.step_count == self._n_settle:
            self._start = (
                float(physics.named.data.sensordata['axlepos'][1]),
                float(physics.named.data.qpos[Wheel.JOINT_HINGE][0]),
                physics.data.time,
            )

    def get_termination(self, physics):
        if self._rotations(physics) >= self.max_rotations:
            return 0.0      # trial complete: terminal state
        return super().get_termination(physics)

    def episode_result(self, physics):
        """
        Measurement for the episode so far:
            avg_velocity  forward velocity over the measurement window (m/s)
            rotations     wheel rotations during the measurement window
            valid         rotations >= min_rotations
            steps         physics steps taken since reset
        avg_velocity and rotations are NaN if settle_time has not elapsed.
        """
        if self._start is None:
            return {'avg_velocity': float('nan'), 'rotations': float('nan'),
                    'valid': False, 'steps': self.step_count}
        y0, hinge0, t0 = self._start
        y1 = float(physics.named.data.sensordata['axlepos'][1])
        hinge1 = float(physics.named.data.qpos[Wheel.JOINT_HINGE][0])
        rotations = -(hinge1 - hinge0) / (2 * math.pi)
        return {
            'avg_velocity': (y1 - y0) / (physics.data.time - t0),
            'rotations': rotations,
            'valid': rotations >= self.min_rotations,
            'steps': self.step_count,
        }

    def get_observation(self, physics):
            return {
                'axle_y_pos':    physics.named.data.sensordata['axlepos'][1].copy(),
                'axle_z_pos':    physics.named.data.sensordata['axlepos'][2].copy(),
                'axle_velocity': physics.named.data.qvel[Wheel.JOINT_Y].copy(),
                'axle_rotation': physics.named.data.qpos[Wheel.JOINT_HINGE].copy(),
            }

    def get_reward(self, physics):
            return float(physics.named.data.qvel[Wheel.JOINT_Y][0])
