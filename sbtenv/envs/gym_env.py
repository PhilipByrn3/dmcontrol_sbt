"""
Gymnasium wrapper for the split-belt treadmill environments.

    import gymnasium as gym
    import sbtenv

    env = gym.make('sbtenv/RimlessWheel-v0', fast_belt_speed=0.6)
    obs, info = env.reset()
    while True:
        obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
        if terminated or truncated:
            print(info['avg_velocity'])
            break
"""
import gymnasium as gym
import numpy as np
from gymnasium import spaces

from sbtenv.envs.sbwm_env import SBWMenv


class SBTGymEnv(gym.Env):
    """
    Gymnasium interface to an SBTenv environment (SBWMenv by default).

    Belt speeds are part of the environment: set them with the
    slow_belt_speed / fast_belt_speed arguments, or per episode with
    reset(options={'slow_belt_speed': ..., 'fast_belt_speed': ...}). They
    stay constant for the whole episode.

    Action:      controls for the walker's actuators, as described by the
                 MuJoCo model (dm_control action_spec). The rimless wheel is
                 passive, so its action space is empty (shape (0,)); add an
                 actuated walker through SBTenv._add_models to train on it.
    Observation: the task's observation dict flattened into one float64
                 vector, in the order given by obs_keys.
    Reward:      the task's reward (forward axle velocity for SBWMenv).
    terminated:  the task ended the episode (e.g. max_rotations reached).
    truncated:   the time limit was reached.
    info:        on the final step, the task's episode_result() (e.g.
                 avg_velocity, rotations, valid, steps), if it has one.

    :param env: An existing SBTenv instance. Default: a new SBWMenv(**env_kwargs).
    :param slow_belt_speed: Slow belt speed in m/s for every episode, unless
        overridden in reset options. Default: 0.
    :param fast_belt_speed: Fast belt speed in m/s (also the wheel's initial
        push) for every episode, unless overridden in reset options. Default: 0.
    :param render_mode: None or 'rgb_array'. gym.make(..., render_mode='human')
        also works: Gymnasium then shows the rgb_array frames in a window
        (requires pygame).
    :param env_kwargs: Passed to rgb_array when env is not given.
    """
    metadata = {'render_modes': ['rgb_array']}

    def __init__(self, env=None, slow_belt_speed=0.0, fast_belt_speed=0.0,
                 render_mode=None, **env_kwargs):
        self.sbt = env if env is not None else SBWMenv(**env_kwargs)
        self.default_belt_speeds = (slow_belt_speed, fast_belt_speed)
        self.render_mode = render_mode
        self.metadata = {**self.metadata,
                         'render_fps': int(round(1 / self.sbt.timestep))}

        self._task = self.sbt._sbt_task
        spec = self.sbt.sbt_env.action_spec()
        low = np.broadcast_to(getattr(spec, 'minimum', -np.inf), spec.shape)
        high = np.broadcast_to(getattr(spec, 'maximum', np.inf), spec.shape)
        self.action_space = spaces.Box(low.astype(np.float64), high.astype(np.float64),
                                       shape=spec.shape, dtype=np.float64)

        example = self.sbt.sbt_env.reset().observation or {}
        self.obs_keys = list(example)
        size = sum(np.size(example[k]) for k in self.obs_keys)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(size,), dtype=np.float64)

    def _flatten(self, observation):
        if not self.obs_keys:
            return np.zeros(0, dtype=np.float64)
        return np.concatenate([np.atleast_1d(np.asarray(observation[k], dtype=np.float64))
                               for k in self.obs_keys])

    def _result(self):
        result = getattr(self._task, 'episode_result', None)
        return dict(result(self.sbt.sbt_physics)) if result else {}

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        options = options or {}
        slow, fast = self.default_belt_speeds
        self._task.set_belt_speeds(options.get('slow_belt_speed', slow),
                                   options.get('fast_belt_speed', fast))
        timestep = self.sbt.sbt_env.reset()
        return self._flatten(timestep.observation), {}

    def step(self, action):
        action = np.asarray(action, dtype=np.float64).reshape(self.action_space.shape)
        timestep = self.sbt.sbt_env.step(action)

        terminated = timestep.last() and timestep.discount == 0.0
        truncated = timestep.last() and not terminated
        info = self._result() if timestep.last() else {}
        return (self._flatten(timestep.observation), float(timestep.reward),
                terminated, truncated, info)

    def render(self):
        if self.render_mode == 'rgb_array':
            return self.sbt.sbt_physics.render(height=480, width=640, camera_id=-1)
        return None
