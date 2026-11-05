import gymnasium as gym
import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

import sbtenv  # noqa: F401  registers sbtenv/RimlessWheel-v0
from sbtenv.utils.sbwm_simulation import SBWMSimulation


def run(env, options=None):
    env.reset(options=options)
    while True:
        obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
        if terminated or truncated:
            return terminated, truncated, info


def test_passes_gymnasium_env_checker():
    check_env(gym.make('sbtenv/RimlessWheel-v0').unwrapped, skip_render_check=True)


def test_passive_wheel_has_empty_action_space():
    assert gym.make('sbtenv/RimlessWheel-v0').action_space.shape == (0,)


def test_matches_run_episode():
    expected = SBWMSimulation(verbose=False).run_episode(0.0, 0.5)
    _, _, info = run(gym.make('sbtenv/RimlessWheel-v0'), {'fast_belt_speed': 0.5})
    assert info['avg_velocity'] == expected


def test_belt_speeds_from_constructor_or_reset():
    from_options = run(gym.make('sbtenv/RimlessWheel-v0'), {'fast_belt_speed': 0.4})[2]
    from_constructor = run(gym.make('sbtenv/RimlessWheel-v0', fast_belt_speed=0.4))[2]
    assert from_options['avg_velocity'] == from_constructor['avg_velocity']


@pytest.mark.parametrize('speed, terminated', [(0.17, False), (0.5, True)])
def test_terminated_vs_truncated(speed, terminated):
    env = gym.make('sbtenv/RimlessWheel-v0', max_rotations=3)
    is_terminated, is_truncated, info = run(env, {'fast_belt_speed': speed})
    assert is_terminated == terminated and is_truncated == (not terminated)
    assert {'avg_velocity', 'rotations', 'valid', 'steps'} <= set(info)
