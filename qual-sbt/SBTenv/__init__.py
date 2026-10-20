try:
    import gymnasium as _gym
except ImportError:
    _gym = None

if _gym is not None:
    _gym.register(id='SBTenv/RimlessWheel-v0',
                  entry_point='SBTenv.envs.gym_env:SBTGymEnv')
