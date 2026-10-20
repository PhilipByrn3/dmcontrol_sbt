# sbtenv

A split-belt treadmill environment for MuJoCo and dm_control. It comes with a passive rimless wheel that reproduces the experimental results of Butterfield et al. (2022), and a Gymnasium interface so you can put your own walker on the treadmill.

## Install

```bash
git clone https://github.com/PhilipByrn3/dmcontrol_sbt.git
cd dmcontrol_sbt
pip install -e .
```

Python 3.10–3.12. The package pins `dm_control==1.0.22` and `mujoco==3.2.2`, the versions the validation results were produced with; other MuJoCo versions give slightly different velocities. For the exact research environment, use `pip install -r requirements.txt`.

## Run a trial

Each trial gives the wheel a push, lets it walk on the treadmill, and returns its average forward velocity in m/s.

```python
from sbtenv.utils.sbwm_simulation import SBWMSimulation

sim = SBWMSimulation()
v = sim.run_episode(slow_speed=0.0, fast_speed=0.6)
```

A trial ends when the wheel has turned 12 times or after 20 seconds. If the wheel makes less than one full rotation (it stalled, or the belts carried it), the trial is invalid and returns `nan`.

## Sweep belt speeds

```python
bsd, v = sim.sweep_episode(0.0, start_bsd=0.15, end_bsd=1.0, bsd_increment=0.05)
```

`bsd` holds the belt speed differences that were run, and `v` the velocity for each. You can also pass your own speeds:

```python
bsd, v = sim.sweep_episode(0.0, custom_bsd_array=[0.2, 0.4, 0.8])
```

## Compare with the experiment

```python
from sbtenv.utils.data import load_experimental
from sbtenv.utils import stats

exp_bsd, exp_v = load_experimental()
_, v = sim.sweep_episode(0.0, custom_bsd_array=exp_bsd)

print(stats.rmse(v, exp_v), stats.r2(v, exp_v))
```

The full validation runs with

```bash
python -m sbtenv.validation.validation
```

and `python -m sbtenv.validation.check_reference` confirms the defaults still reproduce the published numbers. They match exactly on Apple-silicon Macs. Other platforms round floating point slightly differently, and since small differences grow over a trial, their statistics land close by rather than on the same digits.

## Change the model

The defaults are the validated configuration: rubber tips on both sides of the wheel, level belts, a 0.01 s timestep and RK4. They are listed in `sbtenv/defaults/`. Anything can be overridden:

```python
sim = SBWMSimulation(
    timestep=0.005,
    wheel_kwargs={'fast_rubber': False, 'spoke_length': 0.3},
    treadmill_kwargs={'fast_zoffset': 0.0049},   # raise the fast belt 4.9 mm
    max_rotations=6,
)
```

## Use it with Gymnasium

```python
import gymnasium as gym
import sbtenv

env = gym.make('sbtenv/RimlessWheel-v0', fast_belt_speed=0.6)
obs, info = env.reset()

while True:
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
    if terminated or truncated:
        break

print(info['avg_velocity'])
```

Belt speeds belong to the environment, not the agent. Set them when you create it, as above, or per episode:

```python
obs, info = env.reset(options={'slow_belt_speed': 0.0, 'fast_belt_speed': 0.8})
```

The rimless wheel is passive, so its action space is empty. `terminated` means the wheel finished its rotations, `truncated` means time ran out, and the final step's `info` holds `avg_velocity`, `rotations`, `valid` and `steps`. Pass `render_mode='rgb_array'` to get frames from `env.render()`.

## Put your own walker on the treadmill

Subclass `SBTenv` to add your model, and `SBTTask` to define what an episode means. The treadmill, belt driving and time limit are already handled.

```python
from sbtenv.envs.sbt_env import SBTenv
from sbtenv.tasks.sbt_task import SBTTask
from sbtenv.models.wheel import Wheel

class StayCentred(SBTTask):
    def get_observation(self, physics):
        return {'qpos': physics.data.qpos.copy(), 'qvel': physics.data.qvel.copy()}

    def get_reward(self, physics):
        return -abs(physics.named.data.qpos['axleYAxis'][0])   # stay near y = 0

class MotorWheelEnv(SBTenv):
    def _add_models(self):
        Wheel(self._model.worldbody)
        self._model.actuator.add('motor', joint='axleHinge', ctrlrange=[-1, 1], gear=[0.5])

    def _make_task(self):
        return StayCentred(time_limit=10)
```

Run it with dm_control:

```python
import numpy as np

env = MotorWheelEnv()
env.set_belt_speeds(0.0, 0.6)

timestep = env.dm_env.reset()
while not timestep.last():
    timestep = env.dm_env.step(np.array([0.2]))
```

Or hand it to Gymnasium, which builds the action space from your actuators:

```python
env = gym.make('sbtenv/RimlessWheel-v0', env=MotorWheelEnv(), fast_belt_speed=0.6)
print(env.action_space)   # Box(-1.0, 1.0, (1,), float64)
```

## Conventions

- Belt speeds are in m/s, applied to each belt as its velocity along +Y before every physics step.
- The wheel walks toward +Y, and its forward rotation makes the hinge angle negative.
- `env.physics`, `env.task` and `env.dm_env` give you the underlying dm_control objects.

## Tests

```bash
pip install -e ".[dev]"
pytest
```

## License

Apache 2.0. The experimental data is digitized from Butterfield et al. (2022), *The split-belt rimless wheel*.
