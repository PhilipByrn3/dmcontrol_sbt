# CLAUDE.md — qual-sbt

## Project Overview

This project simulates a **rimless wheel on a split-belt treadmill (SBT)** using the DeepMind `dm_control` / MuJoCo physics engine. The goal is to reproduce and validate experimental results from Dr. Julia K. Butterfield's physical rimless wheel experiments, showing how a passive rimless wheel traverses a treadmill whose two belts run at different speeds, and to characterize steady-state wheel velocity as a function of belt speed difference.

The rimless wheel is a canonical passive dynamic walking model: a rigid axle with two sets of spokes (one per belt), representing the simplest possible bipedal locomotor. Running each belt at a different speed creates an asymmetric ground-contact condition analogous to split-belt adaptation experiments in humans and animals.

This work is written up as an IEEE-format paper (`resources/writeup.md`). The paper's central claim is that this MuJoCo environment is experimentally validated, high-throughput, and high-fidelity — making split-belt treadmill research accessible without a physical apparatus. The GitHub repo is at https://github.com/PhilipByrn3/dmcontrol_sbt.

---

## Repository Layout

```
qual-sbt/
├── sbt.py                  # Legacy simulation entry point (procedural)
├── sbt_model_gen.py        # Legacy MuJoCo model builder (XML-based)
├── sbtnew.xml              # Base MuJoCo XML used by legacy code
├── requirements.txt        # Python dependencies
├── common/                 # dm_control asset helpers (legacy)
│   └── __init__.py         # read_model() helper and ASSETS dict
├── resources/
│   ├── writeup.md          # IEEE-format paper draft (Philip Byrne)
│   └── butterfield-et-al-2022-the-split-belt-rimless-wheel.pdf
├── data/
│   ├── digitized_experimental_sbt_data.csv   # Ground-truth from Butterfield paper
│   └── sbt_data.csv                          # Last simulation run output
├── figures/                # Saved SVG/PDF plots (gitignored)
└── validation/             # Final, production version of the code
    ├── config.yaml         # All parameters in one place — edit this
    ├── create_wheel.py     # SpokeSet, Axle, AssembleWheel
    ├── create_treadmill.py # Belt, AssembleTreadmill
    ├── sbt_core.py         # SplitBeltTask, SplitBeltSim — run this
    ├── common/             # Asset XMLs (materials, skybox, visual)
    ├── create_model.py     # Scratch file
    └── test.py             # Scratch file
```

> The `validation/` folder is the active codebase. `sbt.py` / `sbt_model_gen.py` / `sbtnew.xml` are legacy references.

---

## Resources

- `resources/writeup.md` — Philip's IEEE-format paper draft describing the environment, methods, results, and discussion. Note: the Discussion section has a placeholder ("I need some help explaining why RK4 was the best choice") and the Limitations section is empty. The bibliography and author bios are placeholder text from the IEEE template.
- `resources/butterfield-et-al-2022-the-split-belt-rimless-wheel.pdf` — The Butterfield et al. (2022) experimental paper this simulation is validated against.
- `data/digitized_experimental_sbt_data.csv` — Digitized data from the Butterfield paper (columns: `exp_bdiff`, `exp_avgvelo`), used as ground truth in all plots.

---

## Physics Model

### Rimless Wheel

- **Axle**: central cylinder with 3 degrees of freedom — vertical slide (`axleZAxis`/`axle_z_joint`), forward slide (`axleYAxis`/`axle_y_joint`), and rotation around the axle (`axleHinge`/`axle_hinge_joint`).
- **Spokes**: 9 spokes per side (every 40°, starting at 0°), each a capsule 0.254 m long.
- **Spoke additions**: box geoms at the midpoint of each spoke, adding inertia/width.
- **Two sides** (from `config.yaml`):
  - Slow side (`twoalpha = 15.5°` offset) sits on the slow belt (red, position `[-0.051, 0, 0]`)
  - Fast side (`twobeta = 4.5°` offset) sits on the fast belt (blue, position `[0.051, 0, 0]` or offset)
- **Rubber pads** (optional per side): sphere geoms on spoke tips with a compliant spring joint (`stiffness=1000, damping=10`), increasing effective friction and compliance.
- Sensor: `framepos` sensor named `axlepos` tracks the axle body's world position.

### Split-Belt Treadmill

- Two box geoms (belts), each `0.05 × 10 × 0.1` m.
- Each belt has a prismatic (slide) joint along Y-axis; belt motion is imposed by directly setting `qvel` each timestep, not by motors.
- Friction: `1.15 0.1 0.1` (sliding, torsional, rolling) on belt surfaces.
- Solver: PGS (Projected Gauss-Seidel), Elliptic cone friction, RK4 integrator, timestep 0.01 s.

### Fast-Belt Position Variants

Two variants are tested:
- **No offset**: `[0.051, 0, 0]` — belts are coplanar.
- **Offset**: `[0.051, 0, 0.0049]` — fast belt is slightly raised (~5 mm), changing contact geometry.

---

## dm_control Library — Key APIs Used

### `dm_control.mjcf` (programmatic XML model building)

```python
from dm_control import mjcf

# Create a root element (becomes the <mujoco> root)
model = mjcf.RootElement(model='name')
# or load from file:
model = mjcf.from_path('sbtnew.xml')

# Add bodies, geoms, joints, sensors
body = model.worldbody.add('body', name='foo', pos=[0, 0, 0.4])
body.add('geom', type='cylinder', size=[0.01, 0.038], euler=[0, 90, 0], mass=0.095)
body.add('joint', name='slide_y', type='slide', axis=[0, 1, 0])
model.sensor.add('framepos', name='axlepos', objtype='xbody', objname='rimlesswheel')

# Attach sub-models (used in validation/ refactor)
root.attach(sub_model)

# Export to XML string
xml_str = model.to_xml_string()
```

### `dm_control.mujoco` (physics simulation)

```python
from dm_control import mujoco

physics = mujoco.Physics.from_xml_string(xml_string)

# Read/write named state
physics.named.data.qvel['axleYAxis']          # wheel forward velocity
physics.named.data.qvel['fast_belt_conveyor'] # belt velocity (set directly)
physics.named.data.qpos['axleHinge']          # axle rotation (radians)
physics.named.data.sensordata['axlepos']      # [x, y, z] position

physics.step()       # advance one timestep
physics.data.time    # current simulation time
```

### `dm_control.rl.control.Environment` and `Task`

```python
from dm_control.rl.control import Environment
from dm_control.suite.base import Task

class MyTask(Task):
    def get_observation(self, physics): ...
    def get_reward(self, physics): ...
    def initialize_episode(self, physics): ...
    def before_step(self, action, physics): ...

env = Environment(physics=physics, task=task)
env.reset()
action_spec = env.action_spec()
```

Note: In this project the Task is mostly a stub — belt speeds are controlled by directly setting `qvel` rather than through the RL action interface.

---

## Simulation Logic (`sbt.py`)

### `simulate_treadmill(timesteps, belt_diff, ...)`

Runs one simulation trial:
1. Resets environment; sets `axleYAxis` qvel = `belt_diff` (initial forward push).
2. Each step: advances physics, then forces belt qvels (`slow=0`, `fast=belt_diff`).
3. Records time and axle Y-position each step.
4. Stops early if wheel completes ≥ 3 full rotations (steady-state reached).
5. Returns `avg_wheel_velocity = final_y_position / final_time`.

### `loop_sim(...)` / `mass_simulation(...)`

Sweeps `belt_diff` from `starting_belt_diff` to `belt_diff` in steps of `bd_increment`, calling `simulate_treadmill` at each point.

### Experimental Comparison

`data/digitized_experimental_sbt_data.csv` contains digitized data from the Butterfield paper (`exp_bdiff`, `exp_avgvelo`). Simulation output is overlaid for visual and statistical comparison (Wilcoxon rank-sum and Welch's t-test via `scipy.stats`).

### Parameter Sweep in `__main__`

Runs all 4 rubber configurations (none / fast-only / slow-only / both) × 2 belt positions (no-offset / offset) = 8 conditions, plotting side-by-side for comparison.

---

## `validation/` — Final Architecture

The `validation/` directory is the final, production version of the code. It is config-driven, fully OOP, and structured to be compatible with standard RL/ML frameworks while remaining a pure physics model.

**Design principle**: components (`Axle`, `SpokeSet`, `Belt`) add geometry directly to a parent `worldbody` passed in at construction, rather than using `mjcf.attach()`. This avoids mjcf name-scoping (which would prefix joint names with the sub-model name) and keeps joint names simple and stable.

### `config.yaml`

Single source of truth for all physical and simulation parameters. Grouped into wheel parameters, belt parameters, simulation parameters, and solver settings. The default values are the validated configuration that best matches Butterfield experimental data (fast-side rubber + 4.9 mm belt offset).

### `create_wheel.py`

- **`SpokeSet`**: Builds one set of N evenly-spaced spokes for one side of the wheel. Takes the axle body, side sign, offset angle, rubber flag, colour, and config. Adds spoke capsules, spoke addition boxes, and optional rubber pad spheres to the cluster body.
- **`Axle`**: Builds the central axle cylinder with its three joints (`axleZAxis`, `axleYAxis`, `axleHinge`). Takes `worldbody` and config.
- **`AssembleWheel`**: Instantiates `Axle` and two `SpokeSet` instances (slow + fast). Exposes class-level joint name constants: `JOINT_Z`, `JOINT_Y`, `JOINT_HINGE`, `BODY_NAME`.

### `create_treadmill.py`

- **`Belt`**: Builds one belt body (box geom + slide joint). xpos and zoffset are configurable. Joint name is `{name}_conveyor`.
- **`AssembleTreadmill`**: Instantiates slow and fast belts. Exposes `SLOW_JOINT` and `FAST_JOINT` class constants.

### `sbt_core.py`

- **`SplitBeltTask`**: Minimal `dm_control.suite.base.Task` stub. `get_observation()` returns a dict of wheel state variables; `get_reward()` returns forward velocity as a placeholder. Compatible with RL frameworks.
- **`SplitBeltSim`**: Main simulation class. `_build_model()` assembles the full MuJoCo model from config. Exposes:
  - `simulate_trial(belt_diff)` → avg steady velocity for one belt diff value
  - `run_sweep()` → full belt_diff sweep, prints progress, returns `(bdiffs, velocities)`
  - `load_experimental_data()` → Butterfield CSV as numpy arrays
  - `get_stats(exp, sim)` → Wilcoxon rank-sum + Welch's t-test results dict
  - `plot_comparison(...)` → scatter plot of sim vs experimental, optionally saved
  - `save_sim_data(...)` → writes sweep results to CSV
  - `launch_viewer()` → opens dm_control interactive 3-D viewer
  - `env` attribute → the dm_control `Environment` (for RL use)
  - `physics` attribute → the `mujoco.Physics` object (for direct state access)

### Running the validated simulation

```bash
cd qual-sbt/validation
python sbt_core.py
# → prints per-step progress, stats, saves SVG to ../figures/
```

---

## Running the Simulation

```bash
cd qual-sbt
python sbt.py           # runs mass_simulation, saves plot to figures/
```

For the validation/refactored version:
```bash
cd qual-sbt/validation
python sbt_core.py      # launches dm_control viewer
```

---

## Dependencies

- `dm_control==1.0.22` — DeepMind's MuJoCo Python bindings and MJCF tools
- `numpy`, `matplotlib`, `pandas`, `scipy`

Install: `pip install -r requirements.txt`

MuJoCo itself is bundled with `dm_control` (no separate install needed).

---

## Key Design Decisions & Notes

- **Belt velocity is kinematically imposed** (directly writing `qvel`), not motor-driven. This is a deliberate simplification — the belt geoms have a near-zero-range slide joint that keeps them kinematically constrained.
- **Spoke angular offsets** (`twoalpha`, `twobeta`) are the key asymmetry parameter. The slow-side spokes are more "forward-leaning" (15.5° vs 4.5°) to match the physical device.
- **Rubber pads** are modeled as small spheres with a compliant spring joint at spoke tips. They affect friction and energy absorption at ground contact.
- **The `common/` directory** mimics the structure of dm_control's built-in suite domains; `__init__.py` provides `read_model()` and the `ASSETS` dict used to embed XML assets.
- **Figures are gitignored**; simulation output CSVs in `data/` are also gitignored. Only source code and the experimental data CSV are tracked.

## The Bouncing Problem (Critical Design Context)

The most significant challenge encountered was the **"bouncing problem"**: at high belt speed differences, the fast-side spoke strikes the moving belt and bounces rather than maintaining contact. This reduces energy transfer from the belt to the wheel, distorts velocity measurements, and was also observed in the physical experiment.

**Solution**: Rubber pad sphere geoms on fast-side spoke tips act as spring-mass dampers (`stiffness=1000, damping=10`), absorbing impact energy while maintaining contact.

**Why fast-side only**: Rubber on both sides causes the wheel to gain energy monotonically (velocity keeps rising with Δv instead of peaking and falling), because the slow-side pads reduce energy dissipation on the stationary belt too much. The desired qualitative behavior — wheel gains energy from fast belt, dissipates some on slow belt at high Δv — requires the slow side to dissipate energy on impact.

**Alternative fix**: A 4.9 mm Z-offset of the fast belt (`[0.051, 0, 0.0049]`) changes the contact geometry and also reduces bouncing. Best quantitative match to Butterfield data comes from combining both (fast rubber + offset).

## Average Steady Velocity Calculation

The analytical formula for average steady velocity was considered:

$${}^{S}v_{\mathrm{avg}}=\frac{2l(\sin\alpha+\sin\beta)-T_F \Delta v_{TM}}{T_S+T_F}$$

where $l$ is spoke length, $\alpha$ and $\beta$ are internal spoke angles, $\Delta v_{TM}$ is belt speed difference, and $T_S$/$T_F$ are contact times with slow/fast belt. This was abandoned because accurately measuring $T_S$ and $T_F$ during bouncing was impractical.

**Instead**: `avg_wheel_velocity = final_y_position / final_time` — the same method used experimentally. Measurements start after 50 timesteps (stabilization) and end at 3 full rotations or 2000 timesteps.

## Solver / Integrator Choices

RK4 integrator + PGS solver + Elliptic cone friction produced the best match to experimental results. The writeup still needs a physics explanation for why RK4 outperformed other integrators (noted as a TODO in the paper's Discussion section).
