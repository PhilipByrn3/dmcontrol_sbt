# Split-Belt Treadmill Rimless Wheel — MuJoCo Environment

An open-source, experimentally validated split-belt treadmill environment built with [dm_control](https://github.com/google-deepmind/dm_control) and MuJoCo. The environment simulates a passive rimless wheel walking on a split-belt treadmill and is validated against the physical experimental results of Butterfield et al. (2022). It is designed for high-throughput simulation and is compatible with reinforcement learning frameworks.

**Paper:** *An Experimentally Validated Split-Belt Treadmill Environment in MuJoCo* — Philip Byrne, Fairfield University

**Repository:** https://github.com/PhilipByrn3/dmcontrol_sbt

---

## Table of Contents

1. [Overview](#overview)
2. [Repository Layout](#repository-layout)
3. [Installation](#installation)
4. [Quick Start](#quick-start)
5. [Configuration](#configuration)
6. [Simulation Modes](#simulation-modes)
7. [Programmatic Usage](#programmatic-usage)
8. [Visual Inspection](#visual-inspection)
9. [Output Files](#output-files)
10. [Physics Model](#physics-model)

---

## Overview

The environment places a rimless wheel — a canonical model of passive bipedal locomotion — on a split-belt treadmill where the two belts can run at different speeds. By sweeping the belt speed difference and recording the wheel's average steady-state velocity, the simulation reproduces the characteristic velocity curve measured physically by Butterfield et al. (2022).

Key features:

- Config-driven: all physical and simulation parameters live in a single `config.yaml`
- Four rubber contact configurations and two belt offset conditions are sweep-testable out of the box
- Tied-belt mode runs both belts at equal speed for symmetric baseline comparison
- Outputs per-trial EPS plots and a combined grid plot, organised into timestamped folders
- `dm_control` `Environment` / `Task` interface makes the environment drop-in compatible with RL frameworks

---

## Repository Layout

```
qual-sbt/
├── requirements.txt
├── data/
│   ├── Experimental.csv              # Digitized Butterfield et al. (2022) data
│   └── sbt_data.csv                  # Last simulation run output (gitignored)
├── figures/                          # Saved plots (gitignored)
├── resources/
│   ├── writeup.md                    # IEEE paper draft
│   └── butterfield-et-al-2022-...pdf # Reference paper
└── sbt/                              # Active codebase
    ├── config.yaml                   # All parameters — edit this
    ├── sbt_core.py                   # Simulation classes and entry point
    ├── analysis.py                   # Statistical comparison functions
    ├── plotting.py                   # All plotting functions
    ├── create_wheel.py               # Rimless wheel model builder
    ├── create_treadmill.py           # Split-belt treadmill model builder
    └── render.py                     # Visual inspection (frames or interactive viewer)
```

---

## Installation

Python 3.9+ is required. It is strongly recommended to use a virtual environment.

```bash
git clone https://github.com/PhilipByrn3/dmcontrol_sbt.git
cd dmcontrol_sbt/qual-sbt

python -m venv venv
source venv/bin/activate        # Windows: venv\Scripts\activate

pip install -r requirements.txt
```

MuJoCo is bundled with `dm_control` — no separate MuJoCo installation is needed.

---

## Quick Start

```bash
cd qual-sbt/sbt
python sbt_core.py
```

This runs the simulation mode configured in `config.yaml`, prints per-trial progress and statistics to stdout, and saves EPS plots to a timestamped folder inside `figures/`.

---

## Configuration

All parameters are set in `sbt/config.yaml`. You should never need to edit the Python source files to change simulation behaviour.

### Wheel Parameters

| Parameter | Default | Description |
|---|---|---|
| `axle_length` | `0.038` | Half-length of axle cylinder (m) |
| `twoalpha` | `15.5` | Angular offset of slow-side spoke cluster (deg) |
| `twobeta` | `4.5` | Angular offset of fast-side spoke cluster (deg) |
| `spoke_length` | `0.254` | Spoke length (m) |
| `spoke_angle_step` | `40` | Angular spacing between consecutive spokes (deg) — 9 spokes per side |
| `component_mass` | `0.095405` | Mass per spoke / spoke-addition component (kg) |
| `wheel_height` | `0.38` | Initial axle height above ground (m) |

### Rubber Contact Parameters

Rubber pads are modelled as spring-mass-damper spheres at the tip of each spoke.

| Parameter | Default | Description |
|---|---|---|
| `slow_side_rubber` | `true` | Attach rubber pads to slow-side spokes |
| `fast_side_rubber` | `true` | Attach rubber pads to fast-side spokes |
| `rubber_size` | `[0.017]` | Sphere radius (m) |
| `rubber_friction` | `[1, 1, 0.1]` | Sliding, torsional, rolling friction |
| `smd_stiffness` | `50` | Spring stiffness |
| `smd_damping` | `10` | Damping coefficient |
| `smd_range` | `[0, 0.015]` | Slide joint range (m) |

### Belt Parameters

| Parameter | Default | Description |
|---|---|---|
| `slow_belt_xpos` | `-0.051` | X-position of slow (red) belt (m) |
| `fast_belt_xpos` | `0.051` | X-position of fast (blue) belt (m) |
| `fast_belt_zoffset` | `0.0049` | Z-offset of fast belt above slow belt (m). Set to `0.0` for coplanar. |
| `belt_friction` | `[1.15, 0.1, 0.1]` | Sliding, torsional, rolling friction |
| `belt_mass` | `3.0` | Belt mass (kg) |

### Simulation Parameters

| Parameter | Default | Description |
|---|---|---|
| `slow_belt_speed` | `0.0` | Slow belt speed — always stationary (m/s) |
| `starting_belt_diff` | `0.15` | Sweep start value (m/s) |
| `max_belt_diff` | `1.1` | Sweep end value (m/s) |
| `belt_diff_increment` | `0.05` | Sweep step size (m/s) |
| `max_timesteps` | `2000` | Per-trial timestep cap (20 s at 0.01 s/step) |
| `target_rotations` | `12` | End trial early after this many full wheel rotations |

### Output Options

| Parameter | Default | Description |
|---|---|---|
| `run_offset_comparison` | `true` | Run all 8 rubber × offset configurations |
| `run_tied_belt` | `true` | Run tied-belt sweep (both belts at equal speed) |
| `run_all_rubber` | `true` | Run all 4 rubber configurations |
| `show_plot` | `false` | Open plot window after saving |

**Priority order:** `run_offset_comparison` → `run_tied_belt` → `run_all_rubber` → single sweep. Only the first `true` mode runs.

### Solver / Integrator

| Parameter | Default | Description |
|---|---|---|
| `timestep` | `0.01` | Simulation timestep (s) |
| `integrator` | `RK4` | Numerical integrator |
| `solver` | `PGS` | Constraint solver |
| `cone` | `Elliptic` | Contact friction cone |
| `iterations` | `50` | Maximum solver iterations |

These settings produced the best match to experimental data. Change with caution.

---

## Simulation Modes

### Single Split-Belt Sweep

Set all mode flags to `false` in `config.yaml`. Runs one sweep with the rubber configuration specified by `slow_side_rubber` and `fast_side_rubber`. Outputs one plot and prints R², RMSE, MAE, and p-values.

```yaml
run_offset_comparison: false
run_tied_belt: false
run_all_rubber: false
```

### All Rubber Configurations

Runs four sweeps (no rubber / fast only / slow only / both), saves individual EPS per config plus a combined 2×2 grid.

```yaml
run_offset_comparison: false
run_tied_belt: false
run_all_rubber: true
```

### Rubber × Offset Comparison

Runs all 8 combinations (4 rubber configs × 2 offset conditions). Saves 8 individual EPS files plus a combined 4×2 grid.

```yaml
run_offset_comparison: true
```

### Tied-Belt Sweep

Both belts run at the same speed (no split). The sweep range reuses `starting_belt_diff` → `max_belt_diff`. Useful as a symmetric baseline.

```yaml
run_offset_comparison: false
run_tied_belt: true
```

---

## Programmatic Usage

### Basic sweep

```python
from sbt_core import SplitBeltSim, load_config
from analysis import get_stats
from plotting import plot_comparison

config = load_config()
sim = SplitBeltSim(config)

# Split-belt sweep
bdiffs, velocities = sim.run_sweep()

# Tied-belt sweep
speeds, tied_velocities = sim.run_tied_sweep()

# Load experimental data
exp_bdiffs, exp_velocities = sim.load_experimental_data()

# Statistical comparison
s = get_stats(exp_velocities, velocities, exp_bdiffs, bdiffs)
print(s['fit'])   # {'r2': ..., 'rmse': ..., 'mae': ..., 'n_points': ...}

# Plot
plot_comparison(bdiffs, velocities, exp_bdiffs, exp_velocities,
                save_path='my_plot.eps', show=True)
```

### Run a single trial

```python
avg_velocity = sim.simulate_trial(belt_diff=0.5)       # split-belt
avg_velocity = sim.simulate_trial_tied(belt_speed=0.5) # tied-belt
```

### Override config parameters on the fly

```python
cfg = {**load_config(), 'slow_side_rubber': False, 'fast_side_rubber': True}
sim = SplitBeltSim(cfg)
```

### RL framework integration

`SplitBeltSim` exposes a standard `dm_control` `Environment`:

```python
sim = SplitBeltSim(load_config())
env = sim.env          # dm_control.rl.control.Environment
physics = sim.physics  # dm_control.mujoco.Physics

timestep = env.reset()
obs = timestep.observation
# {'axle_y_pos', 'axle_z_pos', 'axle_velocity', 'axle_rotation'}

action = None  # no actuators; belt speeds are set directly
timestep = env.step(action)
```

Belt speeds can be set directly at any point:

```python
from create_treadmill import AssembleTreadmill
physics.named.data.qvel[AssembleTreadmill.SLOW_JOINT] = 0.0
physics.named.data.qvel[AssembleTreadmill.FAST_JOINT] = 0.5
```

---

## Visual Inspection

`render.py` provides two inspection modes:

```bash
cd qual-sbt/sbt

# Static 4-camera grid saved to figures/
python render.py --mode frames

# Interactive 3-D viewer (orbit: left-drag, zoom: scroll, pan: right-drag)
python render.py --mode viewer
```

The default mode can also be changed by editing the `MODE` constant at the top of `render.py`.

---

## Output Files

Each run creates a timestamped folder inside `figures/`:

```
figures/
└── rubber_comparison_2026-06-12__14-30-00/
    ├── no_rubber.eps
    ├── fast_rubber_only.eps
    ├── slow_rubber_only.eps
    ├── both_rubber.eps
    └── all_rubber_configs.eps       # combined grid
```

Simulation data is saved to `data/sbt_data.csv` (single sweep mode only).

---

## Physics Model

### Rimless Wheel

The rimless wheel has two sets of 9 spokes (one per belt), connected by a central axle. The axle has three degrees of freedom: vertical slide (`axleZAxis`), forward slide (`axleYAxis`), and rotation about the axle axis (`axleHinge`). Lateral and roll motion are constrained, matching the physical apparatus.

### Split-Belt Treadmill

Two belt bodies, each with a prismatic slide joint along the Y-axis. Belt motion is kinematically imposed by writing directly to `qvel` each timestep — the joint range is near-zero so the belt does not translate in space, matching how a real treadmill belt circulates around its drive wheels.

### Contact Model

Rubber pads are modelled as sphere geoms at spoke tips, connected to the spoke body via a slide joint configured as a spring-mass damper. This absorbs impact energy at high belt speed differences, reducing the bouncing that occurs when a spoke strikes a fast-moving belt.

### Solver

RK4 integration with PGS solver and elliptic friction cone. This combination produced the best quantitative agreement with Butterfield et al. experimental data across all tested configurations.
