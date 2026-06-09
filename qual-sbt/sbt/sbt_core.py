"""
sbt_core.py
Core simulation environment for the split-belt rimless wheel.

SplitBeltSim builds the MuJoCo model and wraps it in a standard dm_control
Environment / Task. Belt speeds are set by directly writing
physics.named.data.qvel rather than through the action interface.

Validation workflow:
    python sbt_core.py

Modes (set in config.yaml):
    run_offset_comparison — 8 sweeps: all rubber × offset combinations
    run_all_rubber        — 4 sweeps: all rubber configurations
    run_tied_belt         — tied-belt sweep: both belts at the same speed
    (default)             — single split-belt sweep

See also:
    plotting.py  — all plotting functions
    analysis.py  — statistical comparison helpers
"""

import math
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

from dm_control import mujoco, mjcf, viewer
from dm_control.rl.control import Environment
from dm_control.suite.base import Task

from create_wheel import AssembleWheel
from create_treadmill import AssembleTreadmill
from analysis import get_stats, print_stats
from plotting import (plot_comparison, plot_tied_belt,
                      plot_permutations, plot_offset_permutations)

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

CONFIG_PATH   = Path(__file__).with_name('config.yaml')
EXP_DATA_PATH = Path(__file__).parent.parent / 'data' / 'Experimental.csv'
FIGURES_PATH  = Path(__file__).parent.parent / 'figures'
SIM_DATA_PATH = Path(__file__).parent.parent / 'data' / 'sbt_data.csv'


def load_config() -> dict:
    """Load simulation parameters from config.yaml."""
    with CONFIG_PATH.open('r') as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# dm_control Task
# ---------------------------------------------------------------------------

class SplitBeltTask(Task):
    """
    Minimal dm_control Task stub for the split-belt treadmill.

    Belt velocities are not action-driven; SplitBeltSim overwrites
    physics.named.data.qvel directly on each timestep.

    get_observation() returns a dict of wheel state variables — enough for
    a policy to observe the system if used in an RL context.
    get_reward() returns forward velocity as a placeholder.
    """

    def before_step(self, action, physics):
        pass

    def get_observation(self, physics):
        return {
            'axle_y_pos':    physics.named.data.sensordata['axlepos'][1].copy(),
            'axle_z_pos':    physics.named.data.sensordata['axlepos'][2].copy(),
            'axle_velocity': physics.named.data.qvel[AssembleWheel.JOINT_Y].copy(),
            'axle_rotation': physics.named.data.qpos[AssembleWheel.JOINT_HINGE].copy(),
        }

    def get_reward(self, physics):
        return float(physics.named.data.qvel[AssembleWheel.JOINT_Y])

    def initialize_episode(self, physics):
        pass


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

class SplitBeltSim:
    """
    Split-belt treadmill simulation environment.

    Public attributes:
        env     — dm_control.rl.control.Environment (RL-compatible)
        physics — dm_control.mujoco.Physics
        config  — dict of simulation parameters

    Key methods:
        simulate_trial(belt_diff)       → float  split-belt: slow=0, fast=belt_diff
        simulate_trial_tied(belt_speed) → float  tied-belt:  both belts = belt_speed
        run_sweep()                     → (bdiffs, velocities)
        run_tied_sweep()                → (speeds, velocities)
        load_experimental_data()        → (bdiffs, velocities)
        save_sim_data(bdiffs, velocities)
        launch_viewer()
    """

    def __init__(self, config: dict):
        self.config  = config
        self._model  = self._build_model()
        self.physics = mujoco.Physics.from_xml_string(self._model.to_xml_string())
        self._task   = SplitBeltTask()
        self.env     = Environment(physics=self.physics, task=self._task)

    # ------------------------------------------------------------------
    # Model construction
    # ------------------------------------------------------------------

    def _build_model(self) -> mjcf.RootElement:
        """Assemble the MuJoCo model: solver settings + treadmill + wheel."""
        model = mjcf.RootElement(model='sbt_rimless_wheel')

        model.compiler.autolimits = True
        model.compiler.angle      = 'degree'

        model.option.gravity    = [0, 0, -9.81]
        model.option.timestep   = self.config['timestep']
        model.option.integrator = self.config['integrator']
        model.option.cone       = self.config['cone']
        model.option.solver     = self.config['solver']
        model.option.iterations = self.config['iterations']

        AssembleTreadmill(model.worldbody, self.config)
        AssembleWheel(model.worldbody, self.config)

        model.sensor.add('framepos', name='axlepos',
                         objtype='xbody', objname=AssembleWheel.BODY_NAME)

        return model

    # ------------------------------------------------------------------
    # Core trial runner (shared logic)
    # ------------------------------------------------------------------

    def _run_trial(self, slow_speed: float, fast_speed: float) -> float:
        """
        Shared trial loop. Resets the environment, gives the wheel an
        initial push at fast_speed, then steps until target_rotations or
        max_timesteps is reached.

        Returns:
            avg_velocity = final_y_position / final_elapsed_time
        """
        self.env.reset()

        max_steps = self.config['max_timesteps']
        target    = self.config['target_rotations']

        self.physics.named.data.qvel[AssembleWheel.JOINT_Y] = fast_speed

        time_list = []
        pos_list  = []

        for _ in range(max_steps):
            self.physics.step()

            self.physics.named.data.qvel[AssembleTreadmill.SLOW_JOINT] = slow_speed
            self.physics.named.data.qvel[AssembleTreadmill.FAST_JOINT] = fast_speed

            rotation = (
                self.physics.named.data.qpos[AssembleWheel.JOINT_HINGE][0]
                / (2.0 * math.pi) * -1.0
            )

            time_list.append(round(self.physics.data.time, 8))
            pos_list.append(float(self.physics.named.data.sensordata['axlepos'][1]))

            if rotation >= target:
                break

        return pos_list[-1] / time_list[-1]

    # ------------------------------------------------------------------
    # Public simulation methods
    # ------------------------------------------------------------------

    def simulate_trial(self, belt_diff: float) -> float:
        """
        Split-belt trial: slow belt stationary, fast belt at belt_diff m/s.

        Args:
            belt_diff: fast belt speed (m/s)

        Returns:
            Average steady-state wheel velocity (m/s)
        """
        slow_speed = float(self.config['slow_belt_speed'])
        return self._run_trial(slow_speed, slow_speed + belt_diff)

    def simulate_trial_tied(self, belt_speed: float) -> float:
        """
        Tied-belt trial: both belts running at the same speed.

        No speed difference between belts — symmetric ground contact.
        The wheel is pushed at belt_speed and both belts run at belt_speed.

        Args:
            belt_speed: speed of both belts (m/s)

        Returns:
            Average steady-state wheel velocity (m/s)
        """
        return self._run_trial(belt_speed, belt_speed)

    def run_sweep(self) -> tuple:
        """
        Split-belt sweep: slow=0, fast steps from starting_belt_diff to
        max_belt_diff in belt_diff_increment steps.

        Returns:
            (bdiffs, velocities): numpy arrays
        """
        return self._sweep(self.simulate_trial, label='belt_diff')

    def run_tied_sweep(self) -> tuple:
        """
        Tied-belt sweep: both belts step from starting_belt_diff to
        max_belt_diff in belt_diff_increment steps.

        Reuses the same sweep range as the split-belt sweep so results
        are directly comparable on the same x-axis.

        Returns:
            (speeds, velocities): numpy arrays
        """
        return self._sweep(self.simulate_trial_tied, label='belt_speed')

    def _sweep(self, trial_fn, label: str = 'belt_diff') -> tuple:
        """Generic sweep loop shared by run_sweep and run_tied_sweep."""
        start = self.config['starting_belt_diff']
        end   = self.config['max_belt_diff']
        step  = self.config['belt_diff_increment']

        xs         = []
        velocities = []
        x = start

        while x <= end + 1e-9:
            v = trial_fn(x)
            xs.append(round(x, 6))
            velocities.append(v)
            print(f'  {label}={xs[-1]:.3f} m/s  →  avg_vel={v:.4f} m/s')
            x = round(x + step, 6)

        return np.array(xs), velocities

    # ------------------------------------------------------------------
    # Data I/O
    # ------------------------------------------------------------------

    def load_experimental_data(self, path=None) -> tuple:
        """
        Load Butterfield et al. (2022) digitized experimental data from
        Experimental.csv (columns: index, X, Y).

        Returns:
            (exp_bdiffs, exp_velocities): numpy arrays
        """
        p  = Path(path) if path else EXP_DATA_PATH
        df = pd.read_csv(p, index_col=0)
        return df['X'].values.astype(float), df['Y'].values.astype(float)

    def save_sim_data(self, xs, velocities, path=None):
        """Save simulation sweep results to CSV."""
        p = Path(path) if path else SIM_DATA_PATH
        pd.DataFrame({'X': xs, 'AvgVelo': velocities}).to_csv(p, index=False)
        print(f'Simulation data saved → {p}')

    # ------------------------------------------------------------------
    # Viewer
    # ------------------------------------------------------------------

    def launch_viewer(self):
        """Launch the dm_control interactive 3-D viewer."""
        viewer.launch(self.env)


# ---------------------------------------------------------------------------
# Permutation sweep runners
# ---------------------------------------------------------------------------

RUBBER_PERMUTATIONS = [
    (False, False, 'No rubber'),
    (False, True,  'Fast rubber only'),
    (True,  False, 'Slow rubber only'),
    (True,  True,  'Both rubber'),
]

OFFSET_PERMUTATIONS = [
    (0.0,    'No offset'),
    (0.0049, '4.9 mm offset'),
]


def run_rubber_permutations(config: dict) -> list:
    """Run a full split-belt sweep for each rubber attachment configuration."""
    results = []
    for slow_r, fast_r, label in RUBBER_PERMUTATIONS:
        cfg = {**config, 'slow_side_rubber': slow_r, 'fast_side_rubber': fast_r}
        print(f'\n── {label} ──')
        sim = SplitBeltSim(cfg)
        bdiffs, velocities = sim.run_sweep()
        results.append({'label': label, 'slow_rubber': slow_r,
                        'fast_rubber': fast_r, 'bdiffs': bdiffs,
                        'velocities': velocities})
    return results


def run_offset_permutations(config: dict) -> list:
    """Run a full split-belt sweep for all 8 rubber × offset combinations."""
    results = []
    for offset, offset_label in OFFSET_PERMUTATIONS:
        for slow_r, fast_r, rubber_label in RUBBER_PERMUTATIONS:
            cfg = {**config, 'slow_side_rubber': slow_r,
                   'fast_side_rubber': fast_r, 'fast_belt_zoffset': offset}
            label = f'{rubber_label} — {offset_label}'
            print(f'\n── {label} ──')
            sim = SplitBeltSim(cfg)
            bdiffs, velocities = sim.run_sweep()
            results.append({'label': label, 'rubber_label': rubber_label,
                            'offset_label': offset_label, 'slow_rubber': slow_r,
                            'fast_rubber': fast_r, 'offset': offset,
                            'bdiffs': bdiffs, 'velocities': velocities})
    return results


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    config                = load_config()
    RUN_ALL_RUBBER        = config['run_all_rubber']
    RUN_OFFSET_COMPARISON = config['run_offset_comparison']
    RUN_TIED_BELT         = config.get('run_tied_belt', False)
    SHOW_PLOT             = config['show_plot']
    timestamp             = datetime.now().strftime('%Y-%m-%d__%H-%M-%S')

    sim_base = SplitBeltSim(config)
    exp_bdiffs, exp_velocities = sim_base.load_experimental_data()

    if RUN_TIED_BELT:
        save_dir = FIGURES_PATH / f'tied_belt_{timestamp}'
        save_dir.mkdir(parents=True, exist_ok=True)
        print(f'Saving plots to {save_dir}')

        print('Running tied-belt sweep (both belts at equal speed)...')
        speeds, velocities = sim_base.run_tied_sweep()

        sim_base.save_sim_data(speeds, velocities,
                               path=save_dir.parent.parent / 'data' / 'tied_belt_data.csv')

        print('\nSaving plot...')
        plot_tied_belt(speeds, velocities,
                       save_path=save_dir / 'tied_belt.svg', show=SHOW_PLOT)

    elif RUN_OFFSET_COMPARISON:
        save_dir = FIGURES_PATH / f'offset_comparison_{timestamp}'
        save_dir.mkdir(parents=True, exist_ok=True)
        print(f'Saving plots to {save_dir}')

        print('Running sweep for all rubber × offset configurations (8 total)...')
        results = run_offset_permutations(config)

        print('\nStatistical comparison per configuration:')
        for r in results:
            s = get_stats(exp_velocities, r['velocities'], exp_bdiffs, r['bdiffs'])
            print_stats(r['label'], s)

        print('\nSaving plots...')
        plot_offset_permutations(results, exp_bdiffs, exp_velocities,
                                 save_dir=save_dir, show=SHOW_PLOT)

    elif RUN_ALL_RUBBER:
        save_dir = FIGURES_PATH / f'rubber_comparison_{timestamp}'
        save_dir.mkdir(parents=True, exist_ok=True)
        print(f'Saving plots to {save_dir}')

        print('Running sweep for all rubber configurations...')
        results = run_rubber_permutations(config)

        print('\nStatistical comparison per configuration:')
        for r in results:
            s = get_stats(exp_velocities, r['velocities'], exp_bdiffs, r['bdiffs'])
            print_stats(r['label'], s, width=22)

        print('\nSaving plots...')
        plot_permutations(results, exp_bdiffs, exp_velocities,
                          save_dir=save_dir, show=SHOW_PLOT)

    else:
        save_dir = FIGURES_PATH / f'single_sweep_{timestamp}'
        save_dir.mkdir(parents=True, exist_ok=True)
        print(f'Saving plots to {save_dir}')

        print('Running split-belt sweep...')
        bdiffs, velocities = sim_base.run_sweep()

        print('\nStatistical comparison (simulation vs. Butterfield experimental):')
        s   = get_stats(exp_velocities, velocities, exp_bdiffs, bdiffs)
        fit = s.get('fit', {})
        rs  = s['ranksums']
        tt  = s['ttest_ind']
        print(f"  R²                : {fit.get('r2', float('nan')):.4f}  "
              f"(n={fit.get('n_points', '?')} overlapping points)")
        print(f"  RMSE              : {fit.get('rmse', float('nan')):.4f} m/s")
        print(f"  MAE               : {fit.get('mae', float('nan')):.4f} m/s")
        print(f"  Wilcoxon rank-sum : stat={rs['statistic']:+.4f},  p={rs['p_value']:.4f}")
        print(f"  Welch's t-test    : stat={tt['statistic']:+.4f},  p={tt['p_value']:.4f}")

        sim_base.save_sim_data(bdiffs, velocities)

        print('\nSaving plot...')
        plot_comparison(bdiffs, velocities, exp_bdiffs, exp_velocities,
                        save_path=save_dir / 'validation.svg', show=SHOW_PLOT)


if __name__ == '__main__':
    main()
