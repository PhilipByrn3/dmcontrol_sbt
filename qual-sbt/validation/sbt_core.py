"""
sbt_core.py
Core simulation environment for the split-belt rimless wheel.

SplitBeltSim builds the MuJoCo model and wraps it in a standard dm_control
Environment / Task. The Task stub makes this environment compatible with RL
frameworks without adding any learning machinery to what is purely a physics
model. Belt speeds are set by directly writing physics.named.data.qvel rather
than through the action interface.

Validation workflow:
    python sbt_core.py
    → sweeps belt speed differences, compares to Butterfield et al. data,
      saves a plot to figures/ and prints statistical test results.

Programmatic usage:
    from sbt_core import SplitBeltSim, load_config
    sim = SplitBeltSim(load_config())
    bdiffs, velocities = sim.run_sweep()
    sim.plot_comparison(bdiffs, velocities, *sim.load_experimental_data())

Interactive viewer:
    sim.launch_viewer()
"""

import math
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import yaml
from scipy import stats

from dm_control import mujoco, mjcf, viewer
from dm_control.rl.control import Environment
from dm_control.suite.base import Task

from create_wheel import AssembleWheel
from create_treadmill import AssembleTreadmill

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

CONFIG_PATH   = Path(__file__).with_name('config.yaml')
EXP_DATA_PATH = Path(__file__).parent.parent / 'data' / 'digitized_experimental_sbt_data.csv'
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

    Belt velocities are not action-driven; SplitBeltSim.simulate_trial()
    overwrites physics.named.data.qvel directly on each timestep.

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

    Builds a parametric MuJoCo model from config, sweeps belt speed
    differences, and compares steady-state wheel velocity to the
    Butterfield et al. (2022) experimental data.

    Public attributes:
        env     — dm_control.rl.control.Environment (RL-compatible)
        physics — dm_control.mujoco.Physics
        config  — dict of simulation parameters

    Key methods:
        simulate_trial(belt_diff)              → float (avg velocity)
        run_sweep()                            → (bdiffs, velocities)
        plot_comparison(...)                   → (fig, ax)
        load_experimental_data()               → (bdiffs, velocities)
        get_stats(exp_velocities, sim_velocities) → dict
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

        # Compiler
        model.compiler.autolimits = True
        model.compiler.angle      = 'degree'

        # Solver / integrator (RK4 + PGS + Elliptic: best experimental match)
        model.option.gravity    = [0, 0, -9.81]
        model.option.timestep   = self.config['timestep']
        model.option.integrator = self.config['integrator']
        model.option.cone       = self.config['cone']
        model.option.solver     = self.config['solver']
        model.option.iterations = self.config['iterations']

        # Treadmill first so belts are underneath the wheel at load time
        AssembleTreadmill(model.worldbody, self.config)

        # Wheel
        AssembleWheel(model.worldbody, self.config) ## NEEDS TO ALLOW FOR CHANGING RUBBER PLACEMENT

        # Axle position sensor (Y-component gives forward displacement)
        model.sensor.add(
            'framepos',
            name='axlepos',
            objtype='xbody',
            objname=AssembleWheel.BODY_NAME
        )

        return model

    # ------------------------------------------------------------------
    # Simulation
    # ------------------------------------------------------------------

    def simulate_trial(self, belt_diff: float) -> float:
        """
        Run one trial at a fixed belt speed difference.

        The slow belt is stationary; the fast belt runs at `belt_diff` m/s.
        The wheel receives an initial forward impulse equal to the fast belt
        speed, then runs freely until it completes `target_rotations` full
        rotations or `max_timesteps` is reached.

        Average steady velocity is computed as:
            avg_velocity = final_y_position / final_elapsed_time
        This matches the method used by Butterfield et al. experimentally.

        Args:
            belt_diff: fast belt speed (m/s); slow belt is always 0 m/s

        Returns:
            Average steady-state wheel velocity (m/s)
        """
        self.env.reset()

        slow_speed = float(self.config['slow_belt_speed'])
        fast_speed = slow_speed + belt_diff
        max_steps  = self.config['max_timesteps']
        target     = self.config['target_rotations']

        # Initial push: give wheel forward velocity equal to fast belt speed
        self.physics.named.data.qvel[AssembleWheel.JOINT_Y] = fast_speed

        time_list = []
        pos_list  = []

        for _ in range(max_steps):
            self.physics.step()

            # Kinematically enforce belt velocities on every timestep
            self.physics.named.data.qvel[AssembleTreadmill.SLOW_JOINT] = slow_speed
            self.physics.named.data.qvel[AssembleTreadmill.FAST_JOINT] = fast_speed

            # Rotation count: negative hinge direction = forward rolling
            rotation = (
                self.physics.named.data.qpos[AssembleWheel.JOINT_HINGE][0]
                / (2.0 * math.pi) * -1.0
            )

            time_list.append(round(self.physics.data.time, 8))
            pos_list.append(
                float(self.physics.named.data.sensordata['axlepos'][1])
            )

            if rotation >= target:
                break

        return pos_list[-1] / time_list[-1]

    def run_sweep(self) -> tuple:
        """
        Sweep belt speed differences from config['starting_belt_diff'] to
        config['max_belt_diff'] in steps of config['belt_diff_increment'].

        Prints progress to stdout.

        Returns:
            (bdiffs, velocities): numpy array of belt diffs and list of
            corresponding average steady-state wheel velocities
        """
        start = self.config['starting_belt_diff']
        end   = self.config['max_belt_diff']
        step  = self.config['belt_diff_increment']

        bdiffs     = []
        velocities = []
        bd = start

        while bd <= end + 1e-9:
            v = self.simulate_trial(bd)
            bdiffs.append(round(bd, 6))
            velocities.append(v)
            print(f'  belt_diff={bdiffs[-1]:.3f} m/s  →  avg_vel={v:.4f} m/s')
            bd = round(bd + step, 6)

        return np.array(bdiffs), velocities

    # ------------------------------------------------------------------
    # Data I/O
    # ------------------------------------------------------------------

    def load_experimental_data(self, path=None) -> tuple:
        """
        Load Butterfield et al. (2022) digitized experimental data.

        Returns:
            (exp_bdiffs, exp_velocities): numpy arrays
        """
        p = Path(path) if path else EXP_DATA_PATH
        df = pd.read_csv(p)
        return df['exp_bdiff'].values, df['exp_avgvelo'].values

    def save_sim_data(self, bdiffs, velocities, path=None):
        """Save simulation sweep results to CSV."""
        p = Path(path) if path else SIM_DATA_PATH
        pd.DataFrame({'BDiffs': bdiffs, 'AvgVelo': velocities}).to_csv(p, index=False)
        print(f'Simulation data saved → {p}')

    # ------------------------------------------------------------------
    # Statistical comparison
    # ------------------------------------------------------------------

    def get_stats(self, exp_velocities, sim_velocities) -> dict:
        """
        Compare simulation and experimental velocity distributions.

        Runs Wilcoxon rank-sum (non-parametric) and Welch's t-test.

        Returns:
            dict with keys 'ranksums' and 'ttest_ind', each containing
            {'statistic': float, 'p_value': float}
        """
        exp = np.asarray(exp_velocities)
        sim = np.asarray(sim_velocities)
        rs_stat, rs_pval = stats.ranksums(exp, sim)
        tt_stat, tt_pval = stats.ttest_ind(exp, sim)
        return {
            'ranksums':  {'statistic': float(rs_stat), 'p_value': float(rs_pval)},
            'ttest_ind': {'statistic': float(tt_stat), 'p_value': float(tt_pval)},
        }

    # ------------------------------------------------------------------
    # Visualisation
    # ------------------------------------------------------------------

    def plot_comparison(self, sim_bdiffs, sim_velocities,
                        exp_bdiffs, exp_velocities,
                        title: str = '',
                        save_path=None,
                        show: bool = True):
        """
        Scatter plot of simulation results overlaid on Butterfield data.

        Args:
            sim_bdiffs:      belt speed differences from run_sweep()
            sim_velocities:  corresponding simulated avg velocities
            exp_bdiffs:      experimental belt speed differences
            exp_velocities:  experimental avg velocities
            title:           optional plot title
            save_path:       path to save figure (SVG or PDF); None = don't save
            show:            if True, call plt.show() after saving

        Returns:
            (fig, ax)
        """
        fig, ax = plt.subplots(figsize=(7, 5))

        ax.scatter(exp_bdiffs, exp_velocities, s=12, color='red', zorder=3,
                   label='Butterfield et al. (experimental)')
        ax.scatter(sim_bdiffs, sim_velocities, s=12, color='blue', zorder=3,
                   label='MuJoCo simulation')

        ax.set_xlabel('Belt Speed Difference (m/s)', fontsize=13)
        ax.set_ylabel('Average Steady Velocity (m/s)', fontsize=13)
        ax.set_title(title or 'Simulation vs. Experimental Results')
        ax.set_ylim(0, 4)
        ax.legend()
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

        if save_path:
            p   = Path(save_path)
            fmt = 'svg' if p.suffix == '.svg' else 'pdf'
            fig.savefig(p, format=fmt)
            print(f'Plot saved → {p}')

        if show:
            plt.show()

        return fig, ax

    # ------------------------------------------------------------------
    # Viewer
    # ------------------------------------------------------------------

    def launch_viewer(self):
        """Launch the dm_control interactive 3-D viewer."""
        viewer.launch(self.env)


# ---------------------------------------------------------------------------
# Permutation sweeps
# ---------------------------------------------------------------------------

# All four rubber attachment configurations, in display order
RUBBER_PERMUTATIONS = [
    (False, False, 'No rubber'),
    (False, True,  'Fast rubber only'),
    (True,  False, 'Slow rubber only'),
    (True,  True,  'Both rubber'),
]

# Belt offset conditions
OFFSET_PERMUTATIONS = [
    (0.0,    'No offset'),
    (0.0049, '4.9 mm offset'),
]


def run_rubber_permutations(config: dict) -> list:
    """
    Run a full belt-diff sweep for each rubber attachment configuration.

    Iterates over all four (slow_rubber, fast_rubber) combinations,
    rebuilding the model for each. Config values for slow_side_rubber and
    fast_side_rubber are overridden per permutation; all other parameters
    are taken from config unchanged.

    Args:
        config: base simulation config dict (from load_config())

    Returns:
        List of dicts, one per permutation, each with keys:
            label        — human-readable config name
            slow_rubber  — bool
            fast_rubber  — bool
            bdiffs       — numpy array of belt speed differences
            velocities   — list of avg steady velocities
    """
    results = []
    for slow_r, fast_r, label in RUBBER_PERMUTATIONS:
        cfg = {**config, 'slow_side_rubber': slow_r, 'fast_side_rubber': fast_r}
        print(f'\n── {label} ──')
        sim    = SplitBeltSim(cfg)
        bdiffs, velocities = sim.run_sweep()
        results.append({
            'label':       label,
            'slow_rubber': slow_r,
            'fast_rubber': fast_r,
            'bdiffs':      bdiffs,
            'velocities':  velocities,
        })
    return results


def plot_permutations(results: list,
                      exp_bdiffs, exp_velocities,
                      save_path=None,
                      show: bool = True):
    """
    2×2 grid of scatter plots — one panel per rubber configuration — all on a
    shared Y axis so results are directly comparable.

    Args:
        results:          output of run_rubber_permutations()
        exp_bdiffs:       experimental belt speed differences (Butterfield)
        exp_velocities:   experimental avg velocities
        save_path:        path to save figure (SVG or PDF); None = don't save
        show:             if True, call plt.show() after saving

    Returns:
        (fig, axes)
    """
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), sharey=True,
                             constrained_layout=True)
    fig.suptitle('Rubber Configuration Comparison vs. Butterfield et al.',
                 fontsize=14)

    for ax, result in zip(axes.flat, results):
        ax.scatter(exp_bdiffs, exp_velocities, s=10, color='red', zorder=3,
                   label='Butterfield et al.')
        ax.scatter(result['bdiffs'], result['velocities'], s=10, color='blue',
                   zorder=3, label='MuJoCo')
        ax.set_title(result['label'])
        ax.set_xlabel('Belt Speed Difference (m/s)')
        ax.set_ylabel('Average Steady Velocity (m/s)')
        ax.set_ylim(0, 4)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

    # Legend on first panel only (all panels share the same markers)
    axes[0, 0].legend(fontsize=9)

    if save_path:
        p   = Path(save_path)
        fmt = 'svg' if p.suffix == '.svg' else 'pdf'
        fig.savefig(p, format=fmt)
        print(f'Plot saved → {p}')

    if show:
        plt.show()

    return fig, axes


def run_offset_permutations(config: dict) -> list:
    """
    Run a full belt-diff sweep for all 8 combinations of rubber config × belt offset.

    Outer loop: offset (no offset / 4.9 mm offset)
    Inner loop: rubber config (no rubber / fast only / slow only / both)

    Returns:
        List of 8 dicts, each with keys:
            label        — e.g. 'Fast rubber only — 4.9 mm offset'
            slow_rubber  — bool
            fast_rubber  — bool
            offset       — float (m)
            offset_label — str
            bdiffs       — numpy array
            velocities   — list of avg steady velocities
    """
    results = []
    for offset, offset_label in OFFSET_PERMUTATIONS:
        for slow_r, fast_r, rubber_label in RUBBER_PERMUTATIONS:
            cfg = {
                **config,
                'slow_side_rubber':  slow_r,
                'fast_side_rubber':  fast_r,
                'fast_belt_zoffset': offset,
            }
            label = f'{rubber_label} — {offset_label}'
            print(f'\n── {label} ──')
            sim    = SplitBeltSim(cfg)
            bdiffs, velocities = sim.run_sweep()
            results.append({
                'label':        label,
                'rubber_label': rubber_label,
                'offset_label': offset_label,
                'slow_rubber':  slow_r,
                'fast_rubber':  fast_r,
                'offset':       offset,
                'bdiffs':       bdiffs,
                'velocities':   velocities,
            })
    return results


def plot_offset_permutations(results: list,
                             exp_bdiffs, exp_velocities,
                             save_path=None,
                             show: bool = True):
    """
    4×2 grid of scatter plots — rows are rubber configs, columns are belt offset
    conditions — all panels on a shared Y axis.

    Layout:
        col 0: No offset      col 1: 4.9 mm offset
        row 0: No rubber
        row 1: Fast rubber only
        row 2: Slow rubber only
        row 3: Both rubber

    Args:
        results:        output of run_offset_permutations()
        exp_bdiffs:     experimental belt speed differences (Butterfield)
        exp_velocities: experimental avg velocities
        save_path:      path to save figure (SVG or PDF); None = don't save
        show:           if True, call plt.show() after saving

    Returns:
        (fig, axes)
    """
    n_rubber = len(RUBBER_PERMUTATIONS)   # 4 rows
    n_offset = len(OFFSET_PERMUTATIONS)   # 2 cols

    fig, axes = plt.subplots(n_rubber, n_offset,
                             figsize=(12, 16),
                             sharey=True,
                             constrained_layout=True)
    fig.suptitle('Rubber × Offset Configuration Comparison vs. Butterfield et al.',
                 fontsize=14)

    # Column headers
    for col, (_, offset_label) in enumerate(OFFSET_PERMUTATIONS):
        axes[0, col].set_title(offset_label, fontsize=12, fontweight='bold', pad=10)

    for result in results:
        row = [r[2] for r in RUBBER_PERMUTATIONS].index(result['rubber_label'])
        col = [o[1] for o in OFFSET_PERMUTATIONS].index(result['offset_label'])
        ax  = axes[row, col]

        ax.scatter(exp_bdiffs, exp_velocities, s=10, color='red', zorder=3,
                   label='Butterfield et al.')
        ax.scatter(result['bdiffs'], result['velocities'], s=10, color='blue',
                   zorder=3, label='MuJoCo')

        # Row label on left column only
        if col == 0:
            ax.set_ylabel(f"{result['rubber_label']}\n\nAvg Steady Velocity (m/s)",
                          fontsize=9)
        ax.set_xlabel('Belt Speed Difference (m/s)')
        ax.set_ylim(0, 1)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

    axes[0, 0].legend(fontsize=9)

    if save_path:
        p   = Path(save_path)
        fmt = 'svg' if p.suffix == '.svg' else 'pdf'
        fig.savefig(p, format=fmt)
        print(f'Plot saved → {p}')

    if show:
        plt.show()

    return fig, axes


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    config        = load_config()
    RUN_ALL_RUBBER = config['run_all_rubber']
    SHOW_PLOT      = config['show_plot']
    FIGURES_PATH.mkdir(exist_ok=True)
    timestamp = datetime.now().strftime('%Y-%m-%d__%H-%M-%S')

    # Load experimental ground truth once
    sim_base = SplitBeltSim(config)
    exp_bdiffs, exp_velocities = sim_base.load_experimental_data()

    RUN_OFFSET_COMPARISON = config['run_offset_comparison']

    if RUN_OFFSET_COMPARISON:
        print('Running sweep for all rubber × offset configurations (8 total)...')
        results = run_offset_permutations(config)

        print('\nStatistical comparison per configuration:')
        for r in results:
            s = sim_base.get_stats(exp_velocities, r['velocities'])
            rs, tt = s['ranksums'], s['ttest_ind']
            print(f"  {r['label']:<38} "
                  f"ranksums p={rs['p_value']:.4f}  "
                  f"ttest p={tt['p_value']:.4f}")

        save_path = FIGURES_PATH / f'all-figures_{timestamp}.svg'
        plot_offset_permutations(results, exp_bdiffs, exp_velocities,
                                 save_path=save_path, show=SHOW_PLOT)

    elif RUN_ALL_RUBBER:
        print('Running sweep for all rubber configurations...')
        results = run_rubber_permutations(config)

        print('\nStatistical comparison per configuration:')
        for r in results:
            s = sim_base.get_stats(exp_velocities, r['velocities'])
            rs, tt = s['ranksums'], s['ttest_ind']
            print(f"  {r['label']:<22} "
                  f"ranksums p={rs['p_value']:.4f}  "
                  f"ttest p={tt['p_value']:.4f}")

        save_path = FIGURES_PATH / f'all-figures_{timestamp}.svg'
        plot_permutations(results, exp_bdiffs, exp_velocities,
                          save_path=save_path, show=SHOW_PLOT)

    else:
        print('Running belt speed difference sweep...')
        bdiffs, velocities = sim_base.run_sweep()

        print('\nStatistical comparison (simulation vs. Butterfield experimental):')
        s  = sim_base.get_stats(exp_velocities, velocities)
        rs = s['ranksums']
        tt = s['ttest_ind']
        print(f"  Wilcoxon rank-sum : stat={rs['statistic']:+.4f},  p={rs['p_value']:.4f}")
        print(f"  Welch's t-test    : stat={tt['statistic']:+.4f},  p={tt['p_value']:.4f}")

        sim_base.save_sim_data(bdiffs, velocities)

        save_path = FIGURES_PATH / f'validation_{timestamp}.svg'
        sim_base.plot_comparison(
            bdiffs, velocities, exp_bdiffs, exp_velocities,
            save_path=save_path, show=SHOW_PLOT
        )


if __name__ == '__main__':
    main()
