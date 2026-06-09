"""
plotting.py
All plotting functions for the split-belt rimless wheel simulation.

Functions:
    plot_comparison(...)          — single sweep vs experimental
    plot_tied_belt(...)           — tied-belt sweep (no experimental overlay)
    plot_permutations(...)        — 2×2 rubber configuration grid
    plot_offset_permutations(...) — 4×2 rubber × offset grid
"""

from pathlib import Path

import matplotlib.pyplot as plt

# Use Times New Roman for all plot text
plt.rcParams["font.family"] = "serif"
plt.rcParams["font.serif"]  = ["Times New Roman", "Times", "DejaVu Serif"]

# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _label_to_filename(label: str) -> str:
    """Convert a result label to a safe SVG filename."""
    return (label.lower()
                 .replace(' ', '_')
                 .replace('—', '-')
                 .replace('/', '-') + '.svg')


def _style_ax(ax):
    """Remove top/right spines."""
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)


def _plot_single(result: dict, exp_bdiffs, exp_velocities,
                 xlabel: str = 'Belt Speed Difference (m/s)',
                 save_path=None, show: bool = False):
    """
    Single scatter panel for one permutation result overlaid on experimental data.
    Returns (fig, ax); caller is responsible for closing.
    """
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.scatter(exp_bdiffs, exp_velocities, s=10, color='red', zorder=3,
               label='Butterfield et al.')
    ax.scatter(result['bdiffs'], result['velocities'], s=10, color='blue',
               zorder=3, label='MuJoCo')
    ax.set_title(result['label'])
    ax.set_xlabel(xlabel)
    ax.set_ylabel('Average Steady Velocity (m/s)')
    ax.set_ylim(0, 0.4)
    ax.legend(fontsize=9)
    _style_ax(ax)
    if save_path:
        fig.savefig(save_path, format='svg')
        print(f'  saved → {save_path}')
    if show:
        plt.show()
    return fig, ax


# ---------------------------------------------------------------------------
# Public plot functions
# ---------------------------------------------------------------------------

def plot_comparison(sim_bdiffs, sim_velocities,
                    exp_bdiffs, exp_velocities,
                    title: str = '',
                    save_path=None,
                    show: bool = True):
    """
    Single scatter plot of one simulation sweep overlaid on Butterfield data.

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
    ax.set_ylim(0, 0.4)
    ax.legend(fontsize=9)
    _style_ax(ax)

    if save_path:
        fig.savefig(save_path, format='svg')
        print(f'Plot saved → {save_path}')

    if show:
        plt.show()

    return fig, ax


def plot_tied_belt(sim_speeds, sim_velocities,
                   title: str = '',
                   save_path=None,
                   show: bool = True):
    """
    Single scatter plot for a tied-belt sweep (no experimental overlay,
    since both-belts-equal is not part of the Butterfield dataset).

    Args:
        sim_speeds:     array of belt speeds used in the tied sweep
        sim_velocities: corresponding avg steady-state wheel velocities
        title:          optional plot title
        save_path:      SVG save path; None = don't save
        show:           if True, call plt.show()

    Returns:
        (fig, ax)
    """
    fig, ax = plt.subplots(figsize=(7, 5))

    ax.scatter(sim_speeds, sim_velocities, s=12, color='blue', zorder=3,
               label='MuJoCo (tied belts)')

    ax.set_xlabel('Belt Speed (m/s)', fontsize=13)
    ax.set_ylabel('Average Steady Velocity (m/s)', fontsize=13)
    ax.set_title(title or 'Tied-Belt Sweep')
    ax.set_ylim(0, 0.4)
    ax.legend(fontsize=9)
    _style_ax(ax)

    if save_path:
        fig.savefig(save_path, format='svg')
        print(f'Plot saved → {save_path}')

    if show:
        plt.show()

    return fig, ax


def plot_permutations(results: list,
                      exp_bdiffs, exp_velocities,
                      save_dir=None,
                      show: bool = True):
    """
    Saves one SVG per rubber configuration plus a combined 2×2 grid SVG,
    all inside save_dir (if provided).

    Args:
        results:          output of run_rubber_permutations()
        exp_bdiffs:       experimental belt speed differences
        exp_velocities:   experimental avg velocities
        save_dir:         Path to output folder; None = don't save
        show:             if True, call plt.show() on the combined figure

    Returns:
        (fig, axes)
    """
    # Individual panels
    if save_dir:
        for result in results:
            _plot_single(result, exp_bdiffs, exp_velocities,
                         save_path=save_dir / _label_to_filename(result['label']))
            plt.close()

    # Combined 2×2 grid
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
        ax.set_ylim(0, 0.4)
        _style_ax(ax)

    axes[0, 0].legend(fontsize=9)

    if save_dir:
        p = save_dir / 'all_rubber_configs.svg'
        fig.savefig(p, format='svg')
        print(f'  saved → {p}')

    if show:
        plt.show()

    return fig, axes


def plot_offset_permutations(results: list,
                             exp_bdiffs, exp_velocities,
                             save_dir=None,
                             show: bool = True):
    """
    Saves one SVG per rubber × offset combination plus a combined 4×2 grid SVG,
    all inside save_dir (if provided).

    Layout:
        col 0: No offset      col 1: 4.9 mm offset
        row 0: No rubber
        row 1: Fast rubber only
        row 2: Slow rubber only
        row 3: Both rubber

    Returns:
        (fig, axes)
    """
    from sbt_core import RUBBER_PERMUTATIONS, OFFSET_PERMUTATIONS

    # Individual panels
    if save_dir:
        for result in results:
            _plot_single(result, exp_bdiffs, exp_velocities,
                         save_path=save_dir / _label_to_filename(result['label']))
            plt.close()

    # Combined 4×2 grid
    n_rubber = len(RUBBER_PERMUTATIONS)
    n_offset = len(OFFSET_PERMUTATIONS)

    fig, axes = plt.subplots(n_rubber, n_offset,
                             figsize=(12, 16),
                             sharey=True,
                             constrained_layout=True)
    fig.suptitle('Rubber × Offset Configuration Comparison vs. Butterfield et al.',
                 fontsize=14)

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

        if col == 0:
            ax.set_ylabel(f"{result['rubber_label']}\n\nAvg Steady Velocity (m/s)",
                          fontsize=9)
        ax.set_xlabel('Belt Speed Difference (m/s)')
        ax.set_ylim(0, 0.4)
        _style_ax(ax)

    axes[0, 0].legend(fontsize=9)

    if save_dir:
        p = save_dir / 'all_rubber_offset_configs.svg'
        fig.savefig(p, format='svg')
        print(f'  saved → {p}')

    if show:
        plt.show()

    return fig, axes
