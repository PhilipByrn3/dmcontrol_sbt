"""
analysis.py
Statistical comparison between simulation and experimental results.

Functions:
    get_stats(exp_velocities, sim_velocities, exp_bdiffs, sim_bdiffs) → dict
    print_stats(label, stats_dict)
"""

import numpy as np
from scipy import stats


def get_stats(exp_velocities, sim_velocities,
              exp_bdiffs=None, sim_bdiffs=None) -> dict:
    """
    Compare simulation and experimental velocity distributions.

    Distribution tests (no x-alignment needed):
        - Wilcoxon rank-sum (non-parametric)
        - Welch's t-test (parametric)

    Curve-fit metrics (require exp_bdiffs and sim_bdiffs):
        - R²   — coefficient of determination (1 = perfect fit)
        - RMSE — root mean squared error (m/s)
        - MAE  — mean absolute error (m/s)

        The simulation curve is linearly interpolated at each experimental
        x-value; only points within the sim's x-range are used (no
        extrapolation).

    Args:
        exp_velocities: experimental avg velocities (array-like)
        sim_velocities: simulation avg velocities (array-like)
        exp_bdiffs:     experimental belt speed differences (optional)
        sim_bdiffs:     simulation belt speed differences (optional)

    Returns:
        dict with keys:
            'ranksums'  → {'statistic', 'p_value'}
            'ttest_ind' → {'statistic', 'p_value'}
            'fit'       → {'r2', 'rmse', 'mae', 'n_points'}
                          (only present when both bdiff arrays are provided)
    """
    exp = np.asarray(exp_velocities)
    sim = np.asarray(sim_velocities)
    rs_stat, rs_pval = stats.ranksums(exp, sim)
    tt_stat, tt_pval = stats.ttest_ind(exp, sim)

    result = {
        'ranksums':  {'statistic': float(rs_stat), 'p_value': float(rs_pval)},
        'ttest_ind': {'statistic': float(tt_stat), 'p_value': float(tt_pval)},
    }

    if exp_bdiffs is not None and sim_bdiffs is not None:
        exp_x = np.asarray(exp_bdiffs)
        sim_x = np.asarray(sim_bdiffs)
        sim_y = np.asarray(sim_velocities)

        mask         = (exp_x >= sim_x.min()) & (exp_x <= sim_x.max())
        exp_x_in     = exp_x[mask]
        exp_y_in     = np.asarray(exp_velocities)[mask]
        sim_y_interp = np.interp(exp_x_in, sim_x, sim_y)

        residuals = exp_y_in - sim_y_interp
        ss_res    = np.sum(residuals ** 2)
        ss_tot    = np.sum((exp_y_in - exp_y_in.mean()) ** 2)
        r2        = 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')

        result['fit'] = {
            'r2':       float(r2),
            'rmse':     float(np.sqrt(np.mean(residuals ** 2))),
            'mae':      float(np.mean(np.abs(residuals))),
            'n_points': int(mask.sum()),
        }

    return result


def print_stats(label: str, s: dict, width: int = 38):
    """Print a formatted one-line stats summary for a single configuration."""
    fit = s.get('fit', {})
    rs  = s['ranksums']
    tt  = s['ttest_ind']
    print(f"  {label:<{width}} "
          f"R²={fit.get('r2', float('nan')):.3f}  "
          f"RMSE={fit.get('rmse', float('nan')):.4f}  "
          f"MAE={fit.get('mae', float('nan')):.4f}  "
          f"ranksums p={rs['p_value']:.4f}  "
          f"ttest p={tt['p_value']:.4f}")
