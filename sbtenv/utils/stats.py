"""
Agreement statistics between simulated and experimental velocities.

Every function takes paired arrays (simulation[i] and experimental[i] at
the same belt speed difference) in the order (simulation, experimental).
"""
import numpy as np


def rmse(simulation, experimental):
    """Root-mean-square error (m/s)."""
    return float(np.sqrt(np.mean((simulation - experimental) ** 2)))


def bias(simulation, experimental):
    """Mean signed error, simulation - experimental (m/s)."""
    return float(np.mean(simulation - experimental))


def mae(simulation, experimental):
    """Mean absolute error (m/s)."""
    return float(np.mean(np.abs(simulation - experimental)))


def r2(simulation, experimental):
    """Coefficient of determination of the simulation as a predictor of the experiment."""
    ss_res = np.sum((experimental - simulation) ** 2)
    ss_tot = np.sum((experimental - experimental.mean()) ** 2)
    return float(1 - ss_res/ss_tot)


def pearson_r(simulation, experimental):
    """Linear correlation. Measures shape only; ignores offset and scale."""
    return float(np.corrcoef(experimental, simulation)[0,1])
