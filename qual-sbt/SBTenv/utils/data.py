from pathlib import Path

import numpy as np

EXPERIMENTAL_CSV = Path(__file__).resolve().parents[2] / 'data' / 'Experimental.csv'


def load_experimental(path=EXPERIMENTAL_CSV):
    """
    Load the digitized Butterfield et al. (2022) split-belt data.

    Returns (bsd, velocity): belt speed differences (m/s) and average
    wheel velocities (m/s), in file order.
    """
    bsd, velocity = np.loadtxt(path, delimiter=',', skiprows=1, usecols=(1, 2), unpack=True)
    return bsd, velocity
