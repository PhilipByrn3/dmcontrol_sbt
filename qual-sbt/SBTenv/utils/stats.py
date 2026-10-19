import numpy as np
from scipy import stats

class Stats:
    def __init__(self):
        pass
    
    def rmse(self, simulation, experimental):
        return float(np.sqrt(np.mean((simulation - experimental) ** 2)))
    
    def bias(self, simulation, experimental):
        return float(np.mean(simulation-experimental))
    
    def r2(self, simulation, experimental):
        ss_res = np.sum((experimental - simulation) ** 2)
        ss_tot = np.sum((experimental - experimental.mean()) ** 2)
        return float(1 - ss_res/ss_tot)
    
    def pearson_r(self, simulation, experimental):
        return float(np.corrcoef(experimental, simulation)[0,1])
    
    