import numpy as np
import time
import matplotlib.pyplot as plt
import pandas as pd
import math

from datetime import datetime
from dm_control import viewer, mujoco
from dm_control.rl.control import Environment
from dm_control.suite.base import Task
from dm_control import mjcf
from create_wheel import AssembleWheel
import yaml
import common

# with open('/qual-sbt/validation/config.yaml', 'r') as file:
#     config_settings = yaml.safe_load(file)
# slow_belt_speed: float = config_settings['slow_belt_speed']
# belt_speed_difference: list = config_settings['belt_speed_difference']    

class SplitBeltTask(Task):
    def __init__(self, random=None):
        super().__init__(random)
    
    def before_step(self, action, physics, 
                    slow_belt_speed: float,
                    belt_speed_difference: float):
        super().before_step(action, physics)
        physics.named.data.qvel['slow_belt'] = slow_belt_speed
        physics.names.data.qvel['fast_belt'] = slow_belt_speed + belt_speed_difference
    
    def get_observation(self, physics):
        return super().get_observation(physics)

def main():
    task = SplitBeltTask()      
    aw = AssembleWheel()
    physics = mjcf.Physics.from_mjcf_model(aw._mjcf_model) 
    env = Environment(physics=physics, task=task)
    viewer.launch(env)

if __name__ == '__main__':
    main()
    