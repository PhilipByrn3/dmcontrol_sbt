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
from dm_control.utils import xml_tools
from dm_control.utils import io as resources 
from create_wheel import AssembleWheel
from create_treadmill import AssembleTreadmill
import yaml
import common
from pathlib import Path
from lxml import etree



CONFIG_PATH = Path(__file__).with_name('config.yaml')  # same folder as create_wheel.py
with CONFIG_PATH.open('r') as f:
    config_settings = yaml.safe_load(f)
slow_belt_speed: float = config_settings['slow_belt_speed']
belt_speed_difference: list = config_settings['belt_speed_difference']    

class SplitBeltTask(Task):
    def __init__(self, random=None):
        super().__init__(random)
    
    def before_step(self, action, physics):
        super().before_step(action, physics)
        
    def get_observation(self, physics):
        return super().get_observation(physics)
    
    def get_reward(self, physics):
        return super().get_reward(physics)
    
    def initialize_episode(self, physics):
        return super().initialize_episode(physics)

class CreateModel():
    def __init__(self):
        rimless_wheel = AssembleWheel()
        sbt = AssembleTreadmill()
        self._mjcf_model = mjcf.RootElement(model='rimlesssbt')
        
        # self._mjcf_model.include('./common/skybox.xml')
        # self._mjcf_model.include('./common/visual.xml')
        # self._mjcf_model.include('./common/materials.xml')
        
        self._mjcf_model.compiler.autolimits = True
        self._mjcf_model.compiler.angle = 'degree'
        
        self._mjcf_model.option.gravity = [0, 0, -9.81]
        self._mjcf_model.option.timestep = 0.01
        self._mjcf_model.option.integrator = 'RK4'
        self._mjcf_model.option.cone = 'Elliptic'
        self._mjcf_model.option.solver = 'PGS'
        self._mjcf_model.option.iterations = 50
        
        self._mjcf_model.attach(rimless_wheel._mjcf_model)
        self._mjcf_model.attach(sbt._mjcf_model)
        
    

def main():
    task = SplitBeltTask()     
    cm = CreateModel()

    physics = mjcf.Physics.from_mjcf_model(cm._mjcf_model) 
    env = Environment(physics=physics, task=task)
    viewer.launch(env)

if __name__ == '__main__':
    main()
    