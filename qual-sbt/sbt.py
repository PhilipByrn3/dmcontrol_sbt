import numpy as np
import time
import matplotlib.pyplot as plt
import pandas as pd
import math

from datetime import datetime
from dm_control import viewer, mujoco
from dm_control.rl.control import Environment
from dm_control.suite.base import Task


import common
import sbt_model_gen

class SplitBeltTreadmillTask(Task):

    def __init__(self):
        super().__init__()
    
    def get_observation(self, physics):
        #print(sbt_physics.named.data.sensordata['slow_touch'])
        return None
    
    def get_reward(self, physics):
        return None
    
    def initialize_episode(self, physics):
        return None
    
    def before_step(self, action, physics):
        return None

def simulate_treadmill(timesteps, belt_diff):
    sbt_env.reset()
    i=0
    # -- Set slow belt speed to 0 and fast belt speed to belt_diff as sbt_env resets
    slow_belt_speed = 0
    fast_belt_speed = slow_belt_speed + belt_diff
    time_list, wheel_position_list = [], []
    sbt_physics.named.data.qvel['axleYAxis'] = fast_belt_speed

    while i < timesteps:
        # -- At the beginning of each timestep, set belt speeds
        sbt_physics.step()
        sbt_physics.named.data.qvel['slow_belt_conveyor'] = slow_belt_speed
        sbt_physics.named.data.qvel['fast_belt_conveyor'] = slow_belt_speed + belt_diff
        rotation_count = sbt_physics.named.data.qpos['axleHinge']/(2*math.pi) * -1
        timeval = round(sbt_physics.data.time,8)
        time_list.append(timeval)
        if timesteps >= 50:
            wheel_position = sbt_physics.named.data.sensordata['axlepos'][1].astype(np.float32).item()
            wheel_position_list.append(wheel_position)
        if rotation_count[0] >= 3.0:
            break
        i+=1
    # -- Calculate average velocity of wheel by dividing final measured distance over final measured time
    avg_wheel_velocity = wheel_position_list[-1]/time_list[-1]
    print('Bdiff: ', round(belt_diff, 5), '\n        -> Rotation count:', round(rotation_count[0], 4))
    return avg_wheel_velocity, rotation_count 

def loop_sim(timesteps, belt_diff, starting_belt_diff, bd_increment):
    average_velocity_list, rotation_count_list = [], []
    while starting_belt_diff <= belt_diff:
        wheel_velocity, rotation_count = simulate_treadmill(timesteps, starting_belt_diff)
        average_velocity_list.append(wheel_velocity)
        rotation_count_list.append(rotation_count)
        starting_belt_diff+=bd_increment
    return average_velocity_list, rotation_count_list

def robust_plotting(average_velocity_list, bdiffarray):
    
    # -- Create Dataframe for CSV export of simulation data
    sbt_dataset = pd.DataFrame({'BDiffs': bdiffarray[:], 'AvgVelo':average_velocity_list[:]})
    sbt_dataset.to_csv('data/sbt_data.csv', index=False)

    # -- Convert CSV of experimental results into dataframe
    experimental_sbt_dataset = pd.read_csv('data/digitized_experimental_sbt_data.csv')
    exp_bdiffarray = experimental_sbt_dataset['exp_bdiff']
    exp_average_velocity = experimental_sbt_dataset['exp_avgvelo']

    # -- Plot Settings
    plt.scatter(bdiffarray, average_velocity_list, 
                s=10, 
                color='blue'
                )
    plt.scatter(exp_bdiffarray, exp_average_velocity, 
                s=10, 
                color='red'
                )
    plt.title('Belt Speed Difference (m/s) vs. Average Steady Velocity (m/s)')
    plt.ylabel('Average Steady Velocity (m/s)', 
               size=15
               )
    plt.xlabel('Belt Speed Difference (m/s)', 
               size=15
               )

    # -- Show Results
    # plt.show()

    # -- Save Results
    timestamp = datetime.now().strftime('%Y-%m-%d__%H-%M-%S')
    filename = f'figures/figure_{timestamp}.svg'
    plt.savefig(filename,
                format='svg'
                )
    print(f'Plot saved as {filename}')

if __name__ == '__main__':
    # -- Model Parameters for Results
    sbt_fast_spoke_rubber = True
    sbt_slow_spoke_rubber = False

    # -- MuJoCo Setup
    model = sbt_model_gen.create_sbt_model(sbt_fast_spoke_rubber, 
                                           sbt_slow_spoke_rubber
                                           )
    sbt_physics = mujoco.Physics.from_xml_string(model.to_xml_string())
    sbt_task = SplitBeltTreadmillTask()
    sbt_env = Environment(physics=sbt_physics, 
                          task=sbt_task
                          )
    action_spec = sbt_env.action_spec()

    # -- Simulation Parameters
    timesteps = 2000
    starting_belt_diff = 0.15
    belt_diff = 1.1 
    bd_increment = 0.005

    # -- Test single simulation
    # simulate_treadmill(timesteps, belt_diff)

    # -- Test Loop Simulation
    average_velocity_list, rotation_count_list = loop_sim(timesteps, belt_diff, starting_belt_diff, bd_increment)
    bdiffarray = np.linspace((starting_belt_diff+bd_increment), belt_diff, num=len(average_velocity_list))
    robust_plotting(average_velocity_list, bdiffarray)

    # -- Test Rendering and Model
    # viewer.launch(sbt_env)