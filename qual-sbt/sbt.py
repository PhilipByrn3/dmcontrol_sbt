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

def get_experimental_data():
    # -- Convert CSV of experimental results into dataframe
    experimental_sbt_dataset = pd.read_csv('data/digitized_experimental_sbt_data.csv')
    exp_bdiffarray = experimental_sbt_dataset['exp_bdiff']
    exp_average_velocity = experimental_sbt_dataset['exp_avgvelo']
    return exp_bdiffarray, exp_average_velocity

def robust_plotting(average_velocity_list, bdiffarray):
    
    # -- Create Dataframe for CSV export of simulation data
    sbt_dataset = pd.DataFrame({'BDiffs': bdiffarray[:], 'AvgVelo':average_velocity_list[:]})
    sbt_dataset.to_csv('data/sbt_data.csv', index=False)

    exp_bdiffarray, exp_average_velocity = get_experimental_data()

    # -- Plot Settings
    plt.scatter(bdiffarray, average_velocity_list, 
                s=10, 
                color='blue',
                label='MuJoCo Simulation'
                )
    plt.scatter(exp_bdiffarray, exp_average_velocity, 
                s=10, 
                color='red',
                label='Experimental Results'
                )
    plt.set_title('Belt Speed Difference (m/s) vs. Average Steady Velocity (m/s)')
    plt.ylabel('Average Steady Velocity (m/s)', 
               size=15
               )
    plt.xlabel('Belt Speed Difference (m/s)', 
               size=15
               )
    plt.legend()
    # -- Show Results
    # plt.show()

    # -- Save Results
    timestamp = datetime.now().strftime('%Y-%m-%d__%H-%M-%S')
    filename = f'figures/figure_{timestamp}.svg'
    plt.savefig(filename,
                format='svg'
                )
    print(f'Plot saved to {filename}')

def instantiate_environment(sbt_fast_spoke_rubber, sbt_slow_spoke_rubber, fast_belt_pos):
    # -- MuJoCo Setup
    model = sbt_model_gen.create_sbt_model(sbt_fast_spoke_rubber, 
                                           sbt_slow_spoke_rubber,
                                           fast_belt_pos
                                           )
    sbt_physics = mujoco.Physics.from_xml_string(model.to_xml_string())
    sbt_task = SplitBeltTreadmillTask()
    sbt_env = Environment(physics=sbt_physics, 
                          task=sbt_task
                          )
    action_spec = sbt_env.action_spec()

    return model, sbt_physics, sbt_task, sbt_env, action_spec

if __name__ == '__main__':

    mass_simulation = False

    if mass_simulation == True:
        timesteps = 2000
        starting_belt_diff = 0.15
        belt_diff = 1.1 
        bd_increment = 0.005

        # -- Fig 1
        sbt_fast_spoke_rubber = True
        sbt_slow_spoke_rubber = False
        fast_belt_pos = [0.051, 0, 0]
        model, sbt_physics, sbt_task, sbt_env, action_spec = instantiate_environment(sbt_fast_spoke_rubber, 
                                                                                     sbt_slow_spoke_rubber, 
                                                                                     fast_belt_pos)
        average_velocity_list1, rotation_count_list1 = loop_sim(timesteps, belt_diff, starting_belt_diff, bd_increment)
        bdiffarray1 = np.linspace((starting_belt_diff+bd_increment), belt_diff, num=len(average_velocity_list1))

        # -- Fig 2
        sbt_fast_spoke_rubber = True
        sbt_slow_spoke_rubber = False
        fast_belt_pos = [0.051, 0, 0.0049]
        model, sbt_physics, sbt_task, sbt_env, action_spec = instantiate_environment(sbt_fast_spoke_rubber, 
                                                                                     sbt_slow_spoke_rubber, 
                                                                                     fast_belt_pos)
        average_velocity_list2, rotation_count_list2 = loop_sim(timesteps, belt_diff, starting_belt_diff, bd_increment)
        bdiffarray2 = np.linspace((starting_belt_diff+bd_increment), belt_diff, num=len(average_velocity_list2))

        # -- Fig 3
        sbt_fast_spoke_rubber = True
        sbt_slow_spoke_rubber = True
        fast_belt_pos = [0.051, 0, 0]
        model, sbt_physics, sbt_task, sbt_env, action_spec = instantiate_environment(sbt_fast_spoke_rubber, 
                                                                                     sbt_slow_spoke_rubber, 
                                                                                     fast_belt_pos)
        average_velocity_list3, rotation_count_list3 = loop_sim(timesteps, belt_diff, starting_belt_diff, bd_increment)
        bdiffarray3 = np.linspace((starting_belt_diff+bd_increment), belt_diff, num=len(average_velocity_list3))

        # -- Fig 4
        sbt_fast_spoke_rubber = True
        sbt_slow_spoke_rubber = True
        fast_belt_pos = [0.051, 0, 0.0049]
        model, sbt_physics, sbt_task, sbt_env, action_spec = instantiate_environment(sbt_fast_spoke_rubber, 
                                                                                     sbt_slow_spoke_rubber, 
                                                                                     fast_belt_pos)
        average_velocity_list4, rotation_count_list4 = loop_sim(timesteps, belt_diff, starting_belt_diff, bd_increment)
        bdiffarray4 = np.linspace((starting_belt_diff+bd_increment), belt_diff, num=len(average_velocity_list4))

        # -- Fig 5
        sbt_fast_spoke_rubber = False
        sbt_slow_spoke_rubber = False
        fast_belt_pos = [0.051, 0, 0.0049]
        model, sbt_physics, sbt_task, sbt_env, action_spec = instantiate_environment(sbt_fast_spoke_rubber, 
                                                                                     sbt_slow_spoke_rubber, 
                                                                                     fast_belt_pos)
        average_velocity_list5, rotation_count_list5 = loop_sim(timesteps, belt_diff, starting_belt_diff, bd_increment)
        bdiffarray5 = np.linspace((starting_belt_diff+bd_increment), belt_diff, num=len(average_velocity_list5))

        # -- Fig 6
        sbt_fast_spoke_rubber = False
        sbt_slow_spoke_rubber = False
        fast_belt_pos = [0.051, 0, 0]
        model, sbt_physics, sbt_task, sbt_env, action_spec = instantiate_environment(sbt_fast_spoke_rubber, 
                                                                                     sbt_slow_spoke_rubber, 
                                                                                     fast_belt_pos)
        average_velocity_list6, rotation_count_list6 = loop_sim(timesteps, belt_diff, starting_belt_diff, bd_increment)
        bdiffarray6 = np.linspace((starting_belt_diff+bd_increment), belt_diff, num=len(average_velocity_list6))

        # -- Plot Results
        exp_bdiffarray, exp_average_velocity = get_experimental_data()
        fig, ax = plt.subplots(nrows=2, ncols=3, figsize=(9,6), constrained_layout=True)

        # Existing scatter plots and titles
        ax[0,0].scatter(bdiffarray1, average_velocity_list1, color='blue', s=10, label='Simulation Data')
        ax[0,0].scatter(exp_bdiffarray, exp_average_velocity, color='red', s=10, label='Experimenal Data')
        ax[0,0].set_title('(a)', x=0.5, y=0.98)
        ax[0,0].set_ylabel('Average Steady Velocity (m/s)')
        ax[0,0].legend(loc='lower center')

        ax[0,1].scatter(bdiffarray2, average_velocity_list2, color='blue', s=10, label='Simulation Data')
        ax[0,1].scatter(exp_bdiffarray, exp_average_velocity, color='red', s=10, label='Experimenal Data')
        ax[0,1].set_title('(b)', x=0.5, y=0.98)
        ax[0,1].set_xlabel('Belt Speed Difference (m/s)')

        ax[0,2].scatter(bdiffarray3, average_velocity_list3, color='blue', s=10, label='Simulation Data')
        ax[0,2].scatter(exp_bdiffarray, exp_average_velocity, color='red', s=10, label='Experimenal Data')
        ax[0,2].set_title('(c)', x=0.5, y=0.98)

        ax[1,0].scatter(bdiffarray4, average_velocity_list4, color='blue', s=10, label='Simulation Data')
        ax[1,0].scatter(exp_bdiffarray, exp_average_velocity, color='red', s=10, label='Experimenal Data')
        ax[1,0].set_title('(d)', x=0.5, y=0.98)
        ax[1,0].set_ylabel('Average Steady Velocity (m/s)')

        ax[1,1].scatter(bdiffarray5, average_velocity_list5, color='blue', s=10, label='Simulation Data')
        ax[1,1].scatter(exp_bdiffarray, exp_average_velocity, color='red', s=10, label='Experimenal Data')
        ax[1,1].set_title('(e)', x=0.5, y=0.98)
        ax[1,1].set_xlabel('Belt Speed Difference (m/s)')

        ax[1,2].scatter(bdiffarray6, average_velocity_list6, color='blue', s=10, label='Simulation Data')
        ax[1,2].scatter(exp_bdiffarray, exp_average_velocity, color='red', s=10, label='Experimenal Data')
        ax[1,2].set_title('(f)', x=0.5, y=0.98)

        for row in ax:
            for a in row:
                # only keep left & bottom spine
                a.spines['top'].set_visible(False)
                a.spines['right'].set_visible(False)

                # ensure ticks only on bottom & left
                a.xaxis.set_ticks_position('bottom')
                a.yaxis.set_ticks_position('left')

                # same y-range
                a.set_ylim(0, 0.4)



        # example of one panel needing a different y‐limit
        ax[0,1].set_ylim(-0.4, 0.22)

        plt.show()

    # -- Model Parameters for Results
    

    model, sbt_physics, sbt_task, sbt_env, action_spec = instantiate_environment(sbt_fast_spoke_rubber, sbt_slow_spoke_rubber, fast_belt_pos)

    # -- Simulation Parameters
    timesteps = 2000
    starting_belt_diff = 0.15
    belt_diff = 1.1 
    bd_increment = 0.005

    # -- Test single simulation
    # simulate_treadmill(timesteps, belt_diff)

    # -- Test Loop Simulation
    # average_velocity_list, rotation_count_list = loop_sim(timesteps, belt_diff, starting_belt_diff, bd_increment)
    # bdiffarray = np.linspace((starting_belt_diff+bd_increment), belt_diff, num=len(average_velocity_list))
    # robust_plotting(average_velocity_list, bdiffarray)

    # -- Test Rendering and Model
    viewer.launch(sbt_env)