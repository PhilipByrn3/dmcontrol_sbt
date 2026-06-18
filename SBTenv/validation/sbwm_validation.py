from SBTenv.models.treadmill import Treadmill
from SBTenv.models.wheel import Wheel
from SBTenv.envs.sbwm_env import SBWMtask
from dm_control import mujoco, mjcf, viewer
from dm_control.rl.control import Environment
from dm_control.suite.base import Task
import numpy as np
import matplotlib.pyplot as plt

class SBWMsim:
    def __init__(self): 
        self._model = mjcf.RootElement(model='sbt_rimless_wheel')
        
        self._model.compiler.autolimits = True
        self._model.compiler.angle = 'degree'
        self._model.option.gravity = [0, 0, -9.81]
        self._model.option.timestep = 0.01
        self._model.option.integrator = 'RK4'
        self._model.option.solver = 'PGS'
        self._model.option.cone = 'Elliptic'
        self._model.option.iterations = 50

        Wheel(self._model.worldbody)
        Treadmill(self._model.worldbody, 
                  belt_size=[0.05, 10, 0.1])
        self._model.sensor.add('framepos', 
                               name='axlepos',
                               objtype='xbody', 
                               objname=Wheel.BODY_NAME
                               )

        self.time_limit = 20
        self._sbt_task = SBWMtask()
        self.sbt_physics = mujoco.Physics.from_xml_string(self._model.to_xml_string())
        self.sbt_env = Environment(physics=self.sbt_physics, task=self._sbt_task, time_limit=self.time_limit)
        
    def _run_episode(self, slow_belt_speed: float, fast_belt_speed: float) -> float:
        self._sbt_task.slow_belt_speed = slow_belt_speed
        self._sbt_task.fast_belt_speed = fast_belt_speed
        timestep = self.sbt_env.reset()
        observations = []
        while not timestep.last():
            timestep = self.sbt_env.step(None)
            observations.append(timestep.observation)

        axle_y_pos = np.array([obs['axle_y_pos'] for obs in observations])
        net_displacement = axle_y_pos[-1] - axle_y_pos[0]
        avg_velocity = net_displacement / self.time_limit
        
        return avg_velocity
    
    def run_sweep(self, bsd_range: list, bsd_interval: float):
        speeds = np.arange(bsd_range[0], bsd_range[1] + bsd_interval, bsd_interval)
        avg_velocity_list = []

        print('Beginning sweep...')
        for bsd in speeds:
            avg_velocity = self._run_episode(slow_belt_speed=0, fast_belt_speed=bsd)
            avg_velocity_list.append(avg_velocity)
            print(f'  BSD: {bsd:.3f} -> avg velocity: {float(avg_velocity):.4f}')
        print('Sweep complete.')

        plt.figure()
        plt.plot(speeds[:len(avg_velocity_list)], avg_velocity_list, marker='o')
        plt.xlabel('Belt Speed Difference (BSD)')
        plt.ylabel('Average Velocity (m/s)')
        plt.title('Axle Average Velocity vs Belt Speed Difference')
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        return avg_velocity_list


s = SBWMsim()
results = s.run_sweep(bsd_range=[0.15, 1.15], bsd_interval=0.05)



        