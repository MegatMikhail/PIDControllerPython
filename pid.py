# SIGNIFICANTLY based off of Bryan Smith's PID Controller at
# https://medium.com/@bsmith4360/building-a-simulated-pid-controller-in-python-111b08ccae1a

import numpy as np
import matplotlib.pyplot as plt

G = 9.81

class Vertical_Rocket:
    def __init__(self, mass): #Add drag coefficient later?
        self.mass = mass 
    
    def get_acceleration(self, force, v0):
        return (force - self.mass * G) / self.mass
    
    def get_velocity(self, a, v0, time_step):
        return v0 + a * time_step
    
    def get_position(self, a, v0, x0, time_step):
        return x0 + v0 * time_step + 0.5 * a * time_step**2
    
class PID: 
    def __init__(self, kp, ki, kd, dt, desired_value):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.desired_value = desired_value # eq to target in BR's PID
        self.dt = dt # eq to sample rate in BR's PID
        self.error = 0.0
        self.prev_sensor_data = 0.0 # eq to last_reading in BR's PID
        self.derivative_error = 0.0
        self.integral_error = 0.0 # eq to accumulator in BR's PID
        self.max_val = 500 # Deviation from Bayes.
        
    def update(self, sensor_data): # eq to adjust_signal in BR's PID
        self.error = self.desired_value - sensor_data
        self.integral_error += self.error * self.dt 
        self.derivative_error = (sensor_data - prev_sensor_data) / dt
        self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error
        # deviates from Bayes slightly, so that we can plot output
        if self.output > self.max_val:
            self.output = self.max_val
        elif self.output < -self.max_val:
            self.output = -self.max_val
        # Another deviation from Bayes
        prev_sensor_data = sensor_data
    
    def run_simulation(self, system_simulator, duration, commands):
        command_idx = 0
        x = [0]
        v = [0]
        a = []
        if commands[0][0] == 0:
            self.set_new_target(commands[0][1])
            command_idx += 1
        desired_values_list = [self.desired_values]
        self.update(x[0])
        output_list = [self.output]
        
        a = [system_simulator.get_acceleration(output_list[0], v[0], x[0])]
        time_step_size = 0.01
        time_list = np.linspace(0,duration, int(np.ceil(duration/time_step_size)))
        
        for i in range(1, len(time_list)):
            if command_idx <= len(commands)-1:
                if commands[command_idx][0] <= round(time_list[i], 2):
                    self.set_new_target(commands[command_idx][1])
                    command_idx += 1
            desired_values_list.append(self.desired_values)
            x.append(system_simulator.get_position(a[-1], v[-1], x[-1], time_step_size))
            v.append(system_simulator.get_velocity(a[-1], v[-1], time_step_size))
            if time_list[i] % self.dt:
                self.update(x[i])
            output_list.append(self.output)
            a.append(system_simulator.get_acceleration(output_list[-1], v[-1], x[-1]))
        
        return time_list, output_list, x, desired_values_list
    
        #Main code
        controller = PID(kp=1.0, ki=0.1, kd=0.01, dt=0.1, desired_value=500)
        system_simulator = Vertical_Rocket(mass = 50)
        command_list = [[0,100]]
        time_list, output_list, x, desired_values = controller.run_simulation(system_simulator, 100, command_list)

    plt.plot(time_list, x, label='Position')
    plt.plot(time_list, output_list, label='Controller Output')
    plt.plot(time_list, desired_values, label='Desired Value', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position/Output')
    plt.legend()
    plt.show()
