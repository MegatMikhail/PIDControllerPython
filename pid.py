import numpy as np
import matplotlib.pyplot as plt

G = 9.81

class Vertical_Rocket:
    def __init__(self, mass):  # Add drag coefficient later?
        self.mass = mass
    
    def get_acceleration(self, force):
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
        self.desired_value = desired_value  # Target value
        self.dt = dt  # Sample rate
        self.error = 0.0
        self.prev_sensor_data = 0.0  # Last sensor reading
        self.derivative_error = 0.0
        self.integral_error = 0.0  # Accumulator for integral term
        self.max_val = 5000  # Limit on controller output
        
    def update(self, sensor_data):  # PID controller logic
        # Calculate errors
        self.error = self.desired_value - sensor_data
        self.integral_error += self.error * self.dt
        self.derivative_error = (sensor_data - self.prev_sensor_data) / self.dt
        
        # PID output calculation
        self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error
        
        # Saturate output to max_val
        if self.output > self.max_val:
            self.output = self.max_val
        elif self.output < -self.max_val:
            self.output = -self.max_val
        
        # Update previous sensor data
        self.prev_sensor_data = sensor_data
    
    def set_new_target(self, new_target):
        self.desired_value = new_target
    
    def run_simulation(self, system_simulator, duration, commands):
        command_idx = 0
        x = [0]  # Initial position
        v = [0]  # Initial velocity
        a = []   # To store acceleration
        time_step_size = 0.01
        time_list = np.linspace(0, duration, int(np.ceil(duration / time_step_size)))
        
        # Initialize controller with first command
        if commands[0][0] == 0:
            self.set_new_target(commands[0][1])
            command_idx += 1
        
        desired_values_list = [self.desired_value]
        self.update(x[0])  # Initial controller update
        output_list = [self.output]
        a.append(system_simulator.get_acceleration(output_list[0]))
        
        for i in range(1, len(time_list)):
            # Check if a new command should be applied
            if command_idx < len(commands) and commands[command_idx][0] <= time_list[i]:
                self.set_new_target(commands[command_idx][1])
                command_idx += 1
            
            # Update desired value list
            desired_values_list.append(self.desired_value)
            
            # Compute new position and velocity
            x.append(system_simulator.get_position(a[-1], v[-1], x[-1], time_step_size))
            v.append(system_simulator.get_velocity(a[-1], v[-1], time_step_size))
            
            # Controller update at every sample interval
            if i % int(self.dt / time_step_size) == 0:
                self.update(x[-1])
            
            # Compute new acceleration and store output
            output_list.append(self.output)
            a.append(system_simulator.get_acceleration(output_list[-1]))
        
        return time_list, output_list, x, desired_values_list

# Main code
controller = PID(kp=1.0, ki=0.1, kd=0.01, dt=0.1, desired_value=100)
system_simulator = Vertical_Rocket(mass=50)
command_list = [[0, 100], [50, 50]]  # Example commands, change as needed

time_list, output_list, x, desired_values = controller.run_simulation(system_simulator, 100, command_list)

# Plotting the results
plt.plot(time_list, x, label='Position')
plt.plot(time_list, output_list, label='Controller Output')
plt.plot(time_list, desired_values, label='Desired Value', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Position/Output')
plt.legend()
plt.show() 
