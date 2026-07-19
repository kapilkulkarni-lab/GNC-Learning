import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib.pyplot as plt

from governing_functions import PIDController, rk4_step, pendulum_dynamics

'''Pendulum Simulation with PID Control: This section simulates a pendulum system
 under PID control. The pendulum's angle is controlled to reach a target angle using the PID controller.'''

target = 0  # Target angle (rad)
initial_state = np.array([np.pi/4, 0.0])  # Initial state: angle=pi/4 rad, angular_velocity=0 rad/s
u = np.array([0.0])  # Control input (torque applied to the pendulum) (rad/s^2)
l = 1.0  # Length of the pendulum rod (m)
m = 1.0  # Mass of the pendulum bob (kg)
g = 9.81  # Acceleration due to gravity (m/s^2)
b = 0.1  # Damping coefficient (kg*m^2/s)
dt = 0.01  # Time step for simulation
Kp = -7  # Proportional gain
Ki = 0   # Integral gain
Kd = 5   # Derivative gain

pid = PIDController(dt, Kp, Ki, Kd)  # Initialize PID controller with gains

angle_plot_storage = []  # Initialize an empty array for storing angles
angular_velocity_plot_storage = []  # Initialize an empty array for storing angular velocities
time_storage = []  # Initialize an empty array for storing time points



for t in np.arange(0, 10, dt):
    current_angle = initial_state[0]
    error = pid.angle_error(current_angle, target)  # Calculate angle error
    control_input = pid.compute(error)  # Compute control input using PID controller
    u[0] = control_input  # Update control input

    # Perform RK4 step to update the state of the pendulum
    initial_state = rk4_step(lambda t, x, u: pendulum_dynamics(t, x, u, m, l, b, g), t, initial_state, u, dt)

    # Store the results for plotting
    angle_plot_storage.append(initial_state[0])  # Store angle
    angular_velocity_plot_storage.append(initial_state[1])  # Store angular velocity
    time_storage.append(t)  # Store time

angle_plot_storage = np.array(angle_plot_storage)
angular_velocity_plot_storage = np.array(angular_velocity_plot_storage)
time_storage = np.array(time_storage)

settling_time = pid.settling_time(angle_plot_storage, time_storage, target, threshold=0.03)  # Calculate settling time
print(f"Settling Time: {settling_time:.2f} seconds")  # Print the settling time


plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.plot(time_storage, np.degrees(angle_plot_storage), label='Angle (deg)')
plt.axhline(y=np.degrees(target), color='r', linestyle='--', label='Target Angle (deg)')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.legend()
plt.title('Pendulum Angle vs Time')
plt.grid(True)
plt.subplot(2, 1, 2)
plt.plot(time_storage, angular_velocity_plot_storage, label='Angular Velocity (rad/s)', color='orange')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.legend()
plt.title('Pendulum Angular Velocity vs Time')
plt.grid(True) 
plt.show()  
