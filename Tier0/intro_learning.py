import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib.pyplot as plt

from governing_functions import rk4_step, spring_mass_system, pendulum_dynamics

'''Simulating Spring Mass System with RK4'''

plot_storage = []  # Initialize an empty array for storing states
t_storage = []
m=1.0  # Mass of the spring-mass system
c=0.1  # Damping coefficient
k=1.0  # Spring constant
dt = 0.1 # Time step for simulation
u = np.array([0.0])  # Control input (force applied to the mass)
state0 = np.array([1.0, 0.2])  # Initial state: position=1.0, velocity=0.0

for t in np.arange(0, 100, dt):
    state = rk4_step(lambda t, x, u: spring_mass_system(t, x, u, m, c, k), t, state0, u, dt)
    state0 = state  # Update state for the next iteration
    plot_storage.append(state)
    t_storage.append(t)

plot_storage = np.array(plot_storage)
t_storage = np.array(t_storage)

positions = plot_storage[:, 0] #First column of plot_storage corresponds to positions
velocities = plot_storage[:, 1] #Second column of plot_storage corresponds to velocities
times = t_storage

plt.figure(figsize=(10, 6))
plt.plot(times, positions, label='Position')
plt.plot(times, velocities, label='Velocity')
plt.xlabel('Time')
plt.ylabel('State')
plt.legend()
plt.show()

'''Open-loop pendulum swing (no controller) — demonstrates rk4_step + pendulum_dynamics.'''

state0 = np.array([np.pi/4, 0.0])  # Initial state
u = np.array([0.0])  # Control input (torque applied to the pendulum) (rad/s^2)
l = 1.0  # Length of the pendulum rod (m)
m = 1.0  # Mass of the pendulum bob (kg)
g = 9.81  # Acceleration due to gravity (m/s^2)
b = 0.1  # Damping coefficient (kg*m^2/s)

angle_plot_storage = []  # Initialize an empty array for storing states
angle_t_storage = []  # Initialize an empty array for storing time points
angular_velocity_plot_storage = []  # Initialize an empty array for storing angular velocities

for t in np.arange(0, 10, dt):
    state = rk4_step(lambda t, x, u: pendulum_dynamics(t, x, u, m, l, b, g), t, state0, u, dt)
    state0 = state  # Update state for the next iteration
    angle_plot_storage.append(state[0])  # Store angle
    angular_velocity_plot_storage.append(state[1])  # Store angular velocity
    angle_t_storage.append(t)  # Store time

angle_plot_storage = np.array(angle_plot_storage)
angular_velocity_plot_storage = np.array(angular_velocity_plot_storage)
angle_t_storage = np.array(angle_t_storage)

angle_deg_storage = np.degrees(angle_plot_storage)  # Convert angle from radians to degrees

plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.plot(angle_t_storage, angle_deg_storage, label='Angle (deg)')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.legend()
plt.subplot(2, 1, 2)
plt.plot(angle_t_storage, angular_velocity_plot_storage, label='Angular Velocity (rad/s)', color='orange')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.legend()
plt.tight_layout()
plt.show()
