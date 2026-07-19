import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from governing_functions import KalmanFilter_1D, KalmanFilter_multiD, kinematics_3D, Extended_KalmanFilter, rk4_step, pendulum_dynamics

# Define the system parameters

x0 = 0.0  # Initial state
P0 = 1.0  # Initial estimate covariance
Q = 1e-5  # Process noise covariance
R = 1.0  # Measurement noise covariance
dt = 0.1  # Time step
vel = 2.0

kalman_filter = KalmanFilter_1D(x0, P0, Q, R)

kalman_filter_storage = []  # Initialize an empty array for storing Kalman filter estimates
time_storage = []  # Initialize an empty array for storing time points

for t in np.arange(0, 10, dt):
    # Simulate the true state and measurement
    true_state = vel * t  # True state (position)
    measurement = true_state + np.random.normal(0, np.sqrt(R))  # Noisy measurement

    # Kalman filter prediction and update
    kalman_filter.predict(vel, dt)
    kalman_filter.update(measurement)
    time_storage.append(t)

    # Store the Kalman filter estimate
    kalman_filter_storage.append(kalman_filter.x)

kalman_filter_storage = np.array(kalman_filter_storage)
time_storage = np.array(time_storage)

plt.figure(figsize=(10, 6))
plt.plot(time_storage, kalman_filter_storage, label='Kalman Filter Estimate', color='blue')
plt.plot(time_storage, vel * time_storage, label='True State', color='green', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.title('Kalman Filter Performance')
plt.legend()
plt.grid(True)
plt.show()

pos0=np.array([-2.0, 0.0, 0.0])  # Initial state: position=0
vel0=np.array([2.0, 1.0, 3.0])  # Initial velocity: 2 m/s
accel0=np.array([1.0, 0.0, 0.0])  # Initial acceleration: 0 m/s^2
g = np.array([0.0, 0.0, -9.81])  # Gravity vector

# State transition matrix (6x6): constant-velocity model
F = np.array([
    [1, 0, 0, dt, 0,  0 ],
    [0, 1, 0, 0,  dt, 0 ],
    [0, 0, 1, 0,  0,  dt],
    [0, 0, 0, 1,  0,  0 ],
    [0, 0, 0, 0,  1,  0 ],
    [0, 0, 0, 0,  0,  1 ],
])

# Measurement matrix (3x6): we only measure position (like GPS)
H = np.array([
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
])


P0=np.eye(6)  # Initial estimate covariance
Q=np.eye(6) * 0.05  # Process noise covariance
R=np.eye(3)  # Measurement noise covariance
x0 = np.concatenate((pos0, vel0))  

kalman_filter_3d = KalmanFilter_multiD(x0, P0, Q, R, F, H)

kalman_filter_storage_3d = []  # Initialize an empty array for storing Kalman filter estimates
measurement_storage_3d = []  # Initialize an empty array for storing measurements
time_storage_3d = []  # Initialize an empty array for storing time points
true_positions = []  # Initialize an empty array for storing true positions
state0 = np.concatenate((pos0, vel0))  # Initial state vector
accel = accel0 + g  # Acceleration vector including gravity
for t in np.arange(0, 10, dt):
    # Simulate the true state and measurement
    pos, vel = kinematics_3D(t, accel, pos0, vel0)
    true_state = np.concatenate((pos, vel))
    true_positions.append(pos)

    measurement = pos + np.random.normal(0, np.sqrt(R[0, 0]), size=3)  # Noisy measurement of position

    # Kalman filter prediction and update
    kalman_filter_3d.predict()
    x = kalman_filter_3d.update(measurement)

    # Store the Kalman filter estimate
    kalman_filter_storage_3d.append(x)
    measurement_storage_3d.append(measurement)
    time_storage_3d.append(t)


kalman_filter_storage_3d = np.array(kalman_filter_storage_3d)
measurement_storage_3d = np.array(measurement_storage_3d)
time_storage_3d = np.array(time_storage_3d)
true_positions = np.array(true_positions)

fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(kalman_filter_storage_3d[:, 0], kalman_filter_storage_3d[:, 1], kalman_filter_storage_3d[:, 2], label='Kalman Filter Estimate', color='blue')
ax.scatter(measurement_storage_3d[:, 0], measurement_storage_3d[:, 1], measurement_storage_3d[:, 2], label='Measurements', color='red', s=10)
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.set_title('3D Kalman Filter Performance')
ax.legend()
ax.grid(True)   

plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(time_storage_3d, kalman_filter_storage_3d[:, 0], label='Kalman Filter Estimate', color='blue')
plt.plot(time_storage_3d, true_positions[:, 0], label='True State', color='green', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Position X')
plt.title('Kalman Filter Performance')
plt.legend()
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(time_storage_3d, kalman_filter_storage_3d[:, 1], label='Kalman Filter Estimate', color='blue')
plt.plot(time_storage_3d, true_positions[:, 1], label='True State', color='green', linestyle='--')
plt.xlabel('Time (s)')  
plt.ylabel('Position Y')
plt.title('Kalman Filter Performance')  
plt.legend()
plt.grid(True)  

plt.subplot(3, 1, 3)
plt.plot(time_storage_3d, kalman_filter_storage_3d[:, 2], label='Kalman Filter Estimate', color='blue')
plt.plot(time_storage_3d, true_positions[:, 2], label='True State', color='green', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Position Z')
plt.title('Kalman Filter Performance')
plt.legend()
plt.grid(True)  
plt.show() 

initial_condition = np.array([np.pi/3, 0.0])  # save initial condition explicitly
state0 = initial_condition.copy()
u = np.array([0.0])
l = 1.0
m = 1.0
g = 9.81
b = 0.1

true_state_storage = []
angle_t_storage = []

for t in np.arange(0, 10, dt):
    state = rk4_step(lambda t, x, u: pendulum_dynamics(t, x, u, m, l, b, g), t, state0, u, dt)
    state0 = state
    true_state_storage.append(state.copy())
    angle_t_storage.append(t)

true_state_storage = np.array(true_state_storage)   # (N, 2): [angle, angular_velocity]
angle_t_storage = np.array(angle_t_storage)

# --- EKF setup ---
x0 = initial_condition   # EKF starts at the SAME initial condition as the true system
P0 = np.array([[1.0, 0.0], [0.0, 1.0]])
Q = np.eye(2) * 0.001
R = np.eye(2) * 0.1
h = lambda x: np.array([l*np.sin(x[0]), -l*np.cos(x[0])])

f = lambda x, u, dt: rk4_step(lambda t, x, u: pendulum_dynamics(t, x, u, m, l, b, g), 0, x, u, dt)
accel = np.array([0.0])

extended_kalman_filter = Extended_KalmanFilter(x0, P0, Q, R, f, h, u=accel)

ekf_storage = []
measurement_storage = []

for i, t in enumerate(np.arange(0, 10, dt)):
    true_position = h(true_state_storage[i])
    measurement = true_position + np.random.multivariate_normal(mean=[0, 0], cov=R)

    extended_kalman_filter.predict(dt)
    x = extended_kalman_filter.update(measurement)

    ekf_storage.append(x.copy())
    measurement_storage.append(measurement)

ekf_storage = np.array(ekf_storage)
measurement_storage = np.array(measurement_storage)

# Convert angles to degrees for readability, matching your other plots
true_angle_deg = np.degrees(true_state_storage[:, 0])
ekf_angle_deg = np.degrees(ekf_storage[:, 0])

plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)
plt.plot(angle_t_storage, true_angle_deg, label='True Angle', color='green', linestyle='--')
plt.plot(angle_t_storage, ekf_angle_deg, label='EKF Estimated Angle', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.title('Pendulum Angle: True vs EKF Estimate')
plt.legend()
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(angle_t_storage, true_state_storage[:, 1], label='True Angular Velocity', color='green', linestyle='--')
plt.plot(angle_t_storage, ekf_storage[:, 1], label='EKF Estimated Angular Velocity', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Pendulum Angular Velocity: True vs EKF Estimate')
plt.legend()
plt.grid(True)

plt.tight_layout()

# Second figure: the noisy (x,y) position measurements vs true bob position,
# since that's the actual raw signal the EKF had to work with
true_positions = np.array([h(s) for s in true_state_storage])  # true (x,y), noise-free

plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)
plt.plot(angle_t_storage, true_positions[:, 0], label='True X Position', color='green', linestyle='--')
plt.scatter(angle_t_storage, measurement_storage[:, 0], label='Measured X Position', color='red', s=10)
plt.xlabel('Time (s)')
plt.ylabel('X Position (m)')
plt.title('Bob Position: True vs Noisy Measurement')
plt.legend()
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(angle_t_storage, true_positions[:, 1], label='True Y Position', color='green', linestyle='--')
plt.scatter(angle_t_storage, measurement_storage[:, 1], label='Measured Y Position', color='red', s=10)
plt.xlabel('Time (s)')
plt.ylabel('Y Position (m)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()