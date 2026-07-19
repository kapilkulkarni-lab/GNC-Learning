'''Rigid Body Attitude Simulation using quaternions'''
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib.pyplot as plt

from governing_functions import rk4_step, PIDController, quat_to_R, dcm_to_euler, quat_attitude_dynamics, quat_multiply, quat_error

dt = 0.01
state0 = np.array([0.0, 0.0, 0.0, 1.0,
                    0.1, 0.0, 0.0])

I = np.diag([1.0, 2.0, 3.0])
u = np.array([0,0,0])

quat_angle_results = []
quat_omega_results = []
t_storage = []
x_target = np.array([0.0, 0.0, 0.0, 1.0])
Kp = 1
Kd = 1

for t in np.arange(0,10,dt):
    state = rk4_step(lambda t, x, u: quat_attitude_dynamics(t, x, u, I), t, state0, u, dt)
    state0 = state
    state[0:4] = state[0:4] / np.linalg.norm(state[0:4])

    x = state[0:4]
    x_dot = state[4:]

    x_error = quat_error(x, x_target)

    u = -Kp * x_error[0:3] - Kd * x_dot

    quat_angle_results.append(state[0:4])
    quat_omega_results.append(state[4:])
    t_storage.append(t)

quat_angle_results = np.array(quat_angle_results)
euler_data = np.array([dcm_to_euler(quat_to_R(q)) for q in quat_angle_results])
quat_omega_results = np.array(quat_omega_results)
t_storage = np.array(t_storage)

# --- Expected/target values, for comparison ---
target_euler = dcm_to_euler(quat_to_R(x_target))   # target orientation as Euler angles (should be [0,0,0])
target_omega = np.zeros(3)                          # target angular velocity is zero (pointing steady, not tumbling)

# --- Plot 1: Euler angles vs target ---
plt.figure(figsize=(10, 6))
plt.plot(t_storage, np.degrees(euler_data[:, 0]), label='Yaw (deg)')
plt.plot(t_storage, np.degrees(euler_data[:, 1]), label='Pitch (deg)')
plt.plot(t_storage, np.degrees(euler_data[:, 2]), label='Roll (deg)')
plt.axhline(y=np.degrees(target_euler[0]), color='k', linestyle='--', alpha=0.5, label='Target Yaw')
plt.axhline(y=np.degrees(target_euler[1]), color='gray', linestyle=':', alpha=0.5, label='Target Pitch')
plt.axhline(y=np.degrees(target_euler[2]), color='gray', linestyle='-.', alpha=0.5, label='Target Roll')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.title('Orientation as Euler Angles vs Target')
plt.legend()
plt.grid(True)

# --- Plot 2: Quaternion components vs target ---
plt.figure(figsize=(10, 6))
plt.plot(t_storage, quat_angle_results[:, 0], label='qx')
plt.plot(t_storage, quat_angle_results[:, 1], label='qy')
plt.plot(t_storage, quat_angle_results[:, 2], label='qz')
plt.plot(t_storage, quat_angle_results[:, 3], label='qw')
plt.axhline(y=x_target[0], color='k', linestyle='--', alpha=0.4, label='Target qx')
plt.axhline(y=x_target[3], color='gray', linestyle=':', alpha=0.4, label='Target qw')
plt.xlabel('Time (s)')
plt.ylabel('Quaternion components')
plt.title('Orientation Quaternion vs Target')
plt.legend()
plt.grid(True)

# --- Plot 3: Angular velocity vs target (zero) ---
plt.figure(figsize=(10, 6))
plt.plot(t_storage, quat_omega_results[:, 0], label='ωx (rad/s)')
plt.plot(t_storage, quat_omega_results[:, 1], label='ωy (rad/s)')
plt.plot(t_storage, quat_omega_results[:, 2], label='ωz (rad/s)')
plt.axhline(y=0.0, color='k', linestyle='--', alpha=0.5, label='Target ω (zero)')
plt.xlabel('Time (s)')
plt.ylabel('Angular velocity (rad/s)')
plt.title('Body-Frame Angular Velocity vs Target')
plt.legend()
plt.grid(True)

# --- Plot 4 (NEW): scalar error metric — cleanest single view of convergence ---
error_angle_deg = np.array([2 * np.degrees(np.arccos(np.clip(quat_error(q, x_target)[3], -1, 1)))
                             for q in quat_angle_results])

plt.figure(figsize=(10, 4))
plt.plot(t_storage, error_angle_deg, label='Attitude Error Magnitude', color='red')
plt.axhline(y=0.0, color='k', linestyle='--', alpha=0.5, label='Converged')
plt.xlabel('Time (s)')
plt.ylabel('Error angle (deg)')
plt.title('Total Attitude Error vs Time')
plt.legend()
plt.grid(True)

plt.show()