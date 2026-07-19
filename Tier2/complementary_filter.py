'''Complementary Filter for Attitude Estimation'''
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib.pyplot as plt

from governing_functions import (
    rk4_step, quat_to_R, dcm_to_euler, quat_attitude_dynamics,
    simulate_gyro, simulate_accel_gravity, complementary_filter_step, gyro_only_step,
)

# --- Simulation setup: generate a TRUE trajectory to estimate against ---
dt = 0.01
true_state0 = np.array([0.0, 0.0, 0.0, 1.0,
                         0.1, 0.2, 0.0])   # some initial tumble, so there's real motion to track

I = np.diag([1.0, 2.0, 3.0])
u_true = np.array([0.0, 0.0, 0.0])   # free rotation, no controller — isolates the estimator

true_state_storage = []
t_storage = []

for t in np.arange(0, 10, dt):
    true_state = rk4_step(lambda t, x, u: quat_attitude_dynamics(t, x, u, I), t, true_state0, u_true, dt)
    true_state0 = true_state
    true_state0[0:4] = true_state0[0:4] / np.linalg.norm(true_state0[0:4])
    true_state_storage.append(true_state0.copy())
    t_storage.append(t)

true_state_storage = np.array(true_state_storage)
t_storage = np.array(t_storage)

# --- Sensor simulation ---
gyro_noise_std = 0.01     # rad/s
accel_noise_std = 0.02    # unitless (normalized gravity direction)

# --- Complementary filter ---
Kp_comp = 1.0   # correction gain — tune this once the loop runs

# --- Run the filter alongside the true trajectory ---
q_est = np.array([0.0, 0.4, 0.3, 0.95])   # start at identity — deliberately WRONG vs true_state0's tumble,
                                            # so you can see the filter converge, not just track
q_est = q_est/np.linalg.norm(q_est)


q_est_storage = []
gyro_only_storage = [] 

q_gyro_only = q_est

for i, t in enumerate(t_storage):
    true_q = true_state_storage[i, 0:4]
    true_omega = true_state_storage[i, 4:7]

    omega_meas = simulate_gyro(true_omega, gyro_noise_std)
    accel_meas = simulate_accel_gravity(true_q, accel_noise_std)

    q_est = complementary_filter_step(q_est, omega_meas, accel_meas, dt, Kp_comp)
    q_est_storage.append(q_est.copy())

    q_gyro_only = gyro_only_step(q_gyro_only, omega_meas, dt)
    gyro_only_storage.append(q_gyro_only)

q_est_storage = np.array(q_est_storage)
gyro_only_storage = np.array(gyro_only_storage)

# --- Plotting: true vs complementary filter vs gyro-only drift ---
true_euler = np.array([dcm_to_euler(quat_to_R(q)) for q in true_state_storage[:, 0:4]])
gyro_only_euler = np.array([dcm_to_euler(quat_to_R(q)) for q in gyro_only_storage[:, 0:4]])
est_euler = np.array([dcm_to_euler(quat_to_R(q)) for q in q_est_storage])

plt.figure(figsize=(10, 6))
plt.plot(t_storage, np.degrees(true_euler[:, 2]), label='True Roll', color='green', linestyle='--')
plt.plot(t_storage, np.degrees(est_euler[:, 2]), label='Complementary Filter Roll', color='blue')
plt.plot(t_storage, np.degrees(gyro_only_euler[:, 2]), label='Gyro-Only Roll (no correction)', color='red', alpha=0.6)
plt.xlabel('Time (s)')
plt.ylabel('Roll (deg)')
plt.title('Attitude Estimation: True vs Complementary Filter vs Gyro-Only')
plt.legend()
plt.grid(True)
plt.show()