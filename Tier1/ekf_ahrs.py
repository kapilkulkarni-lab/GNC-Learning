"Developing a EKF based AHRS: Attitude estimation with gyro basis "

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib.pyplot as plt
from governing_functions import (
    rk4_step, quat_to_R, dcm_to_euler, quat_attitude_dynamics, Extended_KalmanFilter,
    simulate_gyro, simulate_accel_gravity, complementary_filter_step, gyro_only_step,
    ahrs_process_model, ahrs_measurement_model,
)
# --- Step 1: Generate the TRUE trajectory (reuse your rigid body sim) ---
dt = 0.01
true_state0 = np.array([0.0, 0.0, 0.0, 1.0,
                         0.1, 0.2, 0.0])   # some tumble, so there's real motion
I = np.diag([1.0, 2.0, 3.0])
u_true = np.array([0.0, 0.0, 0.0])          # free rotation, isolates the estimator

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

# --- Step 2: Sensor simulation (reuse the shared sensor-simulation functions) ---
gyro_noise_std = 0.01
accel_noise_std = 0.02
gyro_bias_true = np.array([0.02, -0.01, 0.015])   # constant, UNKNOWN to the filter

# --- Step 3/4: AHRS process/measurement models — see governing_functions.ahrs_models ---
# x = [qx, qy, qz, qw, bx, by, bz]   (7 elements)
# u = omega_gyro_measured             (3 elements, RAW gyro reading, bias NOT yet removed)

# --- Step 5: Noise matrices ---
# TODO: think through these before just copying numbers —
# P0 (7x7): how uncertain are you INITIALLY about q vs bias? These are different kinds of
#           uncertainty — consider whether they deserve the same magnitude.
# Q  (7x7): quaternion rows vs bias rows should likely NOT match — one changes fast (rotation),
#           one should barely change at all per step (slow bias drift). What does that imply
#           about relative magnitudes?
# R  (3x3): matches the actual accel_noise_std you're injecting above.
P0 = np.diag([0.5, 0.5, 0.5, 0.5, 0.01, 0.01, 0.01])
Q = np.diag([0.001, 0.001, 0.001, 0.001, 1e-6, 1e-6, 1e-6])
R = np.eye(3) * (accel_noise_std ** 2)

# --- Step 6: Initialize and run the filter ---
x0 = np.array([0.3, 0.0, 0.0, 0.95, 0.0, 0.0, 0.0])   # deliberately wrong q, zero initial bias guess
x0[0:4] = x0[0:4] / np.linalg.norm(x0[0:4])

ekf_ahrs = Extended_KalmanFilter(x0, P0, Q, R,
                                   f=ahrs_process_model,
                                   h=ahrs_measurement_model,
                                   u=np.array([0.0, 0.0, 0.0]))

ekf_state_storage = []
q_est_storage = []
gyro_only_storage = []
Kp_comp = 1
q_est = np.array([0.3, 0.0, 0.0, 0.95])
q_est = q_est / np.linalg.norm(q_est)
q_gyro_only = q_est.copy()

for i, t in enumerate(t_storage):
    true_q = true_state_storage[i, 0:4]
    true_omega = true_state_storage[i, 4:7]

    omega_meas = simulate_gyro(true_omega, gyro_noise_std, gyro_bias_true)
    accel_meas = simulate_accel_gravity(true_q, accel_noise_std)

    ekf_ahrs.predict(dt, u=omega_meas)
    x = ekf_ahrs.update(accel_meas)
    ekf_state_storage.append(x.copy())
    true_q = true_state_storage[i, 0:4]
    true_omega = true_state_storage[i, 4:7]

    omega_meas = simulate_gyro(true_omega, gyro_noise_std, gyro_bias_true)
    accel_meas = simulate_accel_gravity(true_q, accel_noise_std)

    q_est = complementary_filter_step(q_est, omega_meas, accel_meas, dt, Kp_comp)
    q_est_storage.append(q_est.copy())

    q_gyro_only = gyro_only_step(q_gyro_only, omega_meas, dt)
    gyro_only_storage.append(q_gyro_only)



ekf_state_storage = np.array(ekf_state_storage)
q_est_storage = np.array(q_est_storage)
gyro_only_storage = np.array(gyro_only_storage)

# --- Step 7: Plot true vs EKF-estimated orientation, and estimated bias vs true bias ---
# TODO: reuse your Euler-angle comparison pattern from the complementary filter test,
# PLUS a new plot: ekf_state_storage[:, 4:7] (estimated bias) vs gyro_bias_true (flat lines),
# since watching the bias estimate CONVERGE to the true constant bias is the signature
# result that distinguishes this from the complementary filter.

# --- Plotting: true vs complementary filter vs gyro-only drift ---
true_euler = np.array([dcm_to_euler(quat_to_R(q)) for q in true_state_storage[:, 0:4]])
gyro_only_euler = np.array([dcm_to_euler(quat_to_R(q)) for q in gyro_only_storage[:, 0:4]])
est_euler = np.array([dcm_to_euler(quat_to_R(q)) for q in q_est_storage])
ekf_state_storage_euler = np.array([dcm_to_euler(quat_to_R(q)) for q in ekf_state_storage[:, 0:4]])

plt.figure(figsize=(10, 6))
plt.plot(t_storage, np.degrees(true_euler[:, 2]), label='True Roll', color='green', linestyle='--')
plt.plot(t_storage, np.degrees(est_euler[:, 2]), label='Complementary Filter Roll', color='blue')
plt.plot(t_storage, np.degrees(gyro_only_euler[:, 2]), label='Gyro-Only Roll (no correction)', color='red', alpha=0.6)
plt.plot(t_storage, np.degrees(ekf_state_storage_euler[:, 2]), label = 'EKF Roll', color = 'yellow' )
plt.xlabel('Time (s)')
plt.ylabel('Roll (deg)')
plt.title('Attitude Estimation: True vs Complementary Filter vs Gyro-Only')
plt.legend()
plt.grid(True)
plt.show()
