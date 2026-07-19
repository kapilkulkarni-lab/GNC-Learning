import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib.pyplot as plt

from governing_functions import quad_dynamics, quat_to_R, quat_error, R_to_quat, dcm_to_euler, rk4_step, guidance_multi_waypoint, simulate_gyro, simulate_accel, simulate_gps, complementary_filter_step, KalmanFilter_multiD, Kp_comp_eff

dt = 0.01
state0 = np.array([0,0,0,  0,0,0,  0,0,0,1,  0,0,0])   # rest, identity orientation
params = {'m': 1.0, 'g': 9.81, 'I': np.diag([0.01,0.01,0.02]), 'cT': 3e-5, 'cM': 1e-6, 'd': 0.2}
# inner (attitude) loop tuned faster than outer (position) loop, both near-critically
# damped, so the outer loop's commands don't outrun what attitude can actually track.
# With noisy sensors the outer loop must stay gentle: Kd_pos multiplies the KF's
# velocity ESTIMATE, so high position bandwidth amplifies estimation noise into
# tilt commands (wn_pos >= 1.5 rad/s drives the loop unstable).
Kp = 8        # ~ ωn=20 rad/s, ζ=0.85 given I~0.01 (linearized: I*θ'' = -Kp*θ/2 - Kd*θ')
Kd = 0.34
Kp_pos = 0.25  # ~ ωn=0.5 rad/s, ζ=0.9 given m=1 -> ~40x slower than attitude loop
Kd_pos = 0.9
# accel carries no attitude info here (thrust is always along body z), so the comp
# filter's correction pulls the estimate toward LEVEL — keep it weak: just enough to
# arrest slow gyro drift without fighting the gyro during tilted flight
Kp_comp = 0.20
bias = np.random.normal(0, 0.005, size=3)
full_time = 60
q_est = state0[6:10]
omega = state0[10:]
P0=np.eye(6)  # Initial estimate covariance
# Process noise: discrete white-noise-acceleration model for the constant-velocity
# filter. Unmodeled acceleration enters velocity as a*dt and position as a*dt^2/2,
# so pos/vel rows get dt^4/4 and dt^2 (with dt^3/2 cross terms) — NOT equal weights.
# sigma_a ~ the largest acceleration the quad can command: g*tan(30 deg) ~ 6 m/s^2.
sigma_a = 6.0
Q = sigma_a**2 * np.kron(np.array([[dt**4/4, dt**3/2],
                                   [dt**3/2, dt**2  ]]), np.eye(3))
R=np.eye(3)  # Measurement noise covariance (GPS noise_std = 1 m)
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

x0 = state0[:6] 

F_cmd = params['m'] * params['g']

kalman_filter_3d = KalmanFilter_multiD(x0, P0, Q, R, F, H)

# solve for omega_hover such that 4*cT*omega_hover**2 == m*g
omega_hover = np.sqrt(params['m'] * params['g'] / (4 * params['cT']))
rotor_speeds = [omega_hover] * 4


state = state0.copy()
true_state = state0.copy()

def quad_dynamics_f(t, x, u):
    return quad_dynamics(t, x, u, params)

# Inverse rotor mixer: solve for the 4 rotor thrusts that produce a desired
# total thrust F and body torque tau, i.e. invert rotor_mixer's linear map.
d, cT, cM = params['d'], params['cT'], params['cM']
mixer_A = np.array([
    [1,       1,       1,       1      ],
    [0,       d,       0,      -d      ],
    [-d,      0,       d,       0      ],
    [cM/cT,  -cM/cT,   cM/cT,  -cM/cT  ],
])
mixer_A_inv = np.linalg.inv(mixer_A)

time_steps = np.arange(0, full_time, dt)
pos_hist = np.zeros((len(time_steps), 3))
vel_hist = np.zeros((len(time_steps), 3))
quat_angle_results = np.zeros((len(time_steps), 4))
quat_omega_results = np.zeros((len(time_steps), 3))
ref_pos_hist = np.zeros((len(time_steps), 3))
true_pos_hist = np.zeros((len(time_steps), 3))
true_vel_hist = np.zeros((len(time_steps), 3))
true_quat_hist = np.zeros((len(time_steps), 4))
true_omega_hist = np.zeros((len(time_steps), 3))
gps_hist = np.zeros((len(time_steps), 3))

waypoints = np.array([[0,0,0,0], [5,0,5,0], [5,10,5,0], [5,5,5,0], [0,5,5,0], [0,0,0,0], [0,0,2,0], [2,0,2,0]])
n_segments = len(waypoints) - 1
segment_times = np.full(n_segments, full_time / n_segments)

calib_time = 2.0
N_calib = int(calib_time/dt)
gyro_bias_est = np.mean([simulate_gyro(np.zeros(3), 0.01, bias) for _ in range(N_calib)], axis=0)

for i, t in enumerate(time_steps):
    true_pos_hist[i] = true_state[0:3]
    true_vel_hist[i] = true_state[3:6]
    true_quat_hist[i] = true_state[6:10]
    true_omega_hist[i] = true_state[10:13]

    # outer loop: position/velocity PD -> desired inertial acceleration -> desired
    # thrust magnitude + tilt (as a target attitude), since thrust only acts along body z
    pos_meas = simulate_gps(true_state[0:3])
    accel_meas = simulate_accel(F_cmd, params['m'])
    omega_meas = simulate_gyro(true_state[10:], 0.01, bias) - gyro_bias_est
    gps_hist[i] = pos_meas

    kalman_filter_3d.predict()
    pos_vel_kf = kalman_filter_3d.update(pos_meas)

    Kp_comp_adj = Kp_comp_eff(F_cmd, params['m'], params['g'], Kp_comp)

    q_est = complementary_filter_step(q_est, omega_meas, accel_meas, dt, Kp_comp_adj)

    state = np.concatenate([pos_vel_kf, q_est, omega_meas])

    # log the estimate after the measurement update so it's time-aligned with truth
    pos_hist[i] = state[0:3]
    vel_hist[i] = state[3:6]
    quat_angle_results[i] = state[6:10]
    quat_omega_results[i] = state[10:13]

    ref = guidance_multi_waypoint(t, waypoints, segment_times)
    pos_target = np.array([ref['x'], ref['y'], ref['z']])
    vel_target = np.array([ref['xd'], ref['yd'], ref['zd']])
    
    ref_pos_hist[i] = pos_target

    pos_error = pos_target - state[0:3]
    vel_error = vel_target - state[3:6]
    a_des = Kp_pos * pos_error + Kd_pos * vel_error
    thrust_vec = params['m'] * a_des + np.array([0.0, 0.0, params['m'] * params['g']])

    # saturate commanded tilt so large position errors can't demand a >max_tilt lean
    # (otherwise, as altitude converges faster than lateral offset, required tilt keeps
    # growing toward 90 deg and the thrust vector's vertical component can go non-positive)
    max_tilt = np.radians(30)
    horiz_mag = np.linalg.norm(thrust_vec[0:2])
    max_horiz = np.tan(max_tilt) * max(thrust_vec[2], 0.0)
    if horiz_mag > max_horiz and horiz_mag > 1e-9:
        thrust_vec[0:2] *= max_horiz / horiz_mag

    F_cmd = np.linalg.norm(thrust_vec)


    z_des = thrust_vec / F_cmd
    x_c = np.array([1.0, 0.0, 0.0])  # reference heading direction (yaw = 0)
    y_des = np.cross(z_des, x_c)
    y_des = y_des / np.linalg.norm(y_des)
    x_des = np.cross(y_des, z_des)
    q_des = R_to_quat(np.column_stack([x_des, y_des, z_des]))

    # inner loop: attitude PD -> drive quaternion error and body rate to zero
    q_error = quat_error(state[6:10], q_des)
    omega = state[10:13]
    tau_cmd = -Kp * q_error[0:3] - Kd * omega

    # map (thrust, torque) command to 4 rotor speeds
    thrusts_cmd = mixer_A_inv @ np.array([F_cmd, *tau_cmd])
    thrusts_cmd = np.clip(thrusts_cmd, 0, None)  # rotors can't produce negative thrust
    u = np.sqrt(thrusts_cmd / cT)

    true_state = rk4_step(quad_dynamics_f, t, true_state, u, dt)


    # renormalize quaternion each step to counter numerical drift
    true_state[6:10] = true_state[6:10] / np.linalg.norm(true_state[6:10])
    

# euler columns: yaw, pitch, roll
est_euler_deg = np.degrees(np.array([dcm_to_euler(quat_to_R(q)) for q in quat_angle_results]))
true_euler_deg = np.degrees(np.array([dcm_to_euler(quat_to_R(q)) for q in true_quat_hist]))

# --- Figure 1: truth vs estimate vs reference ---
# solid = truth, dashed = estimate, dotted = reference, faint dots = raw GPS
fig, axs = plt.subplots(4, 1, sharex=True, figsize=(9, 11))
colors = ['C0', 'C1', 'C2']

for j, lbl in enumerate(['x', 'y', 'z']):
    axs[0].plot(time_steps, gps_hist[:, j], '.', color=colors[j], markersize=1.5, alpha=0.2)
    axs[0].plot(time_steps, true_pos_hist[:, j], color=colors[j], label=f'{lbl} true')
    axs[0].plot(time_steps, pos_hist[:, j], color=colors[j], linestyle='--', linewidth=1)
    axs[0].plot(time_steps, ref_pos_hist[:, j], color=colors[j], linestyle=':', linewidth=1)
axs[0].set_ylabel('Position (m)')
axs[0].set_title('solid = truth, dashed = estimate, dotted = reference, dots = GPS')
axs[0].legend(loc='best', fontsize=8)

for j, lbl in enumerate(['vx', 'vy', 'vz']):
    axs[1].plot(time_steps, true_vel_hist[:, j], color=colors[j], label=f'{lbl} true')
    axs[1].plot(time_steps, vel_hist[:, j], color=colors[j], linestyle='--', linewidth=1)
axs[1].set_ylabel('Velocity (m/s)')
axs[1].legend(loc='best', fontsize=8)

for j, lbl in enumerate(['yaw', 'pitch', 'roll']):
    axs[2].plot(time_steps, true_euler_deg[:, j], color=colors[j], label=f'{lbl} true')
    axs[2].plot(time_steps, est_euler_deg[:, j], color=colors[j], linestyle='--', linewidth=1)
axs[2].set_ylabel('Attitude (deg)')
axs[2].legend(loc='best', fontsize=8)

for j, lbl in enumerate(['wx', 'wy', 'wz']):
    axs[3].plot(time_steps, true_omega_hist[:, j], color=colors[j], label=f'{lbl} true')
    axs[3].plot(time_steps, quat_omega_results[:, j], color=colors[j], linestyle='--', linewidth=0.7, alpha=0.6)
axs[3].set_ylabel('Body rate (rad/s)')
axs[3].set_xlabel('Time (s)')
axs[3].legend(loc='best', fontsize=8)

plt.tight_layout()

# --- Figure 2: estimation + tracking errors (does the filtering actually help?) ---
gps_err = np.linalg.norm(gps_hist - true_pos_hist, axis=1)
kf_err = np.linalg.norm(pos_hist - true_pos_hist, axis=1)
vel_err = np.linalg.norm(vel_hist - true_vel_hist, axis=1)
track_err = np.linalg.norm(true_pos_hist - ref_pos_hist, axis=1)
# attitude error angle between estimated and true quaternion: 2*acos(|q_est . q_true|)
att_err_deg = np.degrees(2 * np.arccos(np.clip(
    np.abs(np.sum(quat_angle_results * true_quat_hist, axis=1)), -1.0, 1.0)))
# per-axis euler estimate error, wrapped to [-180, 180) so yaw crossings don't spike
euler_err_deg = (est_euler_deg - true_euler_deg + 180) % 360 - 180


def rms(x):
    return np.sqrt(np.mean(x**2))


def err_label(name, x, unit):
    return f'{name} (RMS {rms(x):.2f}, max {np.max(np.abs(x)):.2f} {unit})'


fig2, axs2 = plt.subplots(4, 1, sharex=True, figsize=(9, 11))

axs2[0].plot(time_steps, gps_err, color='gray', alpha=0.5, linewidth=0.8, label=err_label('raw GPS', gps_err, 'm'))
axs2[0].plot(time_steps, kf_err, color='C0', label=err_label('KF estimate', kf_err, 'm'))
axs2[0].set_ylabel('Position est. error (m)')
axs2[0].legend(loc='best', fontsize=8)

axs2[1].plot(time_steps, vel_err, color='C0', label=err_label('KF estimate', vel_err, 'm/s'))
axs2[1].set_ylabel('Velocity est. error (m/s)')
axs2[1].legend(loc='best', fontsize=8)

axs2[2].plot(time_steps, track_err, color='C3', label=err_label('|true - ref|', track_err, 'm'))
axs2[2].set_ylabel('Tracking error (m)')
axs2[2].legend(loc='best', fontsize=8)

axs2[3].plot(time_steps, att_err_deg, color='k', linewidth=1.2, label=err_label('total angle', att_err_deg, 'deg'))
for j, lbl in enumerate(['yaw', 'pitch', 'roll']):
    axs2[3].plot(time_steps, euler_err_deg[:, j], color=colors[j], linewidth=0.8, alpha=0.8,
                 label=err_label(lbl, euler_err_deg[:, j], 'deg'))
axs2[3].set_ylabel('Attitude est. error (deg)')
axs2[3].set_xlabel('Time (s)')
axs2[3].legend(loc='best', fontsize=8)

plt.tight_layout()

# --- Console summary: RMS / max / final for each error signal ---
print(f'\n{"error signal":<28}{"RMS":>10}{"max":>10}{"final":>10}')
for name, x in [('raw GPS pos err (m)', gps_err),
                ('KF pos est err (m)', kf_err),
                ('KF vel est err (m/s)', vel_err),
                ('tracking |true-ref| (m)', track_err),
                ('attitude angle err (deg)', att_err_deg),
                ('yaw est err (deg)', euler_err_deg[:, 0]),
                ('pitch est err (deg)', euler_err_deg[:, 1]),
                ('roll est err (deg)', euler_err_deg[:, 2])]:
    print(f'{name:<28}{rms(x):>10.3f}{np.max(np.abs(x)):>10.3f}{x[-1]:>10.3f}')
print(f'{"true gyro bias (rad/s)":<28}  [{bias[0]:.4f} {bias[1]:.4f} {bias[2]:.4f}]')
print(f'{"calib residual (rad/s)":<28}  [{gyro_bias_est[0]-bias[0]:.4f} {gyro_bias_est[1]-bias[1]:.4f} {gyro_bias_est[2]-bias[2]:.4f}]')

plt.show()

