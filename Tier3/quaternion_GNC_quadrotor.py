import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib.pyplot as plt

from governing_functions import quad_dynamics, quat_to_R, quat_error, R_to_quat, dcm_to_euler, rk4_step, guidance_multi_waypoint
dt = 0.01
state0 = np.array([0,10,0,  0,0,0,  0,0,0,1,  0,0,0])   # rest, identity orientation
params = {'m': 1.0, 'g': 9.81, 'I': np.diag([0.01,0.01,0.02]), 'cT': 3e-5, 'cM': 1e-6, 'd': 0.2}
# inner (attitude) loop tuned faster than outer (position) loop, both near-critically
# damped, so the outer loop's commands don't outrun what attitude can actually track
Kp = 4       # ~ ωn=20 rad/s, ζ=0.7 given I~0.01 -> settles in ~0.3s
Kd = 0.28
Kp_pos = 1.0  # ~ ωn=1 rad/s, ζ=0.9 given m=1 -> settles in ~4-5s, >>10x slower than attitude loop
Kd_pos = 1.8
full_time = 10

# solve for omega_hover such that 4*cT*omega_hover**2 == m*g
omega_hover = np.sqrt(params['m'] * params['g'] / (4 * params['cT']))
rotor_speeds = [omega_hover] * 4

state = state0.copy()

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

waypoints = np.array([[0,0,0,0], [1,0,1,0], [1,1,1,0]])
n_segments = len(waypoints) - 1
segment_times = np.full(n_segments, full_time / n_segments)

for i, t in enumerate(time_steps):
    pos_hist[i] = state[0:3]
    vel_hist[i] = state[3:6]
    quat_angle_results[i] = state[6:10]
    quat_omega_results[i] = state[10:13]

    ref = guidance_multi_waypoint(t, waypoints, segment_times)
    pos_target = np.array([ref['x'], ref['y'], ref['z']])
    vel_target = np.array([ref['xd'], ref['yd'], ref['zd']])
    ref_pos_hist[i] = pos_target

    # outer loop: position/velocity PD -> desired inertial acceleration -> desired
    # thrust magnitude + tilt (as a target attitude), since thrust only acts along body z
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

    state = rk4_step(quad_dynamics_f, t, state, u, dt)

    # renormalize quaternion each step to counter numerical drift
    state[6:10] = state[6:10] / np.linalg.norm(state[6:10])

euler_data = np.array([dcm_to_euler(quat_to_R(q)) for q in quat_angle_results])

# confirm position tracks the waypoint reference trajectory and velocity/attitude settle
euler_deg = np.degrees(euler_data)  # columns: yaw, pitch, roll

fig, axs = plt.subplots(4, 1, sharex=True, figsize=(8, 10))

axs[0].plot(time_steps, pos_hist, label=['x', 'y', 'z'])
axs[0].set_prop_cycle(None)  # reset color cycle so ref lines match their axis's color
for j, style in enumerate(['C0', 'C1', 'C2']):
    axs[0].plot(time_steps, ref_pos_hist[:, j], color=style, linestyle='--', linewidth=0.8)
axs[0].set_ylabel('Position (m)')
axs[0].legend()

axs[1].plot(time_steps, vel_hist, label=['vx', 'vy', 'vz'])
axs[1].set_ylabel('Velocity (m/s)')
axs[1].legend()

axs[2].plot(time_steps, euler_deg, label=['yaw', 'pitch', 'roll'])
axs[2].set_ylabel('Attitude (deg)')
axs[2].legend()

axs[3].plot(time_steps, quat_omega_results, label=['wx', 'wy', 'wz'])
axs[3].set_ylabel('Body rate (rad/s)')
axs[3].set_xlabel('Time (s)')
axs[3].legend()

plt.tight_layout()
plt.show()

