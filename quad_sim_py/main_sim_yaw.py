"""Quad Sim Script - Multi-Waypoint Trajectory (port of MainSimYaw.m)."""
import numpy as np
import matplotlib.pyplot as plt

from quad_dynamics import quad_dynamics, rk4_step
from quad_controller_pid import quad_controller_pid
from guidance_multi_waypoint import guidance_multi_waypoint
from quad_sensors import quad_sensors
from gnc_functions import KalmanFilter_multiD, simulate_gyro, euler_to_quat

# Parameters
params = {
    'm': 1.062,     # mass
    'g': 9.81,      # gravity
    'd': 0.17,      # arm length
    'J': np.diag([1.07e-2, 1.11e-2, 2.29e-2]),   # moment of inertia
    'cT': 5.724165e-8,      # thrust coefficient
    'cM': 8.881631e-10,     # moment coefficient
}

gains = {
    'pos': {
        'Kp': np.array([1.0, 1.0, 5.0]),
        'Kd': np.array([2.0, 2.0, 3.5]),
        'Ki': np.array([0.05, 0.05, 1.0]),
    },
    'att': {
        # Cascade: Kp maps angle error -> desired rate, Kd is the rate-loop gain.
        # Linearized inner loop: wn = sqrt(Kp*Kd), zeta = sqrt(Kd/Kp)/2, so this
        # is wn ~ 14 rad/s, zeta ~ 0.87 -- well above the position loop, which is
        # required: stiff pos gains with a slow inner loop diverge under sensor noise.
        'Kp': np.array([8.0, 8.0, 8.0]),
        'Kd': np.array([24.0, 24.0, 24.0]),
    },
}

# Guidance Info - Multi-Waypoint Trajectory
# User frame waypoints (z = altitude UP, yaw in degrees)
# Format: [x, y, z, yaw_deg]

# Square, yaw at corners
waypoints_user = np.array([
    # Phase 1: Square spiral staircase (3 floors, growing footprint)
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [1.0, 0.0, 1.0, 0.0],
    [1.0, 1.0, 1.0, 90.0],
    [0.0, 1.0, 1.0, -180.0],
    [0.0, 0.0, 1.0, -90.0],
    [0.0, 0.0, 2.0, 0.0],
    [1.5, 0.0, 2.0, 0.0],
    [1.5, 1.5, 2.0, 90.0],
    [0.0, 1.5, 2.0, -180.0],
    [0.0, 0.0, 2.0, -90.0],
    [0.0, 0.0, 3.0, 0.0],
    [2.0, 0.0, 3.0, 0.0],
    [2.0, 2.0, 3.0, 90.0],
    [0.0, 2.0, 3.0, -180.0],
    [0.0, 0.0, 3.0, -90.0],

    # Phase 2: Expanding helix climb
    [1.254, 0.911, 3.15, 126.0],
    [0.494, 1.522, 3.3, 162.0],
    [-0.51, 1.569, 3.45, -162.0],
    [-1.375, 0.999, 3.6, -126.0],
    [-1.75, 0.0, 3.75, -90.0],
    [-1.456, -1.058, 3.9, -54.0],
    [-0.572, -1.759, 4.05, -18.0],
    [0.587, -1.807, 4.2, 18.0],
    [1.578, -1.146, 4.35, 54.0],
    [2.0, -0.0, 4.5, 90.0],
    [1.658, 1.205, 4.65, 126.0],
    [0.649, 1.997, 4.8, 162.0],
    [-0.664, 2.045, 4.95, -162.0],
    [-1.78, 1.293, 5.1, -126.0],
    [-2.25, 0.0, 5.25, -90.0],
    [-1.861, -1.352, 5.4, -54.0],
    [-0.726, -2.235, 5.55, -18.0],
    [0.742, -2.283, 5.7, 18.0],
    [1.982, -1.44, 5.85, 54.0],
    [2.5, -0.0, 6.0, 90.0],

    # Phase 3: Figure-8 sweep at constant altitude
    [0.0, 0.0, 6.0, 26.6],
    [0.518, 0.25, 6.0, 24.1],
    [1.0, 0.433, 6.0, 16.1],
    [1.414, 0.5, 6.0, 0.0],
    [1.732, 0.433, 6.0, -26.6],
    [1.932, 0.25, 6.0, -59.1],
    [2.0, 0.0, 6.0, -90.0],
    [1.932, -0.25, 6.0, -120.9],
    [1.732, -0.433, 6.0, -153.4],
    [1.414, -0.5, 6.0, -180.0],
    [1.0, -0.433, 6.0, 163.9],
    [0.518, -0.25, 6.0, 155.9],
    [0.0, -0.0, 6.0, 153.4],
    [-0.518, 0.25, 6.0, 155.9],
    [-1.0, 0.433, 6.0, 163.9],
    [-1.414, 0.5, 6.0, -180.0],
    [-1.732, 0.433, 6.0, -153.4],
    [-1.932, 0.25, 6.0, -120.9],
    [-2.0, 0.0, 6.0, -90.0],
    [-1.932, -0.25, 6.0, -59.1],
    [-1.732, -0.433, 6.0, -26.6],
    [-1.414, -0.5, 6.0, -0.0],
    [-1.0, -0.433, 6.0, 16.1],
    [-0.518, -0.25, 6.0, 24.1],
    [-0.0, -0.0, 6.0, 26.6],

], dtype=float)

# Square, yaw while moving
# waypoints_user = np.array([
#     [0, 0, 0, 0],
#     [0, 0, 1, 0],
#     [0, 1, 1, 0],
#     [1, 1, 1, 0],
#     [1, 0, 1, 0],
#     [0, 0, 1, 0],
#     [0, 0, 2, 0],
# ], dtype=float)

# Convert yaw from degrees to radians
waypoints_user_rad = waypoints_user.copy()
waypoints_user_rad[:, 3] = np.deg2rad(waypoints_user[:, 3])

# Time for each segment (seconds)
time_per_segment = 5.0
n_segments = waypoints_user_rad.shape[0] - 1
segment_times = time_per_segment * np.ones(n_segments)
total_trajectory_time = np.sum(segment_times)

# Convert to NED convention (z points DOWN in NED, yaw stays same)
waypoints_ned = waypoints_user_rad.copy()
waypoints_ned[:, 2] = -waypoints_user_rad[:, 2]

# Sim params
t_start = 0.0
t_end = total_trajectory_time + 2.0   # trajectory + 2 seconds
dt = 0.01
time_vector = np.arange(t_start, t_end + dt/2, dt)

# Initial conditions
state0 = np.zeros(12)   # everything starts at zero

integral_state = {
    'pos_int': np.zeros(3),     # position integral [x, y, z]
    'omega_int': np.zeros(3),   # angular velocity integral [p, q, r]
    'last_time': -1.0,          # -1 triggers zero dt on first call
}

# Storage for plotting
n_steps = len(time_vector)
state_history = np.zeros((12, n_steps))
rotor_history = np.zeros((4, n_steps))
est_history = np.zeros((12, n_steps))   # sensor/estimator output, same layout as state
ref_history = {key: np.zeros(n_steps) for key in ('x', 'y', 'z', 'xd', 'yd', 'zd', 'psi')}
state_history[:, 0] = state0

Kp_comp = 0.20
bias = np.random.normal(0, 0.005, size=3)
calib_time = 2.0
N_calib = int(calib_time/dt)
gyro_bias_est = np.mean([simulate_gyro(np.zeros(3), 0.01, bias) for _ in range(N_calib)], axis=0)

P0=np.eye(6)  # Initial estimate covariance
# Process noise: discrete white-noise-acceleration model for the constant-velocity
# filter. Unmodeled acceleration enters velocity as a*dt and position as a*dt^2/2,
# so pos/vel rows get dt^4/4 and dt^2 (with dt^3/2 cross terms) — NOT equal weights.
# sigma_a ~ the largest acceleration the quad can command: g*tan(30 deg) ~ 6 m/s^2.
sigma_a = 6.0
Q = sigma_a**2 * np.kron(np.array([[dt**4/4, dt**3/2],
                                   [dt**3/2, dt**2  ]]), np.eye(3))
R=0.1**2*np.eye(3)  # Measurement noise covariance (GPS noise_std = 1 m)
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

x0 = np.concatenate([state0[9:], state0[6:9]])
kf = KalmanFilter_multiD(x0, P0, Q, R, F, H)

q_est = euler_to_quat(state0[5], state0[4], state0[3])

# %% Main Sim

print('Starting quadcopter simulation...')
print(f'  Trajectory: {waypoints_user.shape[0]} waypoints')
print(f'  Total trajectory time: {total_trajectory_time:.1f} s')
print(f'  Total simulation time: {t_end:.1f} s')
print(f'  Time step: {dt:.4f} s')
print(f'  Total steps: {n_steps}\n')

state = state0.copy()
rotor_speeds =  np.zeros(4)

for k in range(1, n_steps):
    t = time_vector[k]

    # Guidance - multi-waypoint
    ref = guidance_multi_waypoint(t, waypoints_ned, segment_times)
    for key in ref_history:
        ref_history[key][k] = ref[key]

    sensor_state, q_est = quad_sensors(dt, state, rotor_speeds, params, kf, q_est, bias, gyro_bias_est, Kp_comp)
    est_history[:, k] = sensor_state

    # Controller
    rotor_speeds, integral_state = quad_controller_pid(t, sensor_state, ref, gains, params, integral_state)

    # Dynamics for each timestep (RK4 in place of ode45)
    state = rk4_step(lambda tau, s: quad_dynamics(tau, s, rotor_speeds, params),
                     t - dt, state, dt)

    state_history[:, k] = state
    rotor_history[:, k] = rotor_speeds

    if k % 100 == 0:
        print(f'  Progress: {100*k/n_steps:.1f}% (t = {t:.2f} s)')

print('\nSim Complete\n')

# %% PLOTTING AND SUMMARY FOR DATA ANALYSIS

p, q, r = state_history[0], state_history[1], state_history[2]
phi, theta, psi = state_history[3], state_history[4], state_history[5]
u, v, w = state_history[6], state_history[7], state_history[8]
x_pos, y_pos, z_pos = state_history[9], state_history[10], state_history[11]

rotor1, rotor2, rotor3, rotor4 = rotor_history

# Errors
error_x = ref_history['x'] - x_pos
error_y = ref_history['y'] - y_pos
error_z = ref_history['z'] - z_pos

# Total thrust
total_thrust = params['cT'] * (rotor1**2 + rotor2**2 + rotor3**2 + rotor4**2)

# Sim summary
print('=== SIMULATION SUMMARY ===\n')

print('Mission Command (user frame, z=altitude UP):')
print('Waypoints (x, y, z, yaw):')
for i, wp in enumerate(waypoints_user, start=1):
    print(f'  {i}: [{wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f}] m, yaw={wp[3]:.1f} deg')
print()

# Final position error (compared to last waypoint)
final_pos_user = np.array([x_pos[-1], y_pos[-1], -z_pos[-1]])
final_waypoint = waypoints_user[-1, 0:3]
final_error = final_waypoint - final_pos_user
final_error_norm = np.linalg.norm(final_error)

print('Final Position Error (vs last waypoint):')
print(f'  X error: {final_error[0]:.4f} m')
print(f'  Y error: {final_error[1]:.4f} m')
print(f'  Z error: {final_error[2]:.4f} m')
print(f'  Total:   {final_error_norm:.4f} m\n')

# Max errors
idx_max_x = np.argmax(np.abs(error_x))
idx_max_y = np.argmax(np.abs(error_y))
idx_max_z = np.argmax(np.abs(error_z))

print('Max Position Error:')
print(f'  X: {np.abs(error_x[idx_max_x]):.4f} m (t={time_vector[idx_max_x]:.2f} s)')
print(f'  Y: {np.abs(error_y[idx_max_y]):.4f} m (t={time_vector[idx_max_y]:.2f} s)')
print(f'  Z: {np.abs(error_z[idx_max_z]):.4f} m (t={time_vector[idx_max_z]:.2f} s)\n')

print('Max Attitude:')
print(f'  Roll:  {np.rad2deg(np.max(np.abs(phi))):.2f} deg')
print(f'  Pitch: {np.rad2deg(np.max(np.abs(theta))):.2f} deg')
print(f'  Yaw:   {np.rad2deg(np.max(np.abs(psi))):.2f} deg\n')

print('Max Body Velocities:')
print(f'  u: {np.max(np.abs(u)):.4f} m/s')
print(f'  v: {np.max(np.abs(v)):.4f} m/s')
print(f'  w: {np.max(np.abs(w)):.4f} m/s\n')

print('Rotor Utilization:')
print(f'  Max speed: {np.max(rotor_history):.0f} rad/s')
print('======================')


# --- FIGURE 1: 3D Trajectory ---
fig = plt.figure('3D Trajectory')
ax = fig.add_subplot(projection='3d')
ax.plot(x_pos, y_pos, -z_pos, 'b-', linewidth=1.5, label='Actual Path')

ax.plot([x_pos[0]], [y_pos[0]], [-z_pos[0]], 'go', markersize=10, markerfacecolor='g', label='Start')
ax.plot([x_pos[-1]], [y_pos[-1]], [-z_pos[-1]], 'r*', markersize=15, label='End')
ax.plot(ref_history['x'], ref_history['y'], -ref_history['z'], 'k--', linewidth=1, label='Reference')
ax.grid(True)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Altitude (m)')
ax.set_title('3D Trajectory')
ax.legend(loc='best')

# Scaling axes (equal aspect cube around trajectory)
max_range = max(np.ptp(x_pos), np.ptp(y_pos), np.ptp(-z_pos)) * 1.2
axis_limit = max(max_range, 0.5)   # ensure minimum scale
x_center = (np.max(x_pos) + np.min(x_pos)) / 2
y_center = (np.max(y_pos) + np.min(y_pos)) / 2
z_center = (np.max(-z_pos) + np.min(-z_pos)) / 2
ax.set_xlim(x_center - axis_limit, x_center + axis_limit)
ax.set_ylim(y_center - axis_limit, y_center + axis_limit)
ax.set_zlim(z_center - axis_limit, z_center + axis_limit)
ax.view_init(elev=30, azim=45)

# --- FIGURE 2: Position Tracking and Errors ---
fig, axs = plt.subplots(3, 2, num='Position Tracking', figsize=(11, 8))

axs[0, 0].plot(time_vector, x_pos, 'b-', linewidth=1.5, label='Actual X')
axs[0, 0].plot(time_vector, ref_history['x'], 'r--', linewidth=1.5, label='Ref X')
axs[0, 0].set_ylabel('X Position (m)')
axs[0, 0].set_title('X Position Tracking')
axs[0, 0].legend(loc='best')

axs[0, 1].plot(time_vector, error_x, 'r-', linewidth=1.5)
axs[0, 1].set_ylabel('X Error (m)')
axs[0, 1].set_title('X Position Error')

axs[1, 0].plot(time_vector, y_pos, 'b-', linewidth=1.5, label='Actual Y')
axs[1, 0].plot(time_vector, ref_history['y'], 'r--', linewidth=1.5, label='Ref Y')
axs[1, 0].set_ylabel('Y Position (m)')
axs[1, 0].set_title('Y Position Tracking')
axs[1, 0].legend(loc='best')

axs[1, 1].plot(time_vector, error_y, 'g-', linewidth=1.5)
axs[1, 1].set_ylabel('Y Error (m)')
axs[1, 1].set_title('Y Position Error')

axs[2, 0].plot(time_vector, -z_pos, 'b-', linewidth=1.5, label='Actual Altitude')
axs[2, 0].plot(time_vector, -ref_history['z'], 'r--', linewidth=1.5, label='Ref Altitude')
axs[2, 0].set_ylabel('Altitude (m)')
axs[2, 0].set_title('Z Position Tracking')
axs[2, 0].legend(loc='best')

axs[2, 1].plot(time_vector, error_z, 'b-', linewidth=1.5)
axs[2, 1].set_ylabel('Z Error (m)')
axs[2, 1].set_title('Z Position Error')

for ax_i in axs.flat:
    ax_i.grid(True)
    ax_i.set_xlabel('Time (s)')
fig.tight_layout()

# --- FIGURE 3: Body Velocities ---
plt.figure('Body Velocities')
plt.plot(time_vector, u, 'r-', linewidth=1.5, label='u (x-body)')
plt.plot(time_vector, v, 'g-', linewidth=1.5, label='v (y-body)')
plt.plot(time_vector, w, 'b-', linewidth=1.5, label='w (z-body)')
plt.grid(True)
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Body Velocities')
plt.legend(loc='best')

# --- FIGURE 4: Euler Angles ---
fig, axs = plt.subplots(2, 1, num='Euler Angles', figsize=(9, 7))

axs[0].plot(time_vector, np.rad2deg(phi), 'r-', linewidth=1.5, label='Roll (phi)')
axs[0].plot(time_vector, np.rad2deg(theta), 'g-', linewidth=1.5, label='Pitch (theta)')
axs[0].set_title('Roll and Pitch Angles')

axs[1].plot(time_vector, np.rad2deg(psi), 'b-', linewidth=1.5, label='Actual Yaw (psi)')
axs[1].plot(time_vector, np.rad2deg(ref_history['psi']), 'r--', linewidth=1.5, label='Reference Yaw')
axs[1].set_title('Yaw Angle Tracking')

for ax_i in axs:
    ax_i.grid(True)
    ax_i.set_xlabel('Time (s)')
    ax_i.set_ylabel('Angle (deg)')
    ax_i.legend(loc='best')
fig.tight_layout()

# --- FIGURE 5: Rotor Speeds and Total Thrust ---
fig, axs = plt.subplots(2, 1, num='Rotor Speeds and Thrust', figsize=(9, 7))

for i, rotor in enumerate([rotor1, rotor2, rotor3, rotor4], start=1):
    axs[0].plot(time_vector, rotor/1000, linewidth=1.5, label=f'Rotor {i}')
axs[0].set_ylabel('Speed (krad/s)')
axs[0].set_title('Rotor Speeds')

axs[1].plot(time_vector, total_thrust, 'k-', linewidth=1.5, label='Total Thrust')
axs[1].axhline(params['m'] * params['g'], color='r', linestyle='--', linewidth=1.5, label='Hover Thrust')
axs[1].set_ylabel('Thrust (N)')
axs[1].set_title('Total System Thrust')

for ax_i in axs:
    ax_i.grid(True)
    ax_i.set_xlabel('Time (s)')
    ax_i.legend(loc='best')
fig.tight_layout()

# --- FIGURE 6: Filter - truth vs estimate ---
# solid = truth, dashed = estimate
est_omega = est_history[0:3]
est_euler = est_history[3:6]
est_vel = est_history[6:9]
est_pos = est_history[9:12]

fig, axs = plt.subplots(3, 1, sharex=True, num='Filter: Truth vs Estimate', figsize=(9, 10))
colors = ['C0', 'C1', 'C2']

for j, lbl in enumerate(['x', 'y', 'z']):
    axs[0].plot(time_vector, state_history[9 + j], color=colors[j], label=f'{lbl} true')
    axs[0].plot(time_vector, est_pos[j], color=colors[j], linestyle='--', linewidth=1)
axs[0].set_ylabel('Position (m)')
axs[0].set_title('solid = truth, dashed = estimate')

for j, lbl in enumerate(['u', 'v', 'w']):
    axs[1].plot(time_vector, state_history[6 + j], color=colors[j], label=f'{lbl} true')
    axs[1].plot(time_vector, est_vel[j], color=colors[j], linestyle='--', linewidth=1)
axs[1].set_ylabel('Body velocity (m/s)')

for j, lbl in enumerate(['roll', 'pitch', 'yaw']):
    axs[2].plot(time_vector, np.rad2deg(state_history[3 + j]), color=colors[j], label=f'{lbl} true')
    axs[2].plot(time_vector, np.rad2deg(est_euler[j]), color=colors[j], linestyle='--', linewidth=1)
axs[2].set_ylabel('Attitude (deg)')
axs[2].set_xlabel('Time (s)')

for ax_i in axs:
    ax_i.grid(True)
    ax_i.legend(loc='best', fontsize=8)
fig.tight_layout()

# --- FIGURE 7: Filter - estimation errors ---
pos_est_err = np.linalg.norm(est_pos - state_history[9:12], axis=0)
vel_est_err = np.linalg.norm(est_vel - state_history[6:9], axis=0)
# per-axis euler estimate error, wrapped to [-180, 180) so yaw crossings don't spike
euler_est_err = (np.rad2deg(est_euler - state_history[3:6]) + 180) % 360 - 180


def rms(x):
    return np.sqrt(np.mean(x**2))


def err_label(name, x, unit):
    return f'{name} (RMS {rms(x):.3f}, max {np.max(np.abs(x)):.3f} {unit})'


fig, axs = plt.subplots(3, 1, sharex=True, num='Filter: Estimation Errors', figsize=(9, 10))

axs[0].plot(time_vector, pos_est_err, color='C0', label=err_label('KF position', pos_est_err, 'm'))
axs[0].set_ylabel('Position est. error (m)')

axs[1].plot(time_vector, vel_est_err, color='C0', label=err_label('KF velocity', vel_est_err, 'm/s'))
axs[1].set_ylabel('Velocity est. error (m/s)')

for j, lbl in enumerate(['roll', 'pitch', 'yaw']):
    axs[2].plot(time_vector, euler_est_err[j], color=colors[j], linewidth=0.9,
                label=err_label(lbl, euler_est_err[j], 'deg'))
axs[2].set_ylabel('Attitude est. error (deg)')
axs[2].set_xlabel('Time (s)')

for ax_i in axs:
    ax_i.grid(True)
    ax_i.legend(loc='best', fontsize=8)
fig.tight_layout()

# Console summary of estimator performance
print(f'\n{"filter error signal":<28}{"RMS":>10}{"max":>10}{"final":>10}')
for name, x in [('KF pos est err (m)', pos_est_err),
                ('KF vel est err (m/s)', vel_est_err),
                ('roll est err (deg)', euler_est_err[0]),
                ('pitch est err (deg)', euler_est_err[1]),
                ('yaw est err (deg)', euler_est_err[2])]:
    print(f'{name:<28}{rms(x):>10.3f}{np.max(np.abs(x)):>10.3f}{x[-1]:>10.3f}')
print(f'{"true gyro bias (rad/s)":<28}  [{bias[0]:.4f} {bias[1]:.4f} {bias[2]:.4f}]')
print(f'{"calib residual (rad/s)":<28}  [{gyro_bias_est[0]-bias[0]:.4f} {gyro_bias_est[1]-bias[1]:.4f} {gyro_bias_est[2]-bias[2]:.4f}]')

plt.show()
