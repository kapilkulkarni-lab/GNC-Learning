import numpy as np


def quad_controller_pid(t, state, ref, gains, params, integral_state):
    """Cascaded PID position/attitude controller (port of quadControllerPID.m).

    Returns (rotor_speeds, integral_state); integral_state is mutated in place
    and returned for clarity.
    """
    p, q, r = state[0], state[1], state[2]
    phi, theta, psi = state[3], state[4], state[5]
    u, v, w = state[6], state[7], state[8]
    x, y, z = state[9], state[10], state[11]

    # Wrap yaw angle to [-pi, pi]
    psi = np.arctan2(np.sin(psi), np.cos(psi))

    mass = params['m']
    gravity = params['g']

    # Body to Inertial rotation
    Rbn = np.array([
        [np.cos(psi)*np.cos(theta), np.cos(psi)*np.sin(theta)*np.sin(phi) - np.sin(psi)*np.cos(phi), np.cos(psi)*np.sin(theta)*np.cos(phi) + np.sin(psi)*np.sin(phi)],
        [np.sin(psi)*np.cos(theta), np.sin(psi)*np.sin(theta)*np.sin(phi) + np.cos(psi)*np.cos(phi), np.sin(psi)*np.sin(theta)*np.cos(phi) - np.cos(psi)*np.sin(phi)],
        [-np.sin(theta),            np.cos(theta)*np.sin(phi),                                       np.cos(theta)*np.cos(phi)],
    ])

    vel_body = np.array([u, v, w])
    vel_inertial = Rbn @ vel_body

    # Horizontal position control
    pos_error_xy = np.array([ref['x'] - x, ref['y'] - y])
    vel_error_xy = np.array([ref['xd'] - vel_inertial[0], ref['yd'] - vel_inertial[1]])

    # Integral dt (zero on first call)
    if integral_state['last_time'] < 0:
        dt = 0.0
    else:
        dt = t - integral_state['last_time']
    integral_state['pos_int'][0:2] += pos_error_xy * dt

    kp, kd, ki = gains['pos']['Kp'], gains['pos']['Kd'], gains['pos']['Ki']
    accel_xy = (kp[0:2] * pos_error_xy + kd[0:2] * vel_error_xy
                + ki[0:2] * integral_state['pos_int'][0:2]
                + np.array([ref['xdd'], ref['ydd']]))

    Apsi = np.array([[-np.cos(psi), -np.sin(psi)],
                     [-np.sin(psi),  np.cos(psi)]])   # transformation of Rpsi

    attitude_cmd = (1.0 / gravity) * Apsi @ accel_xy   # 11.24 in Quan
    theta_des = attitude_cmd[0]
    phi_des = attitude_cmd[1]

    # Vertical position control
    pos_error_z = ref['z'] - z
    vel_error_z = ref['zd'] - vel_inertial[2]

    integral_state['pos_int'][2] += pos_error_z * dt

    accel_z = (kp[2] * pos_error_z + kd[2] * vel_error_z
               + ki[2] * integral_state['pos_int'][2] + ref['zdd'])

    f_des = mass * (gravity - accel_z)

    integral_state['last_time'] = t

    # Attitude control
    yaw_err = np.arctan2(np.sin(ref['psi'] - psi), np.cos(ref['psi'] - psi))

    euler_error = np.array([phi_des - phi,
                            theta_des - theta,
                            yaw_err])

    omega_des = gains['att']['Kp'] * euler_error
    omega_des[2] += ref['psid']   # feedforward yaw rate

    omega = np.array([p, q, r])
    omega_error = omega_des - omega

    # Torque command
 # Torque command
    tau_des = params['J'] @ (gains['att']['Kd'] * omega_error)

    # Mixer
    d = params['d']
    cT = params['cT']
    kya = params['cM'] / cT

    M = np.array([
        [1.0,  1.0,  1.0,  1.0],
        [0.0,  d,    0.0, -d  ],
        [d,    0.0, -d,    0.0],
        [kya, -kya,  kya, -kya],
    ])

    u_cmd = np.concatenate([[f_des], tau_des])
    T_cmd = np.linalg.solve(M, u_cmd)

    # Protect against negative thrust
    T_cmd = np.maximum(T_cmd, 0.0)

    rotor_speeds = np.sqrt(T_cmd / cT)

    return rotor_speeds, integral_state
