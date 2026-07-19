import numpy as np

from .attitude_math import quat_to_R, quat_multiply


def spring_mass_system(t, state, u, m, c, k):
    """
    Computes the derivatives for a spring-mass system.

    Parameters:
    t : float
        The current time.
    state : array_like
        The current state vector [position, velocity].
    u : array_like
        The control input vector (force applied to the mass).

    Returns:
    derivatives : array_like
        The derivatives [velocity, acceleration].
    """
    position, velocity = state

    # Compute acceleration using Hooke's law and Newton's second law
    acceleration = (-k * position + u[0] - c * velocity) / m

    return np.array([velocity, acceleration])


def pendulum_dynamics(t, state, u, m, l, b, g):
    """
    Computes the derivatives for a simple pendulum system.

    Parameters:
    t : float
        The current time.
    state : array_like
        The current state vector [angle, angular_velocity].
    u : array_like
        The control input vector (torque applied to the pendulum).
    m : float
        Mass of the pendulum bob.
    l : float
        Length of the pendulum rod.
    b : float
        Damping coefficient.
    g : float
        Acceleration due to gravity.

    Returns:
    derivatives : array_like
        The derivatives [angular_velocity, angular_acceleration].
    """
    angle, angular_velocity = state

    # Compute angular acceleration using the equation of motion for a damped pendulum
    angular_acceleration = (-b * angular_velocity - m * g * l * np.sin(angle) + u[0]) / (m * l**2)

    state_dot = np.array([angular_velocity, angular_acceleration])
    return state_dot


def kinematics_3D(t, accel, pos0, vel0):
    """
    Compute the position and velocity in 3D space given acceleration, initial position, and initial velocity.

    Parameters:
    t : float
        The current time.
    accel : array_like
        The acceleration vector (3D).
    pos0 : array_like
        The initial position vector (3D).
    vel0 : array_like
        The initial velocity vector (3D).

    Returns:
    pos : ndarray
        The position vector at time t (3D).
    vel : ndarray
        The velocity vector at time t (3D).
    """
    pos = pos0 + vel0 * t + 0.5 * accel * t**2  # Position update using kinematic equation
    vel = vel0 + accel * t  # Velocity update using kinematic equation
    return pos, vel


def quat_attitude_dynamics(t, state, u, I):
    """
    Rigid body attitude dynamics: quaternion + angular velocity.

    state : [qx, qy, qz, qw, wx, wy, wz]   (7-element)
    u     : [taux, tauy, tauz]              applied torque
    I     : 3x3 inertia tensor
    """
    q = state[0:4]
    w = state[4:7]

    # Quaternion kinematics: q_dot = 0.5 * q ⊗ [w, 0]  (pure quaternion, scalar last convention)
    w_quat = np.array([w[0], w[1], w[2], 0.0])
    q_dot = 0.5 * quat_multiply(q, w_quat)

    # Euler's rotational equation: w_dot = I^-1 @ (tau - w × (I@w))
    tau = np.array(u)
    w_dot = np.linalg.inv(I) @ (tau - np.cross(w, I @ w))

    return np.concatenate([q_dot, w_dot])


def rotor_mixer(rotor_speeds, cT, cM, d):
    """
    Converts 4 rotor speeds into total thrust + body-frame torque.

    rotor_speeds : [omega1, omega2, omega3, omega4]  (rad/s)
    cT : thrust coefficient (T = cT * omega^2)
    cM : moment/drag coefficient
    d  : arm length (m)

    Returns:
    F   : total thrust magnitude (N), along body +z
    tau : [tau_x, tau_y, tau_z] body-frame torque (roll, pitch, yaw)
    """
    thrusts = cT * np.array(rotor_speeds) ** 2
    F = np.sum(thrusts)
    tau_x = d * (thrusts[1] - thrusts[3])                                    # roll
    tau_y = d * (thrusts[2] - thrusts[0])                                    # pitch
    tau_z = (cM / cT) * (thrusts[0] - thrusts[1] + thrusts[2] - thrusts[3])  # yaw
    return F, np.array([tau_x, tau_y, tau_z])


def quad_dynamics(t, state, rotor_speeds, params):
    pos = state[0:3]
    vel = state[3:6]
    q = state[6:10]
    omega = state[10:13]

    m, g, I = params['m'], params['g'], params['I']
    F, tau = rotor_mixer(rotor_speeds, params['cT'], params['cM'], params['d'])

    # Translational dynamics: rotate body thrust into inertial frame, add gravity
    R = quat_to_R(q)
    F_body = np.array([0.0, 0.0, F])
    accel = (R @ F_body + np.array([0.0, 0.0, -m * g])) / m

    # Rotational dynamics: reuse the already-validated attitude dynamics
    attitude_state = np.concatenate([q, omega])
    attitude_dot = quat_attitude_dynamics(t, attitude_state, tau, I)

    return np.concatenate([vel, accel, attitude_dot[0:4], attitude_dot[4:7]])
