import numpy as np

from .attitude_math import quat_to_R, quat_multiply


def ahrs_process_model(x, u, dt):
    """
    EKF process model for a quaternion + gyro-bias AHRS.

    x : [qx, qy, qz, qw, bx, by, bz]   (7 elements)
    u : raw gyro measurement (3 elements, bias NOT yet removed)
    """
    q = x[0:4]
    bias = x[4:7]

    omega_true = u - bias
    omega_quat = [omega_true[0], omega_true[1], omega_true[2], 0.0]
    q_dot = 0.5 * quat_multiply(q, omega_quat)
    q_next = q + q_dot * dt
    q_next = q_next / np.linalg.norm(q_next)

    bias_next = bias
    return np.concatenate([q_next, bias_next])


def ahrs_measurement_model(x):
    """EKF measurement model: predicted accelerometer reading (gravity direction in body frame)."""
    q = x[0:4]
    return quat_to_R(q).T @ np.array([0.0, 0.0, 1.0])
