import numpy as np

from .attitude_math import quat_to_R, quat_multiply


def simulate_gyro(true_omega, noise_std=0.01, bias=None):
    """
    Simulate a noisy (optionally biased) gyro measurement of true angular velocity.

    Parameters:
    true_omega : array_like
        True body-frame angular velocity (3,).
    noise_std : float
        Standard deviation of the additive Gaussian noise (rad/s).
    bias : array_like, optional
        Constant gyro bias (3,). Defaults to zero (unbiased gyro).
    """
    if bias is None:
        bias = np.zeros(3)
    return true_omega + bias + np.random.normal(0, noise_std, size=3)

def simulate_accel(thrust, m, noise_std=0.5):
    """
    Simulate a noisy accelerometer on a quadrotor: specific force from thrust only.

    Thrust is assumed to act purely along body z with no aero drag, so the true
    specific force is [0, 0, thrust/m] in the body frame regardless of attitude.

    Parameters:
    thrust : float
        Total thrust actually applied by the rotors (N).
    m : float
        Vehicle mass (kg).
    noise_std : float
        Standard deviation of the additive Gaussian noise (m/s^2).
    """
    a_true_body = np.array([0.0, 0.0, thrust/m])
    a_noisy = a_true_body + np.random.normal(0, noise_std, size=3)
    return a_noisy


def simulate_accel_gravity(q_true, noise_std=0.02):
    """
    Simulate an accelerometer under the quasi-static (AHRS) assumption: the
    measurement is the gravity direction in the body frame, as a unit vector.

    Matches ahrs_measurement_model, which predicts quat_to_R(q).T @ [0, 0, 1].

    Parameters:
    q_true : array_like
        True attitude quaternion [x, y, z, w].
    noise_std : float
        Standard deviation of the additive Gaussian noise (unit-vector scale).
    """
    a_true_body = quat_to_R(q_true).T @ np.array([0.0, 0.0, 1.0])
    return a_true_body + np.random.normal(0, noise_std, size=3)

def simulate_gps(true_pos, noise_std=1, bias = None):
    """
    Simulate a GPS providing current position data 
    
    Parameters:
    true_pos : array
        True position [x, y, z}
    noise_std
        ST Dev of the Gaussian noise, 1 m if not defined
    bias
        Bias added to the GPS data"""
    
    if bias is None:
        bias = np.zeros(3)
    return true_pos + bias + np.random.normal(0, noise_std, size=3)

def Kp_comp_eff(F_cmd, m, g, Kp, sigma = 0.162):
    ratio = F_cmd/(m*g)
    accel_trust = np.exp(-((ratio-1.0)/sigma)**2)

    return Kp * accel_trust

def complementary_filter_step(q_est, omega_meas, accel_meas, dt, Kp_comp):
    """
    One step of the complementary filter: gyro integration corrected toward
    the accelerometer's implied gravity direction.
    """
    a_predicted = quat_to_R(q_est).T @ np.array([0.0, 0.0, 1.0])
    accel_meas = accel_meas/ np.linalg.norm(accel_meas)
    error = np.cross(accel_meas, a_predicted)
    omega_corrected = omega_meas + Kp_comp * error
    q_dot = 0.5 * quat_multiply(q_est, [omega_corrected[0], omega_corrected[1], omega_corrected[2], 0.0])
    q_est_new = q_est + q_dot * dt

    return q_est_new / np.linalg.norm(q_est_new)


def gyro_only_step(q_est, omega_meas, dt):
    """One step of pure gyro integration (no correction) — drifts over time."""
    q_dot = 0.5 * quat_multiply(q_est, [omega_meas[0], omega_meas[1], omega_meas[2], 0.0])
    q_est_new = q_est + q_dot * dt

    return q_est_new / np.linalg.norm(q_est_new)
