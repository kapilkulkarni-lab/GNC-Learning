"""Local copy of the governing_functions used by the quad sim, so the
quad_sim_py folder is self-contained. Ported from governing_functions
(attitude_math.py, attitude_filters.py, estimators.py)."""
import numpy as np


# --- Attitude math (from governing_functions/attitude_math.py) ---

def euler_to_quat(yaw, pitch, roll):
    """
    Convert Euler angles (yaw, pitch, roll) to a quaternion.

    Uses the same ZYX rotation sequence as euler_to_DCM.

    Parameters:
    yaw : float
        The yaw angle in radians.
    pitch : float
        The pitch angle in radians.
    roll : float
        The roll angle in radians.

    Returns:
    q : ndarray
        The quaternion [qx, qy, qz, qw] where qw is the scalar part.
    """
    cy = np.cos(yaw / 2)
    sy = np.sin(yaw / 2)
    cp = np.cos(pitch / 2)
    sp = np.sin(pitch / 2)
    cr = np.cos(roll / 2)
    sr = np.sin(roll / 2)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = cy * sp * cr + sy * cp * sr
    qz = sy * cp * cr - cy * sp * sr

    return np.array([qx, qy, qz, qw])


def quat_to_euler(q):
    """
    Convert a quaternion to Euler angles (yaw, pitch, roll).

    Uses the same ZYX rotation sequence as euler_to_DCM.

    Parameters:
    q : array_like
        The quaternion [qx, qy, qz, qw] where qw is the scalar part.

    Returns:
    yaw : float
        The yaw angle in radians.
    pitch : float
        The pitch angle in radians.
    roll : float
        The roll angle in radians.
    """
    q = q/np.linalg.norm(q)  # Normalize the quaternion
    qx, qy, qz, qw = q

    yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
    pitch = np.arcsin(np.clip(2*(qw*qy - qx*qz), -1.0, 1.0))
    roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))

    return yaw, pitch, roll


def quat_to_R(q):
    """
    Convert a quaternion to a Direction Cosine Matrix (DCM).

    Parameters:
    q : array_like
        The quaternion [qx, qy, qz, qw] where qw is the scalar part.

    Returns:
    R : ndarray
        The 3x3 Direction Cosine Matrix.
    """
    q = q/np.linalg.norm(q)  # Normalize the quaternion
    qx, qy, qz, qw = q
    R = np.array([[1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
                  [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
                  [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]])
    return R


def quat_multiply(q1, q2):
    """Quaternion multiplication, [qx, qy, qz, qw] convention (scalar last)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])


# --- Sensor models and attitude filter (from governing_functions/attitude_filters.py) ---

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


def simulate_gps(true_pos, noise_std=1, bias=None):
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


def Kp_comp_eff(F_cmd, m, g, Kp, sigma=0.162):
    ratio = F_cmd/(m*g)
    accel_trust = np.exp(-((ratio-1.0)/sigma)**2)

    return Kp * accel_trust


def complementary_filter_step(q_est, omega_meas, accel_meas, dt, Kp_comp):
    """
    One step of the complementary filter: gyro integration corrected toward
    the accelerometer's implied gravity direction.
    """
    a_predicted = quat_to_R(q_est).T @ np.array([0.0, 0.0, 1.0])
    accel_meas = accel_meas / np.linalg.norm(accel_meas)
    error = np.cross(accel_meas, a_predicted)
    omega_corrected = omega_meas + Kp_comp * error
    q_dot = 0.5 * quat_multiply(q_est, [omega_corrected[0], omega_corrected[1], omega_corrected[2], 0.0])
    q_est_new = q_est + q_dot * dt

    return q_est_new / np.linalg.norm(q_est_new)


# --- Kalman filter (from governing_functions/estimators.py) ---

class KalmanFilter_multiD:
    def __init__(self, x0, P0, Q, R, F, H):
        """
        Initialize the Kalman Filter with initial state, covariance,
        process noise, and measurement noise.

        Parameters:
        x0 : array_like
            Initial state estimate.
        P0 : ndarray
            Initial estimate covariance.
        Q : ndarray
            Process noise covariance.
        R : ndarray
            Measurement noise covariance.
        F : ndarray
            State transition matrix.
        H : ndarray
            Measurement matrix.
        """
        self.x = np.array(x0)  # Initial state estimate
        self.P = np.array(P0)  # Initial estimate covariance
        self.Q = np.array(Q)   # Process noise covariance
        self.R = np.array(R)   # Measurement noise covariance
        self.F = np.array(F)   # State transition matrix
        self.H = np.array(H)   # Measurement matrix

    def predict(self):
        """
        Predict the next state and covariance based on the state transition model.
        """
        # State prediction
        self.x = self.F @ self.x  # Update the state estimate using the state transition model
        # Covariance prediction
        self.P = self.F @ self.P @ self.F.T + self.Q  # Update the estimate covariance with process noise

    def update(self, z):
        """
        Update the state estimate and covariance based on the measurement.

        Parameters:
        z : array_like
            The measurement vector.
        """
        z = np.array(z)

        y = z - self.H @ self.x  # Measurement residual
        S = self.H @ self.P @ self.H.T + self.R  # Residual covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain

        # State update
        self.x = self.x + K @ y  # Update the state estimate with the measurement

        # Covariance update
        I = np.eye(self.P.shape[0])  # Identity matrix of appropriate size
        self.P = (I - K @ self.H) @ self.P  # Update the estimate covariance

        return self.x  # Return the updated state estimate
