import numpy as np

'''Coordinate Transformations: Euler Angles, DCM, and Quaternions.
All the following functions are used to convert between Euler angles,
Direction Cosine Matrix (DCM), and quaternions.'''


def euler_to_DCM(yaw, pitch, roll):
    """
    Convert Euler angles (yaw, pitch, roll) to Direction Cosine Matrix (DCM).

    Parameters:
    yaw : float
        The yaw angle in radians.
    pitch : float
        The pitch angle in radians.
    roll : float
        The roll angle in radians.

    Returns:
    dcm : ndarray
        The 3x3 Direction Cosine Matrix.
    """
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cr = np.cos(roll)
    sr = np.sin(roll)

    R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                    [-sp, cp * sr, cp * cr]])

    return R


def dcm_to_euler(R):
    """
    Convert Direction Cosine Matrix (DCM) to Euler angles (yaw, pitch, roll).

    Parameters:
    R : ndarray
        The 3x3 Direction Cosine Matrix.

    Returns:
    yaw : float
        The yaw angle in radians.
    pitch : float
        The pitch angle in radians.
    roll : float
        The roll angle in radians.
    """
    pitch = -np.arcsin(R[2, 0])
    if np.abs(pitch) < np.pi / 2:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        yaw = 0.0

    return yaw, pitch, roll


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


def R_to_quat(R):
    """
    Convert a Direction Cosine Matrix (DCM) to a quaternion.

    Parameters:
    R : ndarray
        The 3x3 Direction Cosine Matrix.

    Returns:
    q : ndarray
        The quaternion [qx, qy, qz, qw] where qw is the scalar part.
    """
    if np.trace(R) > 0:
        S = np.sqrt(np.trace(R) + 1.0) * 2  # S = 4*qw
        qw = S / 4
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2  # S = 4*qx
        qw = (R[2,1] - R[1,2]) / S
        qx = S / 4
        qy = (R[0,1] + R[1,0]) / S
        qz = (R[0,2] + R[2,0]) / S
    elif R[1,1] > R[2,2]:
        S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2  # S = 4*qy
        qw = (R[0,2] - R[2,0]) / S
        qx = (R[0,1] + R[1,0]) / S
        qy = S / 4
        qz = (R[1,2] + R[2,1]) / S
    else:
        S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2  # S = 4*qz
        qw = (R[1,0] - R[0,1]) / S
        qx = (R[0,2] + R[2,0]) / S
        qy = (R[1,2] + R[2,1]) / S
        qz = S / 4

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


def quat_inverse(q):
    """Inverse of a unit quaternion: conjugate. [qx,qy,qz,qw] convention."""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_error(q, q_target):
    return quat_multiply(quat_inverse(q_target), q)
