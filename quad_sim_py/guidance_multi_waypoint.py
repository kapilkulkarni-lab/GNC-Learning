import numpy as np


def guidance_multi_waypoint(t, waypoints, segment_times):
    """Reference trajectory through multiple waypoints using cubic splines
    (port of guidanceMultiWaypoint.m). Yaw is specified at each waypoint and
    smoothly transitioned.

    Inputs:
        t             - current time
        waypoints     - (N, 4) array, each row [x, y, z, psi] in NED (psi in rad)
        segment_times - (N-1,) array of time durations for each segment

    Returns:
        ref - dict with desired position, velocity, acceleration, and yaw
    """
    n_waypoints = waypoints.shape[0]
    n_segments = n_waypoints - 1

    # Cumulative time at each waypoint
    t_cumulative = np.concatenate([[0.0], np.cumsum(segment_times)])

    if t <= 0:
        # Before trajectory starts - use first waypoint
        pos_des = waypoints[0, 0:3].copy()
        vel_des = np.zeros(3)
        accel_des = np.zeros(3)
        psi_des = waypoints[0, 3]
        psid_des = 0.0

    elif t >= t_cumulative[-1]:
        # After trajectory complete - hold final waypoint position and yaw
        pos_des = waypoints[-1, 0:3].copy()
        vel_des = np.zeros(3)
        accel_des = np.zeros(3)
        psi_des = waypoints[-1, 3]
        psid_des = 0.0

    else:
        # Find current segment
        segment_idx = 0
        for i in range(n_segments):
            if t > t_cumulative[i + 1]:
                segment_idx = i + 1
            else:
                break

        pos_start = waypoints[segment_idx, 0:3]
        pos_end = waypoints[segment_idx + 1, 0:3]
        t_segment_start = t_cumulative[segment_idx]
        t_segment_duration = segment_times[segment_idx]

        # Yaw endpoints for this segment
        psi_start = waypoints[segment_idx, 3]
        psi_end = waypoints[segment_idx + 1, 3]

        # Handle yaw wraparound (choose shortest rotation path)
        delta_psi = psi_end - psi_start
        if delta_psi > np.pi:
            delta_psi -= 2*np.pi
        elif delta_psi < -np.pi:
            delta_psi += 2*np.pi

        # Time within current segment, normalized to [0, 1]
        tau = (t - t_segment_start) / t_segment_duration

        # Cubic spline blending function and derivatives
        s = 3*tau**2 - 2*tau**3        # position
        s_dot = 6*tau - 6*tau**2       # velocity (d/dtau)
        s_ddot = 6 - 12*tau            # acceleration (d^2/dtau^2)

        delta_pos = pos_end - pos_start

        pos_des = pos_start + s * delta_pos
        vel_des = (s_dot / t_segment_duration) * delta_pos
        accel_des = (s_ddot / t_segment_duration**2) * delta_pos

        psi_des = psi_start + s * delta_psi
        psid_des = (s_dot / t_segment_duration) * delta_psi

    return {
        'x': pos_des[0], 'y': pos_des[1], 'z': pos_des[2],
        'xd': vel_des[0], 'yd': vel_des[1], 'zd': vel_des[2],
        'xdd': accel_des[0], 'ydd': accel_des[1], 'zdd': accel_des[2],
        'psi': psi_des, 'psid': psid_des,
    }
