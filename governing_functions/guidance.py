import numpy as np


def guidance_multi_waypoint(t, waypoints, segment_times):
    """
    Creates reference trajectory through multiple waypoints using cubic splines.
    Yaw is specified at each waypoint and smoothly transitioned.

    Parameters:
    t : float
        Current time.
    waypoints : array_like
        Nx4 array where each row is [x, y, z, psi] waypoint in NED.
        psi is yaw angle in radians.
    segment_times : array_like
        (N-1)-length array of time durations for each segment.

    Returns:
    ref : dict
        Desired position, velocity, acceleration, and yaw:
        x, y, z, xd, yd, zd, xdd, ydd, zdd, psi, psid.
    """
    waypoints = np.asarray(waypoints, dtype=float)
    segment_times = np.asarray(segment_times, dtype=float)

    n_waypoints = waypoints.shape[0]
    n_segments = n_waypoints - 1

    # Build cumulative time array
    t_cumulative = np.concatenate(([0.0], np.cumsum(segment_times)))

    if t <= 0:
        # Before trajectory starts - use first waypoint
        pos_des = waypoints[0, 0:3]
        vel_des = np.zeros(3)
        accel_des = np.zeros(3)
        psi_des = waypoints[0, 3]
        psid_des = 0.0

    elif t >= t_cumulative[-1]:
        # After trajectory complete - hold final waypoint position and yaw
        pos_des = waypoints[-1, 0:3]
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

        # Get segment info
        pos_start = waypoints[segment_idx, 0:3]
        pos_end = waypoints[segment_idx + 1, 0:3]
        t_segment_start = t_cumulative[segment_idx]
        t_segment_duration = segment_times[segment_idx]

        # Get yaw info for this segment (from waypoints)
        psi_start = waypoints[segment_idx, 3]
        psi_end = waypoints[segment_idx + 1, 3]

        # Handle yaw wraparound (choose shortest rotation path)
        delta_psi = psi_end - psi_start
        if delta_psi > np.pi:
            delta_psi -= 2 * np.pi
        elif delta_psi < -np.pi:
            delta_psi += 2 * np.pi
        psi_end = psi_start + delta_psi  # Adjusted end angle for shortest path

        # Time within current segment
        t_in_segment = t - t_segment_start
        tau = t_in_segment / t_segment_duration  # normalized [0, 1]

        # Cubic spline blending function and derivatives
        s = 3 * tau**2 - 2 * tau**3      # Position
        s_dot = 6 * tau - 6 * tau**2      # Velocity (d/dtau)
        s_ddot = 6 - 12 * tau              # Acceleration (d^2/dtau^2)

        # Displacement for this segment
        delta_pos = pos_end - pos_start

        # Position
        pos_des = pos_start + s * delta_pos

        # Velocity (chain rule: ds/dt = (ds/dtau)(dtau/dt))
        vel_des = (s_dot / t_segment_duration) * delta_pos

        # Acceleration (chain rule: d^2s/dt^2 = (d^2s/dtau^2)(dtau/dt)^2)
        accel_des = (s_ddot / (t_segment_duration**2)) * delta_pos

        # Yaw angle (smooth transition using same spline)
        psi_des = psi_start + s * delta_psi

        # Yaw rate (derivative of yaw)
        psid_des = (s_dot / t_segment_duration) * delta_psi

    return {
        'x': pos_des[0], 'y': pos_des[1], 'z': pos_des[2],
        'xd': vel_des[0], 'yd': vel_des[1], 'zd': vel_des[2],
        'xdd': accel_des[0], 'ydd': accel_des[1], 'zdd': accel_des[2],
        'psi': psi_des, 'psid': psid_des,
    }
