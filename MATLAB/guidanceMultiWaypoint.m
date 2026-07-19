function ref = guidanceMultiWaypoint(t, waypoints, segment_times)
% Creates reference trajectory through multiple waypoints using cubic splines.
% Yaw is specified at each waypoint and smoothly transitioned.
% 
% Inputs:
%   t             - current time
%   waypoints     - Nx4 matrix where each row is [x, y, z, psi] waypoint in NED
%                   psi is yaw angle in radians
%   segment_times - (N-1)x1 vector of time durations for each segment
%
% Output:
%   ref - structure with desired position, velocity, acceleration, and yaw

n_waypoints = size(waypoints, 1);
n_segments = n_waypoints - 1;

% Build cumulative time array
t_cumulative = zeros(n_waypoints, 1);
for i = 2:n_waypoints
    t_cumulative(i) = t_cumulative(i-1) + segment_times(i-1);
end

% Determine which segment we're in
if t <= 0
    % Before trajectory starts - use first waypoint
    pos_des = waypoints(1, 1:3)';
    vel_des = [0; 0; 0];
    accel_des = [0; 0; 0];
    psi_des = waypoints(1, 4);
    psid_des = 0;
    
elseif t >= t_cumulative(end)
    % After trajectory complete - hold final waypoint position and yaw
    pos_des = waypoints(end, 1:3)';
    vel_des = [0; 0; 0];
    accel_des = [0; 0; 0];
    psi_des = waypoints(end, 4);
    psid_des = 0;
    
else
    % Find current segment
    segment_idx = 1;
    for i = 1:n_segments
        if t > t_cumulative(i+1)
            segment_idx = i + 1;
        else
            break;
        end
    end
    
    % Get segment info
    pos_start = waypoints(segment_idx, 1:3)';
    pos_end = waypoints(segment_idx + 1, 1:3)';
    t_segment_start = t_cumulative(segment_idx);
    t_segment_duration = segment_times(segment_idx);
    
    % Get yaw info for this segment (from waypoints)
    psi_start = waypoints(segment_idx, 4);
    psi_end = waypoints(segment_idx + 1, 4);
    
    % Handle yaw wraparound (choose shortest rotation path)
    delta_psi = psi_end - psi_start;
    if delta_psi > pi
        delta_psi = delta_psi - 2*pi;
    elseif delta_psi < -pi
        delta_psi = delta_psi + 2*pi;
    end
    psi_end = psi_start + delta_psi;  % Adjusted end angle for shortest path
    
    % Time within current segment
    t_in_segment = t - t_segment_start;
    tau = t_in_segment / t_segment_duration;  % normalized [0, 1]
    
    % Cubic spline blending function and derivatives
    s = 3*tau^2 - 2*tau^3;           % Position
    s_dot = 6*tau - 6*tau^2;         % Velocity (d/dτ)
    s_ddot = 6 - 12*tau;             % Acceleration (d²/dτ²)
    
    % Displacement for this segment
    delta_pos = pos_end - pos_start;
    
    % Position
    pos_des = pos_start + s * delta_pos;
    
    % Velocity (chain rule: ds/dt = (ds/dτ)(dτ/dt))
    vel_des = (s_dot / t_segment_duration) * delta_pos;
    
    % Acceleration (chain rule: d²s/dt² = (d²s/dτ²)(dτ/dt)²)
    accel_des = (s_ddot / (t_segment_duration^2)) * delta_pos;
    
    % Yaw angle (smooth transition using same spline)
    psi_des = psi_start + s * delta_psi;
    
    % Yaw rate (derivative of yaw)
    psid_des = (s_dot / t_segment_duration) * delta_psi;
end

% Pack into reference structure
ref.x = pos_des(1);
ref.y = pos_des(2);
ref.z = pos_des(3);

ref.xd = vel_des(1);
ref.yd = vel_des(2);
ref.zd = vel_des(3);

ref.xdd = accel_des(1);
ref.ydd = accel_des(2);
ref.zdd = accel_des(3);

ref.psi = psi_des;
ref.psid = psid_des;

end