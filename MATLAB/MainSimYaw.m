clear all; close all; clc;

% Quad Sim Script - Multi-Waypoint Trajectory

%Parameters
params.m = 1.062;    % mass 
params.g = 9.81;     % gravity 
params.d = 0.17;     % arm length
Jxx = 1.07e-2;
Jyy = 1.11e-2;
Jzz = 2.29e-2;
params.J = diag([Jxx, Jyy, Jzz]);    % moment of inertia 
params.cT = 5.724165e-8;             % thrust coefficient
params.cM = 8.881631e-10;            % moment coefficient

% Position gains 
gains.pos.Kp = [0.4; 0.4; 5.0];         
gains.pos.Kd = [1.3; 1.3; 3.5];         
gains.pos.Ki = [0.05; 0.05; 1.0];         

% Attitude gains 
gains.att.Kp = [2.5; 2.5; 2.5];     %Only have proportional for now, need help figuring out the rest
gains.att.Kd = [0.1, 0.1, 0.1];

%Guidance Info - Multi-Waypoint Trajectory
% User frame waypoints (z = altitude UP, yaw in degrees)
% Format: [x, y, z, yaw_deg]

%Square Yaw while moving
% waypoints_user = [
%     0, 0, 0, 0;
%     0, 0, 1, 0;
%     0, 1, 1, 0;
%     1, 1, 1, 0;
%     1, 0, 1, 0;
%     0, 0, 1, 0;
%     0, 0, 2, 0
% ];

%Square Yaw at corners
waypoints_user = [
    0, 0, 0, 0;
    0, 0, 1, 0;
    0, 0, 1, 90;
    0, 1, 1, 90;
    0, 1, 1, 0;
    1, 1, 1, 0;
    1, 1, 1, -90;
    1, 0, 1, -90;
    1, 0, 1, -180;
    0, 0, 1, -180;
    0, 0, 2, 0
];

% Convert yaw from degrees to radians
waypoints_user_rad = waypoints_user;
waypoints_user_rad(:, 4) = deg2rad(waypoints_user(:, 4));

% Time for each segment (seconds)
time_per_segment = 5;  % Set desired time per segment here
n_segments = size(waypoints_user_rad, 1) - 1;  % Number of segments = waypoints - 1
segment_times = time_per_segment * ones(n_segments, 1);  % Create array automatically
total_trajectory_time = sum(segment_times);

% Convert to NED convention (z points DOWN in NED, yaw stays same)
waypoints_ned = waypoints_user_rad;
waypoints_ned(:, 3) = -waypoints_user_rad(:, 3);

%Sim Params
t_start = 0;        % start time 
t_end = total_trajectory_time + 2;  % end time (trajectory + 2 seconds)
dt = 0.01;          % time step 
time_vector = t_start:dt:t_end;   % time vector for plotting

%Initial conditions
state0 = zeros(12, 1); %Everything Starts at Zero

integral_state.pos_int = zeros(3, 1);      % position integral [x; y; z]
integral_state.omega_int = zeros(3, 1);    % angular velocity integral [p; q; r]
integral_state.last_time = -1;             % last time for dt calculation (use -1 to trigger zero dt on first call)

%Storage of Variables for Plotting
n_steps = length(time_vector);
state_history = zeros(12, n_steps);
rotor_history = zeros(4, n_steps);
ref_history = struct('x', zeros(1, n_steps), ...
                     'y', zeros(1, n_steps), ...
                     'z', zeros(1, n_steps), ...
                     'xd', zeros(1, n_steps), ...
                     'yd', zeros(1, n_steps), ...
                     'zd', zeros(1, n_steps), ...
                     'psi', zeros(1, n_steps));
state_history(:, 1) = state0;


%% Main Sim

fprintf('Starting quadcopter simulation...\n');
fprintf('  Trajectory: %d waypoints\n', size(waypoints_user, 1));
fprintf('  Total trajectory time: %.1f s\n', total_trajectory_time);
fprintf('  Total simulation time: %.1f s\n', t_end);
fprintf('  Time step: %.4f s\n', dt);
fprintf('  Total steps: %d\n\n', n_steps);

state = state0;
t = t_start;

for k = 2:n_steps
    t = time_vector(k);
    
    % Guidance - Multi-Waypoint
    ref = guidanceMultiWaypoint(t, waypoints_ned, segment_times);
    
    %Storing Reference Values for Plotting
    ref_history.x(k) = ref.x;
    ref_history.y(k) = ref.y;
    ref_history.z(k) = ref.z;
    ref_history.xd(k) = ref.xd;
    ref_history.yd(k) = ref.yd;
    ref_history.zd(k) = ref.zd;
    ref_history.psi(k) = ref.psi;
    
    % Controller
    [rotor_speeds, integral_state] = quadControllerPID(t, state, ref, gains, params, integral_state);
    
    
    % Dynamics for each timestep
    t_span = [t - dt, t];
    [~, state_int] = ode45(@(tau, s) quadDynamics(tau, s, rotor_speeds, params), t_span, state);
    state = state_int(end, :)';
    
    % Storing Data for Plots
    state_history(:, k) = state;
    rotor_history(:, k) = rotor_speeds;
    
    % Progress indicator
    if mod(k, 100) == 0
        fprintf('  Progress: %.1f%% (t = %.2f s)\n', 100*k/n_steps, t);
    end
end

fprintf('\nSim Complete\n\n');

%% PLOTTING AND SUMMARY FOR DATA ANALYSIS

%State History for Plots and Sim Summary
p = state_history(1, :);
q = state_history(2, :);
r = state_history(3, :);
phi = state_history(4, :);
theta = state_history(5, :);
psi = state_history(6, :);
u = state_history(7, :);
v = state_history(8, :);
w = state_history(9, :);
x_pos = state_history(10, :);
y_pos = state_history(11, :);
z_pos = state_history(12, :);

rotor1 = rotor_history(1, :);
rotor2 = rotor_history(2, :);
rotor3 = rotor_history(3, :);
rotor4 = rotor_history(4, :);

% Calculate errors
error_x = ref_history.x - x_pos;
error_y = ref_history.y - y_pos;
error_z = ref_history.z - z_pos;

%Total Thrust
total_thrust = params.cT * (rotor1.^2 + rotor2.^2 + rotor3.^2 + rotor4.^2);

%Sim Summary
fprintf('=== SIMULATION SUMMARY ===\n\n');

fprintf('Mission Command (user frame, z=altitude UP):\n');
fprintf('Waypoints (x, y, z, yaw):\n');
for i = 1:size(waypoints_user, 1)
    fprintf('  %d: [%.2f, %.2f, %.2f] m, yaw=%.1f deg\n', i, ...
            waypoints_user(i,1), waypoints_user(i,2), waypoints_user(i,3), waypoints_user(i,4));
end
fprintf('\n');

% Final position error (compared to last waypoint)
final_pos_user = [x_pos(end); y_pos(end); -z_pos(end)];
final_waypoint = waypoints_user(end, 1:3)';
final_error_x = final_waypoint(1) - final_pos_user(1);
final_error_y = final_waypoint(2) - final_pos_user(2);
final_error_z = final_waypoint(3) - final_pos_user(3);
final_error_norm = norm([final_error_x, final_error_y, final_error_z]);

fprintf('Final Position Error (vs last waypoint):\n');
fprintf('  X error: %.4f m\n', final_error_x);
fprintf('  Y error: %.4f m\n', final_error_y);
fprintf('  Z error: %.4f m\n', final_error_z);
fprintf('  Total:   %.4f m\n\n', final_error_norm);

% Max errors
[max_error_x, idx_max_x] = max(abs(error_x));
[max_error_y, idx_max_y] = max(abs(error_y));
[max_error_z, idx_max_z] = max(abs(error_z));

fprintf('Max Position Error:\n');
fprintf('  X: %.4f m (t=%.2f s)\n', max_error_x, time_vector(idx_max_x));
fprintf('  Y: %.4f m (t=%.2f s)\n', max_error_y, time_vector(idx_max_y));
fprintf('  Z: %.4f m (t=%.2f s)\n\n', max_error_z, time_vector(idx_max_z));

% Attitude excursions
max_phi = max(abs(phi));
max_theta = max(abs(theta));
max_psi = max(abs(psi));

fprintf('Max Attitude:\n');
fprintf('  Roll:  %.2f deg\n', rad2deg(max_phi));
fprintf('  Pitch: %.2f deg\n', rad2deg(max_theta));
fprintf('  Yaw:   %.2f deg\n\n', rad2deg(max_psi));

% Velocity metrics
max_vel_x = max(abs(u));
max_vel_y = max(abs(v));
max_vel_z = max(abs(w));

fprintf('Max Body Velocities:\n');
fprintf('  u: %.4f m/s\n', max_vel_x);
fprintf('  v: %.4f m/s\n', max_vel_y);
fprintf('  w: %.4f m/s\n\n', max_vel_z);

% Rotor utilization
max_rotor_speed = max(max([rotor1; rotor2; rotor3; rotor4]));

fprintf('Rotor Utilization:\n');
fprintf('  Max speed: %.0f rad/s\n', max_rotor_speed);

fprintf('======================\n');


% --- FIGURE 1: 3D Trajectory ---
figure('Name', '3D Trajectory', 'NumberTitle', 'off');
plot3(x_pos, y_pos, -z_pos, 'b-', 'LineWidth', 1.5);
hold on;

% Plot waypoints
for i = 1:size(waypoints_user, 1)
    plot3(waypoints_user(i,1), waypoints_user(i,2), waypoints_user(i,3), ...
          'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    text(waypoints_user(i,1), waypoints_user(i,2), waypoints_user(i,3), ...
         sprintf(' WP%d', i), 'FontSize', 9);
end

plot3(x_pos(1), y_pos(1), -z_pos(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(x_pos(end), y_pos(end), -z_pos(end), 'r*', 'MarkerSize', 15);
plot3(ref_history.x, ref_history.y, -ref_history.z, 'k--', 'LineWidth', 1);
grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude (m)');
title('3D Trajectory (Waypoints shown)');
legend('Actual Path', 'Waypoints', '', '', '', '', '', '', 'Start', 'End', 'Reference', 'Location', 'best');
axis equal;

% Scaling Axes
x_range = max(x_pos) - min(x_pos);
y_range = max(y_pos) - min(y_pos);
z_range = max(-z_pos) - min(-z_pos);
max_range = max([x_range, y_range, z_range]) * 1.2;

x_center = (max(x_pos) + min(x_pos)) / 2;
y_center = (max(y_pos) + min(y_pos)) / 2;
z_center = (max(-z_pos) + min(-z_pos)) / 2;
axis_limit = max(max_range, 0.5);  % Ensure minimum scale
xlim([x_center - axis_limit, x_center + axis_limit]);
ylim([y_center - axis_limit, y_center + axis_limit]);
zlim([z_center - axis_limit, z_center + axis_limit]);
view(45, 30);

% --- FIGURE 2: Position Tracking and Errors ---
figure('Name', 'Position Tracking', 'NumberTitle', 'off');

subplot(3, 2, 1);
plot(time_vector, x_pos, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual X');
hold on;
plot(time_vector, ref_history.x, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ref X');
grid on; xlabel('Time (s)'); ylabel('X Position (m)');
title('X Position Tracking');
legend('Location', 'best');

subplot(3, 2, 2);
plot(time_vector, error_x, 'r-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('X Error (m)');
title('X Position Error');

subplot(3, 2, 3);
plot(time_vector, y_pos, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Y');
hold on;
plot(time_vector, ref_history.y, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ref Y');
grid on; xlabel('Time (s)'); ylabel('Y Position (m)');
title('Y Position Tracking');
legend('Location', 'best');

subplot(3, 2, 4);
plot(time_vector, error_y, 'g-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Y Error (m)');
title('Y Position Error');

subplot(3, 2, 5);
plot(time_vector, -z_pos, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Altitude');
hold on;
plot(time_vector, -ref_history.z, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ref Altitude');
grid on; xlabel('Time (s)'); ylabel('Altitude (m)');
title('Z Position Tracking');
legend('Location', 'best');

subplot(3, 2, 6);
plot(time_vector, error_z, 'b-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Z Error (m)');
title('Z Position Error');

% --- FIGURE 3: Body Velocities ---
figure('Name', 'Body Velocities', 'NumberTitle', 'off');
plot(time_vector, u, 'r-', 'LineWidth', 1.5, 'DisplayName', 'u (x-body)');
hold on;
plot(time_vector, v, 'g-', 'LineWidth', 1.5, 'DisplayName', 'v (y-body)');
plot(time_vector, w, 'b-', 'LineWidth', 1.5, 'DisplayName', 'w (z-body)');
grid on; xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Body Velocities');
legend('Location', 'best');

% --- FIGURE 4: Euler Angles ---
figure('Name', 'Euler Angles', 'NumberTitle', 'off');

subplot(2, 1, 1);
plot(time_vector, rad2deg(phi), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Roll (φ)');
hold on;
plot(time_vector, rad2deg(theta), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Pitch (θ)');
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Roll and Pitch Angles');
legend('Location', 'best');

subplot(2, 1, 2);
plot(time_vector, rad2deg(psi), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Yaw (ψ)');
hold on;
plot(time_vector, rad2deg(ref_history.psi), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference Yaw');
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Yaw Angle Tracking');
legend('Location', 'best');

% --- FIGURE 5: Rotor Speeds and Total Thrust ---
figure('Name', 'Rotor Speeds and Thrust', 'NumberTitle', 'off');

subplot(2, 1, 1);
plot(time_vector, rotor1/1000, 'LineWidth', 1.5, 'DisplayName', 'Rotor 1');
hold on;
plot(time_vector, rotor2/1000, 'LineWidth', 1.5, 'DisplayName', 'Rotor 2');
plot(time_vector, rotor3/1000, 'LineWidth', 1.5, 'DisplayName', 'Rotor 3');
plot(time_vector, rotor4/1000, 'LineWidth', 1.5, 'DisplayName', 'Rotor 4');
grid on; xlabel('Time (s)'); ylabel('Speed (krpm)');
title('Rotor Speeds');
legend('Location', 'best');

subplot(2, 1, 2);
plot(time_vector, total_thrust, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Total Thrust');
hold on;
yline(params.m * params.g, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Hover Thrust');
grid on; xlabel('Time (s)'); ylabel('Thrust (N)');
title('Total System Thrust');
legend('Location', 'best');