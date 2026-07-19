function [rotorSpeeds, integral_state] = quadControllerPID(t, state, ref, gains, params, integral_state)

%State vec
p = state(1);  q = state(2);  r = state(3);
phi = state(4);  theta = state(5);  psi = state(6);
u = state(7);  v = state(8);  w = state(9);
x = state(10);  y = state(11);  z = state(12);

% Wrap yaw angle to [-pi, pi]
psi = atan2(sin(psi), cos(psi));

%params
mass = params.m;  
gravity = params.g;


%Body to Inertial Rotation

Rbn = [cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
    sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
    -sin(theta),     cos(theta)*sin(phi),                   cos(theta)*cos(phi)];

vel_body = [u; v; w];
vel_inertial = Rbn * vel_body;


% Horizontal Position Control 

pos_error_xy = [ref.x - x; ref.y - y];
vel_error_xy = [ref.xd - vel_inertial(1); ref.yd - vel_inertial(2)];

% Computing Integral
if integral_state.last_time < 0
    dt = 0;
else
    dt = t - integral_state.last_time;
end
integral_state.pos_int(1:2) = integral_state.pos_int(1:2) + pos_error_xy * dt;  

accel_xy = gains.pos.Kp(1:2) .* pos_error_xy + gains.pos.Kd(1:2) .* vel_error_xy + gains.pos.Ki(1:2) .* integral_state.pos_int(1:2) + [ref.xdd; ref.ydd];

Apsi = [-cos(psi), -sin(psi);
        -sin(psi), cos(psi)];  %Transformation of Rpsi

attitude_cmd = (1/gravity) * Apsi * accel_xy;   %11.24 in Quan 
theta_des = attitude_cmd(1);
phi_des = attitude_cmd(2);

% Vertical Position Control

pos_error_z = ref.z - z;
vel_error_z = ref.zd - vel_inertial(3);

integral_state.pos_int(3) = integral_state.pos_int(3) + pos_error_z * dt;

accel_z = gains.pos.Kp(3) * pos_error_z + gains.pos.Kd(3) * vel_error_z + gains.pos.Ki(3) * integral_state.pos_int(3) + ref.zdd;

f_des = mass * (gravity - accel_z);

integral_state.last_time = t;


% Attitude Control

yaw_err = atan2(sin(ref.psi - psi), cos(ref.psi - psi));

euler_error = [phi_des - phi;
               theta_des - theta;
               yaw_err];

omega_des = gains.att.Kp .* euler_error;

omega_des(3) = omega_des(3) + ref.psid;  % Add feedforward yaw rate

% Current angular velocities
omega = [p; q; r];
omega_error = omega_des - omega;

% Torque command 
tau_des = params.J * (gains.att.Kp .* omega_error);

% Mixer
d = params.d;
cT = params.cT;
kya = params.cM / cT;

M = [1,    1,    1,    1;
     0,    d,    0,   -d;
     d,    0,   -d,    0;
     kya, -kya,  kya, -kya];

u_cmd = [f_des; tau_des(:)];
T_cmd = M \ u_cmd;

% Protect against negative thrust (ADDED SAFETY)
T_cmd = max(T_cmd, 0);

rotorSpeeds = sqrt(T_cmd / cT);

end