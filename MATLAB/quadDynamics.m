function [dstate] = quadDynamics(t, state, omega, params)

% States
p = state(1); q = state(2); r = state(3);
phi = state(4); theta = state(5); psi = state(6);
u = state(7); v = state(8); w = state(9);

% Parameters
m = params.m;
g = params.g;
J = params.J;
cT = params.cT;
cM = params.cM;
d = params.d;

% Rotor Thrusts
Thrusts = cT * (omega.^2); 

% Thrust acts upward along body -z axis
F_body = [0; 0; -sum(Thrusts)];

% Torques 
% 1=front(+x), 2=right(+y), 3=back(-x), 4=left(-y)
tau_phi = d * (Thrusts(2) - Thrusts(4));      % roll (x-body)
tau_theta = d * (Thrusts(1) - Thrusts(3));    % pitch (y-body)
tau_psi = (cM/cT) * (Thrusts(1) + Thrusts(3) - Thrusts(2) - Thrusts(4));  % yaw (z-body)
M_b = [tau_phi; tau_theta; tau_psi];

% Rotation matrices 
% Body to Inertial (NED) Rotation Matrix
Rbn = [cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
    sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
    -sin(theta),     cos(theta)*sin(phi),                   cos(theta)*cos(phi)];

Rnb = Rbn';

% Gravity
g_n = [0; 0; g];  % NED gravity (positive downward)
g_b = Rnb * g_n;  % gravity in body frame

% Body-frame velocities
vb = [u; v; w];
omega_b = [p; q; r];

% Acceleration in body frame
vbdot = (F_body / m) + g_b - cross(omega_b, vb);

omegadot = J \ (M_b - cross(omega_b, J * omega_b));

% Euler Angle ROC

W = [1,      sin(phi)*tan(theta),    cos(phi)*tan(theta);
    0,      cos(phi),        -sin(phi);
    0,      sin(phi)/cos(theta), cos(phi)/cos(theta)];

eulerdot = W * omega_b;


% Velocity in NED frame
rdot_n = Rbn * vb;  % [xdot; ydot; zdot]


dstate = [omegadot; eulerdot; vbdot; rdot_n];

end