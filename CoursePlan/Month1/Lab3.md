                        Code Architecture: Building a basic 6 DOF Quadrotor Simulation

State Representation: state - 13 Dimension Vector that includes body position, quaternion state, 
body frane linear and angular velocities [x, y, z, qw, qx, qy, qz, u, v, w, p, q, r] 

x,y,z (NED Frame)
qw,qx,qy,qz (Quaternion State)
u,v,w (Body Frame)
p,q,r (Euler Rates)

Dynamics: Overall goal of the primary dynamcis function is to return the derivative of the state vector. 

V_dot Calc (Acceleration)
F = m*a
F_body = m * (a + omega x V) -> (Linear and Angular Acceleration)
F_body = m * (v_dot + omega x V) -> (v_dot = d/dt[u,v,w]  omega = [p,q,r] V = [u,v,w])

Omega Dot Calc (Angular Acceleration)
M = I*omega_dot
M_body = I * omega_dot + omega x (I * omega) - (Angular Acceleration + Gyroscopic Coupling)
    I = diag([Ixx, Iyy, Izz]), omega = [p, q, r], omega_dot = d/dt[p, q, r]

Quaternion Dynamics: 
q_dot = 0.5 * quat_multiply(q, omega_pure) Omega Pure [0, p, q, r]

x,y,z dot:
Rotate body frame velocities into inertial frame (NED) using the quaternion rotation method


General Quaternion Rotation for Body to Earth - vector_earth = q * vector_body * q*
General Quaternion Rotation for Earth to Body - vector_body = q* * vector_earth * q


Gravity in Body Frame: Gravity is always acting downward however the body frame is constantly changing.
In order to determine the direction of gravity in the body frame, the rotation matrix would have to be used

We would utilize the second equation, rotate the gravity [0, 0, 0, g] (Pure Quat) 

Rotor Thursts: Each rotor produces thrust based on the rotational speed of the rotor and
the thrust coeficient of the blade 
T_i = cT * (omega_i)**2  (Coef of Thrust * Rotor Rotational Speed Squared)

Reaction Torque: When there is an imbalance of thursts from diagonal rotors, there is a reaction torque leading to Yaw.
This is how Yaw is controlled for the quadrotor. 

tau = cM * (omega_i)**2  (Rotational Torque Coef * Rotor Rotational Speed Squared)

Rotor Mixer: Matrix that is based off the geometry of the quadrotor to determine the moments about the CG
created by each rotor. Imbalance of Front/Back or R/L rotors leads to pitch and roll, respectively. 

(+ configuration, arm length L, signs depend on layout)
       1
    4  CG  2
       3  

M = [ F_z  ]     [  1    1    1    1  ] [T1]  (Simple Thurst in Vertical Direction = To all Rotor Thursts)
    [ M_x  ]  =  [  0   -L    0    L  ] [T2]  (Moment about X axis = Pos when Rotor 4 greater than 2) 
    [ M_y  ]     [  L    0   -L    0  ] [T3]  (Moment about y axis = Pos when Rotor 3 greater than 1)
    [ M_z  ]     [ -k    k   -k    k  ] [T4]  (Moment about z axis = Pos when Rotors 2/4 greater than 1/3)

k = cM/cT (Moment/Thrust Ratio for Yaw moment)
L = Distance from CG to Rotor

Inverse of Matrix M provides the desired thrusts for rotors 1,2,3,4 depending on desired F_z, M_x, M_y, M_z
into the body frame based on the current quaternion. This would be used for the dynamic calculations

We would utilize the rk4 step function to integrate the state vector over time to determine state for every timestep

We could use constant velocity and constant acceleration for now before implementing waypoints and a controller

From the desired velocity and accelerations in the body frame we will calculate the desired forces and moments
ahout each axis, leading to the desired thrusts from each rotor from the mixer. 

