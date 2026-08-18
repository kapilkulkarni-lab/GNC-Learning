import numpy as np
import matplotlib.pyplot as plt

def quat_multiply(q1, q2):
    """Quaternion multiplication, [qw, qx, qy, qz]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def gravity_body(q, params):
    '''Returns the gravity vector based on the current quaternion'''

    qw, qx, qy, qz = q

    g_vec = np.array([0, 0, 0, params['g']])
    q_star = np.array([qw, -qx, -qy, -qz])

    step_1 = quat_multiply(q_star, g_vec)

    g_body = quat_multiply(step_1, q)

    return g_body[1:]

def vel_NED(q, inertial_vel):

    ''' Finding Earth Frame Vel (x_dot, y_dot, z_dot)
    vector_earth = q * vector_body * q*'''

    qw, qx, qy, qz = q

    q_star = np.array([qw, -qx, -qy, -qz])

    step1 = quat_multiply(q, inertial_vel)
    vel_earth = quat_multiply(step1, q_star)

    return vel_earth[1:]


def motor_mixer(params):
    '''M = 
    [ F_z  ]     [  1    1    1    1  ] [T1]  (Simple Thurst in Vertical Direction = To all Rotor Thursts)
    [ M_x  ]  =  [  0   -L    0    L  ] [T2]  (Moment about X axis = Pos when Rotor 4 greater than 2) 
    [ M_y  ]     [  L    0   -L    0  ] [T3]  (Moment about y axis = Pos when Rotor 3 greater than 1)
    [ M_z  ]     [ -k    k   -k    k  ] [T4]  (Moment about z axis = Pos when Rotors 2/4 greater than 1/3) '''

    L = params['L']
    k = params['k']

    M = np.array([
        [1, 1, 1, 1], 
        [0, -L, 0, L], 
        [L, 0, -L, 0], 
        [-k, k, -k, k]
        ])

    return M

def trajectory_circle(t, params):
    '''Climbing circular trajectory: flat outputs x(t), y(t), z(t), psi(t)
    and their derivatives, all closed-form.'''
    R = params['radius']
    w = params['circle_omega']       # angular rate around the circle
    climb_rate = params['climb_rate']

    # position
    x = R * np.cos(w * t)
    y = R * np.sin(w * t)
    z = -climb_rate * t              # NED: more negative z = higher altitude

    # velocity
    x_dot = -R * w * np.sin(w * t)
    y_dot =  R * w * np.cos(w * t)
    z_dot = -climb_rate

    # acceleration
    x_ddot = -R * w**2 * np.cos(w * t)
    y_ddot = -R * w**2 * np.sin(w * t)
    z_ddot = 0.0

    pos_des = np.array([x, y, z])
    vel_des = np.array([x_dot, y_dot, z_dot])
    acc_des = np.array([x_ddot, y_ddot, z_ddot])

    psi_des = w * t + np.pi / 2          # nose tangent to the direction of travel

    return pos_des, vel_des, acc_des, psi_des

def flat_to_state(acc_des, psi_des, params):
    '''Invert the flat-output map: given desired earth-frame acceleration
    and desired yaw, return required thrust magnitude and quaternion attitude.
    
    acc_des : array_like, [x_ddot, y_ddot, z_ddot] desired earth-frame accel (NED)
    psi_des : float, desired yaw angle (rad)
    
    Returns:
    T     : float, total thrust magnitude (N)
    q_des : ndarray, [qw, qx, qy, qz] required attitude quaternion
    '''
    m = params['m']
    g = params['g']

    # --- Step 1: thrust vector from desired acceleration ---
    g_vec = np.array([0.0, 0.0, g])          # NED earth-frame gravity
    thrust_vec = m * (g_vec - acc_des)       # = T * b3

    T = np.linalg.norm(thrust_vec)
    b3 = thrust_vec / T                       # required body z-axis, in earth frame

    # --- Step 2: build full orthonormal frame using desired yaw ---
    x_c = np.array([np.cos(psi_des), np.sin(psi_des), 0.0])   # desired heading
    b2 = np.cross(b3, x_c)
    b2 = b2 / np.linalg.norm(b2)
    b1 = np.cross(b2, b3)

    R_des = np.column_stack([b1, b2, b3])     # body axes expressed in earth frame

    # --- Step 3: rotation matrix -> quaternion [qw, qx, qy, qz] ---
    tr = np.trace(R_des)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R_des[2,1] - R_des[1,2]) / S
        qy = (R_des[0,2] - R_des[2,0]) / S
        qz = (R_des[1,0] - R_des[0,1]) / S
    elif R_des[0,0] > R_des[1,1] and R_des[0,0] > R_des[2,2]:
        S = np.sqrt(1.0 + R_des[0,0] - R_des[1,1] - R_des[2,2]) * 2
        qw = (R_des[2,1] - R_des[1,2]) / S
        qx = 0.25 * S
        qy = (R_des[0,1] + R_des[1,0]) / S
        qz = (R_des[0,2] + R_des[2,0]) / S
    elif R_des[1,1] > R_des[2,2]:
        S = np.sqrt(1.0 + R_des[1,1] - R_des[0,0] - R_des[2,2]) * 2
        qw = (R_des[0,2] - R_des[2,0]) / S
        qx = (R_des[0,1] + R_des[1,0]) / S
        qy = 0.25 * S
        qz = (R_des[1,2] + R_des[2,1]) / S
    else:
        S = np.sqrt(1.0 + R_des[2,2] - R_des[0,0] - R_des[1,1]) * 2
        qw = (R_des[1,0] - R_des[0,1]) / S
        qx = (R_des[0,2] + R_des[2,0]) / S
        qy = (R_des[1,2] + R_des[2,1]) / S
        qz = 0.25 * S

    q_des = np.array([qw, qx, qy, qz])
    q_des = q_des / np.linalg.norm(q_des)     # normalize for safety

    return T, q_des

def quat_error(q, q_des):
    '''Rotation from current attitude to desired attitude: q_err = q* x q_des.
    Vector part (q_err[1:]) is the small-angle attitude error used by the
    attitude controller. Sign-corrected to take the shortest path.'''
    qw, qx, qy, qz = q
    q_star = np.array([qw, -qx, -qy, -qz])
    q_err = quat_multiply(q_star, q_des)
    if q_err[0] < 0:
        q_err = -q_err
    return q_err

def quad_dynamics(state, params, thrust_cmd):
    '''F = m*a
    F_body = m * (a + omega x V) -> (Linear and Angular Acceleration)
    F_body = m * (v_dot + omega x V) -> (v_dot = d/dt[u,v,w]  omega = [p,q,r] V = [u,v,w])

    M = I*omega_dot
    M_body = I * omega_dot + omega x (I * omega) - (Angular Acceleration + Gyroscopic Coupling)
        I = diag([Ixx, Iyy, Izz]), omega = [p, q, r], omega_dot = d/dt[p, q, r]

    Quaternion Dynamics: q_dot = 0.5 * quat_multiply(q, omega_pure) Omega Pure [0, p, q, r]
    '''
    Ixx = params['Ixx']
    Iyy = params['Iyy']
    Izz = params['Izz']

    m = params['m']

    x, y, z = state[0:3]
    quat = state[3:7]
    u, v, w = state[7:10]
    p, q, r = state[10:13]

    omega = np.array([p, q, r])
    vel = np.array([u, v, w])

    cT = params['cT']
    motor_lag = params['motor_lag']

    motor_cmd = np.sqrt(thrust_cmd/cT)

    motor_state = state[13:]
    motor_state_dot = (motor_cmd-motor_state)/motor_lag

    mixer = motor_mixer(params)

    thrusts = cT* motor_state**2

    wrench = mixer @ thrusts   # [Fz, Mx, My, Mz]
    F_body = np.array([0, 0, -wrench[0]])
    M_body = wrench[1:4]

    g_body = gravity_body(quat, params)

    #F_body = m * (a_body + g + (omega X vel))
    #a_body = (F_body/m) - g - (omega X vel)

    a_body = F_body/m + g_body - np.cross(omega, vel)

    '''M_body = I * omega_dot + omega x (I * omega)
    I*omega_dot = M_body - omega x (I*omega)
    omega_dot = np.linalg.solve(I, M_body - omega x (I*omega))'''

    I = np.diag([Ixx, Iyy, Izz])
    omega_dot = np.linalg.solve(I, M_body - np.cross(omega, I @ omega))

    omega_pure = [0, p, q, r]
    q_dot = 0.5 * quat_multiply(quat, omega_pure)

    pure_vel = [0, u, v, w]
    vel_earth = vel_NED(quat, pure_vel) 

    state_dot = np.concatenate([vel_earth, q_dot, a_body, omega_dot, motor_state_dot])

    return state_dot

def quad_control_step_test(params, mode='roll', delta=0.5):

    m, g = params['m'], params['g']
    hover = (m * g) / 4
    thrust_cmd = np.array([hover, hover, hover, hover])

    if mode == 'roll':      # perturb rotor 2/4 pair
        thrust_cmd[1] -= delta
        thrust_cmd[3] += delta
    elif mode == 'pitch':   # perturb rotor 1/3 pair
        thrust_cmd[0] += delta
        thrust_cmd[2] -= delta
    elif mode == 'yaw':
        thrust_cmd[1] += delta
        thrust_cmd[3] += delta
        thrust_cmd[0] -= delta
        thrust_cmd[2] -= delta
    elif mode == 'hover':
        pass

    return thrust_cmd


def rk4_step(f, t, x, dt):
    """
    Perform a single step of the 4th-order Runge-Kutta method.

    Parameters:
    f : function
        The function that computes the derivative of x with respect to t.
    t : float
        The current time.
    x : array_like
        The current state vector.
    dt : float
        The time step size.

    Returns:
    x_next : array_like
        The state vector at the next time step.
    """
    x = np.array(x)

    k1 = f(t, x)
    k2 = f(t + dt / 2, x + dt / 2 * k1)
    k3 = f(t + dt / 2, x + dt / 2 * k2)
    k4 = f(t + dt, x + dt * k3)

    x_next = x + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    return x_next


#Main Sim Script

state0 = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] #[x, y, x, qw, qx, qy, qz, u, v, w, p, q, r, m1, m2, m3, m4]
params = {
    'm': 1, 
    'g': 9.81, 
    'Ixx': 1,
    'Iyy': 1, 
    'Izz': 1, 
    'L' : 1,
    'k': 1,
    'cT': 1,
    'motor_lag': 0.1,
    'radius': 2.0,
    'circle_omega': 0.5,
    'climb_rate': 0.1,
    'Kp_att': 400.0,
    'Kd_att': 28.0,
}

mixer = motor_mixer(params)
mixer_inv = np.linalg.inv(mixer)

t = 10
dt = 0.01

steps = int(t/dt)

# Trim the initial state to the trajectory's t=0 flat outputs so the sim
# starts on the desired path instead of snapping to it.
pos0, vel0, acc0, psi0 = trajectory_circle(0.0, params)
T0, q0 = flat_to_state(acc0, psi0, params)
q0_star = np.array([q0[0], -q0[1], -q0[2], -q0[3]])
vel_body0 = vel_NED(q0_star, np.array([0.0, *vel0]))
motor0 = np.sqrt((T0 / 4) / params['cT'])

state0 = [*pos0, *q0, *vel_body0, 0, 0, 0, motor0, motor0, motor0, motor0]
state = np.array(state0)
state_history = np.zeros((17,steps))
pos_ref_history = np.zeros((3,steps))

for i in range(steps):
    t = dt*i

    pos_des, vel_des, acc_des, psi_des = trajectory_circle(t, params)
    pos_ref_history[:, i] = pos_des

    T_cmd, q_des = flat_to_state(acc_des, psi_des, params)

    quat_current = state[3:7]
    omega_current = state[10:13]

    q_err = quat_error(quat_current, q_des)
    M_cmd = params['Kp_att'] * q_err[1:] - params['Kd_att'] * omega_current

    thrusts_cmd = mixer_inv @ np.array([T_cmd, *M_cmd])
    thrusts_cmd = np.clip(thrusts_cmd, 0, None)   # rotors can't produce negative thrust

    state = rk4_step(lambda t, s: quad_dynamics(s, params, thrusts_cmd), t - dt, state, dt)
    state[3:7] = state[3:7] / np.linalg.norm(state[3:7])   # renormalize quaternion

    state_history[:, i] = state

time = np.arange(steps) * dt

fig, axs = plt.subplots(5, 1, figsize=(10, 12), sharex=True)

axs[0].plot(time, state_history[0, :], color='C0', label='x')
axs[0].plot(time, state_history[1, :], color='C1', label='y')
axs[0].plot(time, state_history[2, :], color='C2', label='z')
axs[0].plot(time, pos_ref_history[0, :], color='C0', linestyle='--', linewidth=0.8)
axs[0].plot(time, pos_ref_history[1, :], color='C1', linestyle='--', linewidth=0.8)
axs[0].plot(time, pos_ref_history[2, :], color='C2', linestyle='--', linewidth=0.8)
axs[0].set_ylabel('Position (m)')
axs[0].legend()
axs[0].grid(True)

axs[1].plot(time, state_history[3, :], label='qw')
axs[1].plot(time, state_history[4, :], label='qx')
axs[1].plot(time, state_history[5, :], label='qy')
axs[1].plot(time, state_history[6, :], label='qz')
axs[1].set_ylabel('Quaternion')
axs[1].legend()
axs[1].grid(True)

axs[2].plot(time, state_history[7, :], label='u')
axs[2].plot(time, state_history[8, :], label='v')
axs[2].plot(time, state_history[9, :], label='w')
axs[2].set_ylabel('Body Vel (m/s)')
axs[2].legend()
axs[2].grid(True)

axs[3].plot(time, state_history[10, :], label='p')
axs[3].plot(time, state_history[11, :], label='q')
axs[3].plot(time, state_history[12, :], label='r')
axs[3].set_ylabel('Body Rate (rad/s)')
axs[3].legend()
axs[3].grid(True)

rotor_thrusts = params['cT'] * state_history[13:17, :]**2

axs[4].plot(time, rotor_thrusts[0, :], label='T1')
axs[4].plot(time, rotor_thrusts[1, :], label='T2')
axs[4].plot(time, rotor_thrusts[2, :], label='T3')
axs[4].plot(time, rotor_thrusts[3, :], label='T4')
axs[4].set_ylabel('Rotor Thrust (N)')
axs[4].set_xlabel('Time (s)')
axs[4].legend()
axs[4].grid(True)

fig.suptitle('Quadrotor State History')
plt.tight_layout()
plt.show()

