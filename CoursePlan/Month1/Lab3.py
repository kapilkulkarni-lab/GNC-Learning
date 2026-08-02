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
    p, q, r = state[10:]

    omega = np.array([p, q, r])
    vel = np.array([u, v, w])

    mixer = motor_mixer(params)

    wrench = mixer @ thrust_cmd   # [Fz, Mx, My, Mz]
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

    state_dot = np.concatenate([vel_earth, q_dot, a_body, omega_dot])

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

state0 = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0] #[x, y, x, qw, qx, qy, qz, u, v, w, p, q, r]
params = {
    'm': 1, 
    'g': 9.81, 
    'Ixx': 1,
    'Iyy': 1, 
    'Izz': 1, 
    'L' : 1,
    'k': 1,
}

t = 10
dt = 0.01

steps = int(t/dt)
state = np.array(state0)
state_history = np.zeros((13,steps))

for i in range(steps):
    t = dt*i
    if t <= 4:
        thrusts_cmd = quad_control_step_test(params, 'hover')
    else:
        thrusts_cmd = quad_control_step_test(params, 'pitch')
    
    state = rk4_step(lambda t, s: quad_dynamics(s, params, thrusts_cmd), t - dt, state, dt)

    state_history[:, i] = state

time = np.arange(steps) * dt

fig, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

axs[0].plot(time, state_history[0, :], label='x')
axs[0].plot(time, state_history[1, :], label='y')
axs[0].plot(time, state_history[2, :], label='z')
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
axs[3].set_xlabel('Time (s)')
axs[3].legend()
axs[3].grid(True)

fig.suptitle('Quadrotor State History')
plt.tight_layout()
plt.show()

