import numpy as np
import matplotlib.pyplot as plt

#Creating a simple 3 DOF USV (Dropped Roll, Pitch, Heave)

def build_mass_matrix(params):
    #Building the mass matrix 
    M = np.array([
        [params['m']-params['Xu_dot'], 0.0, 0.0],
        [0.0, params['m']-params['Yv_dot'], 0.0],
        [0.0, 0.0, params['Iz']-params['Nr_dot']]
    ])

    return M

def coriolis_matrix(nu, M):
    #Building the coriolis matrix

    u, v, r  = nu
    m11 = M[0,0]
    m22 = M[1,1]
    m33 = M[2,2]
    C = np.array([
        [0.0,        0.0,       -m22 * v],
        [0.0,        0.0,        m11 * u],
        [m22 * v,   -m11 * u,    0.0]
    ])

    return C

def damping_matrix(nu, params):
    #Building the damping matrix
    u,v,r = nu
    
    D = np.array([
        [params['Xu'] + params['Xuu'] * abs(u), 0.0, 0.0],
        [0.0, params['Yv'] + params['Yvv'] * abs(v), 0.0],
        [0.0, 0.0, params['Nr'] + params['Nrr'] * abs(r)]
    ])

    return D


def nu_dot(nu, tau, M, params, current_nu=np.zeros(3)):
    nu_rel = nu - current_nu  # relative velocity for damping/Coriolis 
    C = coriolis_matrix(nu_rel, M)
    D = damping_matrix(nu_rel, params)
    rhs = tau - C @ nu_rel - D @ nu_rel
    return np.linalg.solve(M, rhs)

def current_velocity(t):
    #Constant Velocity Function for now
    #A real water current only has translational drift, no imposed rotation
    const_vel = [0.3, 0.1, 0.0]

    return const_vel

def eta_dot(eta, nu):
    u, v, r = nu
    x, y, psi = eta
    eta_dot = [u*np.cos(psi) - v*np.sin(psi), u*np.sin(psi) + v*np.cos(psi), r]

    return eta_dot


def rk4_step(f, t, x, u, dt):
    """
    Perform a single step of the 4th-order Runge-Kutta method.

    Parameters:
    f : function
        The function that computes the derivative of x with respect to t.
    t : float
        The current time.
    x : array_like
        The current state vector.
    u : array_like
        The control input vector.
    dt : float
        The time step size.

    Returns:
    x_next : array_like
        The state vector at the next time step.
    """
    x = np.array(x)

    k1 = f(t, x, u)
    k2 = f(t + dt / 2, x + dt / 2 * k1, u)
    k3 = f(t + dt / 2, x + dt / 2 * k2, u)
    k4 = f(t + dt, x + dt * k3, u)

    x_next = x + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    return x_next

def tau_cmd(t, state):

    tau_u_step = 15.0  # N, ~two small thrusters at partial throttle
    tau_r_step = -0.01   # N*m, differential thrust for turning
    tau = [tau_u_step, 0, tau_r_step]

    return tau

def state_dot(t, state, tau, params, M, nu_current):
    #Creating state_dot using eta_dot, nu_dot functions by unpacking state and packing state_dot
    eta = state[0:3]
    nu  = state[3:6]

    d_eta = eta_dot(eta, nu)
    d_nu  = nu_dot(nu, tau, M, params, nu_current)

    return np.concatenate([d_eta, d_nu])

# Assumed vehicle: small twin-hull ASV, L=1.2m, B=0.6m, T=0.12m, freshwater (rho=1000 kg/m^3)
params = {
    'm': 25.0,  #Mass in kg (small ASV incl. battery/electronics)
    'Iz': 4.0,  #Yaw inertia in kg*m^2, ~ m*(L^2+B^2)/12

    #Added Mass Derivs (negative, Fossen convention; scaled off m / Iz)
    'Xu_dot': -2.5,  # surge added mass ~ 0.1*m (slender bow)
    'Yv_dot': -20.0, # sway added mass ~ 0.8*m (broad beam-on area)
    'Nr_dot': -2.0,  # yaw added inertia ~ 0.5*Iz

    #Linear Damping Coefs (skin friction, N per m/s or N*m per rad/s)
    'Xu': 3.0,
    'Yv': 12.0,
    'Nr': 3.0,

    #Quadratic Damping Coefs (form drag, ~0.5*rho*Cd*A)
    'Xuu': 20.0,  # Cd~0.8, frontal area B*T=0.072 m^2
    'Yvv': 90.0,  # Cd~1.0, lateral area L*T=0.144 m^2
    'Nrr': 8.0,
    }


eta = np.array([0.0, 0.0, 0.0]) #3 Dim State of Surge, Sway, Yaw
nu = np.array([0.0, 0.0, 0.0])  #3 Dim state of u, v, r (body frame velocities)


state =  np.append([eta], [nu])

M_mat = build_mass_matrix(params)
sim_time = 30
dt = 0.01
n_steps = int(sim_time/dt)

state_history = np.zeros((6, n_steps))
time_history = np.zeros(n_steps)

for i in range(n_steps):

    t =  i*dt
    tau = tau_cmd(t, eta)
    vel_cur = current_velocity(t)

    f = lambda t_, x_, u_: state_dot(t_, x_, u_, params, M_mat, vel_cur)

    state = rk4_step(f, t, state, tau, dt)
    eta = state[0:3]
    state_history[:, i] = state
    time_history[i] = t

surge, sway, psi = state_history[0], state_history[1], state_history[2]
u, v, r = state_history[3], state_history[4], state_history[5]

fig, axes = plt.subplots(3, 2, figsize=(10, 8), sharex=True)

axes[0, 0].plot(time_history, surge)
axes[0, 0].set_ylabel('surge x (m)')

axes[1, 0].plot(time_history, sway)
axes[1, 0].set_ylabel('sway y (m)')

axes[2, 0].plot(time_history, psi)
axes[2, 0].set_ylabel('psi (rad)')
axes[2, 0].set_xlabel('time (s)')

axes[0, 1].plot(time_history, u)
axes[0, 1].set_ylabel('u (m/s)')

axes[1, 1].plot(time_history, v)
axes[1, 1].set_ylabel('v (m/s)')

axes[2, 1].plot(time_history, r)
axes[2, 1].set_ylabel('r (rad/s)')
axes[2, 1].set_xlabel('time (s)')

fig.tight_layout()
plt.show()

