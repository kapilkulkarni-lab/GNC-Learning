import numpy as np


def quad_dynamics(t, state, omega, params):
    """Quadcopter 6-DOF dynamics (port of quadDynamics.m).

    state: [p, q, r, phi, theta, psi, u, v, w, x, y, z] (NED, body velocities)
    omega: rotor speeds (4,) in rad/s
    Returns dstate (12,)
    """
    p, q, r = state[0], state[1], state[2]
    phi, theta, psi = state[3], state[4], state[5]
    u, v, w = state[6], state[7], state[8]

    m = params['m']
    g = params['g']
    J = params['J']
    cT = params['cT']
    cM = params['cM']
    d = params['d']

    # Rotor thrusts
    thrusts = cT * omega**2

    # Thrust acts upward along body -z axis
    F_body = np.array([0.0, 0.0, -np.sum(thrusts)])

    # Torques
    # 1=front(+x), 2=right(+y), 3=back(-x), 4=left(-y)
    tau_phi = d * (thrusts[1] - thrusts[3])       # roll (x-body)
    tau_theta = d * (thrusts[0] - thrusts[2])     # pitch (y-body)
    tau_psi = (cM / cT) * (thrusts[0] + thrusts[2] - thrusts[1] - thrusts[3])  # yaw (z-body)
    M_b = np.array([tau_phi, tau_theta, tau_psi])

    # Body to Inertial (NED) rotation matrix
    Rbn = np.array([
        [np.cos(psi)*np.cos(theta), np.cos(psi)*np.sin(theta)*np.sin(phi) - np.sin(psi)*np.cos(phi), np.cos(psi)*np.sin(theta)*np.cos(phi) + np.sin(psi)*np.sin(phi)],
        [np.sin(psi)*np.cos(theta), np.sin(psi)*np.sin(theta)*np.sin(phi) + np.cos(psi)*np.cos(phi), np.sin(psi)*np.sin(theta)*np.cos(phi) - np.cos(psi)*np.sin(phi)],
        [-np.sin(theta),            np.cos(theta)*np.sin(phi),                                       np.cos(theta)*np.cos(phi)],
    ])
    Rnb = Rbn.T

    # Gravity
    g_n = np.array([0.0, 0.0, g])   # NED gravity (positive downward)
    g_b = Rnb @ g_n                 # gravity in body frame

    # Body-frame velocities
    vb = np.array([u, v, w])
    omega_b = np.array([p, q, r])

    # Acceleration in body frame
    vbdot = (F_body / m) + g_b - np.cross(omega_b, vb)

    omegadot = np.linalg.solve(J, M_b - np.cross(omega_b, J @ omega_b))

    # Euler angle rates
    W = np.array([
        [1.0, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
        [0.0, np.cos(phi),               -np.sin(phi)],
        [0.0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)],
    ])
    eulerdot = W @ omega_b

    # Velocity in NED frame
    rdot_n = Rbn @ vb   # [xdot, ydot, zdot]

    return np.concatenate([omegadot, eulerdot, vbdot, rdot_n])


def rk4_step(f, t, state, dt):
    """Single fixed-step RK4 integration step (replaces ode45 over one dt)."""
    k1 = f(t, state)
    k2 = f(t + dt/2, state + dt/2 * k1)
    k3 = f(t + dt/2, state + dt/2 * k2)
    k4 = f(t + dt, state + dt * k3)
    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
