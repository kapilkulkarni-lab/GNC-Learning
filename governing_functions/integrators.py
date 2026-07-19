import numpy as np


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
