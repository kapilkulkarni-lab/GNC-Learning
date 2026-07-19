import numpy as np


class KalmanFilter_1D:

    def __init__(self, x0, P0, Q, R):
        """
        Initialize the Kalman Filter with initial state,
        covariance, process noise, and measurement noise. """

        self.x = x0  # Initial state estimate
        self.P = P0  # Initial estimate covariance
        self.Q = Q   # Process noise covariance
        self.R = R   # Measurement noise covariance

    def predict(self, x_dot=0, dt=1):
        """
        Predict the next state and covariance based on the control input.

        Parameters:
        x_dot : float
            The rate of change of the state (default is 0).
        dt : float
            The time step (default is 1).
        """
        # State prediction
        self.x = self.x + x_dot * dt  # Assuming a simple model where the state is updated by the control input
        # Covariance prediction
        self.P = self.P + self.Q  # Update the estimate covariance with process noise

    def update(self, z):
        """
        Update the state estimate and covariance based on the measurement.

        Parameters:
        z : float
            The measurement.
        """
        # Kalman Gain
        K = self.P / (self.P + self.R)

        # State update
        self.x = self.x + K * (z - self.x)  # Update the state estimate with the measurement

        # Covariance update
        self.P = (1 - K) * self.P  # Update the estimate covariance
        return self.x  # Return the updated state estimate


class KalmanFilter_multiD:
    def __init__(self, x0, P0, Q, R, F, H):
        """
        Initialize the Kalman Filter for a 2D system with initial state,
        covariance, process noise, and measurement noise.

        Parameters:
        x0 : array_like
            Initial state estimate (2D vector).
        P0 : ndarray
            Initial estimate covariance (2x2 matrix).
        Q : ndarray
            Process noise covariance (2x2 matrix).
        R : ndarray
            Measurement noise covariance (2x2 matrix).
        F : ndarray
            State transition matrix (2x2).
        H : ndarray
            Measurement matrix (2x2).
        """
        self.x = np.array(x0)  # Initial state estimate
        self.P = np.array(P0)  # Initial estimate covariance
        self.Q = np.array(Q)   # Process noise covariance
        self.R = np.array(R)   # Measurement noise covariance
        self.F = np.array(F)   # State transition matrix
        self.H = np.array(H)   # Measurement matrix

    def predict(self):
        """
        Predict the next state and covariance based on the state transition model.
        """
        # State prediction
        self.x = self.F @ self.x  # Update the state estimate using the state transition model
        # Covariance prediction
        self.P = self.F @ self.P @ self.F.T + self.Q  # Update the estimate covariance with process noise

    def update(self, z):
        """
        Update the state estimate and covariance based on the measurement.

        Parameters:
        z : array_like
            The measurement (2D vector).
        """
        z = np.array(z)

        y = z - self.H @ self.x  # Measurement residual
        S = self.H @ self.P @ self.H.T + self.R  # Residual covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain

        # State update
        self.x = self.x + K @ y  # Update the state estimate with the measurement

        # Covariance update
        I = np.eye(self.P.shape[0])  # Identity matrix of appropriate size
        self.P = (I - K @ self.H) @ self.P  # Update the estimate covariance

        return self.x  # Return the updated state estimate


class Extended_KalmanFilter:
    def __init__(self, x0, P0, Q, R, f, h, u):
        """
        Initialize the Extended Kalman Filter with initial state,
        covariance, process noise, and measurement noise.

        Parameters:
        x0 : array_like
            Initial state estimate.
        P0 : ndarray
            Initial estimate covariance.
        Q : ndarray
            Process noise covariance.
        R : ndarray
            Measurement noise covariance.
        f : function
            Non-linear state transition function.
        h : function
            Non-linear measurement function.
        u : array_like
            Control input vector.
        """
        self.x = np.array(x0, dtype=float)  # Initial state estimate
        self.P = np.array(P0, dtype=float)  # Initial estimate covariance
        self.Q = np.array(Q, dtype=float)   # Process noise covariance
        self.R = np.array(R, dtype=float)   # Measurement noise covariance
        self.f = f             # Non-linear state transition function
        self.h = h             # Non-linear measurement function
        self.u = np.array(u, dtype=float)   # Control input vector

    def predict(self, dt, u=None):
        """
        Predict the next state and covariance based on the non-linear state transition model.

        Parameters:
        dt : float
            The time step for prediction.
        u : array_like, optional
            The control input vector.
        """
        if u is not None:
            self.u = np.array(u, dtype=float)

        # Compute the Jacobian of f at the current state for linearization
        F_jacobian = self.compute_jacobian_f(self.x, self.u, dt)

        # State prediction using the non-linear function f
        self.x = self.f(self.x, self.u, dt)  # Assuming f takes in state, dt, and control input u

        # Covariance prediction using the Jacobian
        self.P = F_jacobian @ self.P @ F_jacobian.T + self.Q

    def update(self, z):
        """
        Update the state estimate and covariance based on the non-linear measurement.

        Parameters:
        z : array_like
            The measurement vector.
        """
        z = np.array(z)

        # Measurement prediction using the non-linear function h
        z_pred = self.h(self.x)

        # Compute the Jacobian of h at the current state for linearization
        H_jacobian = self.compute_jacobian_h(self.x)

        # Measurement residual
        y = z - z_pred

        # Residual covariance
        S = H_jacobian @ self.P @ H_jacobian.T + self.R

        # Kalman Gain
        K = self.P @ H_jacobian.T @ np.linalg.inv(S)

        # State update
        self.x = self.x + K @ y

        # Covariance update
        I = np.eye(self.P.shape[0])  # Identity matrix of appropriate size
        self.P = (I - K @ H_jacobian) @ self.P

        return self.x  # Return the updated state estimate

    def compute_jacobian_f(self, x, u, dt, eps=1e-5):
        x = np.array(x, dtype=float)
        n = len(x)
        f0 = np.array(self.f(x, u, dt))
        J = np.zeros((len(f0), n))
        for i in range(n):
            dx = np.zeros(n)
            dx[i] = eps
            J[:, i] = (self.f(x + dx, u, dt) - self.f(x - dx, u, dt)) / (2 * eps)
        return J

    def compute_jacobian_h(self, x, eps=1e-5):
        x = np.array(x, dtype=float)
        n = len(x)
        h0 = np.array(self.h(x))
        J = np.zeros((len(h0), n))
        for i in range(n):
            dx = np.zeros(n)
            dx[i] = eps
            J[:, i] = (self.h(x + dx) - self.h(x - dx)) / (2 * eps)
        return J
