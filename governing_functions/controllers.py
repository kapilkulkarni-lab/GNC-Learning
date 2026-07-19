import numpy as np

'''PID Controller Implementation: The PIDController class
implements a Proportional-Integral-Derivative (PID) controller.'''


class PIDController:
    """
    A simple PID controller implementation.

    Attributes:
    Kp : float
        Proportional gain.
    Ki : float
        Integral gain.
    Kd : float
        Derivative gain.
    integral : float
        The accumulated integral error.
    last_error : float
        The error from the previous time step.
    """

    def __init__(self, dt, Kp, Ki, Kd):
        """
        Initialize the PID controller with specified gains and setpoint.

        Parameters:
        dt : float
            The time step size.
        Kp : float
            Proportional gain.
        Ki : float
            Integral gain.
        Kd : float
            Derivative gain.
        """
        self.dt = dt
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, error):
        """
        Compute the control output based on the current error.

        Parameters:
        error : float
            The current error (setpoint - measured_value).

        Returns:
        control_output : float
            The control output to be applied to the system.
        """
        dt = self.dt
        self.error = error
        # Update integral and derivative terms
        self.integral += self.error * dt
        derivative = (self.error - self.last_error) / dt if dt > 0 else 0.0

        # Compute PID output
        control_output = self.Kp * self.error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = self.error

        return control_output

    @staticmethod
    def angle_error(current_angle, target_angle):
        """
        Calculate the error between the current angle and the target angle.

        Parameters:
        current_angle : float
            The current angle in radians.
        target_angle : float
            The target angle in radians.

        Returns:
        normalized_angle : float
            The normalized angle in radians.
        """
        error = (target_angle - current_angle + np.pi) % (2 * np.pi) - np.pi
        return error

    @staticmethod
    def settling_time(angle_storage, time_storage, target_angle, threshold=0.05):
        """
        Calculate the settling time of the system based on angle storage.

        Parameters:
        angle_storage : array_like
            The stored angles over time.
        time_storage : array_like
            The corresponding time points for the stored angles.
        target_angle : float
            The target angle in radians.
        threshold : float
            The threshold for determining settling

        Returns:
        settling_time : float
            The time at which the system settles within the threshold of the target angle.
        """
        # Calculate the absolute error from the target angle
        error = np.abs(angle_storage - target_angle)

        # Find indices where error is within the threshold
        outside_threshold = np.where(error > threshold)[0]

        if len(outside_threshold) == 0:
            # If the error is always within the threshold, return the last time point
            return time_storage[0]

        last_violation_index = outside_threshold[-1]

        if last_violation_index == len(error) - 1:
            # If the last violation is at the end of the array, the system never settles
            return None

        return time_storage[last_violation_index + 1]  # First index after the last violation
