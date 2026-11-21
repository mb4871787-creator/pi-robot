"""
pid_controller.py
-----------------
Generic PID controller used by the decision node for alignment.
"""

import time


class PIDController:
    """
    Discrete PID controller with anti-windup and output clamping.

    Usage:
        pid = PIDController(kp=0.1, ki=0.01, kd=0.005, max_output=100)
        output = pid.compute(setpoint=0, measurement=error_value)
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        max_output: float = 100.0,
        integral_limit: float = 500.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output     = max_output
        self.integral_limit = integral_limit

        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None

    def compute(self, setpoint: float, measurement: float) -> float:
        """
        Compute PID output.

        Args:
            setpoint:    Desired value (usually 0 for error-based control)
            measurement: Current measured value

        Returns:
            Clamped control output
        """
        now = time.time()
        error = setpoint - measurement

        # Delta time
        if self._prev_time is None:
            dt = 0.05           # assume 20 Hz on first call
        else:
            dt = now - self._prev_time
            dt = max(dt, 1e-6)  # guard against zero dt

        # Integral with anti-windup
        self._integral += error * dt
        self._integral = max(
            -self.integral_limit,
            min(self.integral_limit, self._integral)
        )

        # Derivative
        derivative = (error - self._prev_error) / dt

        # PID output
        output = (
            self.kp * error
            + self.ki * self._integral
            + self.kd * derivative
        )

        # Clamp
        output = max(-self.max_output, min(self.max_output, output))

        # Store state
        self._prev_error = error
        self._prev_time  = now

        return output

    def reset(self) -> None:
        """Reset integrator and derivative state."""
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None
