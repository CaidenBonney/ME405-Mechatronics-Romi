from pyb import Pin  # pyright: ignore
from Encoder import Encoder
from Motor import Motor
from math import copysign
from IR_Sensor import IRSensor
from Line_Sensor import LineSensor
from Sensor import Sensor


class ClosedLoopControl:
    def __init__(
        self,
        sensor: Sensor,
        max_min: float = 100,
        Kp: float = 0,
        Ki: float = 0,
        Kd: float = 0,
        Kw: float = 0,
        Kff: float = 0,
        PWM_start: float = 0,
        battery_ADC: Pin = None,
    ):
        self.sensor = sensor
        self.max_min = max_min
        self.battery_ADC = battery_ADC

        # Gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Kw = Kw
        self.Kff = Kff
        self.PWM_start = PWM_start

        # Inputs and Outputs
        self.r = 0  #                  Setpoint
        self.x_h = 0  #                Measured value
        self.a = 0  #                  Actuation value
        self.astar = 0  #              Saturated actuation value

        # Intermediate variables
        self.e = 0  #                  Error
        self.prev_error = 0  #         Previous error
        self.integral = 0  #           Integral value
        self.derivative = 0  #         Derivative value

        # Battery Droop Variables
        self.V_nom = 3625  #           Battery nominal voltage
        self.V_bat = 0  #              Battery current voltage

        self.on = True

    def reset(self):
        self.r = 0  #                  Setpoint
        self.e = 0  #                  Error
        self.prev_error = 0  #         Previous error
        self.integral = 0  #           Integral value
        self.derivative = 0  #         Derivative value

    def set_ref(self, speed):
        self.r = speed

    def disable(self):
        self.on = False

    def enable(self):
        self.on = True

    def gain_update(
        self,
        Kp: float = 0,
        Ki: float = 0,
        Kd: float = 0,
        Kw: float = 0,
        Kff: float = 0,
        PWM_start: float = 0,
    ):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Kw = Kw
        self.Kff = Kff
        self.PWM_start = PWM_start

    def run(self):
        """Runs the closed loop control algorithm"""
        if not self.on:
            return 0

        # Get current measured velocity
        self.x_h = self.sensor.get_data()

        # Calculate error
        self.prev_error = self.e
        self.e = self.r - self.x_h

        # Calculate actuation value
        if self.Ki:
            self.integral += self.e * (self.sensor.dt / 1e6)  # convert from uticks to seconds
        if self.Kd:
            self.derivative = (self.e - self.prev_error) / (self.sensor.dt / 1e6)  # convert from uticks to seconds

        if self.battery_ADC is not None:
            # Find current battery voltage
            self.V_bat = self.battery_ADC.read()
            # Dynamically calculate droop gain
            Kdroop = self.V_nom / self.V_bat
        else:
            Kdroop = 1

        self.a = Kdroop * (
            (self.Kp * self.e)  #                                               Proportional
            + (self.integral * self.Ki)  #                                      Integral
            + (self.derivative * self.Kd)  #                                    Derivative
            + ((self.r * self.Kff) + (copysign(1, self.r) * self.PWM_start))  # Feed Forward
        )

        # Saturation
        self.astar = min(self.max_min, max(-self.max_min, self.a))
        if self.astar != self.a:
            # print(f"Saturation: {self.a} -> {self.astar}")
            pass

        # Apply windup gain: Kw
        if self.Kw:
            self.integral -= (self.a - self.astar) * self.Kw  #                   Windup

        # return actuation value
        return self.astar


# # ************* For reference ************* #
# @classmethod
# def set_gain(cls, K="Kp", value=0.0):
#     """Multiply a class-level gain by the given factor."""
#     if not hasattr(cls, K):
#         raise AttributeError(f"'{K}' is not a defined gain on {cls.__name__}")
#     setattr(cls, K, value)
# @classmethod
# def increase_gain(cls, K="Kp", factor=2.0):
#     """Multiply a class-level gain by the given factor."""
#     if not hasattr(cls, K):
#         raise AttributeError(f"'{K}' is not a defined gain on {cls.__name__}")
#     setattr(cls, K, getattr(cls, K) * factor)

# @classmethod
# def decrease_gain(cls, K, factor=2.0):
#     """Divide a class-level gain by the given factor."""
#     if not hasattr(cls, K):
#         raise AttributeError(f"'{K}' is not a defined gain on {cls.__name__}")
#     setattr(cls, K, getattr(cls, K) / factor)
