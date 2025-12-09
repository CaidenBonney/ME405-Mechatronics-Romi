from pyb import Pin  # pyright: ignore
from Encoder import Encoder
from Motor import Motor
from math import copysign
from IR_Sensor import IRSensor
from Line_Sensor import LineSensor
from Sensor import Sensor
from Battery import Battery


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
        battery: Battery | None = None,
    ):
        self.sensor = sensor
        self.max_min = max_min
        self.battery = battery

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

        self.on = True
        self.first_run = True
        self.num_corrections = 75
        self.total_num_corrections = 200

    def reset(self) -> None:
        self.r = 0  #                  Setpoint
        self.e = 0  #                  Error
        self.prev_error = 0  #         Previous error
        self.integral = 0  #           Integral value
        self.derivative = 0  #         Derivative value
        self.first_run = True

    def set_ref(self, speed) -> None:
        self.r = speed

    def disable(self) -> None:
        self.on = False

    def enable(self) -> None:
        self.on = True

    def gain_update(
        self,
        Kp: float = 0,
        Ki: float = 0,
        Kd: float = 0,
        Kw: float = 0,
        Kff: float = 0,
        PWM_start: float = 0,
    ) -> None:
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

        if self.num_corrections > 0:
            self.r_u = self.r * ((self.total_num_corrections - self.num_corrections) / self.total_num_corrections)
            self.num_corrections -= 1
        else:
            self.r_u = self.r

        # Get current measured velocity
        self.x_h = self.sensor.get_data()
        # print("x_h", self.x_h) # -------------------

        # Calculate error
        self.prev_error = self.e
        self.e = self.r_u - self.x_h
        # print("e", self.e) # -------------------

        # Calculate actuation value

        time_delta = self.sensor.dt / 1e6
        if self.first_run:
            time_delta = 0.020

        # print("dt", time_delta) # -------------------
        if self.Ki:
            self.integral += self.e * time_delta  # convert from uticks to seconds
        if self.Kd:
            self.derivative = (self.e - self.prev_error) / time_delta  # convert from uticks to seconds

        # if self.Ki:
        #     self.integral += self.e * (self.sensor.dt / 1e6)  # convert from uticks to seconds
        # if self.Kd:
        #     self.derivative = (self.e - self.prev_error) / (self.sensor.dt / 1e6)  # convert from uticks to seconds

        # print("intgrl", self.integral) # -------------------
        if self.battery is not None:
            # Dynamically calculate droop gain
            Kdroop = 1 / self.battery.get_cur_perc()
        else:
            Kdroop = 1

        self.a = Kdroop * (
            (self.Kp * self.e)  #                                               Proportional
            + (self.integral * self.Ki)  #                                      Integral
            + (self.derivative * self.Kd)  #                                    Derivative
            + ((self.r_u * self.Kff) + (copysign(1, self.r) * self.PWM_start))  # Feed Forward
        )

        # print("a", self.a) # -------------------
        # Saturation
        self.astar = min(self.max_min, max(-self.max_min, self.a))

        # Apply windup gain: Kw
        if self.Kw:
            self.integral -= (self.a - self.astar) * self.Kw  #                 Windup

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
