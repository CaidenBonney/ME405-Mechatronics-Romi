from pyb import Pin  # pyright: ignore
from encoder import Encoder
from motor import Motor
from math import copysign


class ClosedLoopControl:

    # # Global Variables Shared by all instances
    # Battery Droop Variables
    V_nom = 3625  #                    Battery nominal voltage
    V_bat = 0  #                       Battery current voltage

    # Gains
    Kp = 0.1  #                        Proportional gain
    Ki = 1.0  #                        Integral gain
    Kw = 1.0   #                       Windup gain
    Kffr = 0.10  #                     Feed forward gain right Calced: 0.112
    Kffl = 0.10  #                     Feed forward gain left Calced: 0.107
    ff_startr = -2.17  #               Feed forward gain at start regardless of reference speed Calced: -2.17
    ff_startl = 2.49  #                Feed forward gain at start regardless of reference speed Calced: 2.49

    def __init__(self, encoder: Encoder, motor: Motor, battery_ADC: Pin, side: bool):
        self.encoder = encoder
        self.motor = motor
        self.battery_ADC = battery_ADC
        self.side = side  # False = left, True = right

        # Feed Forward
        if self.side:
            # Right Side
            self.Kff = ClosedLoopControl.Kffr  #        Feed forward gain
            self.ff_start = ClosedLoopControl.ff_startr  #    Feed forward gain at start regardless of reference speed
        else:
            # Left Side
            self.Kff = ClosedLoopControl.Kffl  #        Feed forward gain
            self.ff_start = ClosedLoopControl.ff_startl  #    Feed forward gain at start regardless of reference speed

        # Inputs and Outputs
        self.r = 0  #                  Setpoint
        self.x_h = 0  #                Measured value
        self.a = 0  #                  Actuation value
        self.astar = 0  #              Saturated actuation value

        # Intermediate variables
        self.e = 0  #                  Error
        self.prev_error = 0  #         Previous error
        self.integral = 0  #           Integral value

    def reset(self):
        self.r = 0  #                  Setpoint
        self.e = 0  #                  Error
        self.prev_error = 0  #         Previous error
        self.integral = 0  #           Integral value

    @classmethod
    def set_gain(cls, K="Kp", value=0.0):
        """Multiply a class-level gain by the given factor."""
        if not hasattr(cls, K):
            raise AttributeError(f"'{K}' is not a defined gain on {cls.__name__}")
        setattr(cls, K, value)

    # # ************* For reference ************* #
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

    def set_ref(self, speed):
        self.reset()
        if speed == 0:
            self.stop()
        self.r = speed

    def stop(self):
        self.r = 0
        self.motor.set_effort(0)

    def run(self):
        """Runs the closed loop control algorithm"""
        if self.r == 0:
            return

        # Get current measured velocity
        self.x_h = self.encoder.velocity

        # Calculate error
        self.e = self.r - self.x_h

        # Calculate actuation value
        # P controller
        # self.a = ClosedLoopControl.Kp * self.e

        # # P + I Controller
        # self.integral += self.e * self.encoder.dt / 1e6  # convert from uticks to seconds
        # self.a = (ClosedLoopControl.Kp * self.e) + (self.integral * ClosedLoopControl.Ki)

        # P + I + FF Controller
        # self.integral += self.e * self.encoder.dt / 1e6  # convert from uticks to seconds
        # self.a = (
        #     (ClosedLoopControl.Kp * self.e)
        #     + (self.integral * ClosedLoopControl.Ki)
        #     + ((self.r * self.Kff) + (copysign(1, self.r) * self.ff_start))
        # )

        # Droop * (P + I + FF) Controller
        self.integral += self.e * self.encoder.dt / 1e6  # convert from uticks to seconds
        ClosedLoopControl.V_bat = self.battery_ADC.read()  # Pin for battery voltage
        self.a = (ClosedLoopControl.V_nom / ClosedLoopControl.V_bat) * (
            (ClosedLoopControl.Kp * self.e)
            + (self.integral * ClosedLoopControl.Ki)
            + ((self.r * self.Kff) + (copysign(1, self.r) * self.ff_start))
        )

        # Saturation
        self.astar = min(100, max(-100, self.a))
        if self.astar != self.a:
            # print(f"Saturation: {self.a} -> {self.astar}")
            pass

        # # Apply windup gain: Kw
        self.integral -= (self.a - self.astar) * ClosedLoopControl.Kw

        # set actuation value
        self.motor.set_effort(self.astar)
