## @file Closed_Loop_Control.py
#  This file implements a generic closed-loop controller used for motor or
#  mechanism speed/position control on the Romi robot. It supports P, PI, PD,
#  PID, feed-forward, anti-windup, battery droop compensation, and soft-start
#  reference ramping.
#
#  The controller takes in a sensor object (providing @c get_data() and @c dt),
#  and optionally a battery object for dynamic gain adjustment under voltage
#  droop conditions.
#
#  @author Antonio Ventimiglia
#  @author Caiden Bonney
#  @date   2025-Dec-12
#  @copyright GPLv3

from math import copysign


## A class implementing a closed-loop control algorithm.
#
#  This class computes an actuation signal using proportional, integral,
#  derivative, feed-forward, and anti-windup terms. It also supports reference
#  ramping and battery droop compensation.
class ClosedLoopControl:

    ## Constructor for the closed-loop controller.
    #
    #  Initializes controller gains, limits, state variables, reference, and
    #  soft-start parameters.
    #
    #  @param sensor An object providing measurement data via @c get_data()
    #                and a timing attribute @c dt in microseconds.
    #  @param max_min Maximum absolute actuation output (saturation limit)
    #  @param Kp Proportional gain
    #  @param Ki Integral gain
    #  @param Kd Derivative gain
    #  @param Kw Anti-windup gain
    #  @param Kff Feed-forward gain
    #  @param PWM_start Small offset to ensure motor startup torque
    #  @param battery Optional battery object providing @c get_cur_perc()
    def __init__(
        self,
        sensor,
        max_min: float = 100,
        Kp: float = 0,
        Ki: float = 0,
        Kd: float = 0,
        Kw: float = 0,
        Kff: float = 0,
        PWM_start: float = 0,
        battery=None,
    ):
        self.sensor = sensor
        self.max_min = max_min
        self.battery = battery

        # @brief Gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Kw = Kw
        self.Kff = Kff
        self.PWM_start = PWM_start

        # Inputs & Outputs
        self.r = 0  # Reference / setpoint
        self.x_h = 0  # Measured value
        self.a = 0  # Raw actuation value
        self.astar = 0  # Saturated actuation value

        # Internal controller states
        self.e = 0  # Error
        self.prev_error = 0  # Previous cycle error
        self.integral = 0  # Integral accumulator
        self.derivative = 0  # Derivative term

        # Soft-start ramping
        self.on = True
        self.first_run = True
        self.num_corrections = 0
        self.total_num_corrections = 200

    ## Reset the controller.
    #
    #  Clears error, derivative, and integral terms; resets soft-start ramping.
    def reset(self) -> None:
        self.r = 0
        self.e = 0
        self.prev_error = 0
        self.integral = 0
        self.derivative = 0
        self.first_run = True

    ## Reset the ramping counter used for soft-start reference slew.
    #
    #  This shortens the ramp so that the reference reaches its target sooner.
    def reset_num_corrections(self):
        self.num_corrections = 75

    ## Set the controller reference (setpoint).
    #
    #  @param speed The desired reference value.
    def set_ref(self, speed) -> None:
        self.r = speed

    ## Disable the controller (output will remain zero).
    def disable(self) -> None:
        self.on = False

    ## Enable the controller so @c run() produces actuation commands.
    def enable(self) -> None:
        self.on = True

    ## Update controller gains.
    #
    #  Allows runtime tuning of control gains and feed-forward.
    #
    #  @param Kp Proportional gain
    #  @param Ki Integral gain
    #  @param Kd Derivative gain
    #  @param Kw Anti-windup gain
    #  @param Kff Feed-forward gain
    #  @param PWM_start Startup offset for motor
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

    ## Run one iteration of the closed-loop control law.
    #
    #  Applies reference ramping, reads sensor data, computes error, and evaluates:
    #  - proportional term
    #  - integral term
    #  - derivative term
    #  - feed-forward term
    #  - anti-windup correction
    #
    #  Battery droop compensation scales the output gain to maintain performance
    #  under voltage sag.
    #
    #  @return The saturated actuation command to send to hardware.
    def run(self):
        if not self.on:
            return 0

        # Soft-start reference ramping
        if self.num_corrections > 0:
            self.r_u = self.r * ((self.total_num_corrections - self.num_corrections) / self.total_num_corrections)
            self.num_corrections -= 1
        else:
            self.r_u = self.r

        # Get current measured value
        self.x_h = self.sensor.get_data()

        # Error update
        self.prev_error = self.e
        self.e = self.r_u - self.x_h

        # Compute timing delta in seconds
        time_delta = self.sensor.dt / 1e6
        if self.first_run:
            time_delta = 0.020  # Approx. first-cycle assumption

        # Integral and derivative updates
        if self.Ki:
            self.integral += self.e * time_delta
        if self.Kd:
            self.derivative = (self.e - self.prev_error) / time_delta

        # Battery droop compensation
        if self.battery is not None:
            Kdroop = 1 / self.battery.get_cur_perc()
        else:
            Kdroop = 1

        # Controller output
        self.a = Kdroop * (
            (self.Kp * self.e)
            + (self.integral * self.Ki)
            + (self.derivative * self.Kd)
            + ((self.r_u * self.Kff) + copysign(1, self.r) * self.PWM_start)
        )

        # Saturation
        self.astar = min(self.max_min, max(-self.max_min, self.a))

        # Anti-windup correction
        if self.Kw:
            self.integral -= (self.a - self.astar) * self.Kw

        return self.astar
