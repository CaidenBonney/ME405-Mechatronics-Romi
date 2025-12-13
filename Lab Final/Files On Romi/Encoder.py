## @file Encoder.py
#  This file implements a quadrature encoder interface for the Romi robot.
#  The encoder uses a hardware timer in @c Timer.ENC_AB mode to measure
#  wheel rotation and compute velocity. The class inherits from @c Sensor
#  so timing and velocity updates integrate cleanly with the controllers.
#
#  Features:
#  - Reads incremental wheel motion from a hardware timer
#  - Converts tick differences into wheel radians
#  - Accumulates total wheel displacement
#  - Computes velocity in mm/s (using @c dt from Sensor)
#  - Supports resetting via @c zero()
#
#  @authors
#      Antonio Ventimiglia
#      Caiden Bonney
#  @date 2025-Dec-11

from pyb import Pin, Timer  # pyright: ignore
from Sensor import Sensor
from Romi_Props import RomiProps


## A quadrature encoder class for position and velocity measurement.
#
#  This class configures a timer in encoder mode and provides wheel
#  position (radians) and velocity (mm/s). It is compatible with the
#  closed-loop control system through the @c get_data() interface.
class Encoder(Sensor):

    # Default Gains for Motor Control
    Kp = 0.05
    Ki = 0.05
    Kd = 0
    Kw = 0.10
    Kff = 0.075

    PWM_startl = 5.5
    PWM_startr = 5.5

    ## Initialize an encoder.
    #
    #  @param motor      The motor this encoder belongs to
    #  @param tim        pyb.Timer configured for encoder mode
    #  @param chA_pin    Pin for channel A input
    #  @param chA_num    Timer channel number for channel A
    #  @param chB_pin    Pin for channel B input
    #  @param chB_num    Timer channel number for channel B
    def __init__(self, motor, tim: Timer, chA_pin, chA_num: int, chB_pin, chB_num: int):
        super().__init__()

        self.motor = motor
        self._position = 0  # Accumulated wheel position (rad)
        self.prev_count = 0  # Last timer count
        self.delta = 0  # Tick difference between updates

        self.ar = tim.period()  # Auto-reload value
        self.tim = tim

        # Configure pins and timer channels
        self.chA_pin = Pin(chA_pin, mode=Pin.IN)
        self.chB_pin = Pin(chB_pin, mode=Pin.IN)
        self.chA = tim.channel(chA_num, pin=self.chA_pin, mode=Timer.ENC_AB)
        self.chB = tim.channel(chB_num, pin=self.chB_pin, mode=Timer.ENC_AB)

    ## Get the current wheel position.
    #
    #  @return Wheel angle in radians (signed).
    @property
    def position(self) -> float:
        return -self._position

    ## Set the current wheel position.
    #
    #  @param value  New wheel position in radians.
    @position.setter
    def position(self, value: float) -> None:
        self._position = -value

    ## Get the current wheel velocity.
    #
    #  Returns velocity computed using most recent tick delta and @c dt.
    #
    #  @return Wheel velocity in mm/s.
    @property
    def velocity(self) -> float:
        if self.delta == 0 or self.dt == 0:
            return 0
        return -(self.delta * RomiProps.wheel_radius * 1e6) / self.dt

    ## Update encoder position and velocity.
    #
    #  - Reads timer counter
    #  - Computes tick difference with wrap-around handling
    #  - Converts ticks to radians
    #  - Updates total position
    #  - Updates @c dt for timing
    def update(self) -> None:
        count = self.tim.counter()
        self.delta = count - self.prev_count
        self.prev_count = count

        # Wrap-around correction
        half_range = (self.ar + 1) / 2
        if self.delta < -half_range:
            self.delta += self.ar + 1
        if self.delta > half_range:
            self.delta -= self.ar + 1

        # Convert to radians and accumulate
        self.delta *= RomiProps.ticks_to_rads
        self._position += self.delta

        # Update timing from Sensor base class
        self.update_dt()

    ## Return the most recent velocity measurement.
    #
    #  Allows this object to be used as a generic sensor in controllers.
    #
    #  @return Velocity in mm/s.
    def get_data(self) -> float:
        return self.velocity

    ## Reset encoder displacement and timing.
    #
    #  Useful when synchronizing encoder readings with heading or odometry.
    def zero(self) -> None:
        self._position = 0
        self.prev_count = self.tim.counter()
        self.delta = 0
        self.reset_timing()
