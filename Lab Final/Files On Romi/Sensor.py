## @file Sensor.py
#  Base sensor interface for Romi. Provides common timing utilities and
#  a class-level helper to adjust gains on subclasses.
#
#  @author Antonio Ventimiglia
#  @author Caiden Bonney
#  @date   2025-Dec-12
#  @copyright GPLv3

from time import ticks_diff, ticks_us  # pyright: ignore
from pyb import UART  # pyright: ignore

uart = UART(5, 115200)


## Base class for sensors that exposes a common interface.
#
#  Subclasses must implement @c get_data(). Timing helpers keep track of
#  delta-t between measurements for control algorithms.
class Sensor:
    ## Set a class-level gain attribute.
    #
    #  Used to tune controller gains stored on sensor classes.
    #
    #  @param K Name of the gain attribute (default @c Kp)
    #  @param value New value for the gain
    @classmethod
    def set_attr(cls, K="Kp", value=0.0):
        if not hasattr(cls, K):
            raise AttributeError(f"'{K}' is not a defined gain on {cls.__name__}")
        setattr(cls, K, value)
        print(K, value)
        uart.write(f"{K}{value}\r\n".encode("utf-8"))

    ## Initialize timing variables for dt computation.
    def __init__(self) -> None:
        self._prev_t = ticks_us()
        self.dt = 0

    ## Return the most recent scalar measurement for the sensor.
    #
    #  Must be implemented by subclasses.
    def get_data(self) -> float:
        raise NotImplementedError("Sensor subclasses must implement get_data().")

    ## Refresh the stored time differential for subclasses.
    #
    #  Updates @c dt using current and previous @c ticks_us() values.
    def update_dt(self) -> None:
        now = ticks_us()
        self.dt = ticks_diff(now, self._prev_t)
        self._prev_t = now

    ## Reset the timing history used for dt calculations.
    def reset_timing(self) -> None:
        self._prev_t = ticks_us()
        self.dt = 0
