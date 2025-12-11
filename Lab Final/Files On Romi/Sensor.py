from time import ticks_diff, ticks_us  # pyright: ignore
from pyb import UART  # pyright: ignore

uart = UART(5, 115200)


# Base class for sensors that exposes a common interface.
class Sensor:
    # Multiply a class-level gain by the given factor.
    @classmethod
    def set_attr(cls, K="Kp", value=0.0):
        if not hasattr(cls, K):
            raise AttributeError(f"'{K}' is not a defined gain on {cls.__name__}")
        setattr(cls, K, value)
        print(K, value)
        uart.write(f"{K}{value}\r\n".encode("utf-8"))

    def __init__(self) -> None:
        self._prev_t = ticks_us()
        self.dt = 0

    # Return the most recent scalar measurement for the sensor.
    def get_data(self) -> float:
        raise NotImplementedError("Sensor subclasses must implement get_data().")

    # Refresh the stored time differential for subclasses.
    def update_dt(self) -> None:
        now = ticks_us()
        self.dt = ticks_diff(now, self._prev_t)
        self._prev_t = now

    # Reset the timing history used for dt calculations.
    def reset_timing(self) -> None:
        self._prev_t = ticks_us()
        self.dt = 0
