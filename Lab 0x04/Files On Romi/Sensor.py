from time import ticks_diff, ticks_us  # pyright: ignore

class Sensor():
    """Base class for sensors that exposes a common interface."""

    def __init__(self) -> None:
        self._prev_t = ticks_us()
        self._dt = 0

    def get_data(self) -> float:
        """Return the most recent scalar measurement for the sensor."""
        raise NotImplementedError("Sensor subclasses must implement get_data().")

    @property
    def dt(self) -> int:
        """Return the elapsed time [µs] between the last two updates."""
        return self._dt

    def _update_dt(self) -> None:
        """Refresh the stored time differential for subclasses."""
        now = ticks_us()
        self._dt = ticks_diff(now, self._prev_t)
        self._prev_t = now

    def _reset_timing(self) -> None:
        """Reset the timing history used for dt calculations."""
        self._prev_t = ticks_us()
        self._dt = 0
