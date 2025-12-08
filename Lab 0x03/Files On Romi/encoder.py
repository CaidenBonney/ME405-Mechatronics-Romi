from pyb import Pin, Timer  # pyright: ignore
from time import ticks_us, ticks_diff  # pyright: ignore
from math import pi

HardwarePinName = int


class Encoder:
    """A quadrature encoder decoding interface encapsulated in a Python class"""

    gear_ratio = 119.7576  # shaft revolution / wheel revolution
    counts_per_rev = 12  # ticks / shaft revolution
    wheel_radius = 35  # mm

    def __init__(self, tim: Timer, chA_pin: HardwarePinName, chA_num: int, chB_pin: HardwarePinName, chB_num: int):
        """Initializes an Encoder object"""
        self._position = 0  # Total accumulated position of the encoder
        self.prev_count = 0  # Counter value from the most recent update
        self.delta = 0  # Change in count between last two updates
        self.prev_t = ticks_us()  # starting tick value
        self.dt = 0  # Amount of time between last two updates

        self.ar = tim.period()  # default is 16 bit for all timers except Tim 2&3 on our board
        self.tim = tim
        self.chA_pin = Pin(chA_pin, mode=Pin.IN)
        self.chB_pin = Pin(chB_pin, mode=Pin.IN)
        self.chA = tim.channel(chA_num, pin=self.chA_pin, mode=Timer.ENC_AB)
        self.chB = tim.channel(chB_num, pin=self.chB_pin, mode=Timer.ENC_AB)

    def update(self):
        """Runs one update step on the encoder's timer counter to keep
        track of the change in count and check for counter reload"""
        self.delta = self.tim.counter() - self.prev_count
        self.prev_count = self.tim.counter()

        # Overflow
        if self.delta < -(self.ar + 1) / 2:
            self.delta += self.ar + 1
        # Underflow
        if self.delta > (self.ar + 1) / 2:
            self.delta -= self.ar + 1

        # convert from ticks to wheel radians
        self.delta = self.delta * ((2 * pi) / (self.counts_per_rev * self.gear_ratio))

        # update values
        self._position += self.delta

        now = ticks_us()
        self.dt = ticks_diff(now, self.prev_t)
        self.prev_t = now

    @property
    def position(self):
        """Returns the most recently updated value of position as determined
        within the update() method"""
        return -self._position

    @position.setter
    def position(self, value: int):
        """Sets the position of the encoder"""
        self._position = -value

    @property
    def velocity(self):
        """Returns a measure of velocity [mm/s] using the the most recently updated
        value of delta as determined within the update() method"""
        if self.delta == 0:
            return 0
        return -(self.delta * self.wheel_radius * 1e6) / self.dt

    def zero(self):
        """Sets the present encoder position to zero and causes future updates
        to measure with respect to the new zero position"""
        self._position = 0  # Total accumulated position of the encoder
        self.prev_count = self.tim.counter()  # Counter value from the most recent update
        self.delta = 0  # Change in count between last two updates
        self.prev_t = ticks_us()  # starting tick value
        self.dt = 0  # Amount of time between last two updates
