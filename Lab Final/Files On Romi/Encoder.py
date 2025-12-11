from pyb import Pin, Timer  # pyright: ignore

from Sensor import Sensor
from Romi_Props import RomiProps


# A quadrature encoder decoding interface encapsulated in a Python class
class Encoder(Sensor):

    # Motor Gains
    Kp = 0.05  #                            Proportional gain
    Ki = 0.05  #                           Integral gain
    Kd = 0  #                              Derivative gain
    Kw = 0.10  #                           Windup gain
    Kff = 0.075  #                         Feed forward gain
    PWM_startl = 5.5  #                    Feed forward gain at start regardless of reference speed Calced: 2.49
    PWM_startr = 5.5  #                    Feed forward gain at start regardless of reference speed Calced: -2.17

    # Initializes an Encoder object
    def __init__(self, motor, tim: Timer, chA_pin, chA_num: int, chB_pin, chB_num: int):
        super().__init__()
        self.motor = motor
        self._position = 0  # Total accumulated position of the encoder
        self.prev_count = 0  # Counter value from the most recent update
        self.delta = 0  # Change in count between last two updates

        self.ar = tim.period()  # default is 16 bit for all timers except Tim 2&3 on our board
        self.tim = tim
        self.chA_pin = Pin(chA_pin, mode=Pin.IN)
        self.chB_pin = Pin(chB_pin, mode=Pin.IN)
        self.chA = tim.channel(chA_num, pin=self.chA_pin, mode=Timer.ENC_AB)
        self.chB = tim.channel(chB_num, pin=self.chB_pin, mode=Timer.ENC_AB)

    # Returns the most recently updated value of position as determined
    # within the update() method
    @property
    def position(self) -> float:
        return -self._position

    # Sets the position of the encoder
    @position.setter
    def position(self, value: float) -> None:
        self._position = -value

    # Returns a measure of velocity [mm/s] using the the most recently updated
    # value of delta as determined within the update() method
    @property
    def velocity(self) -> float:
        if self.delta == 0 or self.dt == 0:
            return 0
        return -(self.delta * RomiProps.wheel_radius * 1e6) / self.dt

    # Runs one update step on the encoder's timer counter to keep
    # track of the change in count and check for counter reload
    def update(self) -> None:
        self.delta = self.tim.counter() - self.prev_count
        self.prev_count = self.tim.counter()

        # Overflow
        if self.delta < -(self.ar + 1) / 2:
            self.delta += self.ar + 1
        # Underflow
        if self.delta > (self.ar + 1) / 2:
            self.delta -= self.ar + 1

        # convert from ticks to wheel radians
        self.delta = self.delta * RomiProps.ticks_to_rads

        # update values
        self._position += self.delta

        self.update_dt()

    # Return the latest velocity measurement from the encoder.
    # "override" parent class method
    def get_data(self) -> float:
        return self.velocity

    # Sets the present encoder position to zero and causes future updates
    # to measure with respect to the new zero position
    def zero(self) -> None:
        self._position = 0  # Total accumulated position of the encoder
        self.prev_count = self.tim.counter()  # Counter value from the most recent update
        self.delta = 0  # Change in count between last two updates
        self.reset_timing()
