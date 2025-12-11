from pyb import Pin, Timer  # pyright: ignore


# A motor driver interface encapsulated in a Python class. Works with
# motor drivers using separate PWM and direction inputs such as the DRV8838
# drivers present on the Romi chassis from Pololu.
class Motor:

    # Initializes a Motor object
    def __init__(self, PWM_pin, DIR_pin, nSLP_pin, tim: Timer, PWM_ch_num: int):
        self.nSLP_pin = Pin(nSLP_pin, mode=Pin.OUT_PP)
        self.PWM_pin = Pin(PWM_pin, mode=Pin.ANALOG)
        self.DIR_pin = Pin(DIR_pin, mode=Pin.OUT_PP)
        self.PWM_ch = tim.channel(PWM_ch_num, pin=self.PWM_pin, mode=Timer.PWM)
        if self.PWM_ch is None:
            raise ValueError("Invalid PWM channel number for the given timer")
        self.disable()  # Start with motor disabled

    # Sets the present effort requested from the motor based on an input value
    # between -100 and 100
    def set_effort(self, value: float):

        if not -100 <= value <= 100:
            raise ValueError("Effort must be between -100 and 100")

        # print(f"{value}")
        # Set the direction based on the sign of the effort
        self.DIR_pin.low() if value >= 0 else self.DIR_pin.high()

        # Set the PWM duty cycle to the absolute value of the effort
        self.PWM_ch.pulse_width_percent(abs(value))

    # Enables the motor driver by taking it out of sleep mode into brake mode
    def enable(self):
        self.nSLP_pin.high()
        self.PWM_pin.low()

    # Disables the motor driver by taking it into sleep mode
    def disable(self):
        self.nSLP_pin.low()
        self.PWM_pin.low()
