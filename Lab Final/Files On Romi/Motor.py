## @file Motor.py
#  Motor driver interface for DRV8838-style drivers on the Romi chassis.
#  Provides enable/disable control and effort (PWM/direction) setting.
#  @author Antonio Ventimiglia
#  @author Caiden Bonney
#  @date   2025-Dec-12
#  @copyright GPLv3

from pyb import Pin, Timer  # pyright: ignore


## Motor driver abstraction using PWM and direction pins.
class Motor:

    ## Initialize a motor driver.
    #
    #  @param PWM_pin    PWM pin used for speed control
    #  @param DIR_pin    Direction pin
    #  @param nSLP_pin   Sleep pin (active low)
    #  @param tim        Timer configured for PWM output
    #  @param PWM_ch_num Timer channel number for PWM
    def __init__(self, PWM_pin, DIR_pin, nSLP_pin, tim: Timer, PWM_ch_num: int):
        self.nSLP_pin = Pin(nSLP_pin, mode=Pin.OUT_PP)
        self.PWM_pin = Pin(PWM_pin, mode=Pin.ANALOG)
        self.DIR_pin = Pin(DIR_pin, mode=Pin.OUT_PP)
        self.PWM_ch = tim.channel(PWM_ch_num, pin=self.PWM_pin, mode=Timer.PWM)
        if self.PWM_ch is None:
            raise ValueError("Invalid PWM channel number for the given timer")
        self.disable()  # Start with motor disabled

    ## Set motor effort as a percentage (-100..100).
    def set_effort(self, value: float):

        if not -100 <= value <= 100:
            raise ValueError("Effort must be between -100 and 100")

        # print(f"{value}")
        # Set the direction based on the sign of the effort
        self.DIR_pin.low() if value >= 0 else self.DIR_pin.high()

        # Set the PWM duty cycle to the absolute value of the effort
        self.PWM_ch.pulse_width_percent(abs(value))

    ## Enable the motor driver (wakes from sleep).
    def enable(self):
        self.nSLP_pin.high()
        self.PWM_pin.low()

    ## Disable the motor driver (sleep mode).
    def disable(self):
        self.nSLP_pin.low()
        self.PWM_pin.low()
