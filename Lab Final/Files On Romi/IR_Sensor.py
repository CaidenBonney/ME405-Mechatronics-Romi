## @file IR_Sensor.py
#  Infrared reflectance sensor wrapper used by the line sensor array.
#  Provides normalized readings using stored white/black calibration values.
#  @author Antonio Ventimiglia
#  @author Caiden Bonney
#  @date   2025-Dec-12
#  @copyright GPLv3

from pyb import Pin, ADC  # pyright: ignore
from Sensor import Sensor


## Single IR reflectance sensor with calibration offsets.
class IRSensor(Sensor):
    ## Initialize an IR sensor.
    #
    #  @param id       Position index (1..13) in the sensor bar
    #  @param ADC_pin  Pin connected to the analog reflectance output
    def __init__(self, id: int, ADC_pin):
        super().__init__()
        # Sensor number from 1 to 13. ASSUMES 13 IS RIGHT MOST SENSOR
        self.id = id
        # location in mm from line sensor center. ASSUMES 13 IS RIGHT WHICH IS POSITIVE
        self.location = (self.id - 7) * 8
        self.ADC_pin = Pin(ADC_pin, mode=Pin.ANALOG)
        self.ADC = ADC(self.ADC_pin)
        self.white = 0
        self.black = 1

    ## Return the current infrared sensor reading with zeroing applied.
    def get_data(self) -> float:
        return (self.ADC.read() - self.white) / (self.black - self.white)

    ## Raw ADC reading (uncalibrated).
    def read(self):
        return self.ADC.read()
