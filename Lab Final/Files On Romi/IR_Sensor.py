from pyb import Pin, ADC  # pyright: ignore
from Sensor import Sensor


class IRSensor(Sensor):
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

    # Return the current infrared sensor reading with zeroing applied.
    def get_data(self) -> float:
        return (self.ADC.read() - self.white) / (self.black - self.white)

    def read(self):
        return self.ADC.read()
