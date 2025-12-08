from pyb import Pin, ADC  # pyright: ignore
from Sensor import Sensor

HardwarePinName = int


class IRSensor(Sensor):
    def __init__(self, id: int, ADC_pin: HardwarePinName):
        super().__init__()
        self.id = id
        self.location = (self.id - 7) * 8  # location in mm from center. ASSUMES 13 IS RIGHT WHICH IS POSITIVE
        self.ADC_pin = Pin(ADC_pin, mode=Pin.ANALOG)
        self.ADC = ADC(self.ADC_pin)
        self.white = 0
        self.black = 1

    def get_data(self) -> float:
        """Return the current infrared sensor reading with zeroing applied."""
        return (self.ADC.read() - self.white) / (self.black - self.white)
    
    def read(self):
        return self.ADC.read()
