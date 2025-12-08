from pyb import ADC  # pyright: ignore


class Battery:
    def __init__(self, battery_ADC: ADC | None = None):
        self.battery_ADC = battery_ADC
        self.ADC_nom = 3210  # Battery nominal ADC reading
        self.V_nom = 8.5  # Battery nominal voltage reading

    def get_cur_perc(self):
        if self.battery_ADC is None:
            raise ValueError("Battery ADC is not set")
        return self.battery_ADC.read() / self.ADC_nom

    def get_cur_volt(self):
        # Convert remaining battery percentage to volts
        if self.battery_ADC is None:
            raise ValueError("Battery ADC is not set")
        return self.get_cur_perc() * self.V_nom
