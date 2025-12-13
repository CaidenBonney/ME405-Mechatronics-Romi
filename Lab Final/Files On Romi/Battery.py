## @file Battery.py
#  Helper class for monitoring the battery voltage on the Romi robot.
#  Reads an ADC channel and converts the raw reading into an estimated
#  battery percentage and voltage using nominal calibration values.
#
#  The class wraps a @c pyb.ADC instance and uses calibrated nominal
#  readings to estimate actual battery values.
#
#  @author Antonio Ventimiglia
#  @author Caiden Bonney
#  @date   2025-Dec-12
#  @copyright GPLv3

from pyb import ADC  # pyright: ignore


## A class which provides access to Romi's battery voltage.
#
#  This class reads from an ADC channel connected to the battery
#  measurement circuit on the Romi. It converts the raw ADC reading
#  into a normalized percentage of charge and an estimated voltage
#  using nominal calibration constants.
class Battery:
    ## Create a battery monitor object.
    #
    #  Optionally an ADC channel may be passed in; if not, it must be
    #  assigned later before readings are taken.
    #
    #  @param battery_ADC Optional ADC object wired to the battery monitor.
    def __init__(self, battery_ADC: ADC | None = None):
        self.battery_ADC = battery_ADC

        ## Nominal ADC reading corresponding to nominal battery voltage.
        self.ADC_nom = 3210

        ## Nominal battery voltage corresponding to @c ADC_nom.
        self.V_nom = 8.5

    ## Get the current battery percentage.
    #
    #  Reads the ADC and returns the ratio of the current reading to the
    #  nominal calibrated value. The returned number is a decimal between
    #  approximately 0 and 1. Raises @c ValueError if no ADC is configured.
    #
    #  @return A float representing battery charge percentage (0..1).
    def get_cur_perc(self):
        if self.battery_ADC is None:
            raise ValueError("Battery ADC is not set")
        return self.battery_ADC.read() / self.ADC_nom

    ## Get the current battery voltage.
    #
    #  This method scales the normalized battery percentage by the
    #  nominal calibrated voltage to produce an estimated real voltage.
    #
    #  @return The estimated battery voltage in volts.
    def get_cur_volt(self):
        if self.battery_ADC is None:
            raise ValueError("Battery ADC is not set")
        return self.get_cur_perc() * self.V_nom
