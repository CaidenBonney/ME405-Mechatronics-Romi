## @file Romi_Props.py
#  Physical properties and conversion constants for the Romi drivetrain.
#  Provides wheel radius, trackwidth, gear ratio, encoder counts, and a
#  conversion from encoder ticks to radians.
#
#  @author Antonio Ventimiglia
#  @author Caiden Bonney
#  @date   2025-Dec-12
#  @copyright GPLv3

from math import pi


## Holds static geometric and encoder properties for Romi.
class RomiProps:
    ## Wheel radius in millimeters (diameter measured at 69.5 mm).
    wheel_radius = 34.75  # mm
    ## Trackwidth in millimeters.
    trackwidth = 141  # mm
    ## Half of trackwidth, used for differential drive calculations.
    wdiv2 = trackwidth / 2  # mm

    ## Gear ratio: shaft revolutions per wheel revolution.
    gear_ratio = 119.7576  # shaft revolution / wheel revolution
    ## Encoder counts per motor shaft revolution.
    counts_per_rev = 12  # ticks / shaft revolution

    ## Conversion from encoder ticks to radians of wheel rotation.
    ticks_to_rads = (2 * pi) / (counts_per_rev * gear_ratio)
