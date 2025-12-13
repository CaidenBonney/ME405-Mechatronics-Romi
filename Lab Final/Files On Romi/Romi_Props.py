from math import pi


class RomiProps:
    # We measured the wheel diameter with a caliper to be 69.5 mm
    wheel_radius = 34.75  # mm
    trackwidth = 141  # mm
    wdiv2 = trackwidth / 2  # mm

    gear_ratio = 119.7576  # shaft revolution / wheel revolution
    counts_per_rev = 12  # ticks / shaft revolution

    ticks_to_rads = (2 * pi) / (counts_per_rev * gear_ratio)

