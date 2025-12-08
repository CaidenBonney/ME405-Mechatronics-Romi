max_spd: int = 150 * 60  # Maximum expected speed of the encoder in RPM
PPR: int = 12  # Pulses per revolution of the encoder (counting both edges of both channels)
ar = 0xFFFF
min_f_update = (max_spd * PPR) / ((ar + 1) / 2)

print(f"Min_F: {min_f_update}")
print(f"Period: {(1/min_f_update)*1e6}")
print((1/75)*1e6)

