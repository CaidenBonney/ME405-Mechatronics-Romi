from pyb import ADC, Pin, Timer, ExtInt
from array import array
from time import sleep_ms

# defining data storage
data = array('H', 1000 * [0])
idx = 0
done = False
print("idx, val")

# Function definition
def tim_cb(tim):
    global data, idx, done
    if idx >= 1000:
        tim7.callback(None)  # disable the callback
        done = True
    else:
        data[idx] = adc.read()
        idx = idx + 1

# defining Output Pin
PC0 = Pin(Pin.cpu.C0, mode=Pin.ANALOG)  # Output
adc = pyb.ADC(PC0)

# defining Power Pin
PC1 = Pin(Pin.cpu.C1, mode=Pin.OUT_PP, value=0)  # Power
sleep_ms(5000)

# define timer
tim7 = Timer(7, freq=500)  # create the timer object
tim7.callback(tim_cb)  # assign the callback

# Creating button
# button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, #put function here))

# setting Power pin to high setting
sleep_ms(2)
PC1.high()

# Wait for data collection
while not done:
    pass

# Print data collected
for i, val in enumerate(data):
    print(f"{i}, {val}")
