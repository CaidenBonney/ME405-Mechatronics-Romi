from pyb import ADC, Pin, Timer, ExtInt
from array import array
from time import sleep_ms

# Initializing variables
data = array("H", (0 for _ in range(1000)))
idx = 0
done = False
waiting = False
button = False


# Function definition
def tim_cb(tim):
    global data, idx, done
    if idx >= 1000:
        tim7.callback(None)  # disable the callback
        done = True
    else:
        data[idx] = adc.read()
        idx = idx + 1


def button_pressed(pin=None):
    global button
    button = True


def start_step():
    global done, waiting, idx
    if not waiting:
        # Print headers
        print("idx, val")

        # Ensure pin starts low
        PC1.low()
        sleep_ms(5000)  # allow pin to settle

        # Reset index and done flag
        idx = 0
        done = False

        # Start timer callback first
        tim7.callback(tim_cb)

        # Take one sample before stepping high
        data[0] = adc.read()
        idx = 1

        # Step pin high
        PC1.high()

        wait_and_print()


def wait_and_print():
    global data, idx, done, waiting, button
    waiting = True
    # Wait for data collection
    while not done:
        pass

    # Print data collected
    for i, val in enumerate(data):
        print(f"{i}, {val}")
    waiting = False
    button = False


# defining Power Pin
PC1 = Pin(Pin.cpu.C1, mode=Pin.OUT_PP)  # Power
PC1.low()

# defining Output Pin
PC0 = Pin(Pin.cpu.C0, mode=Pin.ANALOG)  # Output
adc = pyb.ADC(PC0)

# define timer
tim7 = Timer(7, freq=500)  # create the timer object
tim7.callback(tim_cb)  # assign the callback

# Creating button
button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, button_pressed)

while True:
    if button:
        start_step()
