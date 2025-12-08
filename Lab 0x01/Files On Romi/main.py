from pyb import Pin, Timer, ExtInt  # pyright: ignore[reportMissingModuleSource]
from motor import Motor
from encoder import Encoder
from time import ticks_us, ticks_diff, ticks_add, sleep_ms, sleep  # Use to get dt value in update()


class Encoder_updater:
    def __init__(self, list_encoder: list[Encoder]):
        self.interval = 13335  # time interval [us]
        start = ticks_us()  # time of 1st run
        self.deadline = ticks_add(start, self.interval)  # first run deadline
        self.list_encoder = list_encoder

    def update_loop(self):
        now = ticks_us()  # present time [us]
        if ticks_diff(self.deadline, now) <= 0:  # deadline elapsed
            for e in self.list_encoder:
                e.update()
            self.deadline = ticks_add(self.deadline, self.interval)  # prep next deadline


def wait(num_loops: int):
    for _ in range(num_loops):
        sleep_ms(13)
        updater.update_loop()
    print("waited")


def wait_and_print_info():
    wait(1000)
    print(f"right pos: {right_encoder.position}")
    print(f"left  pos: {left_encoder.position}")
    print(f"right vel: {right_encoder.velocity}")
    print(f"left  vel: {left_encoder.velocity}\n")


def button_pressed(pin=None):
    global button
    button = True


button = False

# Creating button
button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, button_pressed)

interval = 2_000_000

if __name__ == "__main__":
    # Motor Timer
    tim4 = Timer(4, freq=20_000)

    left_motor = Motor(Pin.cpu.B7, Pin.cpu.H1, Pin.cpu.H0, tim4, 2)
    tim2 = Timer(2, prescaler=0, period=0xFFFF)  # Left encoder counter
    left_encoder = Encoder(tim2, Pin.cpu.A0, 1, Pin.cpu.A1, 2)

    right_motor = Motor(Pin.cpu.B6, Pin.cpu.A7, Pin.cpu.A6, tim4, 1)
    tim3 = Timer(3, prescaler=0, period=0xFFFF)  # right encoder counter
    right_encoder = Encoder(tim3, Pin.cpu.B4, 3, Pin.cpu.B5, 4)

    updater = Encoder_updater([left_encoder, right_encoder])

    state = 1
    while True:
        if state == 1:
            # 1. Each motor can spin forward and backward independently with varying speed.
            print("*************************** Test 1 ***************************")
            print("Zeroing right and left encoders before enabling motors")
            right_encoder.zero()
            left_encoder.zero()
            wait_and_print_info()

            print("Enabling right and left motors")
            right_motor.enable()
            left_motor.enable()
            wait_and_print_info()

            print("left motor effort: 20    -  Right Spin: None      Left Spin: Forward")
            left_motor.set_effort(20)
            wait_and_print_info()

            print("right motor effort: -20  -  Right Spin: Reverse   Left Spin: Forward")
            right_motor.set_effort(-20)
            wait_and_print_info()

            print("left motor effort: -20   -  Right Spin: Reverse   Left Spin: Reverse")
            left_motor.set_effort(-20)
            wait_and_print_info()

            print("right motor effort: 20   -  Right Spin: Forward   Left Spin: Reverse")
            right_motor.set_effort(20)
            wait_and_print_info()

            print("left motor effort: 100   -  Right Spin: Forward   Left Spin: Forward")
            left_motor.set_effort(100)
            wait_and_print_info()

            print("right motor effort: 0    -  Right Spin: None      Left Spin: Forward")
            right_motor.set_effort(0)
            wait_and_print_info()

            print("Disabling both motors    -  Right Spin: None      Left Spin: None")
            left_motor.disable()
            right_motor.disable()
            wait_and_print_info()

        if button:
            left_motor.disable()
            right_motor.disable()
            button = False
            break

        state += 1


left_motor.disable()
right_motor.disable()
