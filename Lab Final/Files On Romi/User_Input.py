## @file User_Input.py
#  Bluetooth/serial command handler for Romi. Polls UART for user commands,
#  decodes configuration changes, calibration triggers, and direct state
#  transitions, then writes results into shared variables for other tasks.
#  This task is cooperative and yields after each loop iteration.
#
#  @author Antonio Ventimiglia
#  @author Caiden Bonney
#  @date   2025-Dec-12
#  @copyright GPLv3

from pyb import UART  # pyright: ignore
from machine import soft_reset  # pyright: ignore
from Path_Director_vars import PD_vars

# Import CLC Sensor Classes
from Encoder import Encoder
from Line_Sensor import LineSensor
from IMU import IMU


## Reads user commands over UART/Bluetooth and updates shared state.
#
#  Provides helpers for pulling fixed-length numeric inputs, changing sensor
#  attributes, and relaying calibration or gain updates to the rest of the
#  system through @c task_share queues.
class UserInput:
    ## Construct a user input handler.
    #
    #  Sets up UART, stores the reset button pin and battery reference, and
    #  clears any buffered characters.
    #
    #  @param button_pin Pin object used to trigger a soft reset
    #  @param battery Battery object for reporting/adjustment hooks
    def __init__(self, button_pin, battery):
        self.cmd_queue = []
        self.uart = UART(5, 115200)
        self.battery = battery

        self.button_pin = button_pin

        # Flush the buffer
        while self.uart.any():
            self.uart.read()

    ## Non-blocking read of raw bytes from UART into the queue.
    #
    #  Filters out carriage-return and line-feed; leaves other characters
    #  untouched.
    def poll(self):

        while self.uart.any():
            char_in = self.uart.read(1).decode()  # pyright: ignore
            self.cmd_queue.append(char_in)

    ## Check if any command is queued.
    #
    #  @return @c True if there is at least one queued character
    def has_cmd(self):
        return bool(self.cmd_queue)

    ## Return next command as a 1-character string, or @c None if none.
    def get_cmd(self):
        if not self.cmd_queue:
            return None
        return self.cmd_queue.pop(0)

    ## Change a sensor class attribute using the next numeric token.
    #
    #  @param sens Sensor class whose attribute is being modified
    #  @param att_str Attribute name to set (e.g., @c Kp, @c Ki)
    def change_attribute(self, sens, att_str):
        value = self.get_next_n_char(5)
        sens.set_attr(att_str, value)

    ## Fetch the next @p n characters (blocking until available).
    #
    #  Converts the retrieved token to @c float and retries on invalid input.
    #
    #  @param n Number of characters to read
    #  @return Parsed floating-point value
    def get_next_n_char(self, n) -> float:
        while len(self.cmd_queue) < n:
            self.poll()
        value = ""
        for _ in range(n):
            char = self.get_cmd()
            if char is not None:
                value += char
        try:
            return float(value)
        except ValueError:
            print("Invalid value try again")
            self.uart.write(b"Invalid value try again\r\n")
            return self.get_next_n_char(n)

    ## Clear any buffered commands.
    def drain(self):
        self.cmd_queue.clear()

    ## Generator task that processes user commands and updates shares.
    #
    #  Handles calibration triggers, segment selection, gain changes, and
    #  reference speed updates. Also supports soft reset via button press.
    #
    #  @param shares Tuple of @c task_share variables for motor flags, speeds,
    #                and calibration/status signaling
    def run(self, shares):
        (
            l_flag_s,
            r_flag_s,
            l_speed_s,
            r_speed_s,
            data_transfer_s,
            test_complete_s,
            cal_white_s,
            cal_black_s,
            set_seg_s,
        ) = shares
        data_transfer_s.put(0)
        self.uart.write(f"Bluetooth Connection Established\r\n".encode("utf-8"))
        while True:
            if self.button_pin.value() == 0:
                soft_reset()

            # Add user input to queue
            self.poll()
            # Gets most recent command
            cmd = self.get_cmd()
            if cmd is not None:
                cmd = cmd.lower()
            # If command is valid do command
            if cmd:
                print(cmd)
                self.uart.write(f"{cmd}\r\n".encode("utf-8"))

                # Calibration commands
                if cmd == "v":
                    # Calibrate White
                    set_seg_s.put(1)
                    cal_white_s.put(1)
                    test_complete_s.put(1)

                if cmd == "b":
                    # Calibrate Black
                    set_seg_s.put(1)
                    cal_black_s.put(1)
                    test_complete_s.put(1)

                # C for Cancel
                elif cmd == "c":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(0)
                    r_speed_s.put(0)
                    test_complete_s.put(1)
                    set_seg_s.put(0)

                elif cmd == ".":
                    self.uart.write(f"Set State (3 chr):".encode("utf-8"))
                    value = int(self.get_next_n_char(3))
                    self.uart.write(f"{value}\r\n".encode("utf-8"))

                    func_id = value // 100

                    if func_id >= 1:
                        PD_vars.next_state = 0
                        # self.uart.write(f"Next State (3 chr):".encode("utf-8"))
                        # PD_vars.var_1 = int(self.get_next_n_char(3))
                        self.uart.write(f"Var1 (5 chr):".encode("utf-8"))
                        PD_vars.var_1 = self.get_next_n_char(5)
                        self.uart.write(f"{PD_vars.var_1}\r\n".encode("utf-8"))
                    if func_id >= 2:
                        self.uart.write(f"Var2 (5 chr):".encode("utf-8"))
                        PD_vars.var_2 = self.get_next_n_char(5)
                        self.uart.write(f"{PD_vars.var_2}\r\n".encode("utf-8"))
                    if func_id >= 3:
                        self.uart.write(f"Var3 (5 chr):".encode("utf-8"))
                        PD_vars.var_3 = self.get_next_n_char(5)
                        self.uart.write(f"{PD_vars.var_3}\r\n".encode("utf-8"))
                    if func_id >= 4:
                        self.uart.write(f"Var4 (5 chr):".encode("utf-8"))
                        PD_vars.var_4 = self.get_next_n_char(5)
                        self.uart.write(f"{PD_vars.var_4}\r\n".encode("utf-8"))
                    set_seg_s.put(value)

                # Gain Control
                elif cmd == "p":
                    which_CLC = self.get_next_n_char(1)
                    self.uart.write(f"{which_CLC}\r\n".encode("utf-8"))
                    if which_CLC == "1":
                        sens = Encoder
                    elif which_CLC == "2":
                        sens = LineSensor
                    else:
                        sens = IMU
                    self.change_attribute(sens, "Kp")
                elif cmd == "i":
                    which_CLC = self.get_next_n_char(1)
                    if which_CLC == "1":
                        sens = Encoder
                    elif which_CLC == "2":
                        sens = LineSensor
                    else:
                        sens = IMU
                    self.change_attribute(sens, "Ki")
                elif cmd == "o":
                    which_CLC = self.get_next_n_char(1)
                    if which_CLC == "1":
                        sens = Encoder
                    elif which_CLC == "2":
                        sens = LineSensor
                    else:
                        sens = IMU
                    self.change_attribute(sens, "Kd")
                elif cmd == "k":
                    sens = Encoder
                    self.change_attribute(sens, "Kff")
                elif cmd == "h":
                    sens = Encoder
                    self.change_attribute(sens, "turn_correctionl")
                elif cmd == "j":
                    sens = Encoder
                    self.change_attribute(sens, "turn_correctionr")

                # Reference Speed
                elif cmd == "z":
                    value = self.get_next_n_char(5)
                    PD_vars.v_ref = value
                    print("v_ref:", PD_vars.v_ref)
                    self.uart.write(f"v_ref:{PD_vars.v_ref}\r\n".encode("utf-8"))
            yield
