from pyb import UART, Pin  # pyright: ignore
from machine import soft_reset  # pyright: ignore
import task_share
from Path_Director import PathDirector
from os import remove, listdir
from Battery import Battery

# Import CLC Sensor Classes
from Encoder import Encoder
from Line_Sensor import LineSensor
from IMU import IMU


class UserInput:
    def __init__(self, button_pin: Pin, battery: Battery):
        self.cmd_queue = []
        self.uart = UART(5, 115200)
        self.battery = battery

        self.button_pin = button_pin

        # Flush the buffer
        while self.uart.any():
            self.uart.read()

    # Non-blocking read of raw bytes.filter out CR and LF (\r, \n)
    # No other whitespace filtering
    # No lowercase→uppercase conversion
    # Enqueues only bytes present in self.valid.
    # Returns: number of bytes pulled this call (including filtered CR/LF).
    def poll(self):

        while self.uart.any():
            char_in = self.uart.read(1).decode()  # pyright: ignore
            self.cmd_queue.append(char_in)

    def has_cmd(self):
        return bool(self.cmd_queue)

    # Return next command as a 1-char string, or None if none.
    def get_cmd(self):
        if not self.cmd_queue:
            return None
        return self.cmd_queue.pop(0)

    def change_attribute(self, sens, att_str):
        value = self.get_next_n_char(5)
        sens.set_attr(att_str, value)

    # Returns the next n characters as a string and blocks until all n characters are received.
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

    def drain(self):
        self.cmd_queue.clear()

    def run(
        self,
        shares: tuple[
            task_share.Share,  # l_flag_s
            task_share.Share,  # r_flag_s
            task_share.Share,  # l_speed_s
            task_share.Share,  # r_speed_s
            task_share.Share,  # data_transfer_s
            task_share.Share,  # test_complete_s
            task_share.Share,  # cal_white_s
            task_share.Share,  # cal_black_s
            task_share.Share,  # new_seg_s
        ],
    ):
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

                # Controlling with W, A, S, D
                if cmd == "w":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(100)
                    r_speed_s.put(100)
                    test_complete_s.put(0)
                elif cmd == "a":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(0)
                    r_speed_s.put(100)
                    test_complete_s.put(0)
                elif cmd == "s":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(-100)
                    r_speed_s.put(-100)
                    test_complete_s.put(0)
                elif cmd == "d":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(100)
                    r_speed_s.put(0)
                    test_complete_s.put(0)

                # R for Rotate in place
                elif cmd == "r":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(100)
                    r_speed_s.put(-100)
                    test_complete_s.put(0)
                elif cmd == "t":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(-100)
                    r_speed_s.put(100)
                    test_complete_s.put(0)

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
                        PathDirector.next_state = 0
                        # self.uart.write(f"Next State (3 chr):".encode("utf-8"))
                        # PathDirector.var_1 = int(self.get_next_n_char(3))
                        self.uart.write(f"Var1 (5 chr):".encode("utf-8"))
                        PathDirector.var_1 = self.get_next_n_char(5)
                        self.uart.write(f"{PathDirector.var_1}\r\n".encode("utf-8"))
                    if func_id >= 2:
                        self.uart.write(f"Var2 (5 chr):".encode("utf-8"))
                        PathDirector.var_2 = self.get_next_n_char(5)
                        self.uart.write(f"{PathDirector.var_2}\r\n".encode("utf-8"))
                    if func_id >= 3:
                        self.uart.write(f"Var3 (5 chr):".encode("utf-8"))
                        PathDirector.var_3 = self.get_next_n_char(5)
                        self.uart.write(f"{PathDirector.var_3}\r\n".encode("utf-8"))

                    set_seg_s.put(value)

                elif cmd == "%":
                    self.uart.write(
                        f"%:{self.battery.get_cur_perc()}, V:{self.battery.get_cur_volt()}\r\n".encode("utf-8")
                    )

                elif cmd == "x":
                    if "IR_cal.txt" in listdir():
                        remove("IR_cal.txt")
                        print("Calibration data removed")
                    set_seg_s.put(1)

                # Motor gains
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

                elif cmd == "z":
                    value = self.get_next_n_char(5)
                    PathDirector.set_attr("v_ref", value)
                    attribute = PathDirector.v_ref
                    print("v_ref:", attribute)
                    self.uart.write(f"z{attribute}\r\n".encode("utf-8"))
            yield
