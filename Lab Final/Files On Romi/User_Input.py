from math import e
from pyb import UART, Pin  # pyright: ignore
from machine import soft_reset  # pyright: ignore
import task_share
from Motor_Controller import MotorController
from Line_Sensor import LineSensor
from Path_Director import PathDirector
from os import remove, listdir
from Battery import Battery


class UserInput:
    def __init__(self, button_pin: Pin, battery: Battery):
        self.cmd_queue = []
        self.uart = UART(5, 115200)
        self.battery = battery

        self.button_pin = button_pin

        # Flush the buffer
        while self.uart.any():
            self.uart.read()

    def poll(self):
        """Non-blocking read of raw bytes.filter out CR and LF (\r, \n)
        No other whitespace filtering
        No lowercase→uppercase conversion
        Enqueues only bytes present in self.valid.
        Returns: number of bytes pulled this call (including filtered CR/LF)."""

        while self.uart.any():
            char_in = self.uart.read(1).decode()  # pyright: ignore
            self.cmd_queue.append(char_in)

    def has_cmd(self):
        return bool(self.cmd_queue)

    def get_cmd(self):
        """Return next command as a 1-char string, or None if none."""
        if not self.cmd_queue:
            return None
        return self.cmd_queue.pop(0)

    def get_next_n_char(self, n) -> float:
        """Returns the next n characters as a string and blocks until all n characters are received."""
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

                # C for Cancel
                elif cmd == "c":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(0)
                    r_speed_s.put(0)
                    test_complete_s.put(1)
                    set_seg_s.put(0)

                # Tests follow
                elif cmd == "1":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(85)
                    r_speed_s.put(85)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "2":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(170)
                    r_speed_s.put(170)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "3":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(255)
                    r_speed_s.put(255)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "4":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(340)
                    r_speed_s.put(340)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "5":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(425)
                    r_speed_s.put(425)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "6":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(510)
                    r_speed_s.put(510)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "7":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(595)
                    r_speed_s.put(595)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "8":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(680)
                    r_speed_s.put(680)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "9":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(765)
                    r_speed_s.put(765)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "0":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(850)
                    r_speed_s.put(850)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)

                elif cmd == ".":
                    value = int(self.get_next_n_char(1))
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
                    value = self.get_next_n_char(5)
                    MotorController.set_gain("motor_Kp", value)
                    attribute = MotorController.motor_Kp
                    print("Kp:", attribute)
                    self.uart.write(f"p{attribute}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "i":
                    value = self.get_next_n_char(5)
                    MotorController.set_gain("motor_Ki", value)
                    attribute = MotorController.motor_Ki
                    print("Ki:", attribute)
                    self.uart.write(f"i{attribute}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "o":
                    value = self.get_next_n_char(5)
                    MotorController.set_gain("motor_Kw", value)
                    attribute = MotorController.motor_Kw
                    print("Kw:", attribute)
                    self.uart.write(f"o{attribute}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "k":
                    value = self.get_next_n_char(5)
                    MotorController.set_gain("motor_Kffl", value)
                    attribute = MotorController.motor_Kff
                    print("Kffl:", attribute)
                    self.uart.write(f"k{attribute}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "h":
                    value = self.get_next_n_char(5)
                    MotorController.set_gain("motor_PWM_startl", value)
                    attribute = MotorController.motor_PWM_startl
                    print("ff_startl:", attribute)
                    self.uart.write(f"h{attribute}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "j":
                    value = self.get_next_n_char(5)
                    MotorController.set_gain("motor_PWM_startr", value)
                    attribute = MotorController.motor_PWM_startr
                    print("ff_startr:", attribute)
                    self.uart.write(f"j{attribute}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")

                # Line sensor gains
                elif cmd == "u":
                    value = self.get_next_n_char(5)
                    LineSensor.set_gain("sensor_Kp", value)
                    attribute = LineSensor.sensor_Kp
                    print("Kp:", attribute)
                    self.uart.write(f"u{attribute}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                # Line sensor gains
                elif cmd == "y":
                    value = self.get_next_n_char(5)
                    LineSensor.set_gain("sensor_Ki", value)
                    attribute = LineSensor.sensor_Ki
                    print("Ki:", attribute)
                    self.uart.write(f"y{attribute}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                # Line sensor gains
                elif cmd == "t":
                    value = self.get_next_n_char(5)
                    LineSensor.set_gain("sensor_Kd", value)
                    attribute = LineSensor.sensor_Kd
                    print("Kd:", attribute)
                    self.uart.write(f"t{attribute}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")

                elif cmd == "z":
                    value = self.get_next_n_char(5)
                    PathDirector.set_attr("v_ref", value)
                    attribute = PathDirector.v_ref
                    print("v_ref:", attribute)
                    self.uart.write(f"z{attribute}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
            yield
