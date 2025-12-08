from math import e
from pyb import UART, Pin  # pyright: ignore
from machine import soft_reset  # pyright: ignore
import task_share
from Motor_Controller import MotorController
from Line_Sensor import LineSensor
from os import remove, listdir


class UserInput:
    def __init__(self, button_pin: Pin):
        self.cmd_queue = []
        self.uart = UART(5, 115200)

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
            cur_seg_s,
        ) = shares
        data_transfer_s.put(0)
        cur_seg_s.put(0)
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
                    cur_seg_s.put(1)
                    cal_white_s.put(1)
                    data_transfer_s.put(1)
                    test_complete_s.put(1)

                if cmd == "b":
                    # Calibrate Black
                    cur_seg_s.put(1)
                    cal_black_s.put(1)
                    data_transfer_s.put(1)
                    test_complete_s.put(1)

                # Controlling with W, A, S, D
                if cmd == "w":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(50)
                    r_speed_s.put(50)
                elif cmd == "a":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(0)
                    r_speed_s.put(50)
                elif cmd == "s":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(-50)
                    r_speed_s.put(-50)
                elif cmd == "d":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(50)
                    r_speed_s.put(0)

                # R for Rotate in place
                elif cmd == "r":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(50)
                    r_speed_s.put(-50)

                # C for Cancel
                elif cmd == "c":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(0)
                    r_speed_s.put(0)
                    test_complete_s.put(1)
                    cur_seg_s.put(0)

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
                    cur_seg_s.put(value)

                elif cmd == "x":
                    if "IR_cal.txt" in listdir():
                        remove("IR_cal.txt")
                        print("Calibration data removed")
                    cur_seg_s.put(1)

                # Motor gains
                elif cmd == "p":
                    value = self.get_next_n_char(5)
                    MotorController.set_gain("motor_Kp", value)
                    print("Kp:", MotorController.motor_Kp)
                    self.uart.write(f"p{MotorController.motor_Kp}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "i":
                    value = self.get_next_n_char(5)
                    MotorController.set_gain("motor_Ki", value)
                    print("Ki:", MotorController.motor_Ki)
                    self.uart.write(f"i{MotorController.motor_Ki}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "o":
                    value = self.get_next_n_char(5)
                    MotorController.set_gain("motor_Kw", value)
                    print("Kw:", MotorController.motor_Kw)
                    self.uart.write(f"o{MotorController.motor_Kw}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "k":
                    value = self.get_next_n_char(5)
                    MotorController.set_gain("motor_Kffl", value)
                    print("Kffl:", MotorController.motor_Kff)
                    self.uart.write(f"k{MotorController.motor_Kff}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "h":
                    value = self.get_next_n_char(5)
                    MotorController.set_gain("motor_PWM_startl", value)
                    print("ff_startl:", MotorController.motor_PWM_startl)
                    self.uart.write(f"h{MotorController.motor_PWM_startl}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "j":
                    value = self.get_next_n_char(5)
                    MotorController.set_gain("motor_PWM_startr", value)
                    print("ff_startr:", MotorController.motor_PWM_startr)
                    self.uart.write(f"j{MotorController.motor_PWM_startr}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                
                # Line sensor gains
                elif cmd == "u":
                    value = self.get_next_n_char(5)
                    LineSensor.set_gain("sensor_Kp", value)
                    print("Kp:", LineSensor.sensor_Kp)
                    self.uart.write(f"u{LineSensor.sensor_Kp}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                # Line sensor gains
                elif cmd == "y":
                    value = self.get_next_n_char(5)
                    LineSensor.set_gain("sensor_Ki", value)
                    print("Ki:", LineSensor.sensor_Ki)
                    self.uart.write(f"y{LineSensor.sensor_Ki}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                # Line sensor gains
                elif cmd == "t":
                    value = self.get_next_n_char(5)
                    LineSensor.set_gain("sensor_Kd", value)
                    print("Kd:", LineSensor.sensor_Kd)
                    self.uart.write(f"t{LineSensor.sensor_Kd}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
            yield
