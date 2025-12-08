from pyb import UART, Pin  # pyright: ignore
from machine import soft_reset  # pyright: ignore
import task_share
from Closed_Loop_Control import ClosedLoopControl

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

    def get_next_five_char(self) -> str:
        """Returns the next five characters as a string and blocks until all five characters are received."""
        while len(self.cmd_queue) < 5:
            self.poll()
        value = ""
        for _ in range(5):
            char = self.get_cmd()
            if char is not None:
                value += char
        return value

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
        ],
    ):
        l_flag_s, r_flag_s, l_speed_s, r_speed_s, data_transfer_s, test_complete_s = shares
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

                # Controlling with W, A, S, D
                if cmd == "w":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(50)
                    r_speed_s.put(50)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "a":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(0)
                    r_speed_s.put(50)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "s":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(-50)
                    r_speed_s.put(-50)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "d":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(50)
                    r_speed_s.put(0)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                # R for Rotate in place
                elif cmd == "r":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(50)
                    r_speed_s.put(-50)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)

                # G for Go
                elif cmd == "g":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(100)
                    r_speed_s.put(100)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)

                # C for Cancel
                elif cmd == "c":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(0)
                    r_speed_s.put(0)
                    data_transfer_s.put(1)
                    test_complete_s.put(1)

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

                elif cmd == "p":
                    value = float(self.get_next_five_char())
                    ClosedLoopControl.set_gain("Kp", value)
                    print("Kp:", ClosedLoopControl.Kp)
                    self.uart.write(f"p{ClosedLoopControl.Kp}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "i":
                    value = float(self.get_next_five_char())
                    ClosedLoopControl.set_gain("Ki", value)
                    print("Ki:", ClosedLoopControl.Ki)
                    self.uart.write(f"i{ClosedLoopControl.Ki}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "o":
                    value = float(self.get_next_five_char())
                    ClosedLoopControl.set_gain("Kw", value)
                    print("Kw:", ClosedLoopControl.Kw)
                    self.uart.write(f"o{ClosedLoopControl.Kw}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "k":
                    value = float(self.get_next_five_char())
                    ClosedLoopControl.set_gain("Kffl", value)
                    print("Kffl:", ClosedLoopControl.Kffl)
                    self.uart.write(f"k{ClosedLoopControl.Kffl}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "l":
                    value = float(self.get_next_five_char())
                    ClosedLoopControl.set_gain("Kffr", value)
                    print("Kffr:", ClosedLoopControl.Kffr)
                    self.uart.write(f"l{ClosedLoopControl.Kffr}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "h":
                    value = float(self.get_next_five_char())
                    ClosedLoopControl.set_gain("ff_startl", value)
                    print("ff_startl:", ClosedLoopControl.ff_startl)
                    self.uart.write(f"h{ClosedLoopControl.ff_startl}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
                elif cmd == "j":
                    value = float(self.get_next_five_char())
                    ClosedLoopControl.set_gain("ff_startr", value)
                    print("ff_startr:", ClosedLoopControl.ff_startr)
                    self.uart.write(f"j{ClosedLoopControl.ff_startr}\r\n".encode("utf-8"))
                    self.uart.write(b"Test Data Transfer Complete\r\n")
                    self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
            yield
