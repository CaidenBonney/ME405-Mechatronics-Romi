from pyb import UART  # pyright: ignore
import task_share


class UserInput:
    def __init__(self):
        self.cmd_queue = []
        self.uart = UART(5, 115200)

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
            # Add user input to queue
            self.poll()
            # Gets most recent command
            cmd = self.get_cmd()
            # If command is valid do command
            if cmd:
                print(cmd)
                # reduce WASD speeds by a factor of 4 for easier control during testing
                scale = 2

                # Controlling with W, A, S, D
                if cmd == "w":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(50 // scale)
                    r_speed_s.put(50 // scale)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "a":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(0)
                    r_speed_s.put(50 // scale)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "s":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(-50 // scale)
                    r_speed_s.put(-50 // scale)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "d":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(50 // scale)
                    r_speed_s.put(0)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)

                # G for Go
                elif cmd == "g":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(100 // scale)
                    r_speed_s.put(100 // scale)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)

                # C for Cancel
                elif cmd == "c":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(0)
                    r_speed_s.put(0)
                    data_transfer_s.put(0)
                    test_complete_s.put(1)
                    self.uart.write(b"Test Data Transfer Complete\r\n")

                # Tests follow
                elif cmd == "1":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(10)
                    r_speed_s.put(10)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "2":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(20)
                    r_speed_s.put(20)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "3":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(30)
                    r_speed_s.put(30)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "4":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(40)
                    r_speed_s.put(40)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "5":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(50)
                    r_speed_s.put(50)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "6":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(60)
                    r_speed_s.put(60)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "7":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(70)
                    r_speed_s.put(70)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "8":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(80)
                    r_speed_s.put(80)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "9":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(90)
                    r_speed_s.put(90)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
                elif cmd == "0":
                    l_flag_s.put(1)
                    r_flag_s.put(1)
                    l_speed_s.put(100)
                    r_speed_s.put(100)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)
            yield
