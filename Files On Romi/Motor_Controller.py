from motor import Motor
from encoder import Encoder
from time import ticks_us, ticks_diff  # pyright: ignore
import task_share


class MotorController:

    time_start = None

    def __init__(self, motor: Motor, encoder: Encoder, duration: int = 2) -> None:
        self.motor = motor
        self.encoder = encoder

        # Zeroing Encoders and Enabling Motors
        self.motor.enable()
        self.encoder.zero()

        # # PID controller data
        # self.kp = 1.0  # Proportional gain
        # self.ki = 0.0  # Integral gain
        # self.kd = 0.0  # Derivative gain
        # self.integral = 0.0
        # self.prev_error = 0.0

        self.queues_were_full = False
        self.duration = duration

    def zero(self):
        self.encoder.zero()
        # self.integral = 0.0
        # self.prev_error = 0.0

    def run(
        self,
        shares: tuple[
            task_share.Share,  # flag_s
            task_share.Share,  # speed_s
            task_share.Share,  # data_transfer_s
            task_share.Share,  # test_complete_s
            task_share.Queue,  # time_q
            task_share.Queue,  # pos_q
            task_share.Queue,  # vel_q
        ],
    ):
        # Seperating the shares
        (
            flag_s,
            speed_s,
            data_transfer_s,
            test_complete_s,
            time_q,
            pos_q,
            vel_q,
        ) = shares

        self.start = ticks_us()

        while True:
            # All motor control code goes here in the needed states
            if flag_s.get():
                flag_s.put(0)
                self.motor.set_effort(speed_s.get())
                self.test_start = ticks_us()
                self.queues_were_full = False
                # note that you could zero the encoder here if you wanted to

            # Updating encoders before next loop
            self.encoder.update()
            if data_transfer_s.get():
                if ticks_diff(ticks_us(), self.test_start) <= self.duration * 1e6:
                    if time_q.full() or pos_q.full() or vel_q.full():
                        self.queues_were_full = True
                        # Test Complete Stop Motors
                        self.motor.set_effort(0)
                    if not self.queues_were_full:
                        time_q.put(ticks_diff(ticks_us(), self.test_start))
                        pos_q.put(self.encoder.position)
                        vel_q.put(int(self.encoder.velocity * 1e6))
                else:
                    test_complete_s.put(1)
                    self.motor.set_effort(0)
                    self.zero()
            yield
