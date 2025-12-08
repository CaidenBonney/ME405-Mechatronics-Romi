from pyb import Pin  # pyright: ignore
from motor import Motor
from encoder import Encoder
from time import ticks_us, ticks_diff  # pyright: ignore
import task_share
from Closed_Loop_Control import ClosedLoopControl


class MotorController:

    time_start = None

    def __init__(self, motor: Motor, encoder: Encoder, battery_ADC: Pin, side: bool, duration: int = 2) -> None:
        self.motor = motor
        self.encoder = encoder
        self.side = side  # False = left, True = right

        # Zeroing Encoders and Enabling Motors
        self.motor.enable()
        self.encoder.zero()

        self.queues_were_full = False
        self.duration = duration

        self.CLC = ClosedLoopControl(self.encoder, self.motor, battery_ADC, self.side)

        # Flag to indicate when the test is complete
        self.done = False

    def zero(self):
        self.encoder.zero()
        self.CLC.reset()

    def test_complete(self, test_complete_s):
        self.done = True
        test_complete_s.put(1)
        self.motor.set_effort(0)
        self.zero()

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
                self.CLC.set_ref(speed_s.get())
                self.test_start = ticks_us()
                self.queues_were_full = False
                self.done = False
                # note that you could zero the encoder here if you wanted to

            # Updating encoders before next loop
            self.encoder.update()


            if not test_complete_s.get():
                # Change effort according to Closed Loop Control
                self.CLC.run()
                if data_transfer_s.get():
                    if ticks_diff(ticks_us(), self.test_start) <= self.duration * 1e6:
                        if time_q.full() or pos_q.full() or vel_q.full():
                            self.queues_were_full = True
                            # Test Complete Stop Motors
                            self.test_complete(test_complete_s)
                        if not self.queues_were_full:
                            time_q.put(ticks_diff(ticks_us(), self.test_start))  #  [us]
                            pos_q.put(self.encoder.position)  #                     [rad]
                            vel_q.put(int(self.encoder.velocity))  #                [mm/s]
                    else:
                        self.test_complete(test_complete_s)
            elif not self.done:
                self.test_complete(test_complete_s)
            yield
