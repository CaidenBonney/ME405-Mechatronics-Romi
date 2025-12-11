from Encoder import Encoder
from time import ticks_us, ticks_diff  # pyright: ignore
from Closed_Loop_Control import ClosedLoopControl


class MotorController:
    time_start = None

    def __init__(self, motor, encoder, battery, side: bool, duration: int = 0):
        self.motor = motor
        self.encoder = encoder
        self.side = side  # False = left, True = right

        # Zeroing Encoders and Enabling Motors
        self.motor.enable()
        self.encoder.zero()

        self.queues_were_full = False
        self.duration = duration  # duration of test in seconds where 0 = infinite

        # Internal flag to indicate when the current test is complete
        self.done = False

        self.CLC = ClosedLoopControl(
            sensor=self.encoder,
            max_min=100,
            Kp=Encoder.Kp,
            Ki=Encoder.Ki,
            Kd=Encoder.Kd,
            Kw=Encoder.Kw,
            Kff=Encoder.Kff,
            PWM_start=Encoder.PWM_startr if self.side else Encoder.PWM_startl,
            battery=battery,
        )

    def run(self, shares):
        # Seperating the shares
        (
            flag_s,
            speed_s,
            data_transfer_s,
            test_complete_s,
            seg_start_s,
            time_q,
            pos_q,
            vel_q,
        ) = shares

        def test_complete():
            self.done = True
            test_complete_s.put(1)
            data_transfer_s.put(0)
            # self.motor.set_effort(0)

        self.start = ticks_us()

        while True:
            if seg_start_s.get():
                seg_start_s.put(seg_start_s.get() - 1)  # decrement seg_start_s because we have two motor controllers
                self.test_start = ticks_us()
                test_complete_s.put(0)
                self.queues_were_full = False
                self.done = False

            # All motor control code goes here in the needed states
            if flag_s.get():
                flag_s.put(0)
                if speed_s.get() == 0:
                    # Speed is zero so disable the closed loop control
                    self.CLC.reset()
                    self.CLC.disable()
                    self.motor.set_effort(0)
                else:
                    # Speed is not zero so enable the closed loop control
                    self.CLC.enable()
                self.CLC.set_ref(speed_s.get())

            # Updating encoders before next loop
            self.encoder.update()

            # first check if test is already complete
            if not test_complete_s.get():
                # change effort by running closed loop control if it is on
                if self.CLC.on:
                    self.CLC.gain_update(
                        Kp=Encoder.Kp,
                        Ki=Encoder.Ki,
                        Kd=Encoder.Kd,
                        Kw=Encoder.Kw,
                        Kff=Encoder.Kff,
                        PWM_start=Encoder.PWM_startr if self.side else Encoder.PWM_startl,
                    )
                    self.motor.set_effort(self.CLC.run())

                # if data transfer is on, then put data into queues
                if data_transfer_s.get():

                    # if no time limit -> always run ; if time limit -> check if less than duration
                    if self.duration <= 0 or ticks_diff(ticks_us(), self.test_start) <= self.duration * 1e6:

                        # if queues are currently full, then set test complete, stop motors and zero encoders
                        if time_q.full() or pos_q.full() or vel_q.full():
                            self.queues_were_full = True
                            test_complete()

                        # if queues are not full and have not been full within the current test, then put data into queues
                        if not self.queues_were_full:
                            time_q.put(ticks_diff(ticks_us(), self.test_start))  #  [us]
                            pos_q.put(self.encoder.position)  #                     [rad]
                            vel_q.put(self.encoder.velocity)  #                     [mm/s]
                    else:
                        # if time limit is reached, then set test complete, stop motors and zero encoders
                        test_complete()
            elif not self.done:
                # if test is not complete, but we are not done then set test to complete
                test_complete()
            yield
