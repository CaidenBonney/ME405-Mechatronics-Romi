from pyb import Pin  # pyright: ignore
from Motor import Motor
from Encoder import Encoder
from time import ticks_us, ticks_diff  # pyright: ignore
import task_share
from Closed_Loop_Control import ClosedLoopControl


class MotorController:
    @classmethod
    def set_gain(cls, K="motor_Kp", value=0.0):
        """Multiply a class-level gain by the given factor."""
        if not hasattr(cls, K):
            raise AttributeError(f"'{K}' is not a defined gain on {cls.__name__}")
        setattr(cls, K, value)

    time_start = None

    # Motor Gains
    motor_Kp = 0.10  #                       Proportional gain
    motor_Ki = 1.00  #                       Integral gain
    motor_Kd = 0  #                          Derivative gain
    motor_Kw = 1.00  #                       Windup gain
    motor_Kff = 0.10  #                      Feed forward gain
    motor_PWM_startl = 2.49  #               Feed forward gain at start regardless of reference speed Calced: 2.49
    motor_PWM_startr = -2.17  #              Feed forward gain at start regardless of reference speed Calced: -2.17

    def __init__(self, motor: Motor, encoder: Encoder, battery_ADC: Pin, side: bool, duration: int = 0) -> None:
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
            self.encoder,
            max_min=100,
            Kp=MotorController.motor_Kp,
            Ki=MotorController.motor_Ki,
            Kd=MotorController.motor_Kd,
            Kw=MotorController.motor_Kw,
            Kff=MotorController.motor_Kff,
            PWM_start=MotorController.motor_PWM_startr if self.side else MotorController.motor_PWM_startl,
            battery_ADC=battery_ADC,
        )

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
            task_share.Share,  # seg_start_s
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
            seg_start_s,
            time_q,
            pos_q,
            vel_q,
        ) = shares

        self.start = ticks_us()

        while True:
            # All motor control code goes here in the needed states
            if flag_s.get():
                flag_s.put(0)
                if speed_s.get() == 0:
                    # Speed is zero so disable the closed loop control
                    self.CLC.disable()
                    self.motor.set_effort(0)
                else:
                    # Speed is not zero so enable the closed loop control
                    if not self.CLC.on:
                        self.CLC.enable()
                self.CLC.set_ref(speed_s.get())

            # Updating encoders before next loop
            self.encoder.update()

            if seg_start_s.get():
                self.zero()
                seg_start_s.put(seg_start_s.get() - 1)  # decrement seg_start_s because we have two motor controllers
                self.test_start = ticks_us()
                self.queues_were_full = False
                self.done = False

            # first check if test is already complete
            if not test_complete_s.get():
                # change effort by running closed loop control if it is on
                if self.CLC.on:
                    self.CLC.gain_update(
                        Kp=MotorController.motor_Kp,
                        Ki=MotorController.motor_Ki,
                        Kd=MotorController.motor_Kd,
                        Kw=MotorController.motor_Kw,
                        Kff=MotorController.motor_Kff,
                        PWM_start=MotorController.motor_PWM_startr if self.side else MotorController.motor_PWM_startl,
                    )
                    self.motor.set_effort(self.CLC.run())

                # if data transfer is on, then put data into queues
                if data_transfer_s.get():

                    # if no time limit -> always run ; if time limit -> check if less than duration
                    if self.duration <= 0 or ticks_diff(ticks_us(), self.test_start) <= self.duration * 1e6:

                        # if queues are currently full, then set test complete, stop motors and zero encoders
                        if time_q.full() or pos_q.full() or vel_q.full():
                            self.queues_were_full = True
                            self.test_complete(test_complete_s)

                        # if queues are not full and have not been full within the current test, then put data into queues
                        if not self.queues_were_full:
                            time_q.put(ticks_diff(ticks_us(), self.test_start))  #  [us]
                            pos_q.put(self.encoder.position)  #                     [rad]
                            vel_q.put(int(self.encoder.velocity))  #                [mm/s]
                    else:
                        # if time limit is reached, then set test complete, stop motors and zero encoders
                        self.test_complete(test_complete_s)
            elif not self.done:
                # if test is not complete, but we are not done then set test to complete
                self.test_complete(test_complete_s)
            yield
