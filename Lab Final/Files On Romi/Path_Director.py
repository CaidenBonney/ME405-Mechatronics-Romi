from pyb import UART, Pin, ExtInt  # pyright: ignore
from time import ticks_diff, ticks_us  # pyright: ignore
import task_share
from math import pi

from Romi_Props import RomiProps
from Closed_Loop_Control import ClosedLoopControl
from Line_Sensor import LineSensor
from IMU import IMU


class PathDirector:
    @classmethod
    def set_attr(cls, K="v_ref", value=0.0):
        """Multiply a class-level gain by the given factor."""
        if not hasattr(cls, K):
            raise AttributeError(f"'{K}' is not a defined gain on {cls.__name__}")
        setattr(cls, K, value)

    v_ref = 200  # mm/s reference speed

    WAIT = 0
    CALIBRATE = 1
    FOLLOW_LINE = 2
    FIXED_FORWARD = 3
    RECORD_HEADING = 4

    # var1 = X = 650 , var2 = Y = 0, var3 = next_state = FORCE_RIGHT_FORK
    # d1 = ----      , d2 = ----   , d3 = ----

    FOLLOW_LINE_B4_FORK = 5  # var1 = X = 650 , var2 = Y = 0, var3 = next_state = FORCE_RIGHT_FORK
    FORCE_RIGHT_FORK = 6
    LINE_FOLLOW_2_DIAMOND = 7

    # function states
    TURN_ANGLE = 21  # var1 = angle
    STRAIGHT_2_POINT = 22  # var1 = X , var2 = Y

    FOLLOW_LINE = 2  # var1 = X +- 10, var2 = Y +- 10, var3 = next_state

    var_1 = 0
    var_2 = 0

    def __init__(self, line_sensor: LineSensor, IMU_obj: IMU, bump_sensors: tuple):
        self.state = PathDirector.CALIBRATE  # this is overwritten on first run
        self.segment_set = False
        self.seg_start_time = 0  # ms

        self.line_sensor = line_sensor
        self.CLC = ClosedLoopControl(
            sensor=self.line_sensor,
            max_min=12,
            Kp=LineSensor.sensor_Kp,
            Ki=LineSensor.sensor_Ki,
            Kd=LineSensor.sensor_Kd,
        )
        self.CLC.disable()
        self.IMU = IMU_obj
        self.uart = UART(5, 115200)
        self.bumped = False  
            
        # Create bumper callbacks for all pins
        def bmp_callback_handler(pin: Pin):
            def bumper_callback(line):
                if pin.value() != 0:
                    self.bumped = True

            return bumper_callback

        for bump_sensor in bump_sensors:
            ExtInt(bump_sensor, ExtInt.IRQ_FALLING, Pin.PULL_UP, bmp_callback_handler(bump_sensor))

    def run(
        self,
        shares: tuple[
            task_share.Share,  # l_flag_s
            task_share.Share,  # l_speed_s
            task_share.Share,  # r_flag_s
            task_share.Share,  # r_speed_s
            task_share.Share,  # data_transfer_s
            task_share.Share,  # test_complete_s
            task_share.Share,  # white_cal_s
            task_share.Share,  # black_cal_s
            task_share.Share,  # set_seg_s
            task_share.Queue,  # seg_start_s
            task_share.Share,  # obsd_lpos_s
            task_share.Share,  # obsd_rpos_s
            task_share.Share,  # obsd_cpos_s
            task_share.Share,  # obsd_yaw_s
            task_share.Share,  # obsd_yawrate_s
            task_share.Share,  # obsd_X_s
            task_share.Share,  # obsd_Y_s
        ],
    ):
        # Seperating the shares
        (
            l_flag_s,
            l_speed_s,
            r_flag_s,
            r_speed_s,
            data_transfer_s,
            test_complete_s,
            white_cal_s,
            black_cal_s,
            set_seg_s,
            seg_start_s,
            obsd_lpos_s,
            obsd_rpos_s,
            obsd_cpos_s,
            obsd_yaw_s,
            obsd_yawrate_s,
            obsd_X_s,
            obsd_Y_s,
        ) = shares

        def set_state(new_state: int):
            self.state = new_state
            set_seg_s.put(new_state)
            self.segment_set = False

        set_state(PathDirector.CALIBRATE)

        while True:
            if self.bumped:
                self.bumped = False
                set_state(PathDirector.WAIT)

            if set_seg_s.get() != self.state:
                set_state(set_seg_s.get())

            if self.CLC:
                self.CLC.gain_update(
                    Kp=LineSensor.sensor_Kp,
                    Ki=LineSensor.sensor_Ki,
                    Kd=LineSensor.sensor_Kd,
                )

            if self.state == PathDirector.WAIT:
                if not self.segment_set:
                    l_flag_s.put(1)
                    l_speed_s.put(0)
                    r_flag_s.put(1)
                    r_speed_s.put(0)
                    self.segment_set = True

                    print("Starting Waiting Segment")
                # Waiting Segment

            elif self.state == PathDirector.CALIBRATE:
                # Calibration Segment
                if not self.segment_set:
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    self.segment_set = True
                    print("Starting Calibration Segment")

                if white_cal_s.get() == 1:
                    self.line_sensor.cal_white = True
                    white_cal_s.put(0)

                if black_cal_s.get() == 1:
                    self.line_sensor.cal_black = True
                    black_cal_s.put(0)

                if not self.line_sensor.calibrated and self.line_sensor.calibrate():
                    print("Line Sensor Calibration Complete")
                    self.CLC.reset()

                if not self.IMU.calibrated and self.IMU.calibrate():
                    print("IMU Calibration Complete")

                if self.line_sensor.calibrated and self.IMU.calibrated:
                    # both calibrations complete
                    # return back to waiting state
                    set_state(PathDirector.WAIT)

            elif self.state == PathDirector.FOLLOW_LINE:
                # Line Following Segment
                if not self.segment_set:  # ran once per segment
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    self.segment_set = True
                    print("Starting Line Following Segment")

                    # Linesensor CLC centroid reference
                    self.CLC.enable()
                    self.CLC.set_ref(0)

                    # motor settings
                    l_flag_s.put(1)
                    l_speed_s.put(self.v_ref)
                    r_flag_s.put(1)
                    r_speed_s.put(self.v_ref)
                    # data_transfer_s.put(1)  # limits time to 500 queues worth of data
                    test_complete_s.put(0)

                yaw_rate = self.CLC.run()
                v_adjust = yaw_rate * RomiProps.wdiv2
                l_flag_s.put(1)
                l_speed_s.put(PathDirector.v_ref - v_adjust)
                r_flag_s.put(1)
                r_speed_s.put(PathDirector.v_ref + v_adjust)

                # self.uart.write(
                #     f"obsd_cpos: {obsd_cpos_s.get()}, IMU heading: {self.IMU.heading}, obsd_X_pos: {obsd_X_s.get()}, obsd_Y_pos: {obsd_Y_s.get()}\r\n".encode(
                #         "utf-8"
                #     )
                # )

            elif self.state == PathDirector.FIXED_FORWARD:
                # Fixed Forward Distance Segment
                if not self.segment_set:  # ran once per segment
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    self.segment_set = True
                    print("Fixed Forward Distance Segment")

                    # motor settings
                    l_flag_s.put(1)
                    l_speed_s.put(self.v_ref)
                    r_flag_s.put(1)
                    r_speed_s.put(self.v_ref)
                    # data_transfer_s.put(1)
                    test_complete_s.put(0)

                distance = 500  # mm

                if obsd_cpos_s.get() >= distance:
                    self.uart.write(f"L: {obsd_lpos_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"R: {obsd_rpos_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"C: {obsd_cpos_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"H: {obsd_yaw_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"X: {obsd_X_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"Y: {obsd_Y_s.get()}\r\n".encode("utf-8"))

                    l_flag_s.put(1)
                    l_speed_s.put(0)
                    r_flag_s.put(1)
                    r_speed_s.put(0)
                    test_complete_s.put(1)

                    set_state(PathDirector.WAIT)

            elif self.state == PathDirector.RECORD_HEADING:
                # Record Heading Segment
                if not self.segment_set:  # ran once per segment
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    self.segment_set = True
                    print("Record Heading Segment")

                self.uart.write(
                    f"obsd_heading: {obsd_yaw_s.get()}, heading: {self.IMU.heading}, IMU_heading: {self.IMU.get_imu_heading()}\r\n".encode(
                        "utf-8"
                    )
                )

            elif self.state == PathDirector.TURN_ANGLE:
                # Turn Arc Segment
                # var1 = angle
                if not self.segment_set:  # ran once per segment
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    self.segment_set = True
                    print("Turn Angle Segment")

                    # degrees to rads
                    self.var_1 = self.var_1 * pi / 180
                    self.var_2 = self.IMU.heading + self.var_1

                    # Set CLC to IMU heading
                    self.CLC = ClosedLoopControl(
                        sensor=self.IMU,
                        max_min=12,
                        Kp=IMU.Kp,
                        Ki=IMU.Ki,
                        Kd=IMU.Kd,
                    )
                    self.CLC.enable()
                    self.CLC.set_ref(self.var_2)

                radian_interval = 3 / 180 * pi  # degrees in radians
                if self.IMU.heading <= self.var_2 + radian_interval and self.IMU.heading > self.var_2 - radian_interval:
                    pass

                yaw_rate = self.CLC.run()
                v_adjust = yaw_rate * RomiProps.wdiv2
                l_flag_s.put(1)
                l_speed_s.put(PathDirector.v_ref - v_adjust)
                r_flag_s.put(1)
                r_speed_s.put(PathDirector.v_ref + v_adjust)

            elif self.state == PathDirector.STRAIGHT_2_POINT:
                # Using current heading and position, move to another point
                if not self.segment_set:  # ran once per segment
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    self.segment_set = True
                    print("Straight 2 Point Segment")

            elif self.state == PathDirector.FOLLOW_LINE_B4_FORK:
                # Line Following Segment until fork start
                if not self.segment_set:  # ran once per segment
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    self.segment_set = True
                    print("Starting Line Following B4 Fork Segment")

                    # Linesensor CLC centroid reference
                    self.CLC.enable()
                    self.CLC.set_ref(0)

                    # motor settings
                    l_flag_s.put(1)
                    l_speed_s.put(self.v_ref)
                    r_flag_s.put(1)
                    r_speed_s.put(self.v_ref)
                    # data_transfer_s.put(1)  # limits time to 500 queues worth of data
                    test_complete_s.put(0)

                if obsd_X_s.get() >= 650:
                    self.CLC.disable()
                    set_state(PathDirector.FORCE_RIGHT_FORK)

                yaw_rate = self.CLC.run()
                v_adjust = yaw_rate * RomiProps.wdiv2
                l_flag_s.put(1)
                l_speed_s.put(PathDirector.v_ref - v_adjust)
                r_flag_s.put(1)
                r_speed_s.put(PathDirector.v_ref + v_adjust)

            elif self.state == PathDirector.FORCE_RIGHT_FORK:
                # Force Right Fork
                if not self.segment_set:  # ran once per segment
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    self.seg_start_time = ticks_us()
                    seg_start_s.put(2)
                    self.segment_set = True
                    print("Starting Force Right Fork Segment")

                    # Linesensor CLC centroid reference
                    self.CLC.set_ref(0)

                    # motor settings
                    l_flag_s.put(1)
                    l_speed_s.put(self.v_ref + 150)
                    r_flag_s.put(1)
                    r_speed_s.put(self.v_ref)
                    # data_transfer_s.put(1)  # limits time to 500 queues worth of data
                    test_complete_s.put(0)

                if ticks_diff(ticks_us(), self.seg_start_time) >= 500_000:
                    set_state(PathDirector.LINE_FOLLOW_2_DIAMOND)

            elif self.state == PathDirector.LINE_FOLLOW_2_DIAMOND:
                # Follow To diamond
                if not self.segment_set:  # ran once per segment
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    self.segment_set = True
                    print("Starting Line Following 2 Diamond Segment")

                    # Linesensor CLC centroid reference
                    self.CLC.set_ref(0)

                    # motor settings
                    l_flag_s.put(1)
                    l_speed_s.put(self.v_ref)
                    r_flag_s.put(1)
                    r_speed_s.put(self.v_ref)
                    # data_transfer_s.put(1)  # limits time to 500 queues worth of data
                    test_complete_s.put(0)

                yaw_rate = self.CLC.run()
                v_adjust = yaw_rate * RomiProps.wdiv2
                l_flag_s.put(1)
                l_speed_s.put(PathDirector.v_ref - v_adjust)
                r_flag_s.put(1)
                r_speed_s.put(PathDirector.v_ref + v_adjust)

            yield self.state

