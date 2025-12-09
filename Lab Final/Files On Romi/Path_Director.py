from pyb import UART, Pin, ExtInt  # pyright: ignore
from time import ticks_diff, ticks_us  # pyright: ignore
import task_share
from math import pi, atan2

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

    ST_B4_FOLLOW_LINE = 5
    FOLLOW_LINE_B4_FORK = 6
    FORCE_RIGHT_FORK = 7
    LINE_FOLLOW_2_DIAMOND = 8
    Diamond_2_CP1 = 9
    CP1_2_CP2 = 10
    CP2_2_Cup2 = 11
    Cup2_2_CP3 = 12
    CP3_2_CP4 = 13

    # function states: Third digit is the number of inputs required
    # One Input: 100-199
    TURN_ANGLE = 101  #       var1 = angle, var2 = -----, var3 = ----
    # Two Inputs: 200-299
    FOLLOW_LINE_2_POINT = 201  #      var1 = X    , var2 = Y    , var3 = ----
    GO_2_POINT = 202  #       var1 = X,   , var2 = Y,   , var3 = ----
    # Three Inputs: 300-399

    next_state: int = 0  # next state
    var_1: float = 0  # next state input
    var_2: float = 0
    var_3: float = 0

    def __init__(self, line_sensor: LineSensor, IMU_obj: IMU, bump_sensors: tuple):
        self.state = PathDirector.CALIBRATE  # this is overwritten on first run
        self.segment_set = False
        self.seg_start_time = 0

        self.line_sensor = line_sensor
        self.Line_CLC = ClosedLoopControl(
            sensor=self.line_sensor,
            max_min=12,
            Kp=LineSensor.Kp,
            Ki=LineSensor.Ki,
            Kd=LineSensor.Kd,
        )
        self.IMU = IMU_obj
        self.Heading_CLC = ClosedLoopControl(
            sensor=self.IMU,
            max_min=12,
            Kp=IMU.Kp,
            Ki=IMU.Ki,
            Kd=IMU.Kd,
        )

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

        # Extra variables used for fixed distance
        self.distance = 0  # mm
        self.end_point = 0  # mm

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

        if self.Line_CLC:
            self.Line_CLC.gain_update(
                Kp=LineSensor.Kp,
                Ki=LineSensor.Ki,
                Kd=LineSensor.Kd,
            )

        if self.Heading_CLC:
            self.Heading_CLC.gain_update(
                Kp=IMU.Kp,
                Ki=IMU.Ki,
                Kd=IMU.Kd,
            )

        def set_state(new_state: int):
            self.state = new_state
            set_seg_s.put(new_state)
            self.segment_set = False

            # tell motors previous segment is complete
            test_complete_s.put(1)

        set_state(PathDirector.CALIBRATE)

        while True:
            if self.bumped:
                self.bumped = False
                set_state(PathDirector.WAIT)

            if set_seg_s.get() != self.state:
                set_state(set_seg_s.get())

            if self.state == PathDirector.WAIT:
                if not self.segment_set:
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    l_flag_s.put(1)
                    l_speed_s.put(0)
                    r_flag_s.put(1)
                    r_speed_s.put(0)

                    print("Starting Waiting Segment")
                    # Waiting Segment
                    self.uart.write(f"L: {obsd_lpos_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"R: {obsd_rpos_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"C: {obsd_cpos_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"H: {obsd_yaw_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"X: {obsd_X_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"Y: {obsd_Y_s.get()}\r\n".encode("utf-8"))

            elif self.state == PathDirector.CALIBRATE:
                # Calibration Segment
                if not self.segment_set:
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    print("Starting Calibration Segment")

                if white_cal_s.get() == 1:
                    self.line_sensor.cal_white = True
                    white_cal_s.put(0)

                if black_cal_s.get() == 1:
                    self.line_sensor.cal_black = True
                    black_cal_s.put(0)

                if not self.line_sensor.calibrated and self.line_sensor.calibrate():
                    print("Line Sensor Calibration Complete")
                    self.Line_CLC.reset()

                if not self.IMU.calibrated and self.IMU.calibrate():
                    print("IMU Calibration Complete")

                if self.line_sensor.calibrated and self.IMU.calibrated:
                    # both calibrations complete
                    # return back to waiting state
                    set_state(PathDirector.WAIT)

            elif self.state == PathDirector.FOLLOW_LINE:
                # Line Following Segment
                if not self.segment_set:  # ran once per segment
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    print("Starting Line Following Segment")

                    # Linesensor CLC centroid reference
                    self.Line_CLC.set_ref(0)

                    # motor settings
                    l_flag_s.put(1)
                    l_speed_s.put(PathDirector.v_ref)
                    r_flag_s.put(1)
                    r_speed_s.put(PathDirector.v_ref)

                yaw_rate = self.Line_CLC.run()
                v_adjust = yaw_rate * RomiProps.wdiv2
                # print("adjust:", v_adjust)
                l_flag_s.put(1)
                l_speed_s.put(PathDirector.v_ref - v_adjust)
                r_flag_s.put(1)
                r_speed_s.put(PathDirector.v_ref + v_adjust)

                # self.uart.write(f"{obsd_X_s.get()},{obsd_Y_s.get()}\r\n".encode("utf-8"))

            elif self.state == PathDirector.FIXED_FORWARD:
                # Fixed Forward Distance Segment
                if not self.segment_set:  # ran once per segment
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    print("Fixed Forward Distance Segment")

                    self.distance = 1000  # mm
                    self.end_point = obsd_cpos_s.get() + self.distance  # mm

                    # motor settings
                    l_flag_s.put(1)
                    l_speed_s.put(PathDirector.v_ref)
                    r_flag_s.put(1)
                    r_speed_s.put(PathDirector.v_ref)

                # self.uart.write(f"{obsd_X_s.get()},{obsd_Y_s.get()}\r\n".encode("utf-8"))

                if obsd_cpos_s.get() >= self.end_point:
                    self.uart.write(f"L: {obsd_lpos_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"R: {obsd_rpos_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"C: {obsd_cpos_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"H: {obsd_yaw_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"X: {obsd_X_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"Y: {obsd_Y_s.get()}\r\n".encode("utf-8"))
                    set_state(PathDirector.WAIT)

            elif self.state == PathDirector.RECORD_HEADING:
                # Record Heading Segment
                if not self.segment_set:  # ran once per segment
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    print("Record Heading Segment")

                self.uart.write(
                    f"obsd_heading: {obsd_yaw_s.get()}, heading: {self.IMU.heading}, IMU_heading: {self.IMU.get_imu_heading()}\r\n".encode(
                        "utf-8"
                    )
                )
            # -------------------------------------------------------- Start of Functions -------------------------------------------------------- #
            elif self.state == PathDirector.TURN_ANGLE:
                # Turn In Place Segment
                # var1 = angle [deg], var2 = ----, var3 = ----
                if not self.segment_set:  # ran once per segment
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    print("Turn Angle Segment")

                    # degrees to rads
                    self.var_1 = self.var_1 * pi / 180  # radians to turn
                    self.var_2 = self.IMU.heading + self.var_1  # new desired heading

                    # Set IMU heading
                    self.Heading_CLC.set_ref(self.var_2)

                if abs(self.IMU.heading - self.var_2) <= 3 / 180 * pi:  # 3 deg tolerance
                    set_state(PathDirector.next_state)

                yaw_rate = self.Heading_CLC.run()
                v_adjust = yaw_rate * RomiProps.wdiv2
                l_flag_s.put(1)
                l_speed_s.put(PathDirector.v_ref - v_adjust)
                r_flag_s.put(1)
                r_speed_s.put(PathDirector.v_ref + v_adjust)

            elif self.state == PathDirector.GO_2_POINT:
                # Using current heading and position, move to another point
                # var1 = X, var2 = Y, var3 = -----
                if not self.segment_set:  # ran once per segment
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    print("Go 2 Point Segment")

                    # motor settings
                    l_flag_s.put(1)
                    l_speed_s.put(PathDirector.v_ref)
                    r_flag_s.put(1)
                    r_speed_s.put(PathDirector.v_ref)

                dY = PathDirector.var_2 - obsd_Y_s.get()
                dX = PathDirector.var_1 - obsd_X_s.get()

                if abs(dY) <= 25 and abs(dX) <= 25:
                    set_state(PathDirector.next_state)

                target_angle = atan2(dY, dX)
                diff = ((target_angle - self.IMU.heading + pi) % (2 * pi)) - pi
                self.Heading_CLC.set_ref(self.IMU.heading + diff)

                yaw_rate = self.Heading_CLC.run()
                v_adjust = yaw_rate * RomiProps.wdiv2
                l_flag_s.put(1)
                l_speed_s.put(PathDirector.v_ref - v_adjust)
                r_flag_s.put(1)
                r_speed_s.put(PathDirector.v_ref + v_adjust)
            # -------------------------------------------------------- End of Functions -------------------------------------------------------- #

            # -------------------------------------------------------- Start of Path for final course -------------------------------------------------------- #
            elif self.state == PathDirector.ST_B4_FOLLOW_LINE:
                # Straight before following line segment
                if not self.segment_set:  # ran once per segment
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    print("Straight before following line segment")

                    # motor settings
                    l_flag_s.put(1)
                    l_speed_s.put(PathDirector.v_ref)
                    r_flag_s.put(1)
                    r_speed_s.put(PathDirector.v_ref)

                    # print("lspeed", l_speed_s.get())
                    # print("rspeed", r_speed_s.get())

                distance = 50  # mm

                if obsd_cpos_s.get() >= distance:
                    set_state(PathDirector.FOLLOW_LINE_B4_FORK)

            elif self.state == PathDirector.FOLLOW_LINE_B4_FORK:
                # Line Following Segment
                if not self.segment_set:  # ran once per segment
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    print("Starting FOLLOW_LINE_B4_FORK Segment")

                    # Linesensor CLC centroid reference
                    self.Line_CLC.reset()
                    self.Line_CLC.set_ref(0)

                    # motor settings
                    l_flag_s.put(1)
                    l_speed_s.put(PathDirector.v_ref)
                    r_flag_s.put(1)
                    r_speed_s.put(PathDirector.v_ref)

                if obsd_X_s.get() >= 625:
                    set_state(PathDirector.FORCE_RIGHT_FORK)

                yaw_rate = self.Line_CLC.run()
                v_adjust = yaw_rate * RomiProps.wdiv2
                l_flag_s.put(1)
                l_speed_s.put(PathDirector.v_ref - v_adjust)
                r_flag_s.put(1)
                r_speed_s.put(PathDirector.v_ref + v_adjust)

                # self.uart.write(f"{obsd_X_s.get()},{obsd_Y_s.get()}\r\n".encode("utf-8"))

            elif self.state == PathDirector.FORCE_RIGHT_FORK:
                # Force Right Fork
                if not self.segment_set:  # ran once per segment
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    print("Starting Force Right Fork Segment")
                    self.seg_start_time = ticks_us()

                    # motor settings
                    l_flag_s.put(1)
                    l_speed_s.put(PathDirector.v_ref + 150)
                    r_flag_s.put(1)
                    r_speed_s.put(PathDirector.v_ref)

                if ticks_diff(ticks_us(), self.seg_start_time) >= 500_000:
                    set_state(PathDirector.LINE_FOLLOW_2_DIAMOND)

            elif self.state == PathDirector.LINE_FOLLOW_2_DIAMOND:
                # Line Follow to Diamond
                if not self.segment_set:  # ran once per segment
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    print("Starting Line Following 2 Diamond Segment")

                    # Linesensor CLC centroid reference
                    self.Line_CLC.reset()
                    self.Line_CLC.set_ref(0)

                    # motor settings
                    l_flag_s.put(1)
                    l_speed_s.put(PathDirector.v_ref)
                    r_flag_s.put(1)
                    r_speed_s.put(PathDirector.v_ref)
                    # print("lspeed", l_speed_s.get())
                    # print("rspeed", r_speed_s.get())

                if (obsd_X_s.get() >= 800 and obsd_Y_s.get() <= -200) or self.IMU.heading <= -pi/2:
                    set_state(PathDirector.Diamond_2_CP1)

                yaw_rate = self.Line_CLC.run()
                v_adjust = yaw_rate * RomiProps.wdiv2
                l_flag_s.put(1)
                l_speed_s.put(PathDirector.v_ref - v_adjust)
                r_flag_s.put(1)
                r_speed_s.put(PathDirector.v_ref + v_adjust)

            elif self.state == PathDirector.Diamond_2_CP1:
                PathDirector.var_1 = 850
                PathDirector.var_2 = -375
                PathDirector.next_state = PathDirector.CP2_2_Cup2
                set_state(PathDirector.GO_2_POINT)

            elif self.state == PathDirector.CP1_2_CP2:
                PathDirector.var_1 = 1400
                PathDirector.var_2 = -300
                PathDirector.next_state = PathDirector.WAIT
                set_state(PathDirector.GO_2_POINT)

            elif self.state == PathDirector.CP2_2_Cup2:
                pass

            # -------------------------------------------------------- End of Path for final course -------------------------------------------------------- #
            yield self.state
