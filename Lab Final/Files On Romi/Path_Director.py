from pyb import UART  # pyright: ignore
from time import ticks_diff, ticks_us  # pyright: ignore
from math import pi, atan2

from gc import collect
from Romi_Props import RomiProps
from Path_Director_vars import PD_vars


class PathDirector:
    WAIT = 0
    CALIBRATE = 1
    FOLLOW_LINE = 2

    ST_B4_FOLLOW_LINE = 4
    FOLLOW_LINE_B4_FORK = 5
    FORCE_RIGHT_FORK = 6
    LINE_FOLLOW_2_DIAMOND = 7
    DIAMOND_2_CP1 = 8
    TURN_2_CP2 = 9
    GO_2_LINE_B4_CP2 = 10
    LINE_2_CP2 = 11
    LINE_B4_CP3 = 12
    TURN_2_LINE_B4_CP3 = 13
    GO_2_LINE_B4_CP3 = 14
    TURN_2_CP3 = 15
    LINE_2_CP3 = 16
    GO_2_CP3 = 17
    GO_2_LINE_B4_CP4 = 18
    FOLLOW_LINE_2_CP4 = 19
    TURN_4_GARAGE = 20
    MOVE_IN_GARAGE = 21
    TURN_2_GARAGE_EXIT = 22
    GO_2_GARAGE_EXIT = 23
    FOLLOW_LINE_2_WALL = 24
    REVERSE_FROM_WALL = 25
    TURN_2_CUP2 = 26
    GO_2_CUP2 = 27
    TURN_2_CP6 = 28
    GO_2_CP6 = 29
    FOLLOW_LINE_2_CP6 = 30

    # function states: Third digit is the number of inputs required
    # One Input: 100-199
    FIXED_FORWARD = 101  #         self.distance = distance [mm]
    TURN_ANGLE = 102  #            var1 = angle,  var2 = -----, var3 = ----
    FOLLOW_LINE_4_TIME = 103  #    var1 = durration [us]
    # Two Inputs: 200-299
    # Three Inputs: 300-399
    # Four Inputs: 400-499
    GO_2_POINT = 401  #            var1 = X,      var2 = Y,      var3 = Skip_x,      var4 = Skip_y
    FOLLOW_LINE_2_POINT = 402  #   var1 = X,      var2 = Y,      var3 = Skip_x,      var4 = Skip_y

    def __init__(self, line_sensor, IMU_obj, bump_sensors: tuple):
        collect()
        from Closed_Loop_Control import ClosedLoopControl
        from Line_Sensor import LineSensor
        from IMU import IMU

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
        self.bump_stop = True
        self.bump_wall = False
        self.bumptimer = 0

        collect()
        from pyb import Pin, ExtInt  # pyright: ignore

        # Create bumper callbacks for all pins
        def bmp_callback_handler(pin: Pin):
            def bumper_callback(line):
                if pin.value() != 0:
                    self.bumped = True

            return bumper_callback

        for bump_sensor in bump_sensors:
            ExtInt(bump_sensor, ExtInt.IRQ_FALLING, Pin.PULL_UP, bmp_callback_handler(bump_sensor))

        # Extra variables used for fixed distance
        self.distance = 1000  # mm
        self.end_point = 0  # mm

    def run(self, shares):
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
            dist_yaw_s,
        ) = shares

        def set_state(new_state: int):
            self.state = new_state
            set_seg_s.put(new_state)
            self.segment_set = False

            # tell motors previous segment is complete
            test_complete_s.put(1)

        def update_motors_CLC(CLC_obj):
            v_adjust = CLC_obj.run() * RomiProps.wdiv2
            l_flag_s.put(1)
            l_speed_s.put(PD_vars.v_ref - v_adjust)
            r_flag_s.put(1)
            r_speed_s.put(PD_vars.v_ref + v_adjust)

        def forward_at_speed(speed: float):
            l_flag_s.put(1)
            l_speed_s.put(speed)
            r_flag_s.put(1)
            r_speed_s.put(speed)

        set_state(PathDirector.CALIBRATE)

        while True:
            if self.bumped:
                if self.bump_stop:
                    self.uart.write(f"bumped\r\n".encode("utf-8"))
                    self.bumped = False
                    set_state(PathDirector.WAIT)
                elif self.bump_wall:
                    self.uart.write(f"wall\r\n".encode("utf-8"))
                    set_state(PathDirector.REVERSE_FROM_WALL)
                else:
                    if self.bumptimer == 0:
                        self.uart.write(f"bumped\r\n".encode("utf-8"))
                        self.bumptimer = ticks_us()
                    forward_at_speed(-100)
                    if ticks_diff(ticks_us(), self.bumptimer) >= 1000_000:
                        self.uart.write(f"unbumped\r\n".encode("utf-8"))
                        forward_at_speed(0)
                        self.bumptimer = 0
                        self.bumped = False

            if self.Line_CLC:
                from Line_Sensor import LineSensor

                self.Line_CLC.gain_update(Kp=LineSensor.Kp, Ki=LineSensor.Ki, Kd=LineSensor.Kd)

            if self.Heading_CLC:
                from IMU import IMU

                self.Heading_CLC.gain_update(Kp=IMU.Kp, Ki=IMU.Ki, Kd=IMU.Kd)

            if set_seg_s.get() != self.state:
                set_state(set_seg_s.get())

            if self.state == PathDirector.WAIT:
                if not self.segment_set:
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    forward_at_speed(0)

                    print("Starting Waiting Segment")
                    # Waiting Segment
                    # self.uart.write(f"curr state: {self.state}\r\n".encode("utf-8"))
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
                    self.Line_CLC.set_ref(0)

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
                    forward_at_speed(PD_vars.v_ref)

                update_motors_CLC(self.Line_CLC)

            # -------------------------------------------------------- Start of Functions -------------------------------------------------------- #

            elif self.state == PathDirector.FIXED_FORWARD:
                # Fixed Forward Distance Segment
                if not self.segment_set:  # ran once per segment
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    print("Fixed Forward Distance Segment")

                    # self.distance = 1000  # mm
                    self.end_point = obsd_cpos_s.get() + self.distance  # mm

                    # motor settings
                    forward_at_speed(PD_vars.v_ref)

                if obsd_cpos_s.get() >= self.end_point:
                    self.uart.write(f"L: {obsd_lpos_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"R: {obsd_rpos_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"C: {obsd_cpos_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"H: {obsd_yaw_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"X: {obsd_X_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"Y: {obsd_Y_s.get()}\r\n".encode("utf-8"))
                    set_state(PD_vars.next_state)

            elif self.state == PathDirector.TURN_ANGLE:
                # Turn In Place Segment
                # var1 = angle [deg], var2 = ----, var3 = ----
                if not self.segment_set:  # ran once per segment
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    print("Turn Angle Segment")

                    PD_vars.v_ref = 0

                    # degrees to rads
                    PD_vars.var_1 = PD_vars.var_1 * pi / 180  # radians to turn
                    PD_vars.var_2 = self.IMU.heading + PD_vars.var_1  # new desired heading

                    # Set IMU heading
                    self.Heading_CLC.set_ref(PD_vars.var_2)

                diff = ((PD_vars.var_1 - self.IMU.heading + pi) % (2 * pi)) - pi
                self.Heading_CLC.set_ref(self.IMU.heading + diff)

                if abs(diff) <= 3 / 180 * pi:  # 3 deg tolerance
                    set_state(PD_vars.next_state)
                    PD_vars.v_ref = PD_vars.v_ref_DEFAULT
                else:
                    update_motors_CLC(self.Heading_CLC)

            elif self.state == PathDirector.GO_2_POINT:
                # Using current heading and position, move to another point
                # var1 = X, var2 = Y,
                # var3 = "boolean for X tolerance": True = ignore X tolerance
                # var4 = "boolean for Y tolerance": True = ignore Y tolerance
                if not self.segment_set:  # ran once per segment
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    print("Go 2 Point Segment")
                    # motor settings
                    forward_at_speed(PD_vars.v_ref)

                    self.Heading_CLC.reset()

                dX = PD_vars.var_1 - obsd_X_s.get()
                dY = PD_vars.var_2 - obsd_Y_s.get()
                target_angle = atan2(dY, dX)
                diff = ((target_angle - self.IMU.heading + pi) % (2 * pi)) - pi
                self.Heading_CLC.set_ref(self.IMU.heading + diff)

                if (PD_vars.v_ref == 0 and abs(diff) <= 3 / 180 * pi) or (
                    (PD_vars.var_3 or abs(dX) <= 25) and (PD_vars.var_4 or abs(dY) <= 25)
                ):
                    self.uart.write(f"{obsd_X_s.get()},{obsd_Y_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"Next State: {PD_vars.next_state}\r\n".encode("utf-8"))
                    set_state(PD_vars.next_state)
                    PD_vars.v_ref = PD_vars.v_ref_DEFAULT
                else:
                    update_motors_CLC(self.Heading_CLC)

            elif self.state == PathDirector.FOLLOW_LINE_4_TIME:
                if not self.segment_set:
                    # comms to motors
                    self.segment_set = True
                    seg_start_s.put(2)
                    print("Starting Line Follow For Time Segment")
                    self.seg_start_time = ticks_us()

                    # motor settings
                    forward_at_speed(PD_vars.v_ref)

                    self.Line_CLC.reset()
                    self.Line_CLC.set_ref(0)

                if ticks_diff(ticks_us(), self.seg_start_time) >= PD_vars.var_1:
                    set_state(PD_vars.next_state)
                else:
                    update_motors_CLC(self.Line_CLC)

            elif self.state == PathDirector.FOLLOW_LINE_2_POINT:
                if not self.segment_set:
                    # comms to motors
                    self.segment_set = True
                    seg_start_s.put(2)
                    print("Starting Line Follow To Point Segment")

                    # motor settings
                    forward_at_speed(PD_vars.v_ref)

                    self.Line_CLC.reset()

                dX = PD_vars.var_1 - obsd_X_s.get()
                dY = PD_vars.var_2 - obsd_Y_s.get()

                if (PD_vars.var_3 or abs(dX) <= 25) and (PD_vars.var_4 or abs(dY) <= 25):
                    set_state(PD_vars.next_state)
                else:
                    update_motors_CLC(self.Line_CLC)
            # -------------------------------------------------------- End of Functions -------------------------------------------------------- #

            # -------------------------------------------------------- Start of Path for final course -------------------------------------------------------- #
            elif self.state == PathDirector.ST_B4_FOLLOW_LINE:
                # Straight before following line segment
                if not self.segment_set:
                    self.segment_set = True
                    # comms to motors
                    seg_start_s.put(2)
                    print("Straight before following line segment")

                    # motor settings
                    forward_at_speed(PD_vars.v_ref)

                    self.distance = 50  # mm

                if obsd_cpos_s.get() >= self.distance:
                    set_state(PathDirector.FOLLOW_LINE_B4_FORK)

            elif self.state == PathDirector.FOLLOW_LINE_B4_FORK:
                # Line Following Segment
                if not self.segment_set:
                    # comms to motors
                    self.segment_set = True
                    seg_start_s.put(2)
                    print("Starting FOLLOW_LINE_B4_FORK Segment")

                    # Linesensor CLC centroid reference
                    self.Line_CLC.reset()
                    self.Line_CLC.set_ref(0)

                    # motor settings
                    forward_at_speed(PD_vars.v_ref)

                if obsd_X_s.get() >= 725:
                    set_state(PathDirector.FORCE_RIGHT_FORK)
                else:
                    update_motors_CLC(self.Line_CLC)

            elif self.state == PathDirector.FORCE_RIGHT_FORK:
                # Force Right Fork
                if not self.segment_set:
                    # comms to motors
                    self.segment_set = True
                    seg_start_s.put(2)
                    print("Starting Force Right Fork Segment")
                    self.seg_start_time = ticks_us()

                    # motor settings
                    l_flag_s.put(1)
                    l_speed_s.put(PD_vars.v_ref * 1.75)
                    r_flag_s.put(1)
                    r_speed_s.put(PD_vars.v_ref)

                if ticks_diff(ticks_us(), self.seg_start_time) >= 500_000:
                    set_state(PathDirector.LINE_FOLLOW_2_DIAMOND)

            elif self.state == PathDirector.LINE_FOLLOW_2_DIAMOND:
                # Line Follow to Diamond
                if not self.segment_set:
                    self.segment_set = True
                    # comms to motors
                    seg_start_s.put(2)
                    print("Starting Line Following 2 Diamond Segment")

                    # Linesensor CLC centroid reference
                    self.Line_CLC.reset()
                    self.Line_CLC.set_ref(0)

                    # motor settings
                    forward_at_speed(PD_vars.v_ref)

                if (obsd_X_s.get() >= 900 and obsd_Y_s.get() <= 600) or self.IMU.heading <= -pi / 2 + 0.1:
                    self.uart.write(f"X_LF2D: {obsd_X_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"Y_LF2D: {obsd_Y_s.get()}\r\n".encode("utf-8"))
                    self.uart.write(f"H_LF2D: {obsd_yaw_s.get()}\r\n".encode("utf-8"))
                    set_state(PathDirector.DIAMOND_2_CP1)
                else:
                    update_motors_CLC(self.Line_CLC)

            elif self.state == PathDirector.DIAMOND_2_CP1:
                PD_vars.var_1 = 950
                PD_vars.var_2 = 450
                PD_vars.var_3 = True
                PD_vars.var_4 = False
                set_state(PathDirector.GO_2_POINT)
                PD_vars.next_state = PathDirector.TURN_2_CP2

            elif self.state == PathDirector.TURN_2_CP2:
                PD_vars.var_1 = 1300
                PD_vars.var_2 = 650
                PD_vars.var_3 = False
                PD_vars.var_4 = False
                PD_vars.v_ref = 0
                set_state(PathDirector.GO_2_POINT)
                PD_vars.next_state = PathDirector.GO_2_LINE_B4_CP2

            elif self.state == PathDirector.GO_2_LINE_B4_CP2:
                obsd_X_s.put(950)
                obsd_Y_s.put(425)
                PD_vars.var_1 = 1300
                PD_vars.var_2 = 650
                PD_vars.var_3 = False
                PD_vars.var_4 = True
                set_state(PathDirector.GO_2_POINT)
                PD_vars.next_state = PathDirector.LINE_2_CP2

            elif self.state == PathDirector.LINE_2_CP2:
                PD_vars.var_1 = 800_000
                set_state(PathDirector.FOLLOW_LINE_4_TIME)
                PD_vars.next_state = PathDirector.LINE_B4_CP3

            elif self.state == PathDirector.LINE_B4_CP3:
                obsd_X_s.put(1400)
                obsd_Y_s.put(800)
                PD_vars.var_1 = 400_000
                set_state(PathDirector.FOLLOW_LINE_4_TIME)
                PD_vars.next_state = PathDirector.TURN_2_LINE_B4_CP3

            elif self.state == PathDirector.TURN_2_LINE_B4_CP3:
                PD_vars.var_1 = 1600
                PD_vars.var_2 = 100
                PD_vars.var_3 = False
                PD_vars.var_4 = False
                PD_vars.v_ref = 0
                set_state(PathDirector.GO_2_POINT)
                PD_vars.next_state = PathDirector.GO_2_LINE_B4_CP3

            elif self.state == PathDirector.GO_2_LINE_B4_CP3:
                PD_vars.var_1 = 1600
                PD_vars.var_2 = 100
                PD_vars.var_3 = True
                PD_vars.var_4 = False
                set_state(PathDirector.GO_2_POINT)
                PD_vars.next_state = PathDirector.TURN_2_CP3

            elif self.state == PathDirector.TURN_2_CP3:
                self.uart.write(f"T2C3 X, Y: {obsd_X_s.get()}, {obsd_Y_s.get()}\r\n".encode("utf-8"))
                self.IMU.set_heading(-pi / 2)
                PD_vars.var_1 = -180
                set_state(PathDirector.TURN_ANGLE)
                PD_vars.next_state = PathDirector.GO_2_LINE_B4_CP4

            elif self.state == PathDirector.GO_2_LINE_B4_CP4:
                self.uart.write(f"X, Y: {obsd_X_s.get()}, {obsd_Y_s.get()}\r\n".encode("utf-8"))
                self.uart.write(f"dist yaw: {dist_yaw_s.get()}\r\n".encode("utf-8"))

                obsd_X_s.put(1600)
                obsd_Y_s.put(100)
                self.IMU.set_heading(dist_yaw_s.get())
                PD_vars.var_1 = 1100
                PD_vars.var_2 = 100
                PD_vars.var_3 = False
                PD_vars.var_4 = True
                set_state(PathDirector.GO_2_POINT)
                PD_vars.next_state = PathDirector.FOLLOW_LINE_2_CP4

            elif self.state == PathDirector.FOLLOW_LINE_2_CP4:
                self.bump_stop = False
                PD_vars.var_1 = 900
                PD_vars.var_2 = 100
                PD_vars.var_3 = False
                PD_vars.var_4 = True
                set_state(PathDirector.FOLLOW_LINE_2_POINT)
                PD_vars.next_state = PathDirector.TURN_4_GARAGE

            elif self.state == PathDirector.TURN_4_GARAGE:
                obsd_X_s.put(800)
                obsd_Y_s.put(150)
                PD_vars.var_1 = -180
                set_state(PathDirector.TURN_ANGLE)
                PD_vars.next_state = PathDirector.MOVE_IN_GARAGE

            elif self.state == PathDirector.MOVE_IN_GARAGE:
                self.uart.write(f"dist, head: {dist_yaw_s.get()}, {self.IMU.heading}\r\n".encode("utf-8"))
                self.IMU.set_heading(-pi)
                PD_vars.var_1 = 160
                PD_vars.var_2 = 150
                PD_vars.var_3 = False
                PD_vars.var_4 = True
                set_state(PathDirector.GO_2_POINT)
                PD_vars.next_state = PathDirector.TURN_2_GARAGE_EXIT

            elif self.state == PathDirector.TURN_2_GARAGE_EXIT:
                self.uart.write(f"dist, head: {dist_yaw_s.get()}, {self.IMU.heading}\r\n".encode("utf-8"))
                PD_vars.var_1 = -260
                set_state(PathDirector.TURN_ANGLE)
                PD_vars.next_state = PathDirector.GO_2_GARAGE_EXIT

            elif self.state == PathDirector.GO_2_GARAGE_EXIT:
                self.bump_wall = True
                self.uart.write(f"dist, head: {dist_yaw_s.get()}, {self.IMU.heading}\r\n".encode("utf-8"))
                obsd_X_s.put(175)
                obsd_Y_s.put(175)
                self.IMU.set_heading(-3 * pi / 2)
                PD_vars.var_1 = 100
                PD_vars.var_2 = 400
                PD_vars.var_3 = True
                PD_vars.var_4 = False
                set_state(PathDirector.GO_2_POINT)
                PD_vars.next_state = PathDirector.FOLLOW_LINE_2_CP4

            elif self.state == PathDirector.FOLLOW_LINE_2_WALL:
                PD_vars.v_ref = 100
                self.bump_wall = True
                PD_vars.var_1 = 2_000_000
                set_state(PathDirector.FOLLOW_LINE_4_TIME)
                PD_vars.next_state = PathDirector.REVERSE_FROM_WALL  # changed in bumped section

            elif self.state == PathDirector.REVERSE_FROM_WALL:
                if not self.segment_set:  # ran once per segment
                    self.segment_set = True
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    print("Fixed Reverse Distance Segment")

                    self.bump_wall = False
                    self.bumped = False
                    self.IMU.set_heading(-3 * pi / 2)
                    obsd_X_s.put(100)
                    obsd_Y_s.put(675)

                    self.distance = -100  # mm
                    self.end_point = obsd_cpos_s.get() + self.distance  # mm

                    # motor settings
                    forward_at_speed(-200)

                if obsd_cpos_s.get() <= self.end_point:
                    self.uart.write(f"reversed\r\n".encode("utf-8"))
                    set_state(PathDirector.TURN_2_CUP2)

            elif self.state == PathDirector.TURN_2_CUP2:
                PD_vars.var_1 = 500
                PD_vars.var_2 = 500
                PD_vars.var_3 = False
                PD_vars.var_4 = False
                PD_vars.v_ref = 0
                set_state(PathDirector.GO_2_POINT)
                PD_vars.next_state = PathDirector.GO_2_CUP2

            elif self.state == PathDirector.GO_2_CUP2:
                self.IMU.set_heading(pi)
                PD_vars.var_1 = 500
                PD_vars.var_2 = 500
                PD_vars.var_3 = False
                PD_vars.var_4 = True
                set_state(PathDirector.GO_2_POINT)
                PD_vars.next_state = PathDirector.TURN_2_CP6

            elif self.state == PathDirector.TURN_2_CP6:
                PD_vars.var_1 = 100
                PD_vars.var_2 = 300
                PD_vars.var_3 = False
                PD_vars.var_4 = False
                PD_vars.v_ref = 0
                set_state(PathDirector.GO_2_POINT)
                PD_vars.next_state = PathDirector.GO_2_CP6

            elif self.state == PathDirector.GO_2_CP6:
                PD_vars.var_1 = 100
                PD_vars.var_2 = 300
                PD_vars.var_3 = False
                PD_vars.var_4 = True
                set_state(PathDirector.GO_2_POINT)
                PD_vars.next_state = PathDirector.TURN_2_CP6

            elif self.state == PathDirector.FOLLOW_LINE_2_CP6:
                PD_vars.var_1 = 400_000
                set_state(PathDirector.FOLLOW_LINE_4_TIME)
                PD_vars.next_state = PathDirector.WAIT

            # -------------------------------------------------------- End of Path for final course -------------------------------------------------------- #
            yield self.state
