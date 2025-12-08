from pyb import UART  # pyright: ignore
import task_share
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

    def __init__(self, line_sensor: LineSensor, IMU_obj: IMU):
        self.segment = 0
        self.segment_set = False
        self.line_sensor = line_sensor
        self.CLC = ClosedLoopControl(
            sensor=self.line_sensor,
            max_min=50,
            Kp=LineSensor.sensor_Kp,
            Ki=LineSensor.sensor_Ki,
            Kd=LineSensor.sensor_Kd,
        )
        self.IMU = IMU_obj
        self.uart = UART(5, 115200)

    def set_state(self, set_seg_s, new_state: int):
        self.state = new_state
        set_seg_s.put(new_state)
        self.segment_set = False

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

        self.state = 0
        self.set_state(set_seg_s, 1)

        while True:
            if set_seg_s.get() != self.state:
                self.set_state(set_seg_s, set_seg_s.get())

            if self.state == 0:
                if not self.segment_set:
                    l_flag_s.put(1)
                    l_speed_s.put(0)
                    r_flag_s.put(1)
                    r_speed_s.put(0)
                    self.segment_set = True

                    print("Starting Waiting Segment")
                # Waiting Segment
                # print(".", end="")

            elif self.state == 1:
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
                    self.set_state(set_seg_s, 0)

            elif self.state == 2:
                # Line Following Segment
                if not self.segment_set:  # ran once per segment
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    self.segment_set = True
                    print("Starting Line Following Segment")

                    # motor settings
                    l_flag_s.put(1)
                    l_speed_s.put(self.v_ref)
                    r_flag_s.put(1)
                    r_speed_s.put(self.v_ref)
                    # data_transfer_s.put(1)  # limits time to 500 queues worth of data
                    test_complete_s.put(0)

                self.CLC.gain_update(
                    Kp=LineSensor.sensor_Kp,
                    Ki=LineSensor.sensor_Ki,
                    Kd=LineSensor.sensor_Kd,
                )
                yaw_rate = self.CLC.run()
                v_adjust = yaw_rate * RomiProps.wdiv2
                l_flag_s.put(1)
                l_speed_s.put(PathDirector.v_ref - v_adjust)
                r_flag_s.put(1)
                r_speed_s.put(PathDirector.v_ref + v_adjust)

                self.uart.write(
                    f"obsd_cpos: {obsd_cpos_s.get()}, IMU heading: {self.IMU.heading}, obsd_X_pos: {obsd_X_s.get()}, obsd_Y_pos: {obsd_Y_s.get()}\r\n".encode(
                        "utf-8"
                    )
                )

                if self.IMU.heading >= 6.283185307179586:
                    self.set_state(set_seg_s, 0)

            elif self.state == 3:
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

                distance = 1000  # mm

                # self.uart.write(
                #     f"obsd_cpos: {obsd_cpos_s.get()}, obsd_heading: {obsd_yaw_s.get()}, obsd_X_pos: {obsd_X_s.get()}, obsd_Y_pos: {obsd_Y_s.get()}\r\n".encode(
                #         "utf-8"
                #     )
                # )

                if obsd_cpos_s.get() >= distance:
                    self.uart.write(f"L: {obsd_lpos_s.get() * RomiProps.wheel_radius}\r\n".encode("utf-8"))
                    self.uart.write(f"R: {obsd_rpos_s.get() * RomiProps.wheel_radius}\r\n".encode("utf-8"))
                    self.uart.write(f"C: {obsd_cpos_s.get()}\r\n".encode("utf-8"))

                    l_flag_s.put(1)
                    l_speed_s.put(0)
                    r_flag_s.put(1)
                    r_speed_s.put(0)
                    test_complete_s.put(1)

                    self.set_state(set_seg_s, 0)

            elif self.state == 4:
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

            yield self.state
