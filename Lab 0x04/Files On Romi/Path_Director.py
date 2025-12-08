import task_share
from Closed_Loop_Control import ClosedLoopControl
from Line_Sensor import LineSensor


class PathDirector:

    def __init__(self, line_sensor: LineSensor):
        self.segment = 0
        self.segment_set = False
        self.line_sensor = line_sensor
        self.CLC = ClosedLoopControl(
            self.line_sensor,
            max_min=50,
            Kp=LineSensor.sensor_Kp,
            Ki=LineSensor.sensor_Ki,
            Kd=LineSensor.sensor_Kd,
        )
        self.v_ref = 0  # mm/s reference speed

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
        ) = shares

        self.state = 0

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
                # Waiting state
                pass

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
                    
                if self.line_sensor.calibrate():
                    print("Calibration Complete")
                    self.CLC.reset()
                    # return back to waiting state
                    self.set_state(set_seg_s, 0)
            elif self.state == 2:
                # Line Following State
                if not self.segment_set:  # ran once per segment
                    # set seg_start_s to 2 because we have two motor controllers
                    # that both have to acknowledge the segment start
                    seg_start_s.put(2)
                    self.segment_set = True

                    self.v_ref = 500  # mm/s

                    # motor settings
                    l_flag_s.put(1)
                    l_speed_s.put(self.v_ref)
                    r_flag_s.put(1)
                    r_speed_s.put(self.v_ref)
                    data_transfer_s.put(1)
                    test_complete_s.put(0)

                self.CLC.gain_update(
                    Kp=LineSensor.sensor_Kp,
                    Ki=LineSensor.sensor_Ki,
                    Kd=LineSensor.sensor_Kd,
                )
                yaw_rate = self.CLC.run()
                v_adjust = yaw_rate * 70.5 # = 141/2 = track width / 2 = radius of rotation by a wheel 
                l_flag_s.put(1)
                l_speed_s.put(self.v_ref - v_adjust)
                r_flag_s.put(1)
                r_speed_s.put(self.v_ref + v_adjust)

                # if done with current seg
                #     # return back to waiting state
                #     self.set_seg(set_seg_s, 0)
            elif self.state == 3:
                pass

            yield self.state
