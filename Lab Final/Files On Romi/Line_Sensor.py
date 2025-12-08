from Sensor import Sensor
from IR_Sensor import IRSensor
from array import array
from os import listdir


class LineSensor(Sensor):
    @classmethod
    def set_gain(cls, K="sensor_Kp", value=0.0):
        """Multiply a class-level gain by the given factor."""
        if not hasattr(cls, K):
            raise AttributeError(f"'{K}' is not a defined gain on {cls.__name__}")
        setattr(cls, K, value)

    # Sensor Gains
    sensor_Kp = 0.05  # Proportional gain
    sensor_Ki = 0.05  # Integral gain
    sensor_Kd = 0.00  # Derivative gain

    def __init__(self, list_of_IR_Sensors: list[IRSensor]):
        super().__init__()
        self.list_IR_Sensors = list_of_IR_Sensors
        self.cal_white = False
        self.cal_black = False
        self.cal_white_complete = False
        self.cal_black_complete = False
        self.use_cal = True  # Set to False to force recalibration
        self.calibrated = False

        # tuple of buffers for readings
        if self.list_IR_Sensors:
            self.buffer = array("f", (0.0 for _ in range(len(self.list_IR_Sensors))))
        else:
            raise ValueError("Line sensor array is empty.")

    # "override" parent class method
    def get_data(self) -> float:
        """Return the centroid position computed from the IR sensor array."""
        # If no sensors, data can't be gotten
        if not self.list_IR_Sensors:
            raise ValueError("Line sensor array is empty.")

        self.update_dt()

        for i, sensor in enumerate(self.list_IR_Sensors):
            self.buffer[i] = sensor.get_data()

        # compute centroid position
        sum_val = 0.0
        sum_pos_val = 0.0
        # each item in buffer is an array
        for i, val in enumerate(self.buffer):
            pos = self.list_IR_Sensors[i].location
            sum_val += val
            sum_pos_val += pos * val

        # print(self.buffer)

        return sum_pos_val / sum_val

    def calibrate(self) -> bool:
        if self.use_cal and "IR_cal.txt" in listdir():
            # Calibration data is present
            print("Found Line Sensor calibration data, skipping calibration")
            with open("IR_cal.txt", "r") as file:
                # Read data from file, strip special characters, split on commas, assign
                # to variables. Similar to HW 0x00 using file.readline()
                white_data = array("H", map(int, file.readline().strip().split(",")))  # H is uint16
                black_data = array("H", map(int, file.readline().strip().split(",")))  # H is uint16

                if len(white_data) != len(black_data) or len(white_data) != len(self.list_IR_Sensors):
                    raise ValueError("Line Sensor Calibration data length does not match number of IR sensors.")

                for idx, IR in enumerate(self.list_IR_Sensors):
                    IR.white = white_data[idx]
                    IR.black = black_data[idx]

            # Calibration successful
            self.calibrated = True
            return True
        else:
            # Calibration data is not present
            # important note: the black reading is already offset by white reading, meaning the order is important.
            if self.cal_white == True and self.cal_white_complete == False:
                for sensor in self.list_IR_Sensors:
                    sensor.read()
                    sensor.white = sensor.ADC.read()
                self.cal_white_complete = True
                # Reset Calibration Flag
                self.cal_white = False
                print("White Calibration Complete")

            if self.cal_black == True and self.cal_black_complete == False:
                for sensor in self.list_IR_Sensors:
                    sensor.read()
                    sensor.black = sensor.ADC.read()
                self.cal_black_complete = True
                # Reset Calibration Flag
                self.cal_black = False
                print("Black Calibration Complete")

            if self.cal_white_complete and self.cal_black_complete:
                with open("IR_cal.txt", "w") as f:
                    whites = ",".join(str(s.white) for s in self.list_IR_Sensors)
                    blacks = ",".join(str(s.black) for s in self.list_IR_Sensors)
                    f.write(f"{whites}\n{blacks}\n")

                # Reset Calibration Flags
                self.cal_white_complete = False
                self.cal_black_complete = False
                self.calibrated = True
                return True
        self.calibrated = False
        return False
