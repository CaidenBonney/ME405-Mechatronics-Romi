from pyb import I2C, Pin, delay  # pyright: ignore
from struct import calcsize, unpack_from
from micropython import const  # pyright: ignore
from os import listdir
from Sensor import Sensor
from math import pi


class IMU(Sensor):

    DEV_ADDR = 0x28
    default_mode_set = 5

    # Sensor Gains
    Kp = 0.15  # Proportional gain
    Ki = 0.20  # Integral gain
    Kd = 0.00  # Derivative gain

    # register addresses class
    class reg:
        # remapping axis
        AXIS_REMAP_CONFIG = (const(0x41), b"<B")  # value = 0x21
        AXIS_REMAP_SIGN = (const(0x42), b"<B")  # value = 0x02

        # page 51/52 table
        EUL_HEADING = (const(0x1A), b"<h")
        EUL_DATA_ALL = (const(0x1A), b"<hhh")  # heading 0x1A/1B, roll 0x1C/1D, pitch 0x1E/1F

        # GYRO_DATA_X = (const(0x14), b"<h")
        # GYRO_DATA_Y = (const(0x16), b"<h")
        GYRO_DATA_Z = (const(0x18), b"<h")
        GYRO_DATA_ALL = (const(0x14), b"<hhh")

        # Settings:
        OPR_MODE = (const(0x3D), b"<B")
        UNIT_SEL = (const(0x3B), b"<B")  # Section 3.6 page 30

        # CALIBRATION: Section 3.10, page 47
        CALIB_STAT = (const(0x35), b"<B")

        # OFFSETS & RADII: Section 3.6.4, page 31
        calibration_coefficients = (
            (const(0x55), b"<h"),  # ACC_OFFSET_X
            (const(0x57), b"<h"),  # ACC_OFFSET_Y
            (const(0x59), b"<h"),  # ACC_OFFSET_Z
            (const(0x5B), b"<h"),  # MAG_OFFSET_X
            (const(0x5D), b"<h"),  # MAG_OFFSET_Y
            (const(0x5F), b"<h"),  # MAG_OFFSET_Z
            (const(0x61), b"<h"),  # GYR_OFFSET_X
            (const(0x63), b"<h"),  # GYR_OFFSET_Y
            (const(0x65), b"<h"),  # GYR_OFFSET_Z
            (const(0x67), b"<h"),  # ACC_RADIUS
            (const(0x69), b"<h"),  # MAG_RADIUS
        )

    def __init__(self, i2c_object: I2C, reset_pin: Pin):
        super().__init__()
        self.i2c = i2c_object  # the pyb board is the controller, meaning the IMU is the peripheral
        self.reset_pin = reset_pin
        self._buf = bytearray((0 for _ in range(22)))  # buffer for unpacking. Each element is a byte
        self.calibrated = False
        self.convert_reg_to_rad = 900
        self.prev_imu_heading = 0
        self.heading = 0
        self.ar = 2 * pi
        self.reset()

    def reset(self):
        self.reset_pin.value(0)
        self.reset_pin.value(1)
        delay(700)  # delay(ms) to allow IMU to reset

        self._write_reg(IMU.reg.AXIS_REMAP_CONFIG, "00100001")  # P3
        self._write_reg(IMU.reg.AXIS_REMAP_SIGN, "xxxxxx10")
        self._write_reg(IMU.reg.UNIT_SEL, "xxx1x110")  # degF, rads, rad/s, m/s^2

    # "override" parent class method
    def get_data(self) -> float:
        return self.get_heading()

    def _read_reg(self, reg: tuple):
        # determine number of bytes from reg read code
        length = calcsize(reg[1])

        # Create a memoryview object of the right size
        buf = memoryview(self._buf)[:length]

        # Read the the I2C bytes into memoryview
        self.i2c.mem_read(buf, self.DEV_ADDR, reg[0])

        return unpack_from(reg[1], buf)

    def _write_reg(self, reg: tuple, input: str):
        # Writes a string input to the IMU: ie input = '01110xxx'
        # This write functionality can work on 8 or 16 bits
        # ASSUMED THAT CONFIGMODE IS SET APPROPRIATELY BEFORE CALLING THIS FUNCTION

        # determine number of bytes in register location
        length = calcsize(reg[1])
        if length > 2 or len(input) // 8 != length:
            raise ValueError("Write to register is not the correct length")

        # Create and mask and or mask
        and_mask = 0b1111_1111 if length == 1 else 0b1111_1111_1111_1111
        or_mask = 0b0000_0000 if length == 1 else 0b0000_0000_0000_0000
        for char_index in range(len(input)):
            char = input[-char_index - 1]
            if char == "x" or char == "X":
                continue  # and mask is already set, or mask is already cleared
            elif char == "0":
                and_mask &= ~(1 << (char_index))  # clear the and bit
            elif char == "1":
                and_mask &= ~(1 << (char_index))  # clear the and bit
                or_mask |= 1 << (char_index)  # set the or bit
            else:
                raise ValueError("mask chars must be '0','1','x', or 'X'")

        self.i2c.mem_write(
            ((self._read_reg(reg)[0] & and_mask) | or_mask),
            self.DEV_ADDR,
            reg[0],
            addr_size=(8 if length == 1 else 16),
        )

    # A method to change the operating mode of the IMU to one of the many "fusion" modes available
    # from the BNO055.
    # operating modes available: (page 20), fusion modes available: (page 21)
    # [OPR_MODE] address = 0x3D (page 70)
    def set_mode(self, mode: int):
        if mode == 0:
            self._write_reg(IMU.reg.OPR_MODE, "xxxx0000")  # 0: CONFIGMODE
        elif mode == 1:
            self._write_reg(IMU.reg.OPR_MODE, "xxxx1000")  # 1: IMU
        elif mode == 2:
            self._write_reg(IMU.reg.OPR_MODE, "xxxx1001")  # 2: COMPASS
        elif mode == 3:
            self._write_reg(IMU.reg.OPR_MODE, "xxxx1010")  # 3: M4G
        elif mode == 4:
            self._write_reg(IMU.reg.OPR_MODE, "xxxx1011")  # 4: NDOF_FMC_OFF
        elif mode == 5:
            self._write_reg(IMU.reg.OPR_MODE, "xxxx1100")  # 5: NDOF
        else:
            raise ValueError("Invalid mode")

    def get_calibration_status(self) -> tuple:
        return self._read_reg(IMU.reg.CALIB_STAT)

    def get_calibration_coefficients(self) -> tuple:
        return tuple(self._read_reg(coeff_reg)[0] for coeff_reg in IMU.reg.calibration_coefficients)

    def calibrate(self) -> bool:
        if "IMU_cal.txt" in listdir():
            # Calibration data is present
            self.set_mode(0)
            print("Found IMU calibration data, skipping calibration")
            with open("IMU_cal.txt", "r") as file:
                entries = file.readline().strip().split(",")
                if len(entries) != 11:
                    raise ValueError("Calibration data length != expected length")

                for idx, coeff_reg in enumerate(IMU.reg.calibration_coefficients):
                    self._write_reg(coeff_reg, entries[idx])

                self.calibrated = True
                self.set_mode(IMU.default_mode_set)
                return True
        else:
            print("No IMU calibration data found")
            self.set_mode(5)  # set to NDOF mode for calibration

            prev_cal = cur_cal = 0
            while not (cur_cal & 0b0011_0011 == 0b0011_0011):  # only check GYR and MAG calibration
                if prev_cal != cur_cal:
                    print(f"{cur_cal:08b}")
                    prev_cal = cur_cal

                cur_cal = self.get_calibration_status()[0]  # Update cur_cal before checking
                if cur_cal & 0b0011_0011 == 0b0011_0011:
                    self.set_mode(0)
                    with open("IMU_cal.txt", "w") as f:
                        coefs = ",".join(f"{s & 0xFFFF:016b}" for s in self.get_calibration_coefficients())
                        f.write(f"{coefs}\n")
                        print(f"{coefs}")
                    print("IMU Calibration Coefficients Saved")
                    self.calibrated = True
                    self.set_mode(IMU.default_mode_set)
                    return True
                delay(100)  # delay(ms) to allow IMU to settle
        return False

    def get_euler_angles(self) -> tuple[float, float, float]:
        # order: head, rolled, pitched
        return tuple(x / -self.convert_reg_to_rad for x in self._read_reg(IMU.reg.EUL_DATA_ALL))

    def get_imu_heading(self) -> float:
        return -self._read_reg(IMU.reg.EUL_HEADING)[0] / self.convert_reg_to_rad

    def get_heading(self) -> float:
        self.delta = self.get_imu_heading() - self.prev_imu_heading
        self.prev_imu_heading = self.get_imu_heading()

        # Overflow
        if self.delta < -self.ar / 2:
            self.delta += self.ar
        # Underflow
        if self.delta > self.ar / 2:
            self.delta -= self.ar

        self.heading += self.delta

        return self.heading

    def set_heading(self, heading: float):
        self.heading = heading
        self.prev_imu_heading = self.get_imu_heading()
        self.delta = 0

    def get_yaw_rate(self) -> float:
        return self._read_reg(IMU.reg.GYRO_DATA_Z)[0] / self.convert_reg_to_rad
