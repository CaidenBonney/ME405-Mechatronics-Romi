from ulab import numpy as np  # pyright: ignore
from Romi_Props import RomiProps
from math import pi


class Observer:

    def __init__(self, IMU, l_encoder, r_encoder, battery):
        self.IMU = IMU
        self.tstep = 0.020  # must assume constant timestep for observer to function

        self.l_encoder = l_encoder
        self.l_motor_pwm_ch = self.l_encoder.motor.PWM_ch
        self.l_motor_nSLP_pin = self.l_encoder.motor.nSLP_pin
        self.r_encoder = r_encoder
        self.r_motor_pwm_ch = self.r_encoder.motor.PWM_ch
        self.r_motor_nSLP_pin = self.r_encoder.motor.nSLP_pin
        self.battery = battery

        # only for printing to bluetooth
        from pyb import UART  # pyright: ignore

        self.uart = UART(5, 115200)

    def run(self, shares):
        # Seperating the shares
        (
            obsd_lpos_s,
            obsd_rpos_s,
            obsd_cpos_s,
            obsd_yaw_s,
            obsd_yawrate_s,
            obsd_X_s,
            obsd_Y_s,
            dist_yaw_s,
        ) = shares

        # Making A_D, B_D, and C matrices
        A_D = np.array(
            [
                [0.0012, 0.0012, 0.0140, 0.0000],
                [0.0012, 0.0012, 0.0140, 0.0000],
                [-0.0001, -0.0001, 0.0039, 0.0000],
                [0.0000, 0.0000, -0.0000, -0.0000],
            ]
        )

        B_D = np.array(
            [
                [0.0506, 0.0436, -0.0070, -0.0070, -0.0000, -2.0241],
                [0.0436, 0.0506, -0.0070, -0.0070, 0.0000, 2.0241],
                [0.0029, 0.0029, 0.4980, 0.4980, -0.0000, 0.0000],
                [0.0000, -0.0000, 0, 0, 1, 0.02],
            ]
        )

        C = np.array(
            [
                [0.0000, 0.0000, 1.0000, -70.5000],
                [0.0000, 0.0000, 1.0000, 70.5000],
                [0.0000, 0.0000, 0.0000, 1.0000],
                [-0.2465, 0.2465, 0.0000, 0.0000],
            ]
        )

        # Preallocate state and input vectors
        x_k = np.zeros((4, 1))
        ustar = np.zeros((6, 1))  #  # real-time input vector and real-time output vector concatenated
        y_k = np.zeros((4, 1))  # calculated output vector

        prev_c = 0

        # zero heading
        self.IMU.set_heading(0)

        # Update encoders based on current IMU heading (which is zeroed)
        self.l_encoder.position = -RomiProps.wdiv2 * self.IMU.get_heading()
        self.r_encoder.position = RomiProps.wdiv2 * self.IMU.get_heading()

        # set coordinate system
        obsd_X_s.put(100)
        obsd_Y_s.put(800)

        while True:
            # Update real-time data
            ustar[0, 0] = self.l_motor_pwm_ch.pulse_width_percent() / 100 * self.battery.get_cur_volt()
            ustar[1, 0] = self.r_motor_pwm_ch.pulse_width_percent() / 100 * self.battery.get_cur_volt()
            ustar[2, 0] = self.l_encoder.position * RomiProps.wheel_radius
            ustar[3, 0] = self.r_encoder.position * RomiProps.wheel_radius
            ustar[4, 0] = self.IMU.get_heading()
            ustar[5, 0] = self.IMU.get_yaw_rate()

            # Perform observer update
            # y_k calculated before next x_k update
            y_k = np.dot(C, x_k)  # D term omitted since zero

            # Calculate C_delta
            C_delta = x_k[2, 0] - prev_c
            prev_c = x_k[2, 0]

            X_delta = C_delta * np.cos(ustar[4, 0])
            Y_delta = C_delta * np.sin(ustar[4, 0])

            # Update Shares Accordingly
            obsd_lpos_s.put(y_k[0, 0])
            obsd_rpos_s.put(y_k[1, 0])
            obsd_cpos_s.put(x_k[2, 0])
            obsd_yaw_s.put(y_k[2, 0])
            obsd_yawrate_s.put(y_k[3, 0])
            obsd_X_s.put(obsd_X_s.get() + X_delta)
            obsd_Y_s.put(obsd_Y_s.get() + Y_delta)
            dist_yaw_s.put(
                (self.r_encoder.position - self.l_encoder.position) * RomiProps.wheel_radius / RomiProps.trackwidth
            )

            # Update x_k to be used in next iteration
            x_k = np.dot(A_D, x_k) + np.dot(B_D, ustar)
            yield 0


# For Reference:
# x_k[0,0] = Omega_L   [rad/s]
# x_k[1,0] = Omega_R   [rad/s]
# x_k[2,0] = s         [mm]
# x_k[3,0] = psi       [rad]

# y_k[0,0] = s_L       [mm]
# y_k[1,0] = s_R       [mm]
# y_k[2,0] = psi       [rad]
# y_k[3,0] = psi_dot   [rad/s]

# ustar[0,0] = v_L     [V]
# ustar[1,0] = v_R     [V]
# ustar[2,0] = s_L     [mm]
# ustar[3,0] = s_R     [mm]
# ustar[4,0] = psi     [rad]
# ustar[5,0] = psi_dot [rad/s]
