import ulab  # pyright: ignore

array, zeros, dot, cos, sin = ulab.numpy.array, ulab.numpy.zeros, ulab.numpy.dot, ulab.numpy.cos, ulab.numpy.sin
from Romi_Props import RomiProps
import task_share
from IMU import IMU
from Encoder import Encoder
from Battery import Battery
from Rolling_Queue import RollingQueue

# only for printing to bluetooth
from pyb import UART  # pyright: ignore


class Observer:
    def __init__(self, IMU: IMU, l_encoder: Encoder, r_encoder: Encoder, battery: Battery):
        self.IMU = IMU
        self.tstep = 0.020  # must assume constant timestep for observer to function

        self.l_encoder = l_encoder
        self.l_motor_pwm_ch = self.l_encoder.motor.PWM_ch
        self.l_motor_nSLP_pin = self.l_encoder.motor.nSLP_pin
        self.r_encoder = r_encoder
        self.r_motor_pwm_ch = self.r_encoder.motor.PWM_ch
        self.r_motor_nSLP_pin = self.r_encoder.motor.nSLP_pin
        self.battery = battery

        self.uart = UART(5, 115200)

    def run(
        self,
        shares: tuple[
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
            obsd_lpos_s,
            obsd_rpos_s,
            obsd_cpos_s,
            obsd_yaw_s,
            obsd_yawrate_s,
            obsd_X_s,
            obsd_Y_s,
        ) = shares

        # Making A_D, B_D, and C matrices
        A_D = array(
            [
                [0.0012, 0.0012, 0.0140, 0.0000],
                [0.0012, 0.0012, 0.0140, 0.0000],
                [-0.0001, -0.0001, 0.0039, 0.0000],
                [0.0000, 0.0000, -0.0000, -0.0000],
            ]
        )

        B_D = array(
            [
                [0.0506, 0.0436, -0.0070, -0.0070, -0.0000, -2.0241],
                [0.0436, 0.0506, -0.0070, -0.0070, 0.0000, 2.0241],
                [0.0029, 0.0029, 0.4980, 0.4980, -0.0000, 0.0000],
                [0.0000, -0.0000, 0, 0, 1, 0.02],
            ]
        )

        C = array(
            [
                [0.0000, 0.0000, 1.0000, -70.5000],
                [0.0000, 0.0000, 1.0000, 70.5000],
                [0.0000, 0.0000, 0.0000, 1.0000],
                [-0.2465, 0.2465, 0.0000, 0.0000],
            ]
        )

        # Preallocate state and input vectors
        x_k = zeros((4, 1))
        ustar = zeros((6, 1))  #  # real-time input vector and real-time output vector concatenated
        y_k = zeros((4, 1))  # calculated output vector

        # Additional variables
        v = 0
        v_avg_rol_queue = RollingQueue(3)
        X_dot = 0
        Y_dot = 0
        X = 0
        Y = 0

        # zero heading
        self.IMU.zero_heading()

        # Update encoders based on current IMU heading (which is zeroed)
        self.l_encoder.position = -RomiProps.wdiv2 * self.IMU.get_heading()
        self.r_encoder.position = RomiProps.wdiv2 * self.IMU.get_heading()

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
            y_k = dot(C, x_k)  # D term omitted since zero

            # Calculate v based on x_k
            v = (self.l_encoder.velocity + self.r_encoder.velocity) / 2

            v_avg_rol_queue.push(v)

            v_avg = (
                v_avg_rol_queue.average()
                if (self.l_motor_pwm_ch.pulse_width_percent() == 0 or self.l_motor_nSLP_pin.value() == 1)
                and (self.r_motor_pwm_ch.pulse_width_percent() == 0 or self.r_motor_nSLP_pin.value() == 1)
                else 0
            )

            X_dot = v_avg * cos(ustar[4, 0])
            Y_dot = v_avg * sin(ustar[4, 0])
            X += X_dot * self.tstep
            Y += Y_dot * self.tstep

            # Update Shares Accordingly
            obsd_lpos_s.put(y_k[0, 0])
            obsd_rpos_s.put(y_k[1, 0])
            obsd_cpos_s.put(x_k[2, 0])
            obsd_yaw_s.put(y_k[2, 0])
            obsd_yawrate_s.put(y_k[3, 0])
            obsd_X_s.put(X)
            obsd_Y_s.put(Y)

            # Update x_k to be used in next iteration
            x_k = dot(A_D, x_k) + dot(B_D, ustar)
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
