import ulab  # pyright: ignore

array, zeros, dot, cos, sin = ulab.numpy.array, ulab.numpy.zeros, ulab.numpy.dot, ulab.numpy.cos, ulab.numpy.sin
from Romi_Props import RomiProps
import task_share
from IMU import IMU
from Encoder import Encoder
from Battery import Battery


class Observer:
    def __init__(self, IMU: IMU, l_encoder: Encoder, r_encoder: Encoder, battery: Battery):
        self.IMU = IMU
        self.tstep = 0.020  # must assume constant timestep for observer to function

        self.l_encoder = l_encoder
        self.r_encoder = r_encoder
        self.battery = battery

        self.A_D = array(
            [
                [0.0012, 0.0012, 0.0140, 0.0000],
                [0.0012, 0.0012, 0.0140, 0.0000],
                [-0.0001, -0.0001, 0.0039, 0.0000],
                [0.0000, 0.0000, -0.0000, -0.0000],
            ]
        )

        self.B_D = array(
            [
                [0.0506, 0.0436, -0.0070, -0.0070, -0.0000, -2.0241],
                [0.0436, 0.0506, -0.0070, -0.0070, 0.0000, 2.0241],
                [0.0029, 0.0029, 0.4980, 0.4980, -0.0000, 0.0000],
                [0.0000, -0.0000, -0.0071, 0.0071, 0.0001, 0.0002],
            ]
        )

        self.C = array(
            [
                [0.0000, 0.0000, 1.0000, -70.5000],
                [0.0000, 0.0000, 1.0000, 70.5000],
                [0.0000, 0.0000, 0.0000, 1.0000],
                [-0.2465, 0.2465, 0.0000, 0.0000],
            ]
        )

        # Preallocate state and input vectors
        self.x_k = zeros((4, 1))
        self.ustar = zeros((6, 1))  #  # real-time input vector and real-time output vector concatenated
        self.y_k = zeros((4, 1))  # calculated output vector

        # Additional variables
        self.v_hat = 0
        self.X_dot = 0
        self.Y_dot = 0
        self.X = 0
        self.Y = 0

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

        # zero heading
        self.IMU.zero_heading()

        # Update encoders based on current IMU heading (which is zeroed)
        self.l_encoder.position = -RomiProps.wdiv2 * self.IMU.get_heading()
        self.r_encoder.position = RomiProps.wdiv2 * self.IMU.get_heading()

        while True:
            # Update real-time data
            self.ustar[0, 0] = self.l_encoder.motor.PWM_ch.pulse_width_percent() / 100 * self.battery.get_cur_volt()
            self.ustar[1, 0] = self.r_encoder.motor.PWM_ch.pulse_width_percent() / 100 * self.battery.get_cur_volt()
            self.ustar[2, 0] = self.l_encoder.position #* RomiProps.wheel_radius
            self.ustar[3, 0] = self.r_encoder.position #* RomiProps.wheel_radius
            self.ustar[4, 0] = self.IMU.get_heading()
            self.ustar[5, 0] = self.IMU.get_yaw_rate()

            # Perform observer update
            # y_k calculated before next x_k update
            self.y_k = dot(self.C, self.x_k)  # D term omitted since zero

            # Calculate v based on x_k
            self.v_hat = RomiProps.wheel_radius / 2 * (self.x_k[0, 0] + self.x_k[1, 0])
            self.v = (self.l_encoder.velocity + self.r_encoder.velocity) / 2
            # PRINT V AND V_HAT TO SEE IF THEY AGREE
            # COULD BE SOURCE OF ERROR FOR X AND Y CALCS
            self.v_avg = (3 * self.v_hat + 2 * self.v) / 5  # weighted average of velocities
            
            self.X_dot = self.v_avg * cos(self.ustar[4,0]) * RomiProps.wheel_radius
            self.Y_dot = self.v_avg * sin(self.ustar[4,0]) * RomiProps.wheel_radius
            self.X += self.X_dot * self.tstep
            self.Y += self.Y_dot * self.tstep

            # Update Shares Accordingly
            obsd_lpos_s.put(self.y_k[0, 0])
            obsd_rpos_s.put(self.y_k[1, 0])
            obsd_cpos_s.put(self.x_k[2, 0] * RomiProps.wheel_radius)
            obsd_yaw_s.put(self.y_k[2, 0])
            obsd_yawrate_s.put(self.y_k[3, 0])
            obsd_X_s.put(self.X / 14.578334)
            obsd_Y_s.put(self.Y / 14.578334)

            # Update x_k to be used in next iteration
            self.x_k = dot(self.A_D, self.x_k) + dot(self.B_D, self.ustar)
            yield 0


# For Reference:
# x_k[0,0] = Omega_L
# x_k[1,0] = Omega_R
# x_k[2,0] = s
# x_k[3,0] = psi

# y_k[0,0] = s_L
# y_k[1,0] = s_R
# y_k[2,0] = psi
# y_k[3,0] = psi_dot

# ustar[0,0] = v_L
# ustar[1,0] = v_R
# ustar[2,0] = s_L
# ustar[3,0] = s_R
# ustar[4,0] = psi
# ustar[5,0] = psi_dot
