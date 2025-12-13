\mainpage

# Repository
https://github.com/caidenbonney/ME405-Mechatronics-Romi

# Project Overview
The Romi project is part of California Polytechnic State University San Luis Obispo's Mechanical Engineering Course ME405 Mechatronics. The project entails the design and implementation of a robotic vehicle named Romi, which is a two-wheeled robot constructed from component sourced from Pololu Robotics and Electronics.

Romi’s objective is to traverse a predefined course with speed and precision, using integrated sensor feedback and actuator control. The course is defined by a series of waypoints, and Romi must navigate from one waypoint to the next following the class-defined rules. The robot's sensors include a line sensor made from several infrared (IR) sensors, bumps sensors to detect wall collisions, an inertial measurement unit (IMU) to measure orientation with respect to Earth's magnetic north, and quadrature encoders to measure the angular displacement of each wheel. Using information from these sensors, a State Observer is constructed to estimate the robot's current state, which importantly includes its current global position and orientation. Utilizing the Observer's estimations, the robot follows a path director and explores the course in order to reach the next waypoint. 

@image html Romi_isometric.jpeg "Figure 1: Romi Isometric" width=450

# Final Romi Performance
\htmlonly
<div style="text-align:center;">
  <video width="700" controls>
    <source src="../assets/videos/best.mp4" type="video/mp4" />
    <source src="../assets/videos/best.mov" type="video/quicktime" />
    Your browser does not support the video tag. You can download the clip
    <a href="../assets/videos/best.mp4">here</a>.
  </video>
</div>
\endhtmlonly


# Hardware
## Chassis and Boards
### Romi Chassis
Romi's chassis is a component purchased from Pololu Robotics and Electronics. The chassis is a 2-wheeled robot with a center of gravity located at the center of the robot.

The chassis can be purchased at https://www.pololu.com/product/3509 with additional components available at https://www.pololu.com/category/204/romi-chassis-components

### Power Distribution Board
Romi's power distribution board is a component purchased from Pololu Robotics and Electronics. The board is used to power Romi's motors and other peripherals. 

The power distribution board can be purchased at https://www.pololu.com/product/3543.

### Shoe of Brian
Shoe of Brian was provided by ME405 Lab Instructor Charlie Refvem it is an open source board that can be used to interface iwth the Nucleo board. This shoe allows for uploading of code through the USB port. The Shoe of Brian also ensures that any USB devices connected are not powered by the Romi's battery which could damage the connected device. 

A Shoe of Brian control board can be purchased at https://oshpark.com/shared_projects/e6X6OnYK and https://spluttflob.github.io/ME405-Support/shoe_info.html details the steps to build one including additional components required. 

### Nucleo Board
The Nucleo board was also provided by our ME405 Lab Instructor Charlie Refvem. The Shoe of Brian is built to be directly compatible with the Nucleo board. 

We used the ST NUCLEO-L476RG, which integrates the STM32L476RG MCU plus an on-board ST-LINK/V2-1 debugger/programmer, user LEDs, and a user button. In code, hardware timers are configured in `main.py` and used by `Motor.py`/`Motor_Controller.py` for PWM drive and by `Encoder.py` for quadrature capture. The I2C peripheral is used in `IMU.py`, ADC channels in `Battery.py` and `IR_Sensor.py`, UART in `User_Input.py`, and GPIO interrupts for bump sensors in `Path_Director.py`. The Shoe of Brian exposes the CN7/CN10 Morpho headers and Arduino headers so these peripherals can be wired cleanly to the Romi.

The Nucleo Board can be purchased directly from https://estore.st.com/en/products/evaluation-tools/product-evaluation-tools/mcu-mpu-eval-tools/stm32-mcu-mpu-eval-tools/stm32-nucleo-boards/nucleo-l476rg.html or from the same source as the datasheet https://os.mbed.com/platforms/ST-Nucleo-L476RG/.

## Motor
Both wheels are driven by a Texas Instruments DRV8833 low-voltage H-bridge motor driver. Each motor shaft is equipped with an encoder (discussed below) and coupled to a gearbox that provides a 119.7576:1 reduction between the motor shaft and the wheel. In code, PWM drive and direction control are wrapped in `Motor.py`, while `Motor_Controller.py` runs a closed-loop controller (`Closed_Loop_Control.py`) using encoder feedback from `Encoder.py` to set motor effort. Higher-level speed commands come from `Path_Director.py`, and hardware timers are configured in `main.py`.

@image html Motors.png "Figure 2: Texas Instruments DRV8833 Motor Drive with gearbox and Encoder" width=350

 The motors follow the logic table below for its three input pins: nSLEEP, PH, and EN. nSLEEP acts as an on/off switch, PH acts as a direction switch, and EN acts as a drive/brake switch, which is manipulated via pulse width modulation (PWM).

 The motor can be purchased directly from https://www.pololu.com/product/1520.

@image html MotorLogicTable.png "Figure 3: DRV8833 Motor Logic Table" width=900


## Quadrature Incremental Encoder
Each motor shaft is instrumented with a quadrature incremental encoder that measures the shaft’s angular displacement. These encoders employ two Hall-effect sensors that detect changes in the magnetic field produced by rotating permanent magnets. The corresponding magnet geometry and representative encoder waveform are shown below. The encoders used on the Romi utilize the same three-pole-pair magnet configuration illustrated.

@image html Quad_Enc_3PP.png "Figure 4: Quadrature Output for Encoder with 3 Pole-Pair Magnets" width=450


By capturing the number of encoder ticks that occur across a measured time interval, the average angular velocity of each shaft can be calculated. Utilizing this information in conjunction with the motor's gear ratio and wheel radii, the linear velocity of each wheel can be calculated. These linear velocities are used to both close loop control Romi's behavior and to estimate its current state. 

The encoder can be purchased directly from https://www.pololu.com/product/3542.

## Line Sensor
In order to follow the 1/2 inch black lines on the course, Romi utilizes Pololu's line sensor composed of 13 infrared (IR) sensors. The line sensor 
array is aggregated in `Line_Sensor.py`, which normalizes each IR reading (via `IR_Sensor.py`) and computes the centroid of the detected line for feedback. The line-following closed-loop controller in `Closed_Loop_Control.py` uses this centroid error in `Path_Director.py` to steer Romi along the course. Calibration (white/black) can be triggered through `User_Input.py` and is persisted by `Line_Sensor.py` when flags are set from the task shares.

### Mounting
The line sensor is mounted on the underside of the chassis using 3D-printed standoffs. The standoffs set the sensor height so the bump sensors contact obstacles first, protecting the board. Mounting holes were drilled out to fit the hardware available in the lab's donated parts bin. The SolidWorks part model for these standoffs lives in the `Mounting Bracket/` folder. Note that two of these standoffs are required to mount the line sensor.

@image html linesensorbracket.png "Figure 5: Line sensor standoff bracket" width=350

## IMU
Romi uses an Adafruit BNO055 9-DOF absolute orientation sensor for heading and yaw-rate feedback. The breakout board includes an onboard microcontroller that fuses accelerometer, gyroscope, and magnetometer data to provide stable Euler angles without heavy processing on the Nucleo. The BNO055 breakout can be purchased at https://www.adafruit.com/product/2472.

In code, the IMU is driven over I2C in `IMU.py`, which handles initialization, calibration loading/saving, and exposes heading/yaw-rate via `get_heading()` and `get_yaw_rate()`. The heading controller in `Closed_Loop_Control.py` uses these readings inside `Path_Director.py` to align the robot for turns and point-to-point moves, and the `Observer.py` task fuses IMU yaw data with encoder motion to estimate pose.




# Algorithms
The following sections describe the algorithms and code architectures utilized to facilitate Romi's complex and intricate behaviors.

## Multitasking Architecture
To achieve rapid and precise motion control, Romi requires a robust and efficient software architecture capable of executing a wide range of complex behaviors within short time intervals. Romi utilizes a priority-based, round-robin multitasking framework that enables effective multitasking on its single-core microprocessor. The architecture is created in @c cotask.py and instantiated in @c main.py.

Within this framework, the system is organized into discrete tasks (@c cotask.Task), each characterized by a specified @c period and @c priority. The @c period defines the desired interval, in milliseconds, between successive executions of a task’s @c run() method. In practice, certain computationally intensive algorithms may prevent tasks from executing exactly at their designated periods. Under such circumstances, a task is executed as soon as it becomes the highest-priority task awaiting processor time. Consequently, higher-priority tasks always preempt lower-priority tasks when multiple tasks are ready to run simultaneously. If two tasks of equal priority are queued to run, then, in accordance with round-robin scheduling, the task that has been awaiting execution for the longer duration is dispatched first.

As mentioned above, each task has a @c run() method that is responsible for performing its specific behavior. If it is desired to communicate information between tasks, a task can use @c task_share.py's @c task_share.Share or @c task_share.Queue objects. These objects are used to transfer data between tasks with protection against data corruption by interrupts, among other features.

### Task Diagram
Below is a diagram of each @c cotask.Task in Romi's system. The @c period and @c priority of each task is specified, as well as each @c task_share.Share and @c task_share.Queue that it uses.

@image html Task_Diagram.png "Figure 6: Romi's Task Diagram" width=800

### User Input
The user input (@c User_Input.py) task is responsible for receiving commands from the user and actualizing them into physical outputs. User input processes the single-character commands from both the USB and Bluetooth interfaces into values that are placed in shares. Those shares then trigger other tasks to perform their respective functions.

@image html User_Input.png "Figure 7: User Input Task Logic Diagram" width=500

## Path Director
With the control tools described above, the remaining requirement for Romi is a structured framework to govern its overall motion. The path director (@c Path_Director.py) fulfills this role by planning and coordinating Romi’s actions throughout the course. Implemented as a state machine, each state corresponds to a distinct behavioral mode, while additional function states provide generalized control capabilities that accept variable inputs. This design enables compact, modular code and facilitates targeted testing of individual behaviors without restarting an entire course run. As the top-level supervisory controller, the path director manages all aspects of Romi’s navigation and ensures coherent execution of its course-traversal strategy.

@image html Path_Director.png "Figure 8: Path Director Task Logic Diagram" width=500


### Motor Controller   
The motor controller (@c Motor_Controller.py) task is responsible for controlling Romi's motors. The motor controller task has the responsibility of determining what the motor should be doing based off the shares and queues passed into it. Every time it is ran, it runs one iteration of the closed loop control law and then updates the effort requested of the motors.

@image html Motor_Controller.png "Figure 9: Motor Controller Task Logic Diagram" width=500

## State Observer
Using the data from Romi's sensors, the state observer (@c Observer.py) estimates Romi’s current state such that behavioral decisions can be made based on Romi’s current position and orientation. To obtain state estimation, two approaches were considered: the fourth-order Runge-Kutta solver or discretizing Romi’s state space. The Runge-Kutta method was not chosen since it was determined that the frequency at which it would have to run would impose too significant of a computational load on Romi's multitasking system. Instead, Romi’s state space was discretized using control theory techniques and Matlab’s @c c2d() functionality. Further elaboration on the discretization process can be found in the Analysis section.

@image html Observer.png "Figure 10: Observer Task Logic Diagram" width=500


### Garbage Collector
The garbage collector (@c Garbage_Collector.py) task is responsible for managing Romi's memory. Upon each iteration of the garbage collector, the task garbage collects and removes any memory that is no longer in use.

@image html Garbage_Collector.png "Figure 11: Garbage Collector Task Logic Diagram" width=500

## Closed Loop Control
Romi’s motion accuracy relies heavily on the versatile and robust closed loop controller implemented in @c Closed_Loop_Control.py. This controller provides a unified framework for regulating motor speed or heading and supports a wide range of classical control features, making it adaptable to all dynamic behaviors required by the robot.

### PID Control:
At its core, the controller computes actuating commands by continually comparing some reference input, \f$ r \f$, to the measured output, \f$ \hat{x} \f$, provided by any sensor object provided. Each sensor supplies both a measurement and a corresponding time increment (\f$ dt \f$), allowing the controller to compute proportional, integral, and derivative error terms. These components form the basis of standard P, PI, PD, and PID control. In addition to these standard terms, the controller incorporates several enhancements that improve reliability under real-world operating conditions:

### Feed-Forward Control:
A feed-forward term scales the commanded reference input directly, improving response time and reducing the burden on feedback terms, particularly during rapid accelerations or when executing pre-planned maneuvers.

### Anti-Windup Compensation:
When actuator commands saturate—such as when the controller demands more torque than the motors can provide—the integral term can accumulate excessively and degrade stability. The controller mitigates this through an anti-windup mechanism that automatically adjusts the integral term whenever saturation occurs, preserving steady-state accuracy without overshoot.

### Battery Droop Compensation:
Battery voltage naturally decreases during operation, affecting motor torque and responsiveness. When provided with a battery-monitoring object, the controller dynamically adjusts its gains to maintain consistent performance despite changes in supply voltage.

### Soft-Start Reference Ramping:
To prevent sudden jerks or torque spikes, the controller gradually ramps its internal reference toward the commanded setpoint. This soft-start feature improves mechanical safety, reduces wheel slip, and enhances tracking smoothness during abrupt transitions.

### Output Saturation:
After computing all relevant components, the controller generates a preliminary actuator command (\f$ a \f$), applies the appropriate saturation constraints, and thereby produces the final output (\f$ a^* \f$). This finalized command is then returned and utilized within one of three closed-loop control modalities: velocity CLC, line centroid CLC, or heading CLC.

### Velocity CLC:
As per the code structure, each @c Motor.py is handled by a @c Motor_Controller.py task. Whenever the motor controller requests a motor to move at a given speed, the desired velocity is passed to the velocity CLC. Upon the scheduled execution of the Motor Controller task, the velocity CLC computes the new motor effort to be applied such that the desired velocity is reached/maintained.

### Line Centroid CLC:
To enable reliable line following, Romi determines the centroid of the detected black region using the line sensor module (@c Line_Sensor.py). Because the control objective is to maintain Romi’s geometric center directly over the course line, the closed-loop controller’s reference value is fixed at zero. By supplying the measured centroid to \f$ \hat{x} \f$, the line-centroid CLC computes the appropriate yaw rate required to realign the robot with the line. This commanded yaw rate is subsequently translated into differential motor actuation, producing the necessary increase and decrease in wheel velocities to achieve accurate line tracking.

### Heading CLC:
The heading CLC is designed for use in conjunction with an IMU object (@c IMU.py). When Romi is required to navigate toward a target that is not located on the black course lines, it must first orient itself toward the desired heading and subsequently maintain that orientation as it moves and undergoes natural drift. The heading CLC computes the yaw rate necessary for Romi to achieve and sustain the specified heading. This commanded yaw rate is then converted into differential motor actuation, providing the appropriate adjustments in wheel velocities to ensure stable and accurate heading control throughout the maneuver.

@image html Closed_Loop_Control.png "Figure 12: Closed Loop Control Diagram" width=500

# Key Analyses and Validations
The following sections provide some of the key analyses of Romi’s dynamics and performance characteristics. These analyses are intended to provide a comprehensive understanding of Romi’s behavior and to facilitate the development of future improvements.

## Closed Loop Control Gain Tuning
The closed-loop controller for Romi’s drive motors was developed using an incremental gain-tuning process in which controller complexity was expanded stepwise and evaluated experimentally. The tuning sequence progressed through proportional (P), proportional–integral (PI), and proportional–integral–derivative (PID) structures, followed by the integration of feed-forward compensation and battery-voltage droop correction. All controller gains were adjustable in real time through the host computer interface, and all experimental plots were generated automatically. The following discussion summarizes the left-wheel results, with qualitative agreement across both wheels. 

### P Controller
Initial testing began with a purely proportional controller. With a relatively high proportional gain, Romi exhibited pronounced oscillatory “jittering,” characterized by rapid forward–backward motion about the reference value.

@image html P_jitter.png "Figure 13: Proportional Controller - Initial High Gain Response" width=1000

The oscillation resulted from overly aggressive corrective action, causing repeated overshoot of the target velocity. The proportional gain was iteratively reduced until the oscillations were eliminated.

@image html P_good.png "Figure 14: Proportional Controller - Tuned Gain Response" width=1000


### PI Controller
After stabilizing the proportional controller, an integral term was introduced to eliminate steady-state error. The initial integral gain produced a stable response with no overshoot—an indication that the selected gain was likely too small to minimize convergence time.

@image html PI_under.png "Figure 15: PI Controller - Initial Small Integral Gain" width=1000

The integral gain was gradually increased to identify an upper bound beyond which overshoot became unacceptable.

@image html PI_over.png "Figure 16: PI Controller - Excessive Integral Gain" width=1000

A gain between these two extremes was selected through iterative testing, yielding minimal overshoot and improved settling behavior.

@image html PI_good.png "Figure 17: PI Controller - Tuned Integral Gain" width=1000

### PID Controller
With the PI controller tuned, a derivative term was introduced. The initial derivative gain caused unstable, jitter-like oscillations similar to those observed with an overly large proportional gain.

@image html PID_initial.png "Figure 18: PID Controller - Initial Derivative Gain" width=1000

Significant reductions in derivative gain were attempted:

@image html PID_reduced1.png "Figure 19: PID Controller - Reduced Derivative Gain Attempt 1" width=1000
@image html PID_reduced2.png "Figure 20: PID Controller - Reduced Derivative Gain Attempt 2" width=1000

However, due to the presence of high-frequency noise in Romi’s sensor data, derivative action consistently amplified this noise and degraded performance. Consequently, derivative control was deemed unsuitable and removed from all further controller configurations.

### PI Controller with Feed-Forward Compensation
Since PID control was abandoned, the next enhancement involved adding a feed-forward term to the existing PI structure. Unlike earlier gains, the feed-forward gain K<sub>ff</sub> was computed analytically. Steady-state velocity data was used to generate linear curve fits of wheel velocity versus PWM input.

@image html SSvsPWM.png "Figure 21: Steady-State Wheel Velocities vs. PWM Input" width=500

The slopes of these fits yielded steady-state gains, and their inverses provided the appropriate feed-forward gains for each motor. The x-intercepts of the fits were used to estimate the minimum PWM values needed to initiate motion,PWM<sub>start</sub>, producing the feed-forward relations:

\f[
PWM = K_{ff}*v + PWM_{start}
\f]

The feed-forward controller was tested with PI control:

@image html PI_KFF.png "Figure 22: PI + Feed-Forward Response" width=1000

Performance with feed-forward alone was notably accurate and feed forward tuning was concluded


### PI + Feed-Forward + Battery Droop Compensation
Lastly, to improve robustness against battery discharge, a voltage droop compensation term was incorporated. As battery voltage decreases, motor steady-state gains diminish, causing unintended reductions in wheel velocity. To counteract this, the commanded PWM was scaled according to:

\f[
PWM_{\text{scaled}} = PWM \left(\frac{V_{\text{full}}}{V_{\text{measured}}}\right)
\f]

This battery-droop compensation has minimal influence during most of Romi’s operation, as the supply voltage remains relatively stable throughout the majority of the battery’s useful life. However, as the battery approaches depletion, the scaling factor becomes increasingly significant. Experimental observations indicate that the rate at which Romi attains its commanded velocity remains effectively unchanged despite substantial voltage droop, providing qualitative confirmation of the compensation scheme’s effectiveness.

## Romi Dynamics
To adequately understand how Romi would behave, a dynamic analysis was performed. To define variables, the space in which Romi exists in was defined as seen below

@image html Romi_body.jpg "Figure 23: Coordinate System Definition for Romi Dynamics" width=500

@image html Romi_global.jpg "Figure 24: Global Coordinate System" width=500


The state of Romi was determined to be constituted of of six variables: global X position, global Y position, global heading, centroid displacement (like an odometer), angular velocity of the left wheel, and angular velocity of the right wheel. Using this state breakdown, Romi’s dynamics were analyzed using the following equations:

@image html State_input_output.jpg "Figure 25: Romi State Space, Input, and Output Matrices" width=500

@image html xdot.png "Figure 26: State Derivative Equations" width=500

@image html y.png "Figure 27: Output Equations" width=500


## State Observer   
To implement Romi’s observer in discrete time, the continuous-time system model was first defined using the continuous matrices 𝐴, 𝐵, 𝐶, and 𝐷 obtained from the dynamics above. These matrices depend on measured physical parameters such as wheel radius, 𝑟, and track width, @c w. Although nominal values were initially used, subsequent measurements indicated small deviations; therefore, an updated wheel radius value was substituted into all system equations to improve model accuracy.

### Selecting Observer Dynamics and Constructing the Continuous Model
Because the observer is a fourth-order system, four desired eigenvalues were required for the observer error dynamics. Two eigenvalues were selected as a complex conjugate pair corresponding to a second-order system designed to achieve a target overshoot percentage and settling time. The remaining two eigenvalues were chosen as real poles placed far to the left in the complex plane to ensure rapid decay of the associated modes. This yields a dominant second-order estimation response while maintaining full-order stability.

Using the classical second-order relationships:

\f[
%OS = e^{\left( -\frac{\zeta\pi}{\sqrt{1-\zeta^2}} \right)}, \qquad
t_s \approx \frac{3}{\zeta\omega_n},
\f]

the desired damping ratio, 𝜁, and natural frequency, 𝜔<sub>n</sub>, were computed from the specified overshoot and settling-time requirements. Substituting 𝜁 and 𝜔<sub>n</sub> into the second-order characteristic equation,

\f[
s^2 + 2\zeta\omega_n s + \omega_n^2 = 0,
\f]

produced the first two poles for the observer. Two additional poles were then selected at large negative real values (e.g., 

𝑠
=
−
25
s=−25 and 
𝑠
=
−
30
s=−30), forming the complete fourth-order characteristic polynomial:

\f[
(s - p_1)(s - p_2)(s - p_3)(s - p_4) = 0. \tag{1}
\f]

The roots of Equation (1) served as the desired observer poles.

Using MATLAB’s @c place() function, the continuous-time observer gain matrix, 𝐿, was computed from the matrices 
@c A and @c C and the pole locations defined above.

### Computing A<sub>d</sub> and B<sub>d</sub> from the Continuous Model
With the continuous-time system matrices established, the next step was to obtain the discrete-time equivalents required for implementation in Romi’s observer task. The discrete model has the form

\f[
x[k+1] = A_d x[k] + B_d u[k],
\f]

where the sampling period, Δt, must match the execution period of the observer task in the cooperative scheduler.

#### Step 1 — Forming the Continuous Augmented System

Using standard continuous-to-discrete relationships, the following continuous expressions were first evaluated:

\f[
A_c = A - LC,\qquad
B_c = \begin{bmatrix} B & L \end{bmatrix},
\f]

which correspond to the continuous-time error-dynamics model used for observer prediction and correction.

#### Step 2 — Using MATLAB’s c2d Function

These continuous matrices were discretized using MATLAB’s zero-order hold conversion:

\f[
(A_d,, B_d) = \mathrm{c2d}(A_c,, B_c,, \Delta t).
\f]

MATLAB returns the discrete-time matrices: A<sub>d</sub>, the discrete-time state-transition matrix, and B<sub>d</sub>, the discrete-time input matrix corresponding to the augmented input vector

These matrices were then exported and embedded into the Observer class to perform the real-time update:

\f[
\hat{x}[k+1] = A_d \hat{x}[k] + B_d u[k].
\f]

The resulting discrete model matches the execution rate of the observer task and ensures that Romi’s estimated pose evolves consistently with the continuous-time dynamics.

@image html obs_xuy.png "Figure 28: Observer State Space, Input, and Output Matrices" width=500


# Hardware Setup
If any efforts are made to emulate the Romi setup, the following wiring diagram should be used:

@image html wiring_cn7.png "Figure 29: Wiring Diagram for Romi's C7 Morpho Headers" width=1000

@image html wiring_cn10.png "Figure 30: Wiring Diagram for Romi's C10 Morpho Headers" width=1000
