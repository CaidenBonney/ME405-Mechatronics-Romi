## @file main.py
#  Entry point for Romi control firmware. Configures hardware, initializes
#  sensors/actuators, creates cooperative tasks, and starts the scheduler.
#  @author Antonio Ventimiglia
#  @author Caiden Bonney
#  @date   2025-Dec-12
#  @copyright GPLv3

from pyb import Pin, Timer, ADC, I2C, UART  # pyright: ignore
import cotask
import task_share
from gc import collect

from Romi_Props import RomiProps
from Motor import Motor
from Encoder import Encoder
from Battery import Battery
from Line_Sensor import LineSensor
from IR_Sensor import IRSensor
from IMU import IMU

collect()

if __name__ == "__main__":

    # Configure default pins
    Pin(Pin.cpu.C12)  # TXD aka write Pin
    Pin(Pin.cpu.D2)  # RXD aka read Pin

    # Configure button pin
    button_pin = Pin(Pin.cpu.C13, mode=Pin.PULL_NONE)

    # Configure battery ADC
    bat_ADC_Pin = Pin(Pin.cpu.C3, mode=Pin.ANALOG)
    Battery_obj = Battery(ADC(bat_ADC_Pin))

    ## Creating and setting up the motors and encoders
    # Create motor timer with PWM frequency of 20kHz
    tim4 = Timer(4, freq=20_000)

    # Create motors
    l_motor = Motor(Pin.cpu.B7, Pin.cpu.H1, Pin.cpu.H0, tim4, 2)
    r_motor = Motor(Pin.cpu.B6, Pin.cpu.A7, Pin.cpu.A6, tim4, 1)

    # Create encoders and their respective timers for counting pulses
    tim2 = Timer(2, prescaler=0, period=0xFFFF)  # Left encoder counter
    l_encoder = Encoder(l_motor, tim2, Pin.cpu.A0, 1, Pin.cpu.A1, 2)
    tim3 = Timer(3, prescaler=0, period=0xFFFF)  # right encoder counter
    r_encoder = Encoder(r_motor, tim3, Pin.cpu.B4, 3, Pin.cpu.B5, 4)

    # Create IR sensor objects
    IRSensor1 = IRSensor(1, Pin.cpu.A4)
    # IRSensor2 = IRSensor(2, Pin.cpu.A3) # RANDOM PIN SELECTED
    IRSensor3 = IRSensor(3, Pin.cpu.B0)
    # IRSensor4 = IRSensor(4, Pin.cpu.A5) # RANDOM PIN SELECTED
    IRSensor5 = IRSensor(5, Pin.cpu.C1)
    # IRSensor6 = IRSensor(6, Pin.cpu.A7) # RANDOM PIN SELECTED
    IRSensor7 = IRSensor(7, Pin.cpu.C0)
    # IRSensor8 = IRSensor(8, Pin.cpu.A9) # RANDOM PIN SELECTED
    IRSensor9 = IRSensor(9, Pin.cpu.C4)
    # IRSensor10 = IRSensor(10, Pin.cpu.A11) # RANDOM PIN SELECTED
    IRSensor11 = IRSensor(11, Pin.cpu.B1)
    # IRSensor12 = IRSensor(12, Pin.cpu.A13) # RANDOM PIN SELECTED
    IRSensor13 = IRSensor(13, Pin.cpu.C5)

    # Linesensor = LineSensor([IRSensor1, IRSensor2, IRSensor3, IRSensor4, IRSensor5, IRSensor6, IRSensor7, IRSensor8, IRSensor9, IRSensor10, IRSensor11, IRSensor12, IRSensor13])
    Linesensor = LineSensor([IRSensor1, IRSensor3, IRSensor5, IRSensor7, IRSensor9, IRSensor11, IRSensor13])

    # Create IMU object
    I2C_obj = I2C(2, I2C.CONTROLLER)
    SCL2_pin = Pin(Pin.cpu.B10, Pin.ALT, alt=4)
    SDA2_pin = Pin(Pin.cpu.B11, Pin.ALT, alt=4)
    IMU_reset_pin = Pin(Pin.cpu.B2, Pin.OUT_PP)
    IMU_reset_pin.value(1)
    IMU_obj = IMU(I2C_obj, IMU_reset_pin)

    # Setting inital encoder positions based on initial IMU heading
    l_encoder.position = -RomiProps.wdiv2 * (IMU_obj.get_heading())
    r_encoder.position = RomiProps.wdiv2 * (IMU_obj.get_heading())

    # Create bump sensor pins
    bump0 = Pin(Pin.cpu.C7, mode=Pin.IN)
    bump1 = Pin(Pin.cpu.B8, mode=Pin.IN)
    bump2 = Pin(Pin.cpu.C9, mode=Pin.IN)
    bump3 = Pin(Pin.cpu.C11, mode=Pin.IN)
    bump4 = Pin(Pin.cpu.C10, mode=Pin.IN)
    bump5 = Pin(Pin.cpu.A15, mode=Pin.IN)

    bump_sensors = (bump0, bump1, bump2, bump3, bump4, bump5)

    ## Create Shares and Queues required for the tasks
    # Share creation here
    l_flag_s = task_share.Share("B", thread_protect=False, name="left flag share")
    r_flag_s = task_share.Share("B", thread_protect=False, name="right flag share")
    l_speed_s = task_share.Share("f", thread_protect=False, name="left speed share")
    r_speed_s = task_share.Share("f", thread_protect=False, name="right speed share")
    data_transfer_s = task_share.Share("B", thread_protect=True, name="data transfer share")
    test_complete_s = task_share.Share("B", thread_protect=True, name="test complete share")
    cal_white_s = task_share.Share("B", thread_protect=True, name="white calibration share")
    cal_black_s = task_share.Share("B", thread_protect=True, name="black calibration share")
    set_seg_s = task_share.Share("H", thread_protect=True, name="new segment share")
    seg_start_s = task_share.Share("B", thread_protect=True, name="segment start share")
    obsd_lpos_s = task_share.Share("f", thread_protect=False, name="Oberserved left position share")
    obsd_rpos_s = task_share.Share("f", thread_protect=False, name="Oberserved right position share")
    obsd_cpos_s = task_share.Share("f", thread_protect=False, name="Oberserved center position share")
    obsd_yaw_s = task_share.Share("f", thread_protect=False, name="Oberserved yaw share")
    obsd_yawrate_s = task_share.Share("f", thread_protect=False, name="Oberserved yawrate share")
    obsd_X_s = task_share.Share("f", thread_protect=False, name="Oberserved X position share")
    obsd_Y_s = task_share.Share("f", thread_protect=False, name="Oberserved Y position share")
    dist_yaw_s = task_share.Share("f", thread_protect=False, name="distance derived yaw share")

    ## Create objects of each task for the Task objects
    # Collect garbage data for defragmentation before large imports and object creation
    collect()
    from Path_Director import PathDirector
    collect()

    # Task object imports
    from User_Input import UserInput
    from Observer import Observer
    from Motor_Controller import MotorController
    from Garbage_Collector import GarbageCollector

    collect()

    user_input_obj = UserInput(button_pin, Battery_obj)
    observer_obj = Observer(IMU_obj, l_encoder, r_encoder, Battery_obj)
    path_director_obj = PathDirector(Linesensor, IMU_obj, bump_sensors)
    LMC_obj = MotorController(l_motor, l_encoder, Battery_obj, False)  # False = left
    RMC_obj = MotorController(r_motor, r_encoder, Battery_obj, True)  # True = right
    garbage_collector_obj = GarbageCollector()

    ## Create Task objects
    # Create the tasks. If trace is enabled for any task, memory will be
    # allocated for state transition tracing, and the application will run out
    # of memory after a while and quit. Therefore, use tracing only for
    # debugging and set trace to False when it's not needed
    task_User_Input = cotask.Task(
        user_input_obj.run,
        name="User Input Task      ",
        priority=10,
        period=100,
        profile=True,
        trace=False,
        shares=(
            l_flag_s,
            r_flag_s,
            l_speed_s,
            r_speed_s,
            data_transfer_s,
            test_complete_s,
            cal_white_s,
            cal_black_s,
            set_seg_s,
        ),
    )
    task_Observer = cotask.Task(
        observer_obj.run,
        name="Observer Task        ",
        priority=4,
        period=20,
        profile=True,
        trace=False,
        shares=(
            obsd_lpos_s,
            obsd_rpos_s,
            obsd_cpos_s,
            obsd_yaw_s,
            obsd_yawrate_s,
            obsd_X_s,
            obsd_Y_s,
            dist_yaw_s,
        ),
    )
    task_Path_Director = cotask.Task(
        path_director_obj.run,
        name="Path Director Task   ",
        priority=3,
        period=30,
        profile=True,
        trace=False,
        shares=(
            l_flag_s,
            l_speed_s,
            r_flag_s,
            r_speed_s,
            data_transfer_s,
            test_complete_s,
            cal_white_s,
            cal_black_s,
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
        ),
    )
    task_LMC = cotask.Task(
        LMC_obj.run,
        name="Left Motor Cont Task ",
        priority=2,
        period=20,
        profile=True,
        trace=False,
        shares=(l_flag_s, l_speed_s, data_transfer_s, test_complete_s, seg_start_s),
    )
    task_RMC = cotask.Task(
        RMC_obj.run,
        name="Right Motor Cont Task",
        priority=2,
        period=20,
        profile=True,
        trace=False,
        shares=(r_flag_s, r_speed_s, data_transfer_s, test_complete_s, seg_start_s),
    )
    task_Garbage_Collection = cotask.Task(
        garbage_collector_obj.run,
        name="Garbage Collect Task ",
        priority=0,
        period=0,
        profile=True,
        trace=False,
        shares=None,
    )

    cotask.task_list.append(task_User_Input)
    cotask.task_list.append(task_Observer)
    cotask.task_list.append(task_Path_Director)
    cotask.task_list.append(task_LMC)
    cotask.task_list.append(task_RMC)
    cotask.task_list.append(task_Garbage_Collection)

    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    collect()

    # Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
    while True:
        try:
            cotask.task_list.pri_sched()
        except BaseException as e:
            uart = UART(5, 115200)
            
            # If the error is an OSError, print notify the user and soft reset
            if isinstance(e, OSError):
                print(f"OSError: {e}")
                uart.write(f"OSError: {e}\r\n".encode("utf-8"))
                from machine import soft_reset  # pyright: ignore

                soft_reset()

            LMC_obj.motor.set_effort(0)
            RMC_obj.motor.set_effort(0)

            # Print a table of task data and a table of shared information data
            print("\n" + str(cotask.task_list))
            print(task_share.show_all())
            # print(task_User_Input.get_trace())
            # print("")
            # print(task_Observer.get_trace())
            # print("")
            # print(task_Path_Director.get_trace())
            # print("")
            # print(task_LMC.get_trace())
            # print("")
            # print(task_RMC.get_trace())
            # print("")
            # print(task_Garbage_Collection.get_trace())
            # print("")

            # Print the current battery voltage and percentage after every error including keyboard interrupts
            print(f"Battery - %:{Battery_obj.get_cur_perc()}, V:{Battery_obj.get_cur_volt()}\n")

            # Write the error type and message to the UART when only bluetooth is connected
            uart.write(f"Error: {type(e)} - {e}\r\n".encode("utf-8"))
            raise e
