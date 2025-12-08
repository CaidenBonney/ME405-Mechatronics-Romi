from pyb import Pin, Timer, ADC  # pyright: ignore
import cotask
import task_share
from gc import collect

from motor import Motor
from encoder import Encoder

# Task object imports
from User_Input import UserInput
from Motor_Controller import MotorController
from Data_Transfer import DataTransfer
from Garbage_Collector import GarbageCollector


if __name__ == "__main__":

    # Configure default pins
    Pin(Pin.cpu.C12)  # TXD aka write Pin
    Pin(Pin.cpu.D2)  # RXD aka read Pin

    # Configure button pin
    button_pin = Pin(Pin.cpu.C13, mode=Pin.PULL_NONE)

    # Configure battery ADC pin
    bat_ADC_Pin = Pin(Pin.cpu.C3, mode=Pin.ANALOG)
    battery_ADC = ADC(bat_ADC_Pin)

    ## Creating and setting up the motors and encoders
    # Create motor timer with PWM frequency of 20kHz
    tim4 = Timer(4, freq=20_000)

    # Create motors
    l_motor = Motor(Pin.cpu.B7, Pin.cpu.H1, Pin.cpu.H0, tim4, 2)  # type: ignore
    r_motor = Motor(Pin.cpu.B6, Pin.cpu.A7, Pin.cpu.A6, tim4, 1)  # type: ignore

    # Create encoders and their respective timers for counting pulses
    tim2 = Timer(2, prescaler=0, period=0xFFFF)  # Left encoder counter
    l_encoder = Encoder(tim2, Pin.cpu.A0, 1, Pin.cpu.A1, 2)  # type: ignore
    tim3 = Timer(3, prescaler=0, period=0xFFFF)  # right encoder counter
    r_encoder = Encoder(tim3, Pin.cpu.B4, 3, Pin.cpu.B5, 4)  # type: ignore

    ## Create Shares and Queues required for the tasks
    # Share creation here
    l_flag_s = task_share.Share("B", thread_protect=False, name="left flag share")
    r_flag_s = task_share.Share("B", thread_protect=False, name="right flag share")
    l_speed_s = task_share.Share("f", thread_protect=False, name="left speed share")
    r_speed_s = task_share.Share("f", thread_protect=False, name="right speed share")
    data_transfer_s = task_share.Share("B", thread_protect=True, name="data transfer share")
    test_complete_s = task_share.Share("B", thread_protect=True, name="test complete share")

    # Queue creation here
    # All queues have overwrite=True because if the data is being overwritten because the queue is full, we can see with discontinuous data
    l_time_q = task_share.Queue("L", 1000, thread_protect=False, overwrite=True, name="time queue")
    r_time_q = task_share.Queue("L", 1000, thread_protect=False, overwrite=True, name="time queue")
    l_pos_q = task_share.Queue("f", 1000, thread_protect=False, overwrite=True, name="left position queue")
    r_pos_q = task_share.Queue("f", 1000, thread_protect=False, overwrite=True, name="right position queue")
    l_vel_q = task_share.Queue("f", 1000, thread_protect=False, overwrite=True, name="left velocity queue")
    r_vel_q = task_share.Queue("f", 1000, thread_protect=False, overwrite=True, name="right velocity queue")

    ## Create objects of each task for the Task objects
    user_input_obj = UserInput(button_pin)
    LMC_obj = MotorController(l_motor, l_encoder, battery_ADC, False)  # False = left
    RMC_obj = MotorController(r_motor, r_encoder, battery_ADC, True)  # True = right
    data_transfer_obj = DataTransfer()
    garbage_collector_obj = GarbageCollector()

    ## Create Task objects
    # Create the tasks. If trace is enabled for any task, memory will be
    # allocated for state transition tracing, and the application will run out
    # of memory after a while and quit. Therefore, use tracing only for
    # debugging and set trace to False when it's not needed
    task_User_Input = cotask.Task(
        user_input_obj.run,
        name="User Input Task            ",
        priority=10,
        period=50,
        profile=True,
        trace=False,
        shares=(l_flag_s, r_flag_s, l_speed_s, r_speed_s, data_transfer_s, test_complete_s),
    )
    task_LMC = cotask.Task(
        LMC_obj.run,
        name="Left Motor Controller Task ",
        priority=2,
        period=20,
        profile=True,
        trace=False,
        shares=(l_flag_s, l_speed_s, data_transfer_s, test_complete_s, l_time_q, l_pos_q, l_vel_q),
    )
    task_RMC = cotask.Task(
        RMC_obj.run,
        name="Right Motor Controller Task",
        priority=2,
        period=20,
        profile=True,
        trace=False,
        shares=(r_flag_s, r_speed_s, data_transfer_s, test_complete_s, r_time_q, r_pos_q, r_vel_q),
    )
    task_Data_Transfer = cotask.Task(
        data_transfer_obj.run,
        name="Data Transfer Task         ",
        priority=1,
        period=20,
        profile=True,
        trace=False,
        shares=(data_transfer_s, test_complete_s, l_time_q, r_time_q, l_pos_q, r_pos_q, l_vel_q, r_vel_q),
    )
    task_Garbage_Collection = cotask.Task(
        garbage_collector_obj.run,
        name="Garbage Collection Task    ",
        priority=0,
        period=0,
        profile=True,
        trace=False,
        shares=None,
    )

    cotask.task_list.append(task_User_Input)
    cotask.task_list.append(task_LMC)
    cotask.task_list.append(task_RMC)
    cotask.task_list.append(task_Data_Transfer)
    cotask.task_list.append(task_Garbage_Collection)

    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    collect()

    # Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
    while True:
        try:
            cotask.task_list.pri_sched()
        except (Exception, KeyboardInterrupt) as e:
            LMC_obj.motor.set_effort(0)
            RMC_obj.motor.set_effort(0)

            list_of_queues = [l_time_q, r_time_q, l_pos_q, r_pos_q, l_vel_q, r_vel_q]
            for queue in list_of_queues:
                queue.clear()

            # Print a table of task data and a table of shared information data
            print("\n" + str(cotask.task_list))
            print(task_share.show_all())
            print(task_User_Input.get_trace())
            print("")
            print(task_LMC.get_trace())
            print("")
            print(task_RMC.get_trace())
            print("")
            print(task_Data_Transfer.get_trace())
            print("")
            print(task_Garbage_Collection.get_trace())
            print("")

            raise e
