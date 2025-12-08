if state == 2:
    # 2. Each motor can be enabled and disabled individually and does not start moving when enabled.
    print("*************************** Test 2 ***************************")
    wait_and_print_info()
    print("Zeroing right and left encoders before enabling motors")
    right_encoder.zero()
    left_encoder.zero()
    wait_and_print_info()

    print("Enable left motor")
    left_motor.enable()
    wait_and_print_info()

    print("left motor effort: 50    -  Right Spin: None      Left Spin: Forward")
    left_motor.set_effort(50)
    wait_and_print_info()

    print("Disable left motor")
    left_motor.disable()
    wait_and_print_info()

    print("Enable right motor")
    right_motor.enable()
    wait_and_print_info()

    print("right motor effort: 50   -  Right Spin: Forward   Left Spin: None")
    right_motor.set_effort(50)
    wait_and_print_info()

    print("Disable right motor")
    right_motor.disable()
    wait_and_print_info()

elif state == 3:
    print("*************************** Test 3 ***************************")
    # 4. Timer reload has no effect on the encoder speed or position
    print("Zeroing right and left encoders before enabling motors")
    right_encoder.zero()
    left_encoder.zero()
    wait_and_print_info()

    print("Setting both encoder timer counts to 0xFFFE")
    # Simulate timer reload by manually setting the timer counter to near max value
    tim2.counter(0xFFFE)
    tim3.counter(0xFFFE)

    print("Enabling both motors")
    left_motor.enable()
    right_motor.enable()
    wait_and_print_info()
    
    print("left and right motor efforts: 50")
    left_motor.set_effort(50)
    right_motor.set_effort(50)
    wait_and_print_info()

    print("Disabling both motors")
    left_motor.disable()
    right_motor.disable()
    wait_and_print_info()

    print("Code Complete")