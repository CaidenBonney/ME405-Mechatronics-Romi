California Polytechnic State University San Luis Obispo Mechanical Engineering Course, ME405 Mechatronics. Romi Project. 

This repository holds code used in the ME405 course at Cal Poly.

## Webpage
https://caidenbonney.github.io/ME405-Mechatronics-Romi/html/index.html

## Firmware Files
The firmware files are located in the `Full Reset Firmware` folder. This firmware file was provided by... This firmware allows the user to upload files to the Shoe of Brain and then run the code on the Romi.

## Runtime Code Overview
Core files that run on the Romi live in `Lab Final/Files On Romi/`:
- `main.py`: configures hardware, builds tasks, and starts the scheduler.
- `Path_Director.py` / `Path_Director_vars.py`: top-level state machine and shared segment parameters.
- `Motor_Controller.py`, `Motor.py`, `Encoder.py`: closed-loop motor control stack and encoder interface.
- `Closed_Loop_Control.py`: generic PID/PI/P control with feed-forward, anti-windup, and droop compensation.
- `Line_Sensor.py`, `IR_Sensor.py`: line array aggregation and IR sensor normalization.
- `IMU.py`: BNO055 driver for heading and yaw rate; handles calibration storage.
- `Observer.py`: state observer that fuses sensors to estimate pose.
- `User_Input.py`: UART/Bluetooth command handler for calibration and tuning.
- `Battery.py`, `Romi_Props.py`, `Sensor.py`, `Garbage_Collector.py`: support utilities and shared constants.
