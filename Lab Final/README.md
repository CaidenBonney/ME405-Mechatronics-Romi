## Lab Final Folder
This folder contains the files necessary for the robot to run as well as supplementary files used to calculate certain performance parameters. Important test data is also stored here.

### Files On Romi
- Runtime code and supporting modules the robot executes: `main.py`, `Path_Director.py`, `Motor_Controller.py`, `Closed_Loop_Control.py`, `Observer.py`, sensor drivers (`Encoder.py`, `Line_Sensor.py`, `IR_Sensor.py`, `IMU.py`, `Battery.py`, `Sensor.py`), utility modules (`Romi_Props.py`, `Garbage_Collector.py`), and shared libraries (`cotask.py`, `task_share.py`).
- Calibration text files (`IMU_cal.txt`, `IR_cal.txt`) plus a local README.

### Calibration Data
- Stored calibration snapshots (e.g., `11.19.25@10.30PM/IMU_cal.txt`, `IR_cal.txt`) captured during setup and testing.

### Not Code
- Supporting analysis/design artifacts such as Simulink/Matlab files (`BlockDiagram.slx`, `GainMatrix.mlx`).

### Console Logs
- `Bluetooth_Connection_Console.txt` and `USB_Connection_Console.txt` capture representative serial I/O sessions for reference/troubleshooting.
