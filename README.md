# AI-Powered Autonomous Vacuum Cleaner with adaptive floor Robot
6-month  UH IMP project — an autonomous vacuum robot 
using LiDAR SLAM, wheel odometry, and CNN-based floor/dirt classification <br>
**Progress on Project:** Assembly & calibration phase (odometry calibrated, SLAM integration in progress) <br>
components nearly bought progress: 98% <br>

## Hardware
- Raspberry Pi 5 (8GB) — main compute, SLAM, CNN inference, Ir Sesnors
- Arduino Uno R3 — motor control, encoder reading, Analog sensors
- Slamtec RPLIDAR A1M8 — SLAM mapping
- Pi Camera Module 2 — floor/dirt CNN input
- JGA25-370 DC gear motors x2 (65mm wheels, measured 859 ticks/rev)
- ONBO 2800mAh 3S LiPo + BMS
- IR Sensor - wall detection

## Software Architecture

- `hardware/` — sensor and actuator drivers (lidar, motors, camera, sensors)
- `navigation/` — SLAM, path planning, PID control
- `ai/` — CNN floor/dirt classifiers
- `app/` — Flask API + MQTT for mobile app communication

## Calibration Constants
- Ticks per wheel revolution: 859
- mm per encoder tick: 0.2486
- Wheelbase: 173 mm
- Wheel diameter: 65 mm

## Known Issues
- Robot tilts/veers slightly left during straight-line driving (under investigation)
- Loop closure (g2o/GTSAM) not implemented — SLAM currently odometry + LiDAR scan only, tuned for ~2m map size
- LiDAR not yet mounted on chassis — currently tethered via USB/12V on base plate
