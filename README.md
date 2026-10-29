# AI-Powered Autonomous Vacuum Cleaner with adaptive floor Robot
6-month  UH IMP project — an autonomous vacuum robot 
using LiDAR SLAM, wheel odometry, and CNN-based floor/dirt classification <br>
**Progress on Project:** Assembly & calibration phase (odometry calibrated, SLAM integration in progress) <br>
components nearly bought progress: 98% <br>

## Hardware
- Raspberry Pi 5 (4GB) — main compute, SLAM, CNN inference
- Arduino Uno R3 — motor control, encoder reading
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
