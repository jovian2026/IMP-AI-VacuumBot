# AI-Powered Autonomous Vacuum Cleaner with Adaptive Floor Robot
6-month UH IMP project — an autonomous vacuum robot
using LiDAR SLAM, wheel odometry, and CNN-based floor/dirt classification <br>
**Progress on Project:** Assembly & calibration phase (odometry calibrated, SLAM integration in progress, dataset in progress) <br>
**Components:** all bought, waiting for delivery (13 Nov 2026) <br>

## Hardware
- Raspberry Pi 5 (8GB) — main compute, SLAM, CNN inference, IR sensors
- Arduino Uno R3 — motor control, encoder reading, analog sensors
- Slamtec RPLIDAR A1M8 — SLAM mapping
- Pi Camera Module 2 — floor/dirt CNN input
- JGA25-370 DC gear motors x2 (65mm wheels, measured 859 ticks/rev)
- ONBO 3S 11.1V 2800mAh LiPo + 3S BMS (XT60)
- HW-688 5V 5A step-down (Pi 5) + 3.3V regulator
- IR Sensor — wall detection

## Software Architecture
- `hardware/` — sensor and actuator drivers (lidar, motors, camera, sensors)
- `navigation/` — SLAM, path planning, PID control
- `ai/` — CNN floor/dirt classifiers
- `app/` — Flask API + MQTT for mobile app communication

## Calibration Constants
- Ticks per wheel revolution: 859
- mm per encoder tick: 0.2486
- Wheelbase: 173 mm (to recheck on the new chassis)
- Wheel diameter: 65 mm

## Known Issues
- Robot tilts/veers slightly left during straight-line driving (under investigation)
- Loop closure (g2o/GTSAM) not implemented — SLAM currently odometry + LiDAR scan only, tuned for ~2m map size
- LiDAR not yet mounted on chassis — currently tethered via USB/12V on base plate
- Odometry under-reads turns by ~6% with run-to-run scatter
- Map smears after sharp corners and when the robot sits idle with the scan running
- Timed key presses can't hit an exact distance
- Wheelbase, LiDAR offset and LiDAR front angle need recalibrating once the new chassis is printed

## Removed / Changed Scope
- ~~Flutter mobile app~~ — replaced by a Flask dashboard running on the Pi
- ~~Charging station~~ — docking station only (QR code docking with the Pi Camera, no charging circuit)
- ~~IR beacon homing (555 timer)~~ — moved to optional, only if time allows
- ~~ROS2 / TurtleBot~~ — dropped, SLAM is done in Python with BreezySLAM instead
- ~~Arduino Mega~~ — Arduino Uno is enough for motors and encoders
- ~~Raspberry Pi 4~~ — upgraded to Raspberry Pi 5
- ~~Teachable Machine~~ — floor classifier now trained on Edge Impulse
- ~~Pi AI camera / Camera Module 3~~ — not needed, Camera Module 2 from the lab is used


## What's Left
- [ ] Receive components, then mount BMS, 3.3V regulator, step-down and dust sensor
- [ ] Print new chassis, fan mount and camera mount
- [ ] Recalibrate wheelbase, LiDAR offset and front angle
- [ ] Exact distance/angle commands on the Uno
- [ ] Skip SLAM updates while the robot is still
- [ ] Finish floor classifier (Edge Impulse) and run it on the Pi
- [ ] Dirt detection
- [ ] Brush motor and blower control, printed dustbin
- [ ] QR code docking with the Pi Camera (IR beacon only if time)
- [ ] Flask dashboard
- [ ] Final report (25 Nov), viva slides (30 Nov), defence (2 Dec)
