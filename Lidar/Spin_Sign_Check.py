import time, math, serial
from rplidar import RPLidar
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel

A0 = 179
MM_PER_TICK, WHEELBASE_MM = 0.2486, 170

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
time.sleep(2)
ser.reset_input_buffer()
ser.write(b"R\n")

lidar = RPLidar('/dev/ttyUSB0', timeout=3)
slam = RMHC_SLAM(LaserModel(), 500, 4, random_seed=9999)

th = 0.0          # odometry heading, radians, left turn positive
t = 0.0           # SLAM heading, degrees
prev = None

def drain():
    global th, prev
    while ser.in_waiting:
        p = ser.readline().decode(errors='ignore').strip().split(',')
        if len(p) == 4 and p[0] == 'E':
            try:
                l, r = int(p[2]), int(p[3])
            except ValueError:
                continue
            if prev is not None:
                th += ((r - prev[1]) - (l - prev[0])) * MM_PER_TICK / WHEELBASE_MM
            prev = (l, r)

try:
    lidar.start_motor()
    time.sleep(2)
    lidar.clean_input()
    spinning, stop_i = True, 0
    for i, scan in enumerate(lidar.iter_scans(max_buf_meas=5000)):
        drain()
        if spinning:
            if math.degrees(th) < 90:
                ser.write(b"M,-120,120\n")       # spin left
            else:
                ser.write(b"B\n")
                spinning, stop_i = False, i
        dists, angs = [], []
        for q, a, d in scan:
            if q > 0 and 150 < d < 3000:
                dists.append(d)
                angs.append((A0 - a) % 360)
        if len(dists) > 80:
            slam.update(dists, scan_angles_degrees=angs)
        x, y, t = slam.getpos()
        print(f"odometry heading {math.degrees(th):6.1f}   SLAM heading {t:6.1f}")
        if not spinning and i > stop_i + 8:
            break
finally:
    ser.write(b"B\n")
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    ser.close()

print(f"FINAL: odometry {math.degrees(th):.1f} deg   SLAM {t:.1f} deg")