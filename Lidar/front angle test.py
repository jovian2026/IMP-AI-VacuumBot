import time
from rplidar import RPLidar

lidar = RPLidar('/dev/ttyUSB0', timeout=3)
try:
    lidar.start_motor()
    time.sleep(2)
    lidar.clean_input()
    for i, scan in enumerate(lidar.iter_scans(max_buf_meas=5000)):
        pts = [(d, a) for q, a, d in scan if q > 0 and 150 < d < 4000 and 150 < a < 210] #tigthen up the section to scan front
        if pts:
            d, a = min(pts)          # the nearest object
            print(f"nearest: {d:6.0f} mm at lidar angle {a:6.1f} deg")
        if i >= 15:
            break
finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

