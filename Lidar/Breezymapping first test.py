import time
import numpy as np
import matplotlib.pyplot as plt
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar, RPLidarException

PORT = '/dev/ttyUSB0'
MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 6
MIN_SAMPLES = 80
MAX_DIST_MM = 5500
MIN_DIST_MM = 150

def open_lidar():
    l = RPLidar(PORT, timeout=3)
    l.stop()
    time.sleep(0.5)
    l.reset()
    time.sleep(1.5)
    l.clean_input()
    l.start_motor()
    time.sleep(2)          # let the motor reach full speed
    return l

lidar = open_lidar()
slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS,random_seed=9999)
mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

plt.ion()
fig, ax = plt.subplots()
img = ax.imshow(np.zeros((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)),
                cmap='gray', vmin=0, vmax=255)

count = 0
try:
    while True:
        try:
            for scan in lidar.iter_scans(max_buf_meas=5000):
                #updated line
                distances = []
                angles = []
                for quality, angle, dist in scan:
                    if quality > 0 and MIN_DIST_MM < dist < MAX_DIST_MM:
                        distances.append(dist)
                        angles.append(angle)
                        
                print(len(distances), "points")
                if len(distances) > MIN_SAMPLES:
                    slam.update(distances, scan_angles_degrees=angles)
                count += 1
                if count % 5 == 0:
                    slam.getmap(mapbytes)
                    img.set_data(np.reshape(
                        np.frombuffer(mapbytes, dtype=np.uint8),
                        (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)))
                    plt.pause(0.001)
        except RPLidarException as e:
            print("Recovering:", e)
            try:
                lidar.stop_motor()
                lidar.disconnect()
            except Exception:
                pass
            time.sleep(1)
            lidar = open_lidar()
except KeyboardInterrupt:
    pass
finally:
    #updated line
    try:
        slam.getmap(mapbytes)
        plt.imsave('map_result.png',
                   np.reshape(np.frombuffer(mapbytes, dtype=np.uint8),
                              (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)),
                   cmap='gray')
    except Exception:
        pass
    try:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    except Exception:
        pass