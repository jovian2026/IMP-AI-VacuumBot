from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar
import numpy as np
import matplotlib.pyplot as plt

MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 10
MIN_SAMPLES = 100

lidar = RPLidar('/dev/ttyUSB0')
slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

plt.ion()
fig, ax = plt.subplots()
img = ax.imshow(np.zeros((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)), cmap='gray', vmin=0, vmax=255)

try:
    for scan in lidar.iter_scans():
        distances = [item[2] for item in scan]
        angles = [item[1] for item in scan]
        if len(distances) > MIN_SAMPLES:
            slam.update(distances, scan_angles_degrees=angles)
        x, y, theta = slam.getpos()
        slam.getmap(mapbytes)
        img.set_data(np.reshape(np.frombuffer(mapbytes, dtype=np.uint8),
                                (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)))
        plt.pause(0.01)
except KeyboardInterrupt:
    pass
finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()