import time
import numpy as np
import matplotlib.pyplot as plt
import gpiod
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar, RPLidarException

# ---------- Settings ----------
PORT = '/dev/ttyUSB0'
HOLD_S = 10                  # keep mapping this long after the last IR detection
MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 3          # your tuned small-room value
MIN_SAMPLES = 80
MAX_DIST_MM = 3000
MIN_DIST_MM = 200
IR_ACTIVE_VALUE = 0          # change to 1 if your module is reversed

# ---------- IR sensor (gpiod) ----------
ir_line = gpiod.find_line("GPIO17")
ir_line.request(consumer="ir_lidar_slam", type=gpiod.LINE_REQ_DIR_IN)

def ir_triggered():
    return ir_line.get_value() == IR_ACTIVE_VALUE

# ---------- SLAM + live map ----------
slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS,
                 map_quality=50, hole_width_mm=300, random_seed=9999)
mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

plt.ion()
fig, ax = plt.subplots()
img = ax.imshow(np.zeros((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)),
                cmap='gray', vmin=0, vmax=255)

def refresh_map():
    slam.getmap(mapbytes)
    img.set_data(np.reshape(np.frombuffer(mapbytes, dtype=np.uint8),
                            (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)))
    plt.pause(0.001)

def save_map():
    slam.getmap(mapbytes)
    name = time.strftime("map_%Y%m%d_%H%M%S.png")
    plt.imsave(name, np.reshape(np.frombuffer(mapbytes, dtype=np.uint8),
                                (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)), cmap='gray')
    print("Saved", name)

# ---------- Lidar ----------
def open_lidar():
    l = RPLidar(PORT, timeout=3)
    l.stop()
    time.sleep(0.5)
    l.reset()
    time.sleep(1.5)
    l.clean_input()
    return l

lidar = open_lidar()
lidar.stop_motor()

def run_session():
    print("IR detected: lidar ON, mapping")
    lidar.start_motor()
    time.sleep(2)                       # let the motor reach full speed
    lidar.clean_input()
    last_seen = time.time()
    count = 0
    for scan in lidar.iter_scans(max_buf_meas=5000):
        if ir_triggered():
            last_seen = time.time()

        distances, angles = [], []
        for quality, angle, dist in scan:
            if quality > 0 and MIN_DIST_MM < dist < MAX_DIST_MM:
                distances.append(dist)
                angles.append(angle)

        if len(distances) > MIN_SAMPLES:
            slam.update(distances, scan_angles_degrees=angles)

        count += 1
        if count % 5 == 0:
            refresh_map()

        if time.time() - last_seen > HOLD_S:
            break

    lidar.stop()
    lidar.stop_motor()
    refresh_map()
    save_map()
    print("No IR for", HOLD_S, "s: lidar OFF")

# ---------- Main loop ----------
print("Waiting for IR trigger...")
try:
    while True:
        if ir_triggered():
            try:
                run_session()
            except RPLidarException as e:
                print("Recovering:", e)
                try:
                    lidar.stop_motor()
                    lidar.disconnect()
                except Exception:
                    pass
                time.sleep(1)
                lidar = open_lidar()
                lidar.stop_motor()
        plt.pause(0.05)                 # keeps the map window responsive while idle
except KeyboardInterrupt:
    pass
finally:
    try:
        save_map()
    except Exception:
        pass
    ir_line.release()
    try:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    except Exception:
        pass