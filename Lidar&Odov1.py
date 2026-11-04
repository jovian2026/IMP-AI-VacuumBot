import time, math, serial
import numpy as np
import matplotlib.pyplot as plt
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar, RPLidarException

# ---------- Settings ----------
LIDAR_PORT = '/dev/ttyUSB0'
UNO_PORT = '/dev/ttyACM0'
A0 = 179                     # lidar angle that points at the robot's front
LIDAR_OFFSET_MM = 100        # + if the lidar is AHEAD of the wheel axle, - if behind
MM_PER_TICK = 0.2486
WHEELBASE_MM = 173.0
MAP_PIXELS = 500
MAP_METERS = 4
MIN_SAMPLES = 80
MIN_DIST_MM, MAX_DIST_MM = 200, 3000
DRIVE_PWM, TURN_PWM = 110, 100
MOVE_S, TURN_S = 1.0, 0.4    # how long one key press drives for

# ---------- SLAM (one with odometry, one lidar-only) ----------
def make_laser():
    try:
        return LaserModel(offset_mm=LIDAR_OFFSET_MM)
    except TypeError:
        print("This BreezySLAM version has no offset_mm, running without it")
        return LaserModel()

slam_odo = RMHC_SLAM(make_laser(), MAP_PIXELS, MAP_METERS,
                     map_quality=50, hole_width_mm=300, random_seed=9999)
slam_lid = RMHC_SLAM(make_laser(), MAP_PIXELS, MAP_METERS,
                     map_quality=50, hole_width_mm=300, random_seed=9999)
mb_odo = bytearray(MAP_PIXELS * MAP_PIXELS)
mb_lid = bytearray(MAP_PIXELS * MAP_PIXELS)

# ---------- Uno link and odometry ----------
ser = serial.Serial(UNO_PORT, 115200, timeout=0.05)
time.sleep(2)                    # the Uno resets when the port opens
ser.reset_input_buffer()
ser.write(b"R\n")

last = None                      # latest (millis, left, right) from the Uno
prev_odo = None

def drain():
    global last
    while ser.in_waiting:
        p = ser.readline().decode(errors='ignore').strip().split(',')
        if len(p) == 4 and p[0] == 'E':
            try:
                last = (int(p[1]), int(p[2]), int(p[3]))
            except ValueError:
                pass

def pose_change():
    """(dxy_mm, dtheta_deg, dt_s) since the last SLAM update, or None."""
    global prev_odo
    if last is None:
        return None
    if prev_odo is None:
        prev_odo = last
        return None
    t0, l0, r0 = prev_odo
    t1, l1, r1 = last
    dt = (t1 - t0) / 1000.0
    if dt <= 0:
        return None
    prev_odo = last
    dl = (l1 - l0) * MM_PER_TICK
    dr = (r1 - r0) * MM_PER_TICK
    return ((dl + dr) / 2.0, math.degrees((dr - dl) / WHEELBASE_MM), dt)

# ---------- Driving with keys (click the plot window first) ----------
cmd, cmd_until, moving = (0, 0), 0.0, False

def on_key(event):
    global cmd, cmd_until
    now = time.time()
    if event.key == 'w':   cmd, cmd_until = (DRIVE_PWM, DRIVE_PWM), now + MOVE_S
    elif event.key == 's': cmd, cmd_until = (-DRIVE_PWM, -DRIVE_PWM), now + MOVE_S
    elif event.key == 'a': cmd, cmd_until = (-TURN_PWM, TURN_PWM), now + TURN_S   # left
    elif event.key == 'd': cmd, cmd_until = (TURN_PWM, -TURN_PWM), now + TURN_S   # right
    elif event.key == 'x': cmd, cmd_until = (0, 0), 0.0

def send_drive():
    global moving
    if time.time() < cmd_until:
        ser.write(f"M,{cmd[0]},{cmd[1]}\n".encode())
        moving = True
    elif moving:
        ser.write(b"B\n")
        moving = False

# ---------- Plot ----------
plt.rcParams['keymap.save'] = []
plt.rcParams['keymap.quit'] = []
plt.ion()
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
blank = np.zeros((MAP_PIXELS, MAP_PIXELS))
img_odo = ax1.imshow(blank, cmap='gray', vmin=0, vmax=255)
img_lid = ax2.imshow(blank, cmap='gray', vmin=0, vmax=255)
ax1.set_title('Lidar + odometry')
ax2.set_title('Lidar only')
dot, = ax1.plot([], [], 'ro', markersize=4)
fig.canvas.mpl_connect('key_press_event', on_key)

def as_image(mb):
    return np.frombuffer(mb, dtype=np.uint8).reshape(MAP_PIXELS, MAP_PIXELS)

def refresh():
    slam_odo.getmap(mb_odo)
    slam_lid.getmap(mb_lid)
    img_odo.set_data(as_image(mb_odo))
    img_lid.set_data(as_image(mb_lid))
    x, y, th = slam_odo.getpos()
    scale = MAP_PIXELS / (MAP_METERS * 1000.0)
    dot.set_data([x * scale], [y * scale])

# ---------- Lidar ----------
def open_lidar():
    l = RPLidar(LIDAR_PORT, timeout=3)
    l.stop(); time.sleep(0.5)
    l.reset(); time.sleep(1.5)
    l.clean_input()
    l.start_motor(); time.sleep(2)
    return l

lidar = open_lidar()
count = 0
try:
    while True:
        try:
            for scan in lidar.iter_scans(max_buf_meas=5000):
                drain()
                send_drive()

                dists, angs = [], []
                for q, a, d in scan:
                    if q > 0 and MIN_DIST_MM < d < MAX_DIST_MM:
                        dists.append(d)
                        angs.append((A0 - a) % 360)

                if len(dists) > MIN_SAMPLES:
                    pc = pose_change()
                    if pc:
                        slam_odo.update(dists, pose_change=pc, scan_angles_degrees=angs)
                    else:
                        slam_odo.update(dists, scan_angles_degrees=angs)
                    slam_lid.update(dists, scan_angles_degrees=angs)

                count += 1
                if count % 3 == 0:
                    refresh()
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
    try:
        ser.write(b"B\n")
    except Exception:
        pass
    stamp = time.strftime("%Y%m%d_%H%M%S")
    for name, s, mb in (("odo", slam_odo, mb_odo), ("lidar", slam_lid, mb_lid)):
        try:
            s.getmap(mb)
            plt.imsave(f"map_{name}_{stamp}.png", as_image(mb), cmap='gray')
        except Exception:
            pass
    try:
        lidar.stop(); lidar.stop_motor(); lidar.disconnect()
    except Exception:
        pass
    ser.close()
    print("Saved maps:", stamp)