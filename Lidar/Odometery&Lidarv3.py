import glob, time, math, serial
import numpy as np
import matplotlib.pyplot as plt
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar

# ---------- Settings ----------
UNO_PORT = '/dev/ttyACM0'
A0 = 179                      # lidar angle that points at the robot's front
LIDAR_OFFSET_MM = 20         # + if the lidar is AHEAD of the wheel axle, - if behind
MM_PER_TICK = 0.230
WHEELBASE_MM = 157.0
MAP_PIXELS = 500
MAP_METERS = 4
MIN_SAMPLES = 80
MIN_DIST_MM, MAX_DIST_MM = 200, 3000
DRIVE_PWM, TURN_PWM = 110, 100
STEP_MM, STEP_DEG = 100, 15              # one press of w/s, one press of a/d
ROLL_FWD_TICKS, ROLL_TURN_TICKS = 50, 29 # step goes too far -> raise, too short -> lower
MAX_TURN_DPS = 40                        # no map updates while turning faster than this

# ---------- SLAM: one with odometry, one lidar-only ----------
slam_odo = RMHC_SLAM(LaserModel(), MAP_PIXELS, MAP_METERS,
                     map_quality=50, hole_width_mm=150, random_seed=9999,sigma_xy_mm=40, sigma_theta_degrees=5)
slam_lid = RMHC_SLAM(LaserModel(), MAP_PIXELS, MAP_METERS,
                     map_quality=50, hole_width_mm=150, random_seed=9999) # hole width for thinner walls
mb_odo = bytearray(MAP_PIXELS * MAP_PIXELS)
mb_lid = bytearray(MAP_PIXELS * MAP_PIXELS)

# ---------- Uno link ----------
ser = serial.Serial(UNO_PORT, 115200, timeout=0.05)
time.sleep(2)                    # the Uno resets when the port opens
ser.reset_input_buffer()
ser.write(b"R\n")
time.sleep(0.3)
ser.reset_input_buffer()

last = None                      # latest (millis, left, right)
prev_odo = None
raw_prev = None
rx = ry = rth = 0.0              # raw dead-reckoned pose: mm, mm, radians
zero = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
busy, busy_since, report_at = False, 0.0, 0.0

def integrate_raw(l, r):
    global raw_prev, rx, ry, rth
    if raw_prev is not None:
        dl = (l - raw_prev[0]) * MM_PER_TICK
        dr = (r - raw_prev[1]) * MM_PER_TICK
        d = (dl + dr) / 2.0
        dth = (dr - dl) / WHEELBASE_MM
        rx += d * math.cos(rth + dth / 2)
        ry += d * math.sin(rth + dth / 2)
        rth += dth
    raw_prev = (l, r)

def drain():
    global last, busy, report_at
    while ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        if line in ('D', 'X'):
            busy = False
            report_at = time.time() + 0.6        # print the poses once it has settled
            continue
        p = line.split(',')
        if len(p) == 4 and p[0] == 'E':
            try:
                t, l, r = int(p[1]), int(p[2]), int(p[3])
            except ValueError:
                continue
            last = (t, l, r)
            integrate_raw(l, r)
    if busy and time.time() - busy_since > 12:
        busy = False

def pose_change():
    """(dxy_mm, dtheta_deg, dt_s) since the last call, or None."""
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

def slam_pose():
    sx, sy, sth = slam_odo.getpos()
    half = MAP_METERS * 500.0
    return sx - half, sy - half, sth

def mark_zero():
    global zero
    sx, sy, sth = slam_pose()
    zero = (rx, ry, rth, sx, sy, sth)
    print("Zero marked. Press w / a / d, then compare with the tape.")

def report():
    sx, sy, sth = slam_pose()
    zx, zy, zth, zsx, zsy, zsth = zero
    print(f"raw odo: dist={math.hypot(rx - zx, ry - zy):6.1f} mm  th={math.degrees(rth - zth):6.1f} deg"
          f"   |   SLAM: dist={math.hypot(sx - zsx, sy - zsy):6.1f} mm  th={sth - zsth:6.1f} deg")

# ---------- Driving: exact steps, run on the Uno (click the plot window first) ----------
def move_mm(mm, pwm=DRIVE_PWM):
    global busy, busy_since
    ticks = max(0, int(abs(mm) / MM_PER_TICK) - ROLL_FWD_TICKS)
    ser.write(f"F,{ticks if mm >= 0 else -ticks},{pwm}\n".encode())
    busy, busy_since = True, time.time()

def turn_deg(deg, pwm=TURN_PWM):
    global busy, busy_since
    ticks = int(math.radians(abs(deg)) * WHEELBASE_MM / 2 / MM_PER_TICK) - ROLL_TURN_TICKS
    ticks = max(0, ticks)
    ser.write(f"T,{ticks if deg >= 0 else -ticks},{pwm}\n".encode())
    busy, busy_since = True, time.time()

def on_key(event):
    global busy
    k = event.key
    if k == 'x':
        ser.write(b"B\n")
        busy = False
        return
    if k == 'z':
        mark_zero()
        return
    if busy:
        return                       # wait for the current step to finish
    if   k == 'w': move_mm(STEP_MM)
    elif k == 's': move_mm(-STEP_MM)
    elif k == 'a': turn_deg(STEP_DEG)        # left
    elif k == 'd': turn_deg(-STEP_DEG)       # right

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
def lidar_port():
    while True:
        ports = glob.glob('/dev/serial/by-id/*CP210*')
        if ports:
            return ports[0]
        print("Waiting for the lidar...")
        time.sleep(1)

def open_lidar():
    l = RPLidar(lidar_port(), timeout=3)
    l.stop(); time.sleep(0.5)
    l.reset(); time.sleep(1.5)
    l.clean_input()
    l.start_motor(); time.sleep(2)
    return l

print("Keys: z = mark zero, w/s = 100 mm, a/d = 15 deg, x = stop (click the plot window first)")
lidar = open_lidar()
count = 0
settle = 0
try:
    while True:
        try:
            for scan in lidar.iter_scans(max_buf_meas=5000):
                drain()
                if report_at and time.time() >= report_at:
                    report_at = 0.0
                    report()

                dists, angs = [], []
                for q, a, d in scan:
                    if q > 0 and MIN_DIST_MM < d < MAX_DIST_MM:
                        phi = math.radians((A0 - a) % 360)        # robot frame, left is positive
                        x = d * math.cos(phi) + LIDAR_OFFSET_MM   # lidar is ahead of the axle
                        y = d * math.sin(phi)
                        dists.append(math.hypot(x, y))
                        angs.append(math.degrees(math.atan2(y, x)) % 360)

                if len(dists) > MIN_SAMPLES:
                    pc = pose_change()
                    if pc is None:
                        slam_odo.update(dists, scan_angles_degrees=angs)          # first scan
                    else:
                        dxy, dth, dt = pc
                        moving_now = abs(dxy) > 1.0 or abs(dth) > 0.3
                        if moving_now:
                            settle = 4                                            # two extra scans once it stops
                        if moving_now or settle > 0:
                            fast = abs(dth) / dt > MAX_TURN_DPS
                            slam_odo.update(dists, pose_change=pc,
                                            scan_angles_degrees=angs,
                                            should_update_map=not moving_now)
                            if not moving_now:
                                settle -= 1
                    slam_lid.update(dists, scan_angles_degrees=angs)              # lidar-only baseline always runs

                count += 1
                if count % 3 == 0:
                    refresh()
                plt.pause(0.001)
                if not plt.fignum_exists(fig.number):
                    raise KeyboardInterrupt                                       # window closed
        except Exception as e:
            print("Lidar hiccup:", e)
            try:
                lidar.stop()                 # stops scanning, the motor keeps spinning
                lidar.clean_input()
            except Exception:                # the port itself has gone, so reopen it
                try:
                    lidar.stop_motor(); lidar.disconnect()
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