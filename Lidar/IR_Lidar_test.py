import time
import gpiod
from rplidar import RPLidar, RPLidarException

PORT = '/dev/ttyUSB0'
HOLD_S = 10                      # keep the lidar on this long after the last detection

ir_line = gpiod.find_line("GPIO17")
ir_line.request(consumer="ir_lidar", type=gpiod.LINE_REQ_DIR_IN)

def ir_triggered():
    return ir_line.get_value() == 0      # change to == 1 if your module is reversed

def process_scan(scan):
    # Replace with your BreezySLAM update later
    print(len(scan), "points")

lidar = RPLidar(PORT, timeout=3)
lidar.stop_motor()
print("Waiting for IR trigger...")

def run_session():
    print("IR detected: lidar ON")
    lidar.start_motor()
    time.sleep(2)
    lidar.clean_input()
    last_seen = time.time()
    for scan in lidar.iter_scans(max_buf_meas=5000):
        if ir_triggered():
            last_seen = time.time()
        process_scan(scan)
        if time.time() - last_seen > HOLD_S:
            break
    lidar.stop()
    lidar.stop_motor()
    print("No IR for", HOLD_S, "s: lidar OFF")

try:
    while True:
        if ir_triggered():
            try:
                run_session()
            except RPLidarException as e:
                print("Recovering:", e)
                lidar.stop()
                lidar.reset()
                time.sleep(1.5)
                lidar.clean_input()
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    ir_line.release()
    try:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    except Exception:
        pass