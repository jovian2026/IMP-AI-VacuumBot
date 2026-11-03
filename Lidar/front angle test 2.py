import time
from rplidar import RPLidar

lidar = RPLidar('/dev/ttyUSB0', timeout=3)
try:
    lidar.start_motor()
    time.sleep(2)
    lidar.clean_input()
    best = [None] * 12                      # nearest distance in each 30 degree sector
    for i, scan in enumerate(lidar.iter_scans(max_buf_meas=5000)):
        if i < 3:
            continue                        # skip the first partial scans
        for q, a, d in scan:
            if q > 0 and 150 < d < 4000:
                s = int(a // 30) % 12
                if best[s] is None or d < best[s]:
                    best[s] = d
        if i >= 15:
            break
    for s in range(12):
        txt = '---' if best[s] is None else f"{int(best[s])} mm"
        print(f"{s*30:3d}-{s*30+30:3d} deg : {txt}")
finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()