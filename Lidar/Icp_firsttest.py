import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from rplidar import RPLidar

PORT = '/dev/ttyUSB0'

def get_scan(lidar):
    lidar.stop()
    lidar.clean_input()
    time.sleep(0.5)
    gen = lidar.iter_scans(max_buf_meas=5000)
    for _ in range(3):                      # skip the first partial scans
        scan = next(gen)
    lidar.stop()
    pts = []
    for q, ang, d in scan:
        if q > 0 and 150 < d < 3000:
            a = np.radians(ang)
            pts.append((d * np.cos(a), d * np.sin(a)))
    return np.array(pts)

def icp(A, B, iters=40):
    """Align scan B onto scan A. Returns R, t and the moved B."""
    tree = cKDTree(A)
    cur = B.copy()
    R_tot, t_tot = np.eye(2), np.zeros(2)
    for i in range(iters):
        dist, idx = tree.query(cur)
        thresh = max(100, 500 - i * 15)     # shrink the match radius
        m = dist < thresh
        if m.sum() < 10:
            break
        src, dst = cur[m], A[idx[m]]
        cs, cd = src.mean(0), dst.mean(0)
        H = (src - cs).T @ (dst - cd)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = Vt.T @ U.T
        t = cd - R @ cs
        cur = cur @ R.T + t
        R_tot, t_tot = R @ R_tot, R @ t_tot + t
    return R_tot, t_tot, cur

lidar = RPLidar(PORT, timeout=3)
try:
    lidar.start_motor()
    time.sleep(2)
    input("Keep the lidar still, then press Enter (scan 1)...")
    A = get_scan(lidar)
    input("Now slide it about 10-20 cm FORWARD (no rotation), press Enter (scan 2)...")
    B = get_scan(lidar)
finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

R, t, B_aligned = icp(A, B)
print("Translation (mm):", t.round(1))
print("Rotation (deg):", round(np.degrees(np.arctan2(R[1, 0], R[0, 0])), 2))

fig, ax = plt.subplots(1, 2, figsize=(10, 5))
ax[0].scatter(*A.T, s=4, label='scan 1'); ax[0].scatter(*B.T, s=4, label='scan 2')
ax[0].set_title('Before ICP'); ax[0].legend(); ax[0].set_aspect('equal')
ax[1].scatter(*A.T, s=4, label='scan 1'); ax[1].scatter(*B_aligned.T, s=4, label='scan 2 aligned')
ax[1].set_title('After ICP'); ax[1].legend(); ax[1].set_aspect('equal')
plt.show()