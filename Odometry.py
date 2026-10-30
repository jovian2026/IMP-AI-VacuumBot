import time, math, serial

PORT = '/dev/ttyACM0'
MM_PER_TICK  = 0.2486
WHEELBASE_MM = 173.0
TARGET_MM    = 1000
DRIVE_PWM    = 120
STOP_EARLY_MM = 12     # brake roll
#STOP_EARLY_MM = 13 (new update)     

ser = serial.Serial(PORT, 115200, timeout=0.1)
time.sleep(2)               # the Uno resets when the port opens
ser.reset_input_buffer()
ser.write(b"R\n")

x = y = th = 0.0            # mm, mm, radians (left turn positive)
prev = None

def read_ticks():
    line = ser.readline().decode(errors='ignore').strip()
    p = line.split(',')
    if len(p) == 4 and p[0] == 'E':
        try:
            return int(p[1]), int(p[2]), int(p[3])
        except ValueError:
            return None
    return None

def update(t_ms, l, r):
    global prev, x, y, th
    if prev is None:
        prev = (t_ms, l, r)
        return
    dl = (l - prev[1]) * MM_PER_TICK
    dr = (r - prev[2]) * MM_PER_TICK
    prev = (t_ms, l, r)
    d = (dl + dr) / 2
    dth = (dr - dl) / WHEELBASE_MM
    x += d * math.cos(th + dth / 2)
    y += d * math.sin(th + dth / 2)
    th += dth

driving = True
last_cmd = 0
last_print = 0
t_stop = None
try:
    while True:
        msg = read_ticks()
        if msg:
            update(*msg)
        now = time.time()
        dist = math.hypot(x, y)

        if driving:
            if dist >= TARGET_MM - STOP_EARLY_MM:
                ser.write(b"B\n")
                driving = False
                t_stop = now
            elif now - last_cmd > 0.1:
                ser.write(f"M,{DRIVE_PWM},{DRIVE_PWM}\n".encode())
                last_cmd = now
        elif now - t_stop > 1.5:
            break

        if now - last_print > 0.25:
            last_print = now
            print(f"x={x:7.1f} mm  y={y:6.1f} mm  heading={math.degrees(th):6.1f} deg")
finally:
    ser.write(b"B\n")
    ser.close()

print(f"FINAL: x={x:.1f} mm  y={y:.1f} mm  heading={math.degrees(th):.1f} deg")
