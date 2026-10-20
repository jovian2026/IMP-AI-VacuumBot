import serial, time

ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
time.sleep(2)

while True:
    line = ser.readline().decode().strip()
    if line:
        print("IR:", line)