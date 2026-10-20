import serial, time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  # the Uno resets when the port opens

ser.write(b"hello\n")
print(ser.readline().decode().strip())