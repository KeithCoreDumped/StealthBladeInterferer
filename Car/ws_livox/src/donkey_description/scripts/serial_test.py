import serial

ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)

while True:
    s = ser.readline().decode('utf-8').strip()
    # xiaoba 261 136 375 271
    print("recv:", s)
    