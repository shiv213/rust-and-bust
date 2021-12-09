import serial
import json
import time

with serial.Serial() as ser:
    ser.baudrate = 9600
    ser.port = '/dev/ttyACM0'
    ser.open()
    while True:
        ser.write(b'0')
        raw = ser.readline().decode('utf-8')
        data = json.loads(raw)
        # print(data)
        if abs(data['accel_z']) > 1.99:
            print("VERTICAL")
            time.sleep(1)
        if abs(data['accel_x']) > 1.99:
            print("HORIZONTAL")
            time.sleep(1)
    ser.close()
