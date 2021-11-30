import serial
import json

with serial.Serial() as ser:
    ser.baudrate = 9600
    ser.port = '/dev/ttyACM0'
    ser.open()
    ser.write(b'0')
    raw = ser.readline().decode('utf-8')
    print(json.loads(raw))
    ser.close()

