import serial
import json

CONST_THRESHOLD = 5

with serial.Serial() as ser:
    aX = 100
    aY = 100
    aZ = 100
    ser.baudrate = 9600
    print("1")
    ser.port = 'COM5'
    print("2")
    ser.open()
    print("3")
    items = []
    while(abs(aX) > CONST_THRESHOLD or abs(aY) > CONST_THRESHOLD or abs(aZ) > CONST_THRESHOLD):
        ser.write(b'0')
        print("4")
        raw = ser.readline()
        raw = raw.decode('utf-8')
        print(type(raw))
        # stringlist = raw.split("|")
        dictionary = {}
        dict(x.split('=') for x in raw.split('|'))
        for x in raw.split('|'):
            key, value = x.split('=')
            value = float(value.strip())
            dictionary[key.strip()] = value
            # print(key)
            # print(value)
        items.append(dictionary)
        print(raw)
        print(dictionary)
        # raw = raw.decode('utf-8')
        # print(json.loads(dictionary))
        aX = dictionary["aX"]
        aY = dictionary["aY"]
        aZ = dictionary["aZ"]
ser.close()

