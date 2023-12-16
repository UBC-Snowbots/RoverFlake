#!/usr/bin/env python3

import serial
import time

arduinoData = serial.Serial('/dev/ttyUSB0',115200)

debugData = "$V(100,2,3,4,5,6)"

time.sleep(3)

count = 0

while True:
    arduinoData.write(str(debugData).encode())
    print("Data sent.")
    count +=1
    time.sleep(5)

    if count > 1000:
        break