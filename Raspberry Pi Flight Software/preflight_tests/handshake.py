# initiates handshake between pi and nano
# checks connection is good
# recieves logs from nano
# displays them

import time
import serial

ser = serial.Serial("/dev/ttyUSB0",9600)
ser.flushInput()
while ser.isOpen():
    ser.write('A'.encode('utf-8'))
    var = ser.write('A'.encode('utf-8'))
    print(var)
