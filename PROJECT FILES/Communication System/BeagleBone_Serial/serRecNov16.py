#!/usr/bin/python
import Adafruit_BBIO.UART as UART
import serial

UART.setup("UART2")

ser = serial.Serial(port = "/dev/ttyO2", baudrate=57600)
ser.close()
ser.open()
if ser.isOpen():
			print 'Open: ' + ser.portstr
while 1:
	#line = ser.readline().decode('utf-8')[:-1]
	line = ser.readline()[:-1]
        words=line.split(" ")
	print('timestamp',words[0])
        print(line)
	if line=='exit':
		break
ser.close()
