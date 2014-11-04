import Adafruit_BBIO.UART as UART
import serial

UART.setup("UART2")

ser = serial.Serial(port = "/dev/ttyO2", baudrate=57600)
ser.close()
ser.open()
while 1:
	#line = ser.readline().decode('utf-8')[:-1]
	line = ser.readline()[:-1]
	print(line)
	if line=='exit':
		break
ser.close()
