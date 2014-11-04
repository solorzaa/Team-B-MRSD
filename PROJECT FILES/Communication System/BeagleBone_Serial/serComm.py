import Adafruit_BBIO.UART as UART
import serial

UART.setup("UART1")

ser = serial.Serial(port = "/dev/ttyO1", baudrate=57600)
ser.close()
ser.open()
n=0
while ser.isOpen():
	ser.write(str(n)+'\n')
	n+=1
ser.close()
