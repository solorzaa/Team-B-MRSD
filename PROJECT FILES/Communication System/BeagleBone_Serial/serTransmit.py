import serial

#####Global Variables######################################
#be sure to declare the variable as 'global var' in the fxn
ser = 0

#####FUNCTIONS#############################################
#initialize serial connection 
def init_serial():
	
	global ser #must be declared in each fxn used
	ser = serial.Serial()
	ser.baudrate = 57600
	ser.port = '/dev/ttyUSB0'
	
	#you must specify a timeout (in seconds) so that the
	# serial port doesn't hang
	ser.timeout = 1
	ser.open() #open the serial port

	# print port open or closed
	if ser.isOpen():
		print 'Open: ' + ser.portstr
#####SETUP################################################
#this is a good spot to run your initializations 
init_serial()

#####MAIN LOOP############################################
while 1:
	#prints what is sent in on the serial port
	temp = raw_input('Type what you want to send, hit enter:\n\r')
	ser.write(temp+'\n') #write to the serial port
	bytes = ser.readline() #reads in bytes followed by a newline 
	print 'You sent: ' + bytes #print to the console
	break #jump out of loop 
#hit ctr-c to close python window
