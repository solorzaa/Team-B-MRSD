import serial
import time
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
	 with open('surveyData.txt') as f:
	   data = f.readlines()
	   for line in data:
            #print line
       	     
	#prints what is sent in on the serial port
	#print file.readlines()('Type what you want to send, hit enter:\n\r')
	     ser.write(line) #write to the serial port
	     time.sleep(0.9)	
	    # bytes = ser.readline() #reads in bytes followed by a newline 
	     #print 'You sent: ' + bytes #print to the console
	     #break #jump out of loop 
	
#hit ctr-c to close python window
