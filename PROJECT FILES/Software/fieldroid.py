#!/usr/bin/python
"""
Base Software file for Fieldroid System
"""

###### TO DO ######
'''
Encoders?
IMU?

'''

import time
import os
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.UART as UART
from math import cos, pi, sin, atan
#import serial

### Pins and Ports
pumpTriggerPin = "p9_11"
emergencyStopPin = "p9_12"
motorControllerSerialUART = "UART2"
motorControllerSerialPort = "/dev/ttyO2"
motorControllerBaudRate = 9600
 
### Pins and Ports Setup
GPIO.setup(pumpTriggerPin, GPIO.OUT)
#UART.setup(motorControllerSerialPort)
#motorControllerSerial = serial.Serial(port = motorControllerSerialPort, baudrate = motorControllerBaudRate, timeout=1)
#motorControllerSerial.close()

### EMERGENCY STOP
GPIO.setup(emergencyStopPin, GPIO.IN)
# Check to make sure we can do this
GPIO.add_event_detect(emergencyStopPin, GPIO.FALLING)

### Localization Information
trackingStationX = 0.0
trackyngStationY = 0.0
trackingStationTheta = 0.0
prevTrackingStationY = 0.0
prevTrackingStationX = 0.0
trackingStationCount = 0
encoderX = 0.0
encoderY = 0.0
encoderTheta = 0.0
prevEncoderX = 0.0
prevEncoderY = 0.0

### Time Information
trackingStationTime = 0.0
BBBTime = 0.0
timeDifference = 0.0

### Path Setup
pointsFile = 'points.dat'
goalPoints = []
paintPoints = []
absoluteX = 0.0 # what were these for? (absolutes)
absoluteY = 0.0
absoluteTheta = 0.0
offsetTSX = 0.0
offsetTSX = 0.0
currentGoalPoint = []

def turnOnPump():
	'''Turn on the pump through the 5V relay'''
	GPIO.output(pumpTriggerPin,GPIO.HIGH)
	
def turnOffPump():
	'''Turn off the pump through 5V relay'''
	GPIO.output(pumpTriggerPin,GPIO.LOW)
	
def turnOnLED(led):
	''' Turns on the led with numerical reference led.  0<=led<=3'''
	os.system("echo none > /sys/class/leds/beaglebone:green:usr"+str(led)+"/trigger")
	os.system("echo 1 > /sys/class/leds/beaglebo	e:green:usr"+str(led)+"/brightness")

def turnOffLED(led):
	''' Turns off the led with numerical reference led.  0<=led<=3'''
	os.system("echo none > /sys/class/leds/beaglebone:green:usr"+str(led)+"/trigger")
	os.system("echo 0 > /sys/class/leds/beaglebone:green:usr"+str(led)+"/brightness")
	
def readPointsFile():
	''' Interprets the data in the points file, which contains the control points '''
	points = ''
	with open(pointsFile,'rb') as f:
		points = f.read()
	points = points.split('\r\n')
	global paintPoints = [x[-1]=='t' for x in 	points]
	points = [x[0:-2].split() for x in points]
	points = [[float(y) for y in x] for x in points]
	global goalPoints = points
	return True
	
def getTrackingStationData():
	''' Gets the data from the tracking station and converts it to useful information
	Returns whether it succeeded
	'''
	# Update the previous position points to the current points
	global prevTrackingStationX = trackingStationX
	global prevTrackingStationY = trackingStationY
	
	# Get the most previous serial data from the tracking station
	line = ''
	line = motorControllerSerial.readLine()
	if line=='':
		return False
		
	# Convert the string line into an array of floats
	data = [float(x) for x in line.split()]
	
	# Get exact values for count and time
	if trackingStationCount != int(data[1])-1:
		print "The newest tracking station data point's count is off..."
	global trackingStationCount = int(data[1])
	if abs(trackingStationTime-data[0])<1.0: # not sure about this value - what value would be weird to have as a time difference?
		print "The tracking station time stamp seems funny..."
	global trackingStationTime = data[0]
	print 'Time Difference Between Entities: ' + str(checkTimeDifference())
	
	# x = R*cos(phi)*cos(theta) | y = R*cos(phi)*sin(theta)
	global trackingStationX = data[2]*cos((data[4]-90.)*pi/180.)*cos(data[3]*pi/180)
	global trackingStationY = data[2]*cos((data[4]-90.)*pi/180.)*sin(data[3]*pi/180)
	# theta = arctan((new X - last X)/(new Y - last Y))
	# might want to check for small changes in position, as this will throw off theta
	global trackingStationTheta = atan((trackingStationX-prevTrackingStationX)/(trackingStationY-prevTrackingStationY))

	# Exit with success
	return True

def getEncoderData():
	''' Gets the data from the encoders and converts to useful information '''
	return False
	
def getIMUData():
	''' Gets the data from the IMU and converts it to useful information '''
	return False
	
def obtainErrors():
	''' Obtain the error in, y, and z of the position and the next point based on
	the localization data that is currently available.  This uses the timestamp to
	determine relevancy of the information'''
	# Calibrate the position for the 0,0,0 starting position  ################################################## CHECK THIS
	global trackingStationX = trackingStationX-offsetTSX
	global trackingStationY = trackingStationY-offsetTSY
	
	#TODO: Check if deltaT is in seconds (i.e. most of the time it will be less than 1)
	deltaT = checkTimeDifference()
	
	# Use the time since the last tracking station input to weight the position data
	weightTS = (1.0-deltaT)
	if weightTS<0.0:
		weightTS = 0.0;
	global errorX = (currentGoalPoint[0]-trackingStationX)*weightTS+((currentGoalPoint[0]-encoderX)*(1-weightTS))
	global errorY = (currentGoalPoint[1]-trackingStationY)*weightTS+((currentGoalPoint[1]-encoderY)*(1-weightTS))
	global errorTheta = (currentGoalPoint[2]-trackingStationTheta)*weightTS+((currentGoalPoint[2]-encoderTheta)*(1-weightTS))
	return None
	
def checkTimeDifference():
	''' Check the time difference between the beagle bone and the tracking station '''
	global BBBTime = time.time()-timeDifference
	return (BBBTime - trackingStationTime)
	
def checkEStop():
	''' Checks for an EMERGENCY STOP trigger and kills the paint and motors if pressed '''
	if GPIO.event_detected(emergencyStopPin):
		turnOffPump()
		# STOP THE MOTORS!!!
		while GPIO.input(emergencyStopPin)==False:
			print 'Emergency Stop Button Pressed'
	else:
		return

### Turn Off All LEDS to Start
turnOffLED(0)
turnOffLED(1)
turnOffLED(2)
turnOffLED(3)

### Turn relays on???
	#5V Relay
	#12V Relay
#If SUCCESS
turnOnLED(0)

### Initialize Localization Systems

## Tracking Station - LED D3
status = getTrackingStationData()
while status==False:
	status = getTrackingStationData()
	print 'No Tracking Station Data Yet'
# Set the absolute difference between the tracking station time and the robot
timeDifference = BBBTime - trackingStationTime
turnOnLED(1)

## Encoders - LED D4
status = getEncoderData()
while status==False:
	status = getEncoderData()
	print 'No Encoder Data Yet'
	
## IMU - LED D4
status = getIMUData()
while status==False:
	status = getIMUData()
	print 'No IMU Data Yet'
turnONLED(2)

if readPointsFile()==False:
	print 'Failed to read in the points in the points file'

offsetTSX = trackingStationX;
offsetTSY = trackingStationY;

currentGoalPoint = goalPoints[1]
obtainErrors()

#Advance to next point
	
# getEncoderPosition()
	# pull encoder data
	# Calculate position and orientation
		# distance travelled by left wheel = delta encoder angle left (radians) * Wheel radius
		# distance travelled by right wheel = delta encoder angle right (radians) * Wheel radius
		# if dL > dR
			# turn radius right wheel = 2 * dR * radius of robot/(dL-dR)
			# turn angle (theta) = dR/Rr
			# x = last x + (Rr+r)(1-cos(theta))
			# y = last y + (Rr+r)sin(theta)
			# orientation = theta
		# if dR > dL
			# See above (opposite)
	# return x,y,theta
	
# GetIMU
	# Get orientation from IMU
	# Convert to reference frame
	# return theta
