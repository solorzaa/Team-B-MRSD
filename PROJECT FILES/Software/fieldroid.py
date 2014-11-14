#!/usr/bin/python
"""
Base Software file for Fieldroid System
"""

# 	import serial
import time
import os

timeDifference = 0;
trackingStationSerialData = ""

#Turn everything on
	#5V Relay
	#12V Relay
	#If SUCCESS
		#Turn on LED 1
	#Check Data from Tracking Station
	#If SUCCESS
		#Turn on LED 2
		#Get BBB Time and compare to Tracking Station Time
			#Store time difference
	#Check Data from encoders
	#Check Data from IMU
	#If SUCCESS
		#Turn on LED 3

#Read in points file
	#Each line contains:
	#x,y,theta,paint
	#x position, y position, orientation, whether it should be painting at that point

#Set initial position to (0,0,0,t) [first point in file]

#Sync Time
	#Read Tracknig Station Data
	#Check Time Difference
	#Read Encoder Data
	#Read IMU Data
	
#Advance to next point



#___Functions___
# getTime()
	# Get BBB Time
	# Add timeDifference (to line up with tracking station time)   --- Assume no drift between clocks
	
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
	
# getTrackingStation
	# Get Serial
	# Convert Serial to x, y, and time stamp
	# calculate theta from last tracking data
		# theta = arctan(x
	# update time stamp [Function?]
	# return x,y,theta
	
# GetIMU
	# Get orientation from IMU
	# Convert to reference frame
	# return theta
	
# updateTimeStamp(trackingStationTimeStamp)
	# clock Time = getTime()
	# print trackingStationTimeStamp - clock Time  (print the drift between the two entities)
	
# readPointsFile()
	# read the points file into an array
	
def turnOnLED(led):
	''' Turns on the led with numerical reference led.  0<=led<=3'''
	os.system("echo none > /sys/class/leds/beaglebone:green:usr"+str(led)+"/trigger")
	os.system("echo 1 > /sys/class/leds/beaglebone:green:usr"+str(led)+"/brightness")

def turnOffLED(led):
	''' Turns off the led with numerical reference led.  0<=led<=3'''
	os.system("echo none > /sys/clas/leds/beaglebone:green:usr"+str(led)+"/trigger")
	os.system("echo 0 > /sys/class/leds/beaglebone:green:usr"+str(led)+"/brightness")
