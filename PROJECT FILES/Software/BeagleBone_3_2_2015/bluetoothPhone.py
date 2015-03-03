# file: rfcomm-server.py
# auth: Albert Huang <albert@csail.mit.edu>
# desc: simple demonstration of a server application that uses RFCOMM sockets
#
# $Id: rfcomm-server.py 518 2007-08-10 07:20:07Z albert $

from bluetooth import *
import sys
import os
import re
import time

# Create the Bluetooth Connection
server_sock=BluetoothSocket( RFCOMM )
server_sock.bind(("",PORT_ANY))
server_sock.listen(1)

port = server_sock.getsockname()[1]

uuid = "00001101-0000-1000-8000-00805f9b34fb"

advertise_service( server_sock, "SampleServer",
                   service_id = uuid,
                   service_classes = [ uuid, SERIAL_PORT_CLASS ],
                   profiles = [ SERIAL_PORT_PROFILE ], 
#                   protocols = [ OBEX_UUID ] 
                    )
                   
print("Waiting for connection on RFCOMM channel %d" % port)

client_sock, client_info = server_sock.accept()
print("Accepted connection from ", client_info)

#Initialize the Handshake
try:
    while True:
        data = client_sock.recv(1024)
        if len(data) == 0: break
        print("received [%s]" % data)
	if data[0] == '@': break
except IOError:
    pass

#Accept the Handshake
try:
	client_sock.send("Fieldroid - Connected")
	
except IOError:
	print "not client sock?"
	pass

#Handle different options in fieldroid
try:
	while True:
		#data = raw_input()
		data = client_sock.recv(1024)
		#if len(data) == 0: break
		#client_sock.send(data)
		if data == "LAUNCH":
			print("Launching")
			#call(["./runRobot.o",">","lastLog"])
			os.system("cd /root/Software/ && ./runRobot.o > /root/Software/logs/lastLog &");
			print "The Robot Should be Moving... "
			client_sock.send("The Robot Should Be Moving... ")
		if data == "STOP":
			print "STOPPING!!!"
			os.system("ps aux | grep ./runRobot.o | awk '{print $2}' | xargs kill > logs/errors 2>&1")
			client_sock.send("The Robot Should Be Stopped... ")
			print "The Robot Should be Stopped... "
		if data == "FORWARD":
			print "Forward"
			os.system("cd /root/Software/remote/ && ./forward.o &")
		if data == "LEFT":
			print "Left"
			os.system("cd /root/Software/remote/ && ./left.o &")
		if data == "RIGHT":
			print "Right"
			os.system("cd /root/Software/remote/ && ./right.o &")
		if data == "BACKWARD":
			print "Backward"
			os.system("cd /root/Software/remote/ && ./backward.o &")
		if data == "FETCHGOAL":
			print "Attempting to send Data"
			xg = ()
			yg = ()
			client_sock.send("SENDING")
			j = 0;
			reduceData = 7;
			for line in open("/root/Software/logs/lastLog",'r'):
				tmpGX = re.findall('xGoal:(\-?[0-9.]+)',line)
				tmpGY = re.findall('yGoal:(\-?[0-9.]+)',line)
				if tmpGX:
					if j%reduceData==0:
						xg += tmpGX,
				if tmpGY:
					if j%reduceData==0:
						yg += tmpGY,
					j+=1
			print "xg: "
			print xg
			for i in xg:
				try:
					client_sock.send(i[0]+'%')
					#sleep(.01)
				except:
					print "Couldn't Send Gx's"
			client_sock.send('$%')
			time.sleep(.1)
			print "yg: "
			print yg
			for i in yg:
				try:
					client_sock.send(i[0]+'%')
					#sleep(.01)
				except:
					print "Couldn't Send Gy's"
			client_sock.send('$%')
			print "Sent All Data"
		if data == "FETCHACTUAL":
			print "Attempting to send Data"
			x = ()
			y = ()
			client_sock.send("SENDING")
			j = 0;
			reduceData = 7;
			for line in open("/root/Software/logs/lastLog",'r'):
				tmpX = re.findall('absoluteX:\ ?(\-?[0-9.]+)',line)
				tmpY = re.findall('absoluteY:\ ?(\-?[0-9.]+)',line)
				if tmpX:
					if j%reduceData==0:
						x += tmpX,
				if tmpY:
					if j%reduceData==0:
						y += tmpY,
					j+=1
			print "x: "
			print x
			for i in x:
				try:
					client_sock.send(i[0]+'%')
					#sleep(.01)
				except:
					print "Couldn't Send X's"
			client_sock.send('$%')
			time.sleep(.1)
			print "y: "
			print y
			for i in y:
				try:
					client_sock.send(i[0]+'%')
					#sleep(.01)
				except:
					print "Couldn't Send Y's"
			client_sock.send('$%')
			print "Sent All Data"
			
			
except: 
	print "Connection Error"

print("disconnected")

client_sock.close()
server_sock.close()
print("all done")
