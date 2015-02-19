# file: rfcomm-server.py
# auth: Albert Huang <albert@csail.mit.edu>
# desc: simple demonstration of a server application that uses RFCOMM sockets
#
# $Id: rfcomm-server.py 518 2007-08-10 07:20:07Z albert $

from bluetooth import *
import sys
import os
import re

# Create the 
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

try:
    while True:
        data = client_sock.recv(1024)
        if len(data) == 0: break
        print("received [%s]" % data)
	if data[0] == '@': break
except IOError:
    pass

print "Send Stuff: "
try:
	client_sock.send("Fieldroid - BeagleBone")
	
except IOError:
	print "not client sock?"
	pass
try:
	while True:
		#data = raw_input()
		data = client_sock.recv(1024)
		#if len(data) == 0: break
		#client_sock.send(data)
		if data == "LAUNCH":
			print("Launching")
			#call(["./runRobot.o",">","lastLog"])
			os.system("cd /root/Software/ && ./runRobot.o > /root/Software/logs/lastLog");
			client_sock.send("The Robot Should Be Moving... ")
		if data == "FETCH":
			x = ()
			y = ()
			for line in open("/root/Software/logs/lastLog",'r')
				x = re.findall('absoluteX:\ ?(\-?[0-9.]+)',line),
				y = re.findall('absoluteY:\ ?(\-?[0-9.]+)',line),
except: 
	print "Connection Error"

print("disconnected")

client_sock.close()
server_sock.close()
print("all done")
