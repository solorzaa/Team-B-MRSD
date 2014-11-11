import Adafruit_BBIO.GPIO as GPIO
import time

GPIO.setup("P9_11", GPIO.OUT)
GPIO.setup("P9_12", GPIO.IN)

while True:

	   	
	   #GPIO.add_event_detect("P9_30", GPIO.FALLING)
	   # if GPIO.ed("P9_30"):
	   #	 print "event detected!"
           #	 time.sleep(1)
	   input1=raw_input()
	   #input1=int(input1)
	   
	   if input1=='paint': 
	        GPIO.output("P9_11", GPIO.HIGH)
	   #	time.sleep(.5)
	   #	GPIO.output("P9_11", GPIO.LOW)
	   #     time.sleep(.5)
    	   elif input1=='npaint':
		GPIO.output("P9_11",GPIO.LOW)
                print('do not paint')
	   else:
		   print('Error: inavlid paint input') 
