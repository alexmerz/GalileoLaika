'''
/**
 * Copyright (c), Klass&Ihlenfeld Verlag GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the Klass&Ihlenfeld Verlag GmbH nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Alexander Merz, am@golem.de
 */
'''

'''
This script is a small, working example of face recognition and servo control on
the Intel Galileo

Requirements:
- a working OpenCV-modul for Python
- the python-smbus modul for Python (http://www.acmesystems.it/i2c)
You have to patch the module to get it working on the Galileo:
In the file "smbusmodule.c", line 158 replace the I2C_SLAVE with
I2C_SLAVE_FORCE
- a Webcam attached via USB
'''

import cv2
from thread import start_new_thread
import signal
import sys
import os
import time
from smbus import SMBus

'''
The pins in this project are select on purpose
They are not multiplexed making the initialization easier
The first three pins controls the LEDs: 4, 7 und 8
The pin (9) for the servo, has two constants, because we need to address them
as GPIO and as PWM
'''

GREEN 		= 28	# arduino pin 4
YELLOW 		= 27 	# arduino pin 7
RED 		= 26	# arduino pin 8
SERVO 		= 1		# arduino pin 9 as PWM
SERVO_GPIO 	= 19	# arduino pin 9 as GPIO

img 	= None	# The current cam image
cascade = None	# The recognition data of OpenCV
bus 	= None	# Instance of the i2c/smbus to use

capture_not_stopped = True	# Indicate to start or stop the cam capturing

'''
Detect a face, in case of success the array contains the coordinates of the face in the
image
'''
def detectFace(img) : 

	if cascade is not None : # Make sure, the recognition data was already loaded
		rects =  cascade.detectMultiScale(img, 1.3, 4, cv2.cv.CV_HAAR_SCALE_IMAGE, (20,20))
				
		if len(rects) == 0 :
			return []
		rects[:, 2:] += rects[:, :2]
	
		return rects
		
	return []		
	
'''
Move the servo position from left to right, right to left and neutral position
The hard coded values may differ, depending on the servo
'''	
def wink() : 
	for a in range(29, 89) :		
		setPwmPin(SERVO, a)
		time.sleep(0.01)
	for a in range(-88, -29) :
		setPwmPin(SERVO, 0-a)
		time.sleep(0.01)		
	setPwmPin(SERVO, 59)			

'''
Analyzes the current image in the global variable "img" from the cam and tries to detect a face
'''	
def analyse() : 
	global img

	# Switch on the yellow LED to indicate "processing"
	setGPIOPin(RED, 0)
	setGPIOPin(GREEN, 0)	
	setGPIOPin(YELLOW, 1)	 
	
	# Make sure we really have an image
	if img is not None :
		height, width, depth = img.shape
	
		# sometimes OpenCV provides an "empty" image
		if width > 0 and height > 0 :	
		
			# resize the image to make detection faster
			_img = cv2.resize(img, (176, 144))	
			
			# check for a face
			rects = detectFace(_img)			

			# Indicate processing is done
			setGPIOPin(YELLOW, 0)				
					
			if 0 < len(rects) : # Face detected, switch on green LED and do something with the servo
				setGPIOPin(GREEN, 1)			
				wink()
			else : # No face detected, switch on red LED
				setGPIOPin(RED, 1)				

	img = None 				# Delete processed image
	setGPIOPin(YELLOW, 0)	# Indicate processing is done
	
'''
If the program was killed, try to clean up
'''	
def signal_handler(signal, frame):
	capture_not_stopped = False
	disable()
	sys.exit(0)	

'''
Disables the pin, does not reset the pin state!
'''
def disableGPIOPin(pin) :
	f = open("/sys/class/gpio/unexport", "w")
	f.write(str(pin))
	f.close()
	
'''
Set the value for a pin, possible values are 0/1
similar to digitalWrite()
'''	
def setGPIOPin(pin, value) :
	f = open("/sys/class/gpio/gpio"+str(pin)+"/value", "w")
	f.write(str(value))
	f.close()		

'''
Sets the value for the PWM signal
similar to analogWrite()
'''
def setPwmPin(pin, value) :
	global bus

	bus.write_byte_data(0x20, 0x28, pin)	# We want to set the config for the pin
	bus.write_byte_data(0x20,0x2B, value)	# Set pulse width

'''
De-enable PWM functionality for the pin
'''
def disablePwmPin(pin) :
	f = open("/sys/class/pwm/pwmchip0/unexport", "w");
	f.write(str(pin))
	f.close()

'''
Enable PWM for an output pin
and sets the required values on the
IOExpander chip to get the right timing for the servo
'''
def enablePwmPin(pin) :	
	global bus

	if not os.path.exists("/sys/class/pwm/pwmchip0/pwm"+str(pin)) :
		f = open("/sys/class/pwm/pwmchip0/export", "w");
		f.write(str(pin))
		f.close()
	
	# initialize I/O-Chip through I2C using the python-smbus module
	# see http://www.cypress.com/?docID=31413 for the meaning of the values
	bus = SMBus(0)		
	bus.write_byte_data(0x20, 0x28, pin) 	# We want to set the config for the pin
	bus.write_byte_data(0x20, 0x29, 0x04) 	# Set pwm clock source to the programmable 367,6Hz
	bus.write_byte_data(0x20, 0x2A, 0xff) 	# Set rising pulse edge
	bus.write_byte_data(0x20, 0x2C, 0x02) 	# Set the divider of the clock source

	f = open('/sys/class/pwm/pwmchip0/pwm'+str(pin)+'/enable', "w")	
	f.write('1')
	f.close()


'''
Special init for the pin to control the servo
'''
def enableServo(pin, gpio_pin) :
	# activate the pin to act as an output pin
	initGPIOPin(gpio_pin)	
	setGPIOPin(gpio_pin, 1)
	disableGPIOPin(gpio_pin)
	
	# enable PWM functionality for the pin
	# and set it to a default value
	enablePwmPin(pin)
	setPwmPin(pin, 59)

'''		
Activates the pins as output pins
similar to pinMode(pin, 'OUT') in an Arduino sketch
'''
def initGPIOPin(pin) :
	if not os.path.exists("/sys/class/gpio/gpio"+str(pin)) :
		f = open("/sys/class/gpio/export", "w");
		f.write(str(pin))
		f.close()	
	
	f = open("/sys/class/gpio/gpio"+str(pin)+"/direction", "w")
	f.write("out")
	f.close()
	
	f = open("/sys/class/gpio/gpio"+str(pin)+"/drive", "w")
	f.write("strong")
	f.close()

'''
Deactivates the pins
'''	
def disable() :	
	setGPIOPin(RED, 0)
	setGPIOPin(GREEN, 0)	
	setGPIOPin(YELLOW, 0)

	disablePwmPin(SERVO)

	disableGPIOPin(GREEN)	
	disableGPIOPin(YELLOW)
	disableGPIOPin(RED)	
					
'''
Initializes the required pins for the LEDs and the servo
'''					
def initPins() :
	initGPIOPin(GREEN)
	initGPIOPin(YELLOW)
	initGPIOPin(RED)	
	
	enableServo(SERVO, SERVO_GPIO)		
	
'''
Load the data for face recognition
The xml file is part of the OpenCV distribution
Copy the file in the same directory as this python script
or adjust the path to the file location
'''	
def loadRecognition() :
    global cascade 
    cascade = cv2.CascadeClassifier("haarcascade_frontalface_alt2.xml")	
	
'''
Initializes the cam and read the image data continuously
Stores the current image in a global variable called "img"
'''	
def initCam() :
	global img
	global capture_not_stopped
	
	capture = cv2.VideoCapture(0)
	
	while(capture.isOpened() and capture_not_stopped) :
		ret, img = capture.read()

'''
Main program
'''
# react if the program was killed 
signal.signal(signal.SIGINT, signal_handler) 

# init the Galileo pins
initPins() 	

# Load the data for face recognition
start_new_thread(loadRecognition, ())

# Start the cam
start_new_thread(initCam, ())

# main loop
while True :
	analyse()
