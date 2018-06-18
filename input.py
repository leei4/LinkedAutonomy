#! /usr/bin/env python

import serial
import string
import time
import math
from time import sleep

import boat_info

class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, P=2.0, I=1.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

		self.Kp=P
		self.Ki=I
		self.Kd=D
		self.Derivator=Derivator
		self.Integrator=Integrator
		self.Integrator_max=Integrator_max
		self.Integrator_min=Integrator_min

		self.set_point=0.0
		self.error=0.0

	def update(self,current_value):
		"""
		Calculate PID output value for given reference input and feedback
		"""

		self.error = self.set_point - current_value

		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * ( self.error - self.Derivator)
		self.Derivator = self.error

		self.Integrator = self.Integrator + self.error

		if self.Integrator > self.Integrator_max:
			self.Integrator = self.Integrator_max
		elif self.Integrator < self.Integrator_min:
			self.Integrator = self.Integrator_min

		self.I_value = self.Integrator * self.Ki

		PID = self.P_value + self.I_value + self.D_value

		return PID

	def setPoint(self,set_point):
		"""
		Initilize the setpoint of PID
		"""
		self.set_point = set_point
		self.Integrator=0
		self.Derivator=0

	def setIntegrator(self, Integrator):
		self.Integrator = Integrator

	def setDerivator(self, Derivator):
		self.Derivator = Derivator

	def setKp(self,P):
		self.Kp=P

	def setKi(self,I):
		self.Ki=I

	def setKd(self,D):
		self.Kd=D

	def getPoint(self):
		return self.set_point

	def getError(self):
		return self.error

	def getIntegrator(self):
		return self.Integrator

	def getDerivator(self):
		return self.Derivator

def setRudderangle(targetHeading):
    currentHeading = this.heading
    turnAngle = targetHeading - currentHeading
    
    #calculation for rudder shift
    returnAngle = 90.0 - turnAngle / 10.0
    return returnAngle

def destinationReached(currentLat, currentLong, finalLat, finalLong):
        temp1 = finalLong     #temporarily stores the destination longitude
        temp2 = finalLat      #temporarily stores the destination latitude
        finalLong = currentLong    #flips longitude of destination and start
        finalLat = currentLat      #flips latitude of destination and start
        currentLong = temp1     #sets current longitude
        currentLat = temp2      #sets current latitude
        return currentLat, currentLong, finalLat, finalLong

def Exit():
    self.break()
    
if __name__ == "__main__":
    
    #time.sleep(100)
    f_lat = 41.131412
    f_long = -73.28398 
    course = 32.0
    
    output = " "
    ser = serial.Serial('/dev/ttyACM0',9600)
    while(True):  
        #ser.write('3')
        output = ser.readline()
        print(output)
    print("stopped") 

    #Function that occurs if we reach our destination. 
    if(c_long == f_long and c_lat == f_lat):
        destinationReached(c_lat, c_long, f_lat, f_long)