#! /usr/bin/env python
from __future__ import division
from twisted.internet import reactor, protocol
import serial
import string
import time
import math
import wiringpi
import random #remove in final build for testing only
from time import sleep

import Adafruit_PCA9685
import boat_info

class QuoteProtocol(protocol.Protocol):
    def __init__(self, factory):
        self.factory = factory
    def connectionMade(self):
        self.sendQuote()
    def sendQuote(self):
        self.transport.write(self.factory.quote)
    def dataReceived(self, data):
        print("Received quote:", data)
        self.transport.loseConnection()
        
class QuoteClientFactory(protocol.ClientFactory):
    def __init__(self, quote):
        self.quote = quote
    def buildProtocol(self, addr):
        return QuoteProtocol(self)
    def clientConnectionFailed(self, connector, reason):
        print 'connection failed:', reason.getErrorMessage()
        maybeStopReactor()
    def clientConnectionLost(self, connector, reason):
        print 'connection lost:', reason.getErrorMessage()
        maybeStopReactor()

def maybeStopReactor():
    global quote_counter
    quote_counter -= 1
    if not quote_counter:
        reactor.stop()
def communicate(quotes):
    quote_counter = len(quotes)

    for quote in quotes:
        reactor.connectTCP('localhost', 8000, QuoteClientFactory(quote))
    reactor.run()
    
def PID(current_heading, target):
    P=2.0
    I=1.0
    D=0.0
    Derivator=0
    Integrator=0
    Integrator_max= 250     #1850
    Integrator_min= 0     #1120

    set_point=0.0
    error=0.0
    
    error = target - current_heading
    
    if(error > 180):
         error = 360 - error
    error * (-1)
    
    P_value = P * error
    D_value = D * ( error - Derivator)
    Derivator = error

    Integrator = Integrator + error

    if Integrator > Integrator_max:
        Integrator = Integrator_max
    elif Integrator < Integrator_min:
        Integrator = Integrator_min

    I_value = Integrator * I
	
    PID = P_value + I_value + D_value
    
    return PID
#####################################################################

def translate(value, leftMin, leftMax, rightMin, rightMax):
    #calculate how 'wide' the range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax -  rightMin
    
    #convert each value to a range from 0 to 1 as a float
    valueScaled = float(value - leftMin) / float(leftSpan)
    
    #convert the 0 to 1 range into a value in the right range
    return rightMin + (valueScaled * rightSpan)

#calculations are pulled from sailbot
def direction_to_point(c_lat,c_long, f_lat, f_long):
    a = math.radians(c_lat)
    b = math.radians(f_lat) 
    d = math.radians(f_long - c_long)
    
    y = math.sin(d) * math.cos(b)
    x = math.cos(a) * math.sin(b) - math.sin(a) * math.cos(b) * math.cos(d)
    
    return (math.degrees(math.atan2(y,x)) +360) % 360

def setRudderangle(course, targetHeading):
    turnAngle = targetHeading - course
    
    #calculation for rudder shift
    returnAngle = 90.0 - turnAngle / 10.0
    return returnAngle

def destinationReached(currentLat, currentLong, finalLat, finalLong):
    if(currentLat == finalLat and currentLong == finalLong):
        return True
    else:
        return False

def wiringInit():
    pwm = Adafruit_PCA9685.PCA9685()
    servo_min = 150
    servo_max = 600
    
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000
    pulse_length //= 60
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwn(channel,0,pulse)
    
def adjust_rudder(val):
        val= int(translate(val, -45, 45, 1070, 1900))
        val = int(translate(val, 590, 2378, 200, 500))
        pwm.set_pwm(4,0,val)
        print("rudder",val)
        
def adjust_sails(val):
    if(val > 180): val = 360-val
    
    if(val <= 40): val = 40
    
    val = int(translate(val, 40, 180, 1070, 1900))
    val = int(translate(val, 590, 2378, 150, 600))
    pwm.set_pwm(5,0,val)
    
def manual_sails(val):
    val = int(translate(val, 590, 2378, 150, 600))
    pwm.set_pwm(5,0,val)
    print("sails", val)
    
def manual_rudder(val):
    val = int(translate(val, 590, 2378, 200, 400))
    pwm.set_pwm(4,0,val)
    print("rudder", val)
    
    
if __name__ == "__main__":
    
    f_lat = 41.178363 #these will be automatically be transmitted by rpi base station
    f_long = -73.271786 #see above comment
    c_lat = -12
    c_long = 23
    course = 32.0
    currentAngle = 250
    
    output = " "
    ser = serial.Serial('/dev/ttyACM0', 115200)
    
    wiringInit()
    pwm = Adafruit_PCA9685.PCA9685()
    pwm.set_pwm_freq(60)
    servo_min = 150
    servo_max = 600
    
    while(True):  
        time.sleep(1)
        output = ser.readline()
        
        #insert code to take inputs from arduino and make calculations
        #latitude,longitude, course(degrees), # of satellites, wind angle(degrees)
        output1 = output.split(",")
        
        while(len(output1) < 7 or output1[0] == ""):
            print("bad output")
            output = ser.readline()
            output1 = output.split(",")
        print(output)
        c_lat = float(output1[0])
        c_long = float(output1[1])
        course = float(output1[2])
        satelliteCount = (output1[3])
        windAngle = (output1[4])
        rudderAngle = (output1[5])
        winchAngle = (output1[6])
        
        satelliteCount = int(satelliteCount.strip("\r\n"))

        
        if(winchAngle != 0 and rudderAngle != 0):
            print("manual")
            manual_sail(winchAngle)
            manual_rudder(rudderAngle) #Shift this back when reenabling the if check for automation 
        else:
            windAngle = int(windAngle)
            adjust_sails(windAngle)
      
            #Function that occurs if we reach our destination. 
            if(destinationReached(c_lat, c_long, f_lat, f_long) == False):
                #calculate destination heading
                #dest_heading = findHeading(c_lat, c_long, f_lat, f_long,course)
                targetCourse = direction_to_point(c_lat, c_long, f_lat, f_long)
                rudderAngle = int(PID(course,targetCourse))
                print("PID",rudderAngle)
                rudderAngle = int(translate(rudderAngle, 0, 360, -45, 45))

                currentAngle = adjust_rudder(int(rudderAngle))  
                commValues = [str(c_lat), str(c_long), str(course),str(windAngle.toString)]
                communicate(commValues)
    print("stopped") #this line should never occur  