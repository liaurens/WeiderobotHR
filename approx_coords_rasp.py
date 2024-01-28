import time
import smbus
import math
import board
import busio
import adafruit_gps
import serial
import RPi.GPIO as GPIO
import mysql.connector
import json
from time import sleep

GPIO.setup(29, GPIO.OUT)
GPIO.setup(31, GPIO.OUT)
waypointLat1  =51.91777 
waypointLong1 =4.48330
#51.91777135560369, 4.4833013728143385
waypointLat2  =51.91769
waypointLong2 =4.48276
waypointLatCur = waypointLat1
waypointLongCur = waypointLong1
#51.9176993906935, 4.482767613215477
#random nearby waypoints chosen for testing purposes, in future versions only waypointlat and waypointcur are 
#neccessary , connection to database will allow for downloading of future waypoints,



#magnetometer addresses 
#might be more practical to move all this to a separate library to import later -
ADDRESS = 0x1E
CONFIG_A = 0x00
CONFIG_B = 0x01
MODE = 0x02
X_MSB = 0x03
Z_MSB = 0x05
Y_MSB = 0x07

bus = smbus.SMBus(1)
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial
#setup done with use of the Adafruit Ultimate GPS breakout python example, requires next to no changes.

def servosetup(): #do not forget, board is currently set to GPIO.BCM, not GPIO.BOARD
    steer = GPIO.PWM(5,100)    #steer:100        throttle:400
    steer.start(0)
    throttle = GPIO.PWM(6,400)
    throttle.start(0)#values determined with testing, do not assume they will work on any servo, this will have to be adapted

def approx(current, goal, margin): #basic function used to evaluate whether a value has approximated another 
    if(goal - margin < 0):
        if (goal - margin) % 360 < current: # MOD360 used to wrap values greater than 360 / less than 0 back around to a homogenous
            return True                     # degree
    if(goal + margin > 360):
        if (goal + margin) % 360 > current:
            return True          
    if(current<= goal + margin and current >= goal - margin):
        return True
    return False
    
def HMC5883Lsetup(): #basic setup of the HMC5883L 
    bus.write_byte_data(ADDRESS, CONFIG_A, 0x70)
    bus.write_byte_data(ADDRESS, CONFIG_B, 0x20)
    bus.write_byte_data(ADDRESS, MODE, 0x00)

def read_HMC(addr):
    high = bus.read_byte_data(ADDRESS, addr)
    low = bus.read_byte_data(ADDRESS, addr+1)
    
    value = (high<<8) + low
    if value > 32768:
        value = value - 65536
    return value

def compute_heading(x,y):
    
    heading_rad = math.atan2(y,x)
    
    declinationAngle = 0.034 #declination angle determined with https://ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination
    heading_rad += declinationAngle
    
    if heading_rad <0:
        heading_rad += 2*math.pi
        
    if heading_rad > 2 * math.pi:
        heading_rad -= 2*math.pi
        
    heading_deg = heading_rad * (180.0 / math.pi)
    
    return heading_deg #function for determining angle in relation to magnetic north.




def goLeft(): #values determined with testing, do not assume they will work on any servo, this will have to be adapted
    throttle.ChangeDutyCycle(0)
    steer.ChangeDutyCycle(10)
    sleep(0.7)
    throttle.ChangeDutyCycle(30)
    sleep(1)
    throttle.ChangeDutyCycle(0)
    steer.ChangeDutyCycle(15)
    sleep(0.7)
    steer.ChangeDutyCycle(0)

def goRight():
    throttle.ChangeDutyCycle(0)
    steer.ChangeDutyCycle(20)
    sleep(0.7)
    throttle.ChangeDutyCycle(30)
    sleep(1)
    throttle.ChangeDutyCycle(0)
    steer.ChangeDutyCycle(15)
    sleep(0.7)
    steer.ChangeDutyCycle(0)


# Initialize the GPS module by changing what data it sends and at what rate.
# These are NMEA extensions for PMTK_314_SET_NMEA_OUTPUT and
# PMTK_220_SET_NMEA_UPDATERATE but you can send anything from here to adjust
# the GPS module behavior:
#   https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf

def gpssetup():
# Turn on the basic GGA and RMC info (what you typically want)
    gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
# Set update rate to once a second (1hz) 
    gps.send_command(b"PMTK220,1000")
    print("gps setup done")

def calcBearing(curLat, curLon, desLat, desLon):
    bearing = math.atan2((math.sin((desLon - curLon)*math.pi/180) * cos(desLat*math.pi/180)),((math.cos(curLat*math.pi/180)*sin(desLat*math.pi/180)) - (math.sin(curLat*math.pi/180)*cos(desLat*math.pi/180)*cos((desLon-curLon)*math.pi/180))))
    if(bearing < 0):
        bearing += 2*math.pi
    if(bearing > 2*math.pi):
        bearing -= 2*math.pi
    bearingDegrees = bearing * (180/math.pi)
    return bearingDegrees   #
def determine_turn_direction(target_angle, current_angle):
    """ Determine the most efficient turn direction (left or right) to reach the target angle from the current angle """
    # Normalize angles
    target_angle = target_angle % 360
    current_angle = current_angle % 360

    # Calculate the difference in both directions
    right_turn =(target_angle - current_angle) % 360
    left_turn = (current_angle - target_angle) % 360

    # Determine the most efficient turn direction
    if right_turn < left_turn:
        goRight()
    else:
        goLeft()
#currently very impractical, it will always just move a "set"amount and reevaluate, future system will want to change to turn 
#untill a certain threshold is met. Sadly enough could not be added in time

def totSetup():#function that set
    gpssetup()
    HMC5883Lsetup()
    servosetup()

totSetup()
last_print = time.monotonic()
while True:
    magX = read_HMC(X_MSB)
    magY = read_HMC(Y_MSB)
    gps.update()      
    heading = compute_heading(magX,magY)
    print(heading)
    if gps.has_fix: 
        if not approx(gps.latitude, waypointLatCur, 0.0005) or not approx(gps.longitude, waypointLongCur, 0.0005):
            bearing = calcBearing(latDD, lonDD, waypointLatCur, waypointLongCur)
            print(bearing)
            if not approx(bearing, heading, 5):
                determine_turn_direction(bearing, heading):
                
                
            throttle.ChangeDutyCycle(30)
        throttle.ChangeDutyCycle(0)
        if approx(latDD, waypointLat, 0.0005) and approx(lonDD, waypointLong, 0.0005):
            waypointLatCur = waypointLat2
            waypointLongCur = waypointLong2
    
        

        
        

