#!/usr/bin/python3

import rospy
from gps_driver.msg import Customgps
import pandas as pd
import numpy as np
import sys
import utm
import time
from serial import Serial
from std_msgs.msg import String
from std_msgs.msg import Header

custom_gps_msg = Customgps()

def isGPGGAinString(inputString):
    if (inputString[0] == '$GPGGA'):
        print("Great Success GPGGA FOUND")
        return 1
    else:
        print('GPGGA not found in string')
        return 0

def degMinstoDegDec(LatOrLong):
    deg = float(LatOrLong[:2])
    mins = float(LatOrLong[2:])
    degDec = float(mins/60)
    return (deg+degDec)

def LatLongSignConvetion(LatOrLong, LatOrLongDir):
    if LatOrLongDir.lower() == "s" or LatOrLongDir.lower() == "w":
        LatOrLong = float((-1)*(float(LatOrLong)))
    return LatOrLong


def convertToUTM(LatitudeSigned, LongitudeSigned):
    try:
        LatitudeFloat = float(LatitudeSigned)
        LongitudeFloat = float(LongitudeSigned)
    except ValueError:
        print("LatitudeSigned and LongitudeSigned must be numerical values.")
        return None

    UTMVals = utm.from_latlon(LatitudeFloat, LongitudeFloat)
    UTMEasting = UTMVals[0]
    UTMNorthing = UTMVals[1]
    UTMZone = UTMVals[2]
    UTMLetter = UTMVals[3]

    return [UTMEasting, UTMNorthing, UTMZone, UTMLetter]


def UTCtoUTCEpoch(UTC):
    UTC = str(UTC)
    hours = int(UTC[0:2])
    minutes = int(UTC[2:4])
    seconds = float(UTC[4:])
    UTCinSecs = hours * 3600 + minutes * 60 + seconds

    TimeSinceEpoch = time.time()
    UTCinSecs = float(UTC)
    TimeSinceEpochBOD = float(TimeSinceEpoch - UTCinSecs)
    CurrentTime = TimeSinceEpochBOD + UTCinSecs
    CurrentTimeSec = int(CurrentTime)
    CurrentTimeNsec = float((CurrentTime - CurrentTimeSec) * 1e9)

    return [CurrentTimeSec, CurrentTimeNsec]

serialPortAddr = '/dev/ttyUSB0'
serial_baud = int(4800)

def ReadFromSerial(serialPortAddr, serial_baud):
    serialPort = Serial(serialPortAddr, serial_baud)
    gpggaRead = serialPort.readline()
    serialPort.close()
    return gpggaRead

print("Executing the main code")

def driver():
    pub = rospy.Publisher('chatter', Customgps, queue_size=10)
    rospy.init_node('driver', anonymous=True)
    rate = rospy.Rate(10)

    CurrentTimeinSec = 0
    CurrentTimeinNSec = 0

    while not rospy.is_shutdown():
        string = ReadFromSerial(serialPortAddr, serial_baud).decode('latin-1')
        inputString = string.split(',')

        if isGPGGAinString(inputString) != True:
            continue

        UTC=float(inputString[1])
        if inputString[2]:
            Latitude = float(inputString[2])
        else:
            print("Latitude value is empty.")
            continue
        if inputString[3]:
            LatitudeDir = str(inputString[3])
        else:
            print("Latitudedir value is empty.")
            continue
        if inputString[4]:
            Longitude = float(inputString[4])
        else:
            print("Longi value is empty.")
            continue
        if inputString[5]:
            LongitudeDir = str(inputString[5])
        else:
            print("Longidir value is empty.")
            continue
        if inputString[8]:
            HDOP = float(inputString[8])
        else:
            print("HDOP value is empty.")
            continue
        
        
        

        
        Latitude_str = str(Latitude)
        Longitude_str = str(Longitude)

        LatitudeSigned = LatLongSignConvetion(degMinstoDegDec(Latitude_str), LatitudeDir)
        LongitudeSigned = LatLongSignConvetion(degMinstoDegDec(Longitude_str), LongitudeDir)

        CurrentTimeinSec, CurrentTimeinNSec = UTCtoUTCEpoch(UTC)

        custom_gps_msg.header = Header(frame_id='GPS1_Frame', stamp=rospy.Time(CurrentTimeinSec, CurrentTimeinNSec))
        custom_gps_msg.latitude = LatitudeSigned
        custom_gps_msg.longitude = LongitudeSigned
        custom_gps_msg.altitude = float(inputString[9])
        custom_gps_msg.utm_easting, custom_gps_msg.utm_northing, custom_gps_msg.zone, custom_gps_msg.letter = convertToUTM(LatitudeSigned, LongitudeSigned)
        custom_gps_msg.hdop = float(HDOP)
        custom_gps_msg.gpgga_read = string
        rospy.loginfo(custom_gps_msg)
        rospy.loginfo("   ")
        pub.publish(custom_gps_msg)

if __name__ == '__main__':
    try:
        driver()
    except rospy.ROSInterruptException:
        pass