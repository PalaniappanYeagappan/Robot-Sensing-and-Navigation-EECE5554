#!/usr/bin/python3

import rospy
import serial
import utm
from gps_driver.msg import Customrtk
from std_msgs.msg import Header
import math
import time
import sys
from datetime import datetime, timezone

def lat_degMinstoDec(latitude):
    deg = float(latitude[:2])
    mins = float(latitude[2:])
    degDec = mins / 60
    return deg + degDec

def long_degMinstoDec(longitude):
    if len(longitude) > 9:
        deg = int(longitude[:3])
        mins = float(longitude[3:])
    else:
        deg = int(longitude[:2])
        mins = float(longitude[2:])
    degDec = mins / 60
    return deg + degDec

def LatLongSign(LatLong, LatLongDir):
    if LatLongDir == "S" or LatLongDir == "W":
        LatLong *= -1
    return LatLong

def convertToUTM(LatitudeSigned, LongitudeSigned):
    UTM_Vals = utm.from_latlon(float(LatitudeSigned), float(LongitudeSigned))
    UTM_Easting, UTM_Northing, UTM_Zone, UTM_Letter = UTM_Vals
    return [UTM_Easting, UTM_Northing, UTM_Zone, UTM_Letter]

def UTCtoUTCEpoch(UTC):
    UTC = str(UTC)
    hours = int(UTC[0:2])
    minutes = int(UTC[2:4])
    seconds = float(UTC[4:6])
    nsecs = float(UTC[6:9])
    millisecond = float(UTC[7:])
    
    UTCinSecs = hours * 3600 + minutes * 60 + seconds + millisecond / 1000.0
    TimeSinceEpoch = time.time()
    TimeSinceEpochBOD = float(TimeSinceEpoch - (TimeSinceEpoch % (24*60*60)) )
    CurrentTime = TimeSinceEpochBOD + UTCinSecs
    CurrentTimeSec = int(CurrentTime)
    CurrentTimeNsec = int(nsecs * 1e9)

    return [CurrentTimeSec, CurrentTimeNsec]

def isGNGGAinString(inputString):
    return "$GNGGA" in inputString

if __name__ == '__main__':
    rospy.init_node('rtk_driver')
    args = rospy.myargv(argv=sys.argv)
    if len(args) < 2:
        print("Usage: standalone_driver.py <port>")
        sys.exit(1)
        
    # Read port name from command line arguments
    port_param = args[1]
    
    serial_port = serial.Serial(port_param, 4800)
    pub = rospy.Publisher('gps', Customrtk, queue_size=10)
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            gngga_Read = str(serial_port.readline())
            if isGNGGAinString(gngga_Read):
                gnggaSplit = gngga_Read.split(',')
                UTC = gnggaSplit[1]
                Latitude = str(gnggaSplit[2])
                LatitudeDir = str(gnggaSplit[3])
                Longitude = str(gnggaSplit[4])
                LongitudeDir = str(gnggaSplit[5])
                Altitude = float(gnggaSplit[9])
                HDOP = float(gnggaSplit[8])
                fix_quality = int(gnggaSplit[6])
                LatitudeDec = lat_degMinstoDec(Latitude)
                LongitudeDec = long_degMinstoDec(Longitude)
                LatitudeSigned = LatLongSign(LatitudeDec, LatitudeDir)
                LongitudeSigned = LatLongSign(LongitudeDec, LongitudeDir)
                UTM_Vals = convertToUTM(LatitudeSigned, LongitudeSigned)
                CurrentTimeSec, CurrentTimeNsec = UTCtoUTCEpoch(UTC)
                rospy.loginfo('publishing data')
                gps_msg = Customrtk()
                gps_msg.header.frame_id = 'GPS1_Frame'
                gps_msg.header.stamp = rospy.Time(CurrentTimeSec, CurrentTimeNsec)
                gps_msg.latitude = float(Latitude)
                gps_msg.longitude = float(Longitude)
                gps_msg.altitude = Altitude
                gps_msg.utm_easting = UTM_Vals[0]
                gps_msg.utm_northing = UTM_Vals[1]
                gps_msg.zone = int(UTM_Vals[2])
                gps_msg.letter = UTM_Vals[3]
                gps_msg.hdop = HDOP
                gps_msg.gngga_read = gngga_Read
                gps_msg.header.stamp = rospy.Time.now()
                pub.publish(gps_msg)

            rate.sleep()

    except rospy.ROSInterruptException:
        serial_port.close()

    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down node")
