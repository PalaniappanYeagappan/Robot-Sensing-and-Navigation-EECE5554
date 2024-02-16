#!/usr/bin/python3

import rospy
import serial
import utm
from gps_driver.msg import Customrtk
from std_msgs.msg import Header
import math
import time
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
    seconds = float(UTC[4:])
    UTCinSecs = hours * 3600 + minutes * 60 + seconds
    TimeSinceEpoch = time.time()
    UTCinSecs = float(UTC)
    TimeSinceEpochBOD = float(TimeSinceEpoch - UTCinSecs)
    CurrentTime = TimeSinceEpochBOD + UTCinSecs
    CurrentTimeSec = int(CurrentTime)
    CurrentTimeNsec = float((CurrentTime - CurrentTimeSec) * 1e9)

    return [CurrentTimeSec, CurrentTimeNsec]

def isGNGGAinString(inputString):
    return "$GNGGA" in inputString

if __name__ == '__main__':
    rospy.init_node('rtk_driver')
    serialPortAddr = rospy.get_param('~port', '/dev/pts/4')
    serialPort = serial.Serial(serialPortAddr, 4800)
    pub = rospy.Publisher('gps', Customrtk, queue_size=10)

    try:
        while not rospy.is_shutdown():
            gngga_Read = str(serialPort.readline())
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
                Current_Time = UTCtoUTCEpoch(UTC)
                rospy.loginfo('publishing data')
                gps_msg = Customrtk()
                gps_msg.header.frame_id = 'GPS1_Frame'
                gps_msg.header.stamp = rospy.Time(int(time.time()), int((time.time() % 1) * 1e9))
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

            rospy.sleep(1)

    except rospy.ROSInterruptException:
        serialPort.close()

    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down node")