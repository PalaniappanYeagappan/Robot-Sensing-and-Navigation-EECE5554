#!/usr/bin/python3

import rospy
import utm
import time
from gps_driver.msg import Customgps
from std_msgs.msg import Header
import serial
import sys
import math
from datetime import datetime, timezone

custom_gps_msg = Customgps()

def lat_degMinstoDec(latitude):
    deg = int(latitude[:2])
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
    hours = int(UTC[0:1])
    minutes = int(UTC[2:3])
    seconds = float(UTC[4:])
    UTCinSecs = hours * 3600 + minutes * 60 + seconds
    TimeSinceEpoch = time.time()
    TimeSinceEpochBOD = float(TimeSinceEpoch - UTCinSecs)
    CurrentTime = TimeSinceEpochBOD + UTCinSecs
    CurrentTimeSec = float(CurrentTime)
    CurrentTimeNsec = float(((CurrentTime - CurrentTimeSec) % 1) * 1e9)

    return [CurrentTimeSec, CurrentTimeNsec]

def isGPGGAinString(inputString):
    return "$GPGGA" in inputString

if __name__ == '__main__':
    rospy.init_node('gps_driver')
    
    # Parse command line arguments
    args = rospy.myargv(argv=sys.argv)
    if len(args) < 2:
        print("Usage: standalone_driver.py <port>")
        sys.exit(1)
        
    # Read port name from command line arguments
    port_param = args[1]
    
    serial_port = serial.Serial(port_param, 4800)
    pub = rospy.Publisher('gps', Customgps, queue_size=10)
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            gpgga_Read = str(serial_port.readline())
            
            if isGPGGAinString(gpgga_Read):
                gpggaSplit = gpgga_Read.split(',')
                UTC = gpggaSplit[1]
                Latitude = str(gpggaSplit[2])
                LatitudeDir = str(gpggaSplit[3])
                Longitude = str(gpggaSplit[4])
                LongitudeDir = str(gpggaSplit[5])
                Altitude = float(gpggaSplit[9])
                HDOP = float(gpggaSplit[8])
                LatitudeDec = lat_degMinstoDec(Latitude)
                LongitudeDec = long_degMinstoDec(Longitude)
                LatitudeSigned = LatLongSign(LatitudeDec, LatitudeDir)
                LongitudeSigned = LatLongSign(LongitudeDec, LongitudeDir)
                UTM_Vals = convertToUTM(LatitudeSigned, LongitudeSigned)
                CurrentTimeSec, CurrentTimeNsec = UTCtoUTCEpoch(UTC)
                
                custom_gps_msg.header.frame_id = 'GPS1_Frame'
                custom_gps_msg.header.stamp = rospy.Time(CurrentTimeSec, CurrentTimeNsec)
                custom_gps_msg.latitude = float(LatitudeSigned)
                custom_gps_msg.longitude = float(LongitudeSigned)
                custom_gps_msg.altitude = Altitude
                custom_gps_msg.utm_easting = UTM_Vals[0]
                custom_gps_msg.utm_northing = UTM_Vals[1]
                custom_gps_msg.zone = int(UTM_Vals[2])
                custom_gps_msg.letter = UTM_Vals[3]
                custom_gps_msg.hdop = HDOP
                custom_gps_msg.gpgga_read = gpgga_Read

                print("secs ", rospy.Time(int(time.time())))
                print("nsecs ", rospy.Time(int((time.time() % 1) * 1e9)))
                print("Latitude ", float(LatitudeSigned))
                print("Longitude ", float(LongitudeSigned))
                print("Easting ", UTM_Vals[0])
                print("Northing ", UTM_Vals[1])
                print("Zone", UTM_Vals[2])
                print("letter ", UTM_Vals[3])
                print("Altitude ", Altitude)
                
                pub.publish(custom_gps_msg)
                rospy.loginfo("Publishing GPS data...")
            
            rate.sleep()

    except rospy.ROSInterruptException:
        serial_port.close()

    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down node")