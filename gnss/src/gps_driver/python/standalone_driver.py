#!/usr/bin/env python3
# Collaborated with Yogeshwaran Eswaran, Shiva Kumar Dhandapani, Sai Sreekar K S and Poojith Maddineni 

import rospy
import serial
import utm
from gps_driver.msg import Customgps
from std_msgs.msg import Header
import math
import time
import rosbag
from datetime import datetime, timezone

def lat_degMinstoDec(latitude):
    deg = int(latitude[:2])
    mins = float(latitude[2:])
    degDec = mins / 60
    return deg + degDec

def long_degMinstoDec(longitude):
    if len(longitude)>9:
        deg = int(longitude[:3])
        mins = float(longitude[3:])
    else:
        deg = int(longitude[:2])
        mins = float(longitude[2:])
    degDec = mins / 60
    return deg + degDec

def LatLongSign(LatLong, LatLongDir):
   if LatLongDir == "S" or LatLongDir =="W" :
       LatLong *= -1
   return LatLong

def convertToUTM(LatitudeSigned, LongitudeSigned):
    UTM_Vals = utm.from_latlon(float(LatitudeSigned), float(LongitudeSigned))
    UTM_Easting, UTM_Northing, UTM_Zone, UTM_Letter = UTM_Vals
    return [UTM_Easting, UTM_Northing, UTM_Zone, UTM_Letter]

def UTCtoUTCEpoch(UTC):
    UTC = float(UTC)
    UTCinSecs = (UTC // 10000) * 3600 + ((UTC % 10000) // 100) * 60 + (UTC % 100)
    Current_Time = datetime.utcnow()
    Current_UTC_Time = datetime.utcnow()
    Midnight_UTC = datetime.combine(Current_UTC_Time, datetime.min.time(), tzinfo=timezone.utc)    
    TimeSinceEpochBOD = Midnight_UTC.timestamp()
    Current_Time_Sec = (Current_Time)
    Current_Time_Nsec = ((Current_Time - Current_Time_Sec) * 1e9)
    return [Current_Time_Sec, Current_Time_Nsec]

if __name__ == '__main__':
    rospy.init_node('gps_driver')
    serialPortAddr = rospy.get_param('~port', '/dev/ttyUSB0')
    # serialPortAddr = rospy.get_param('~port', '/dev/pts/1')
    serialPort = serial.Serial(serialPortAddr, 4800)
    pub =rospy.Publisher('gps',Customgps,queue_size=10) 
    try:
        while not rospy.is_shutdown():
            gpgga_Read = str(serialPort.readline())
            if "$GPGGA" in gpgga_Read:
                gpggaSplit = gpgga_Read.split(',')
                print(f'the GPGGA string is:{gpggaSplit}')
                UTC = gpggaSplit[1]
                Latitude = str(gpggaSplit[2])
                LatitudeDir = str(gpggaSplit[3])
                Longitude = str(gpggaSplit[4])
                LongitudeDir = str(gpggaSplit[5])
                Altitude = float(gpggaSplit[9])
                hdop =float (gpggaSplit[8])
                LatitudeDec = lat_degMinstoDec(Latitude)
                LongitudeDec = long_degMinstoDec(Longitude)
                LatitudeSigned = LatLongSign(LatitudeDec, LatitudeDir)
                LongitudeSigned = LatLongSign(LongitudeDec, LongitudeDir)
                UTM_Vals = convertToUTM(LatitudeSigned, LongitudeSigned)    
                Current_Time = UTCtoUTCEpoch(UTC)             
                rospy.loginfo('publishing data')
                gps_msg = Customgps()
                gps_msg.header.frame_id = 'GPS1_Frame'
                gps_msg.header.stamp = rospy.Time(int(time.time()), int((time.time() % 1) * 1e9))
                gps_msg.latitude = float(Latitude)
                gps_msg.longitude = float(Longitude)
                gps_msg.altitude = Altitude
                gps_msg.utm_easting = UTM_Vals[0]
                gps_msg.utm_northing = UTM_Vals[1]
                gps_msg.zone = int(UTM_Vals[2])
                gps_msg.letter = UTM_Vals[3]
                gps_msg.hdop = hdop
                gps_msg.gpgga_read=gpgga_Read
                gps_msg.header.stamp = rospy.Time.now()
                pub.publish(gps_msg)            
            rospy.sleep(1)

    except rospy.ROSInterruptException:
        serialPort.close()
       
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down")
