#!/usr/bin/env python3

import time
import rospy
import serial
import sys
from datetime import date
import utm
from gps_driver.msg import Customgps
from std_msgs.msg import Header

date_format = '%d%m%Y%H%M%S.%f'
custom_gps_msg = Customgps()

def latdegMinstoDeg(latitude):
    deg = int(latitude[:2])
    mins = float(latitude[2:])
    degDec = mins / 60
    return deg + degDec

def longdegMinstoDeg(longitude):
    if len(longitude>9):
        deg = int(longitude[:3])
        mins = float(longitude[3:])
    else:
        deg = int(longitude[:2])
        mins = float(longitude[2:])
    degDec = mins / 60
    return deg + degDec

def LatLongSignConvetion(LatOrLong, LatOrLongDir):
    if LatOrLongDir == "S" or LatOrLongDir =="W" :
        LatOrLong *= -1
    return LatOrLong

def convertToUTM(LatitudeSigned, LongitudeSigned):
    UTMVals = utm.from_latlon(float(LatitudeSigned), float(LongitudeSigned))
    UTMEasting, UTMNorthing, UTMZone, UTMLetter = UTMVals
    return [UTMEasting, UTMNorthing, UTMZone, UTMLetter]

def UTCtoUTCEpoch(UTC):
    UTC = float(UTC)
    UTCinSecs = (UTC // 10000) * 3600 + ((UTC % 10000) // 100) * 60 + (UTC % 100)
    CurrentTime = time.time()
    TimeSinceEpochBOD = CurrentTime - UTCinSecs
    CurrentTimeSec = int(TimeSinceEpochBOD)
    CurrentTimeNsec = (TimeSinceEpochBOD - CurrentTimeSec) * 1e9
    return [CurrentTimeSec, CurrentTimeNsec]

def parse_line(line, publisher):
    global custom_gps_msg
    if line.startswith("$GPGGA"):
        custom_gps_msg.gpgga_read = line.strip()
        data = line.split(",")
        if not data[2]:
            rospy.logwarn("Warning: GPS puck is unable to receive data")
        else:
            custom_gps_msg.hdop = float(data[8])
            process_data(data)
            publisher.publish(custom_gps_msg)

def process_data(data):
    global custom_gps_msg
    epoch_time = UTCtoUTCEpoch(data[1])
    custom_gps_msg.header.stamp.secs = epoch_time[0]
    custom_gps_msg.header.stamp.nsecs = epoch_time[1]
    custom_gps_msg.latitude = LatLongSignConvetion(latdegMinstoDeg(data[2]), data[3])
    custom_gps_msg.longitude = LatLongSignConvetion(longdegMinstoDeg(data[4]), data[5])
    custom_gps_msg.altitude = float(data[9])
    utm_data = convertToUTM(custom_gps_msg.latitude, custom_gps_msg.longitude)
    custom_gps_msg.utm_easting, custom_gps_msg.utm_northing = utm_data[0], utm_data[1]
    custom_gps_msg.zone, custom_gps_msg.letter = utm_data[2], utm_data[3]

def main():
    rospy.init_node('gps_publisher_node')
    rospy.logdebug('See expected launch format: roslaunch gps_driver driver.launch port:=/dev/ttyUSB0')
    args = rospy.myargv(argv=sys.argv)

    serial_port = None
    port_name = rospy.get_param('~port',"/dev/ttyUSB0")
    baud_rate = rospy.get_param('~baudrate', 4800)
    try:
        serial_port = serial.Serial(port_name, baud_rate, timeout=1.0)
        rospy.logdebug('GPS sensor has been initialized on port', port_name)
    except serial.serialutil.SerialException as e:
        rospy.logerr('SerialException: ' + str(e))
        sys.exit(1)

    publ = rospy.Publisher('/gps', Customgps, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            line = str(serial_port.readline())[2:]
            rospy.loginfo(line)
            parse_line(line, publ)
        except serial.serialutil.SerialException as e:
            rospy.logerr('SerialException: ' + str(e))
        rate.sleep()

    serial_port.close()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr('Error: An unexpected error has occurred\n', str(e))
