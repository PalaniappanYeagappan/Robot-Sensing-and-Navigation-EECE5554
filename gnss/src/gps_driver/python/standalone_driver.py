#!/usr/bin/env python3

import time
import rospy
import serial
import sys
from datetime import date
import utm.conversion as utm
from gps_driver.msg import Customgps

date_format = '%d%m%Y%H%M%S.%f'
latitude = 0.0
longitude = 0.0
current_date = date.today().strftime("%d%m%Y")


def parse_line(line, publisher):
    gps_data = Customgps()
    gps_data.header.frame_id = "GPS1_Frame"

    if line.startswith("$GPGGA"):
        gps_data.gpgga_read = line.strip() 
        data = line.split(",")
        rospy.loginfo(data)

        if not data[2]:
            rospy.logwarn("Warning: GPS puck is unable to receive data")
        else:
            gps_data.hdop = float(data[8]) 
            process_data(data, gps_data)
            publisher.publish(gps_data)


def process_data(data, gps_data):
    set_time(data[1], gps_data)
    set_lat_long_alt(data[2], data[3], data[4], data[5], data[9], gps_data)
    set_utm(gps_data)


def set_time(time_data, gps_data):
    epoch_time = time.mktime(time.strptime(current_date + time_data, date_format))
    gps_data.header.stamp.secs = int(epoch_time)
    gps_data.header.stamp.nsecs = 0  

def set_lat_long_alt(lat, lat_dir, lon, lon_dir, alt, gps_data):
    global latitude, longitude
    latitude = convert_to_decimal(lat, lat_dir == 'N')
    longitude = convert_to_decimal(lon, lon_dir == 'E')
    gps_data.altitude = float(alt)
    gps_data.latitude = latitude
    gps_data.longitude = longitude


def convert_to_decimal(coord, is_positive):
    base, minutes = float(coord[:2]), float(coord[2:]) / 60

    decimal_coord = base + minutes

    if is_positive:
        return decimal_coord
    else:
        return -decimal_coord


def set_utm(gps_data):
    utm_data = utm.from_latlon(latitude, longitude)
    gps_data.utm_easting, gps_data.utm_northing = utm_data[0], utm_data[1]
    gps_data.zone, gps_data.letter = utm_data[2], utm_data[3]


def main():
    rospy.init_node('gps_node')
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

    publisher = rospy.Publisher('/gps', Customgps, queue_size=5)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            line = str(serial_port.readline())[2:]
            rospy.loginfo(line)
            parse_line(line, publisher)
        except serial.serialutil.SerialException as e:
            rospy.logerr('SerialException: ' + str(e))
        rate.sleep()

    serial_port.close()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr('Error: An unexpected error has occurred\n', str(e))
