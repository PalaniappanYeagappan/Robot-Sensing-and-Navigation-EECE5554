#!/usr/bin/env python3

import time
import rospy
import serial
import sys
from datetime import date
import utm.conversion as utm
from gps_driver.msg import Customgps as gps_msg

date_format = '%d%m%Y%H%M%S.%f'
latitude_val = 0.0
longitude_val = 0.0
current_date_str = date.today().strftime("%d%m%Y")
gps_data_msg = gps_msg()
gps_data_msg.header.frame_id = "GPS1_Frame"

def parse_gps_line(line):
    global latitude_val, longitude_val, current_date_str, gps_data_msg

    if line.startswith("$GPGGA"):
        gps_data_msg.gpgga_read = line.strip()
        data = line.split(",")
        if not data[2]:
            rospy.logwarn("Warning:: GPS puck is unable to receive data")
        else:
            gps_data_msg.hdop = float(data[8])
            process_gps_data(data)
            gps_publisher.publish(gps_data_msg)

def process_gps_data(data):
    set_gps_time(data[1])
    set_gps_lat_long_alt(data[2], data[3], data[4], data[5], data[9])
    set_gps_utm()

def set_gps_time(time_data):
    global current_date_str, gps_data_msg

    epoch_time = time.mktime(time.strptime(current_date_str + time_data, date_format))
    gps_data_msg.header.stamp.secs = int(epoch_time)
    gps_data_msg.header.stamp.nsecs = 0

def set_gps_lat_long_alt(lat, lat_dir, lon, lon_dir, alt):
    global latitude_val, longitude_val, gps_data_msg

    latitude_val = convert_coord_to_decimal(lat, lat_dir == 'N')
    longitude_val = convert_coord_to_decimal(lon, lon_dir == 'E')
    gps_data_msg.altitude = float(alt)
    gps_data_msg.latitude = latitude_val
    gps_data_msg.longitude = longitude_val

def convert_coord_to_decimal(coord, is_positive):
    base, minutes = float(coord[:2]), float(coord[2:]) / 60
    decimal_coord = base + minutes

    if is_positive:
        return decimal_coord
    else:
        return -decimal_coord

def set_gps_utm():
    global latitude_val, longitude_val, gps_data_msg

    utm_data = utm.from_latlon(latitude_val, longitude_val)
    gps_data_msg.utm_easting, gps_data_msg.utm_northing = utm_data[0], utm_data[1]
    gps_data_msg.zone, gps_data_msg.letter = utm_data[2], utm_data[3]

def main_gps_node():
    rospy.init_node('gps_sensor_node')
    rospy.logdebug('See expected launch format:: roslaunch gps_driver driver.launch port:=/dev/ttyUSB0')
    args = rospy.myargv(argv=sys.argv)

    serial_port = None
    port_name = rospy.get_param('~port', '/dev/ttyUSB0')
    baud_rate = rospy.get_param('~baudrate', 4800)
    try:
        serial_port = serial.Serial(port_name, baud_rate, timeout=1.0)
        rospy.logdebug('GPS sensor has been initialized on port', port_name)
    except serial.serialutil.SerialException as e:
        rospy.logerr('SerialException:: ' + str(e))
        sys.exit(1)

    gps_publisher = rospy.Publisher('/gps', gps_msg, queue_size=5)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            line = str(serial_port.readline())[2:]
            parse_gps_line(line)
        except serial.serialutil.SerialException as e:
            rospy.logerr('SerialException:: ' + str(e))
        rate.sleep()

    serial_port.close()

if __name__ == '__main__':
    try:
        main_gps_node()
    except Exception as e:
        rospy.logerr('Error:: An unexpected error has occurred\n', str(e))
