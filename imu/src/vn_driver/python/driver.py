#!/usr/bin/env python3

import rospy
import serial
import math
from vn_driver.msg import Vectornav  
import numpy as np
import sys

def isVNYMRinString(inputString):
    return "$VNMYR" in inputString

def read_data(port):
    raw_data = port.readline().decode('utf-8').rstrip()
    # if isVNYMRinString(str(raw_data)):
    imu_data = raw_data.split(',')
    temp = imu_data[-1].split('*')
    rospy.logdebug(imu_data)

    data_arr = np.ones(13)
    for i in range(1, 12):
        data_arr[i] = float(imu_data[i])
    data_arr[12] = float(temp[0])
    rospy.logdebug(data_arr)
    print(data_arr)
    return data_arr, raw_data

def to_quaternions(data_arr):
    y = math.radians(data_arr[1])
    p = math.radians(data_arr[2])
    r = math.radians(data_arr[3])
    cy = math.cos(y/2)
    cp = math.cos(p/2)
    cr = math.cos(r/2)
    sy = math.sin(y/2)
    sp = math.sin(p/2)
    sr = math.sin(r/2)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    # rospy.logdebug("Quaternions: %f, %f, %f, %f" % (qx, qy, qz, qw))
    return [qx, qy, qz, qw]

def configure_vectornav(port, data_freq, data_output_mode):
    try:
        port.write(b"$VNWRG,07,%d*xx\r" % data_freq)
        port.write(b"$VNWRG,06,%d*xx\r" % data_output_mode)
        check = 0
        while check == 0:
            aodf_req = port.write(b"VNRRG,07*xx\r")
            AODF = port.readline()
            if str(data_freq) in str(aodf_req) or str(data_freq) in str(AODF):
                check = 1

        while check == 1:
            aodf_req = port.write(b"VNRRG,06*xx\r")
            AODF = port.readline()
            if str(data_output_mode) in str(aodf_req) or str(data_output_mode) in str(AODF):
                check = 0
        rospy.loginfo('IMU configured')
    except Exception as e:
        rospy.logerr('IMU not configured as per requirement: %s' % str(e))

if __name__ == "__main__":
    rospy.init_node('imu_driver', anonymous=True)
    args = rospy.myargv(argv=sys.argv)
    if len(args) < 2:
        print("Usage: standalone_driver.py <port>")
        sys.exit(1)
        
    port_param = args[1]
    
    port = serial.Serial(port_param, 115200)
    data_freq = 40
    data_output_mode = 14
    configure_vectornav(port,data_freq,data_output_mode)
    rate = rospy.Rate(10) 

    imu_pub = rospy.Publisher('imu', Vectornav, queue_size=10)

    try:
        while not rospy.is_shutdown():
            try:
                data_arr, raw_data = read_data(port)
                quaternions = to_quaternions(data_arr)

                msg = Vectornav()

                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "imu1_frame"
                msg.imu.header.frame_id = 'imu1_frame'
                msg.imu.header.stamp = rospy.Time.now()
                msg.imu.orientation.x = quaternions[0]
                msg.imu.orientation.y = quaternions[1]
                msg.imu.orientation.z = quaternions[2]
                msg.imu.orientation.w = quaternions[3]
                msg.mag_field.header.frame_id = 'imu1_frame'
                msg.mag_field.header.stamp = rospy.Time.now()
                msg.mag_field.magnetic_field.x = data_arr[4]
                msg.mag_field.magnetic_field.y = data_arr[5]
                msg.mag_field.magnetic_field.z = data_arr[6]
                msg.imu.linear_acceleration.x = data_arr[7]
                msg.imu.linear_acceleration.y = data_arr[8]
                msg.imu.linear_acceleration.z = data_arr[9]
                msg.imu.angular_velocity.x = data_arr[10]
                msg.imu.angular_velocity.y = data_arr[11]
                msg.imu.angular_velocity.z = data_arr[12]
                msg.raw_imudata = raw_data
                rospy.loginfo("Publishing info")
                rospy.loginfo(msg)
                imu_pub.publish(msg)

            except ValueError as e:
                rospy.logwarn("ValueError: Could not convert data. " + str(e))
            except IndexError as e:
                rospy.logwarn("IndexError: Data out of range. " + str(e))

    except rospy.ROSInterruptException:
        pass

    port.close()
