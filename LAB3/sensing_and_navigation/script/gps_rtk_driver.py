#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import utm
from std_msgs.msg import String
from std_msgs.msg import Header
from sensing_and_navigation.msg import utm_rtk_msg


def gngga_to_degrees(x):
    deg = int(x/100)
    deg = deg+(x/100-deg)*10/6
    return deg


if __name__ == '__main__':
    SENSOR_NAME = "gps"
    rospy.init_node('gps_talker', anonymous=True)
    port = serial.Serial('/dev/ttyUSB0', 57600, timeout=3.)

    gps_pub = rospy.Publisher('gps_data', utm_rtk_msg, queue_size=10)

    try:
        while not rospy.is_shutdown():
            line = port.readline()

            if line == '':
                rospy.logwarn("Port: No data")
            else:
                if line.startswith('$GNGGA'):
                    gngga_data = line.split(",")
                    e_w = -1 if gngga_data[5] == 'W' else 1
                    n_s = -1 if gngga_data[3] == 'S' else 1
                    utm_data = utm.from_latlon(n_s*gngga_to_degrees(
                        float(gngga_data[2])), e_w*gngga_to_degrees(float(gngga_data[4])))

                    utm_msg_values = utm_rtk_msg()

                    utm_msg_values.header.stamp = rospy.Time.now()
                    utm_msg_values.header.frame_id = 'gps'
                    utm_msg_values.latitude = gngga_to_degrees(
                        float(gngga_data[2]))
                    utm_msg_values.longitude = gngga_to_degrees(
                        float(gngga_data[4]))
                    utm_msg_values.altitude = float(gngga_data[9])
                    utm_msg_values.utm_easting = utm_data[0]
                    utm_msg_values.utm_northing = utm_data[1]
                    utm_msg_values.zone = utm_data[2]
                    utm_msg_values.letter = utm_data[3]
                    utm_msg_values.quality = gngga_data[6]

                    gps_pub.publish(utm_msg_values)

    except rospy.ROSInterruptException:
        port.close()

    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gps node...")
