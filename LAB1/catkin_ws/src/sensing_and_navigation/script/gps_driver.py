#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import utm
from std_msgs.msg import String
from std_msgs.msg import Header
from sensing_and_navigation.msg import utm_msg


def gpgga_to_degrees(x):
    deg = int(x/100)
    deg = deg+(x/100-deg)*10/6
    return deg


if __name__ == '__main__':
    SENSOR_NAME = "gps"
    rospy.init_node('gps_talker', anonymous=True)
    port = serial.Serial('/dev/ttyUSB0', 4800, timeout=3.)

    gps_pub = rospy.Publisher('gps_data', utm_msg, queue_size=10)

    try:
        while not rospy.is_shutdown():
            line = port.readline()

            if line == '':
                rospy.logwarn("Port: No data")
            else:
                if line.startswith('$GPGGA'):
                    gpgga_data = line.split(",")
                    e_w = -1 if gpgga_data[5] == 'W' else 1
                    n_s = -1 if gpgga_data[3] == 'S' else 1
                    utm_data = utm.from_latlon(n_s*gpgga_to_degrees(
                        float(gpgga_data[2])), e_w*gpgga_to_degrees(float(gpgga_data[4])))

                    utm_msg_values = utm_msg()

                    utm_msg_values.header.stamp = rospy.Time.now()
                    utm_msg_values.header.frame_id = 'gps'
                    utm_msg_values.latitude = gpgga_to_degrees(
                        float(gpgga_data[2]))
                    utm_msg_values.longitude = gpgga_to_degrees(
                        float(gpgga_data[4]))
                    utm_msg_values.altitude = float(gpgga_data[9])
                    utm_msg_values.utm_easting = utm_data[0]
                    utm_msg_values.utm_northing = utm_data[1]
                    utm_msg_values.zone = utm_data[2]
                    utm_msg_values.letter = utm_data[3]

                    gps_pub.publish(utm_msg_values)

    except rospy.ROSInterruptException:
        port.close()

    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gps node...")
