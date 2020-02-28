#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import utm
import math
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
# from sensing_and_navigation.msg import utm_msg


def vnymr_to_quat(yaw, pitch, roll):
    cy = math.cos(math.radians(yaw) * 0.5)
    sy = math.sin(math.radians(yaw) * 0.5)
    cp = math.cos(math.radians(pitch) * 0.5)
    sp = math.sin(math.radians(pitch) * 0.5)
    cr = math.cos(math.radians(roll) * 0.5)
    sr = math.sin(math.radians(roll) * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr
    return w, x, y, z


# def data_publish(imu_pub_data):


if __name__ == '__main__':
    SENSOR_NAME = "imu"

    rospy.init_node('imu_talker', anonymous=True)
    imu_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=3.)
    # gps_port = serial.Serial('/dev/ttyUSB1', 4800, timeout=3.)

    # g_pub = rospy.Publisher('gps_pub', utm_msg, queue_size=10)
    line_pub = rospy.Publisher('imu_line', String, queue_size=10)
    imu_pub = rospy.Publisher('imu_data', Imu, queue_size=10)
    mag_pub = rospy.Publisher('mag_data', MagneticField, queue_size=10)

    try:
        while not rospy.is_shutdown():
            # gps_line = gps_port.readline()
            imu_line = imu_port.readline()

            if imu_line == '':
                rospy.logwarn("GPS Port: No data")
            else:
                if imu_line.startswith('$VNYMR'):
                    vnymr_data = imu_line.split(",")
                    # e_w = -1 if vnymr_data[5] == 'W' else 1
                    # n_s = -1 if vnymr_data[3] == 'S' else 1
                    # utm_data = utm.from_latlon(n_s*vnymr_to_degrees(
                    #     float(vnymr_data[2])), e_w*vnymr_to_degrees(float(vnymr_data[4])))

                    imu_values = Imu()
                    mag_values = MagneticField()

                    q_w, q_x, q_y, q_z = vnymr_to_quat(
                        float(vnymr_data[1]), float(vnymr_data[2]), float(vnymr_data[3]))

                    imu_values.header.stamp = rospy.Time.now()
                    imu_values.header.frame_id = 'imu'
                    imu_values.orientation.x = q_x
                    imu_values.orientation.y = q_y
                    imu_values.orientation.z = q_z
                    imu_values.orientation.w = q_w
                    imu_values.angular_velocity.x = float(
                        vnymr_data[10].split("*")[0])
                    imu_values.angular_velocity.y = float(
                        vnymr_data[11].split("*")[0])
                    imu_values.angular_velocity.z = float(
                        vnymr_data[12].split("*")[0])
                    imu_values.linear_acceleration.x = float(vnymr_data[7])
                    imu_values.linear_acceleration.y = float(vnymr_data[8])
                    imu_values.linear_acceleration.z = float(vnymr_data[9])

                    imu_values.header.stamp = rospy.Time.now()
                    imu_values.header.frame_id = 'mag'
                    mag_values.magnetic_field.x = float(vnymr_data[4])
                    mag_values.magnetic_field.y = float(vnymr_data[5])
                    mag_values.magnetic_field.z = float(vnymr_data[6])

                    line_pub.publish(imu_line)
                    imu_pub.publish(imu_values)
                    mag_pub.publish(mag_values)

    except rospy.ROSInterruptException:
        port.close()

    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gps node...")
