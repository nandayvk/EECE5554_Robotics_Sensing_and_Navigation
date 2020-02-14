#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

# Simple talker demo that listens to std_msgs/Strings published
# to the 'chatter' topic

import rospy
import utm
from std_msgs.msg import String
from std_msgs.msg import Header
from sensing_and_navigation.msg import utm_msg


def gpgga_to_degrees(deg):
    x = (deg-0.4*int(deg))*10/6
    corrected_deg = int(x)+(x-int(x))*10/6
    # deg = deg+(x-deg)*10/6
    return corrected_deg


def callback(data):
    pub = rospy.Publisher('corrected_gps_data', utm_msg, queue_size=10)
    utm_data = utm.from_latlon(gpgga_to_degrees(
        data.latitude), (-1)*gpgga_to_degrees(data.longitude))
    utm_msg_values = utm_msg()

    utm_msg_values.header.seq = data.header.seq
    utm_msg_values.header.stamp = data.header.stamp
    utm_msg_values.header.frame_id = data.header.frame_id
    utm_msg_values.latitude = gpgga_to_degrees(data.latitude)
    utm_msg_values.longitude = (-1)*gpgga_to_degrees(data.longitude)
    utm_msg_values.altitude = data.altitude
    utm_msg_values.utm_easting = utm_data[0]
    utm_msg_values.utm_northing = utm_data[1]
    utm_msg_values.zone = utm_data[2]
    utm_msg_values.letter = utm_data[3]

    pub.publish(utm_msg_values)

    # print("header = {}, latitude = {}, longitude = {}, altitude = {}, utm_easting = {}, utm_northig = {}, zone = {}, letter = {} ".format(
    # data.header, data.latitude, data.longitude, data.altitude, data.utm_easting, data.utm_northing, data.zone, data.letter))

    # # rospy.loginfo('header = %d', data.header.seq)
    # rospy.loginfo('latitude = %f', data.latitude)
    # rospy.loginfo('longitude = %f', data.longitude)
    # rospy.loginfo('altitude = %f', data.altitude)
    # rospy.loginfo('utm_easting = %f', data.utm_easting)
    # rospy.loginfo('utm_northing = %f', data.utm_northing)
    # rospy.loginfo('zone = %d', data.zone)
    # rospy.loginfo('letter = %s', data.letter)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('converter', anonymous=True)
    rospy.Subscriber('gps_data', utm_msg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
