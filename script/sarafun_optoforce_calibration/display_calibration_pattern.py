#!/usr/bin/env python
"""
@package sarafun_optoforce_calibration
@author Anthony Remazeilles
@brief provide a service for localizing the calibration pattern in the camera sensor

Copyright (C) 2016 Tecnalia Research and Innovation
Distributed under the GNU GPL v3. For full terms see https://www.gnu.org/licenses/gpl.txt
"""

import rospy
import tf
import math
import numpy

import math
from geometry_msgs.msg import Point, TransformStamped
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
import tf_conversions.posemath as pm
from copy import deepcopy
import PyKDL

class DisplayCalibrationPattern(object):
    def __init__(self):
        self._is_init_ok = False

        self._br = tf.TransformBroadcaster()
        self._marker_pub = rospy.Publisher('calibration_board', Marker, queue_size=1)
        self._object_marker = Marker()
        self._object_marker.header.frame_id = '/ar_marker_0'
        self._object_marker.ns = 'board_pattern'
        self._object_marker.id = 1
        self._object_marker.type = Marker.CUBE
        self._object_marker.action = Marker.ADD
        self._object_marker.color.r = 1.0
        self._object_marker.color.a = 0.8
        self._object_marker.scale.x = 0.297
        self._object_marker.scale.y = 0.211
        self._object_marker.scale.z = 1e-6

        self._object_marker.pose.position.x = 0.0655
        self._object_marker.pose.position.y = 0
        self._object_marker.pose.position.z = 0
        self._object_marker.pose.orientation.x = 0
        self._object_marker.pose.orientation.y = 0
        self._object_marker.pose.orientation.z = 0
        self._object_marker.pose.orientation.w = 1.0

        self._is_init_ok = True
        self._listener = tf.TransformListener()
        self._service = rospy.Service("optoforce_calibrate", Trigger, self.calibration_service_)

    def calibration_service_(self, req):
        rospy.loginfo("[{}] Optoforce calibrate".format(rospy.get_name()))

        try:
            (trans, rot) = self._listener.lookupTransform('/world', '/optoforce', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return TriggerResponse(True, "Exception raised")
        rospy.loginfo("Read t= {} rot = {}".format(trans, rot))
        msg = "Update launch file with the following indication:\n"
        msg+= "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"optoforce_world\""
        msg+= " args=\"{} {} {} {} {} {} {} /world /optoforce_sensor  100\"/>".format(trans[0],
                                                                              trans[1],
                                                                              trans[2],
                                                                              rot[0],
                                                                              rot[1],
                                                                              rot[2],
                                                                              rot[3])
        rospy.logwarn(msg)
        return TriggerResponse(True, msg)
    
    def loop(self):
        while not rospy.is_shutdown():

            time_now = rospy.Time.now()
            self._marker_pub.publish(self._object_marker)
            rospy.sleep(0.01)
        
if __name__ == '__main__':
    rospy.init_node('debug_plane', anonymous=True)
    display = DisplayCalibrationPattern()

    if display._is_init_ok:
        display.loop()
        rospy.spin()
    else:
        rospy.logerr("Prb in initialization. Bye")
