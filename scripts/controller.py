#!/usr/bin/env python
import geometry_msgs.msg
import tf
import math
import rospy
import roslib

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (pos, rot) = listener.lookupTransform(
                '/world', '/drone_0', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        euler = tf.transformations.euler_from_quaternion(rot)

        print("pos: ", pos)
        print("rot: ", euler)
rate.sleep()
