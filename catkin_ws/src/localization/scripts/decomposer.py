#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion


class Decomposer:
    '''
    Decomposer class to monitor tf and publish raw floats to ROS PID
    '''

    def __init__(self):
        '''
        Constructor for Decomposer
        '''

        # Publishers and Subscribers
        self.point_pub = rospy.Publisher(
                '~point', Point, queue_size=1)
        self.yaw_pub = rospy.Publisher(
                '~heading', Float64, queue_size=1)

        # tf Listener
        self._listener = tf.TransformListener()

        # Relevant tf Frames
        self.base_link_frame = rospy.get_param(
            '~base_link_frame', default='base_link')
        self.map_frame = rospy.get_param(
            '~map_frame', default='odom')

    def spin(self):
        '''
        Lookup tf transform and convert to euler angles
        '''
        try:
            trans, rot = self._listener.lookupTransform(
                    self.map_frame, self.base_link_frame, rospy.Time(0))

            (x, y, z) = trans
            (roll, pitch, yaw) = euler_from_quaternion(rot)

            point = Point()
            point.x = x
            point.y = y
            point.z = z

            self.point_pub.publish(point)
            self.yaw_pub.publish(yaw)

            return

        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logdebug(e.message)


if __name__ == "__main__":
    rospy.init_node('decomposer')
    decomposer = Decomposer()

    # Decomposer update rate
    rate = rospy.Rate(rospy.get_param('state_estimation/decomposer_rate', 10))

    # Cycle forever at fixed rate
    while not rospy.is_shutdown():
        decomposer.spin()
        rate.sleep()
    rospy.spin()