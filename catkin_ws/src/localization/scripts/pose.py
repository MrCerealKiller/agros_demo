#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class PoseEcho(object):
	def __init__(self):
		self.sub = rospy.Subscriber('/odometry/filtered', Odometry, self.callback)
		self.pub = rospy.Publisher('~pose', PoseStamped, queue_size=1)
		self.ts = None
		self.frame = None
		self.pose = None

	def callback(self, msg):
		self.ts = msg.header.stamp
		self.frame = msg.child_frame_id
		self.pose = msg.pose.pose

		pose_msg = PoseStamped()
		pose_msg.header = Header()
		pose_msg.header.stamp = self.ts
		pose_msg.header.frame_id = self.frame
		pose_msg.pose = self.pose

		self.pub.publish(pose_msg)

if __name__ == '__main__':
	rospy.init_node('pose_echo')
	poser = PoseEcho()
	rospy.spin()
