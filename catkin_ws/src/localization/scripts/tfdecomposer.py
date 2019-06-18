import rospy
import tf2

from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion


class TFDecomposer:
	def __init__(self):
		self.rollPub = rospy.Publisher(
                '/state/roll', Float64, queue_size=1)
        self.pitchPub = rospy.Publisher(
                '/state/pitch', Float64, queue_size=1)
        self.yawPub = rospy.Publisher(
                '/state/yaw', Float64, queue_size=1)

        self._listener = tf.TransformListener()

    def update(self):
    	try:
    		trans, rot = self._listener.lookupTransform(
    				'map', 'base_link', rospy.Time(0))

    		(roll, pitch, yaw) = euler_from_quaternion(rot)

    		self.rollPub.publish(roll)
    		self.pitchPub.publish(pitch)
    		self.yawPub.publish(yaw)

    		return

    	except (tf.LookupException,
    			tf.ConnectivityException,
    			tf.ExtrapolationException) as e:
    		rospy.logwarn(e.message)


if __name__ == '__main__':
	rospy.init_node('tfDecomposer')
	decomposer = TFDecomposer()

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		decomposer.update()
		rate.sleep()
	rospy.spin()