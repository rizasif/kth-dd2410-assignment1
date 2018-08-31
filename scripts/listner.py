#!/usr/bin/env python
import rospy
from ras_lab1_msgs.msg import Encoders

last_encoder = Encoders()

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "Data recieved at {}".format(data.data.timestamp))
	last_encoder = data.data
	
def listener():
	
	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('encoder_listener', anonymous=True)
	rospy.Subscriber("/kobuki/encoders", Encoders, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()