#!/usr/bin/env python
import rospy
from ras_lab1_msgs.msg import Encoders

last_encoder = Encoders()

def GetEncoderData():
	return last_encoder

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "Data recieved: {} {}".format(data.delta_encoder1, data.delta_encoder2))
	last_encoder = data
	
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