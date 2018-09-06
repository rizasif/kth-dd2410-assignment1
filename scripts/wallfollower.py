#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from ras_lab1_msgs.msg import ADConverter
from geometry_msgs.msg import Twist

freq = 10
l = 0.2

speed = 0.1

sum_theta = 0
last_theta = 0

d_dist = 0.35

# controller
Kp = 0.5
Ki = 0.05
Kd = 0.001

pub = rospy.Publisher('/motor_controller/twist', Twist, queue_size=10)

def callback_adc(data):
	global rate
	global last_theta
	global sum_theta
	global Kp
	global Ki

	ch1 = data.ch1
	ch2 = data.ch2

	rospy.loginfo("Channels Recieved: {}, {}".format(ch1,ch2) )

	# d1 = 1.114*math.exp(-0.004*ch1)
	# d2 = 1.114*math.exp(-0.004*ch2)

	# rospy.loginfo("Distances: {}, {}".format(d1,d2) )

	# num = d1-d2
	# dnum =  math.pow(d1-d2, 2) + math.pow(l, 2)
	# dnum = math.sqrt(dnum)
	# theta = math.atan2(num,dnum)
	
	# avg_dist = (d1+d2)/2.0
	# ed =  d_dist - avg_dist
	# theta += ed

	# sum_theta += theta/freq
	
	# new_theta = (Kp*theta + Ki*sum_theta) 

	twist = Twist()
	
	twist.linear.x = speed
	twist.angular.x = 0.02*(ch1-ch2)
	last_theta = new_theta

	rospy.loginfo("Publishing: {}, {}".format(twist.linear.x,twist.angular.x) )

	pub.publish(twist)
	rate.sleep()

	pass

def main():
	rospy.Subscriber("/kobuki/adc", ADConverter, callback_adc)
	pass

if __name__ == '__main__':
	try:
		global rate
		global last_theta

		rospy.init_node('wallfollower_node', anonymous=True)
		rate = rospy.Rate(freq)
		last_theta = 0
		
		main()

		rospy.spin()

	except rospy.ROSInterruptException:
		pass