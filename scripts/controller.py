#!/usr/bin/env python
# license removed for brevity
import rospy
# from std_msgs.msg import String
from ras_lab1_msgs.msg import PWM
from geometry_msgs.msg import Twist
from ras_lab1_msgs.msg import Encoders
from listner import GetEncoderData
import math

ticks = 360
b = 0.115
r = 0.0352

pub = rospy.Publisher('/kobuki/pwm', PWM)

# def talker():
# 	pub = rospy.Publisher('/kobuki/pwm', PWM, queue_size=10)
# 	rospy.init_node('controller_node', anonymous=True)
# 	rate = rospy.Rate(10) # 10hz
# 	pwm = PWM()
# 	while not rospy.is_shutdown():
# 		pwm.PWM1 = 100
# 		pwm.PWM2 = 100
# 		rospy.loginfo("Publishing PWM: {}, {}".format(pwm.PWM1,pwm.PWM2) ) 
# 		pub.publish(pwm)
# 		rate.sleep()

def callback(data):

	v = data.linear
	w = data.angular

	rospy.loginfo("Received Command: {} {}".format(v, w))

	encoder = GetEncoderData()
	pwm = PWM()

	vm = (v.x+v.y)/2.0
	wm = (w.y-w.x)/(2*b)

	pwm.PWM1 = (2*math.pi*r*encoder.delta_encoder1)/ticks
	pwm.PWM2 = (2*math.pi*r*encoder.delta_encoder2)/ticks

	rospy.loginfo("Publishing PWM: {}, {}".format(pwm.PWM1,pwm.PWM2) ) 
	pub.publish(pwm)
	
def listener():
	rospy.init_node('pwm_listener', anonymous=True)
	rospy.Subscriber("/motor_controller/twist", Twist, callback)

	rospy.spin()

def initialize():
	pwm = PWM()
	pwm.PWM1 = 100
	pwm.PWM2 = 100
	pub.publish(pwm)

if __name__ == '__main__':
	try:
		# talker()
		initialize()
		listener()
	except rospy.ROSInterruptException:
		pass