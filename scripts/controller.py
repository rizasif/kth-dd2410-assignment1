#!/usr/bin/env python
# license removed for brevity
import rospy
# from std_msgs.msg import String
from ras_lab1_msgs.msg import PWM
from geometry_msg.msg import twist
from ras_lab1_msgs.msg import Encoders
from listner import GetEncoderData
import math

ticks = 360
b = 0.115
r = 0.0352

pub = rospy.Publisher('/kobuki/pwm', PWM, queue_size=10)

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
	encoder = GetEncoderData()
	pwm = PWM()

	vm = (v[0]+v[1])/2
	wm = (w[1]-w[0])/(2*b)

	pwm.PWM1 = (2*math.pi*r*encoder.delta_encoder1)/ticks
	pwm.PWM2 = (2*math.pi*r*encoder.delta_encoder2)/ticks

	rospy.loginfo("Publishing PWM: {}, {}".format(pwm.PWM1,pwm.PWM2) ) 
	pub.publish(pwm)
	
def listener():
	rospy.init_node('pwm_listener', anonymous=True)
	rospy.Subscriber("/motor_controller/twist", twist, callback)

	rospy.spin()

if __name__ == '__main__':
	try:
		# talker()
		listener()
	except rospy.ROSInterruptException:
		pass