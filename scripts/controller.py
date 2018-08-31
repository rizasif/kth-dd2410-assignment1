#!/usr/bin/env python
# license removed for brevity
import rospy
# from std_msgs.msg import String
from ras_lab1_msgs.msg import PWM
# from geometry_msg.msg import Twist
from ras_lab1_msgs.msg import Encoders
from listner import GetEncoderData


pub = rospy.Publisher('/kobuki/pwm', PWM, queue_size=10)

def talker():
	pub = rospy.Publisher('/kobuki/pwm', PWM, queue_size=10)
	rospy.init_node('controller_node', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	pwm = PWM()
	while not rospy.is_shutdown():
		pwm.PWM1 = 100
		pwm.PWM2 = 100
		rospy.loginfo("Publishing PWM: {}, {}".format(pwm.PWM1,pwm.PWM2) ) 
		pub.publish(pwm)
		rate.sleep()

# def callback(data):
# 	v = data.linear
# 	w = data.angular
# 	encoder = GetEncoderData()


# 	pwm = PWM()
# 	rospy.loginfo("Publishing PWM: {}, {}".format(pwm.PWM1,pwm.PWM2) ) 
# 	pub.publish(pwm)
	
# def listener():
# 	rospy.init_node('pwm_listener', anonymous=True)
# 	rospy.Subscriber("/motor_controller/twist", Twist, callback)

# 	rospy.spin()

if __name__ == '__main__':
	try:
		talker()
		# listener()
	except rospy.ROSInterruptException:
		pass