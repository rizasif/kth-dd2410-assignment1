#!/usr/bin/env python
# license removed for brevity
import rospy
# from std_msgs.msg import String
from ras_lab1_msgs.msg import PWM

def talker():
	pub = rospy.Publisher('/motor_controller/twist', PWM, queue_size=10)
	rospy.init_node('controller_node', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	pwm = PWM()
	while not rospy.is_shutdown():
		pwm.PWM1 = 100
		pwm.PWM2 = 100
		rospy.loginfo("Publishing PWM: ", pwm)
		pub.publish(pwm)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass