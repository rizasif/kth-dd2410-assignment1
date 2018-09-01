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
freq = 10

rate = 0
# is_initialized = False

pwm = PWM()

# desired velocity
lind = 0
angd = 0

# error sum
error_sum1 = 0.0
error_sum2 = 0.0

# controller
Kp = 0.2
Ki = 0.0

pub = rospy.Publisher('/kobuki/pwm', PWM, queue_size=10)


# def talker():
# 	pub = rospy.Publisher('/kobuki/pwm', PWM, queue_size=10)
# 	rate = rospy.Rate(10) # 10hz
# 	pwm = PWM()
# 	while not rospy.is_shutdown():
# 		pwm.PWM1 = 100
# 		pwm.PWM2 = 100
# 		rospy.loginfo("Publishing PWM: {}, {}".format(pwm.PWM1,pwm.PWM2) ) 
# 		pub.publish(pwm)
# 		rate.sleep()

def callback_encoder(data):
	global error_sum1
	global error_sum2
	global pub
	global lind
	global angd
	global rate

	# rospy.loginfo(rospy.get_caller_id() + "Encoder Data recieved: {} {}".format(data.delta_encoder1, data.delta_encoder2))
	w1 = (float(2)*math.pi*r*data.delta_encoder1*freq)/ticks
	w2 = (float(2)*math.pi*r*data.delta_encoder2*freq)/ticks

	lina = (w1+w2)/2.0
	anga = (w2-w1)/(2.0*b)

	elin = lind-lina
	eang = angd-anga

	evw1 = elin - (b*eang)
	evw2 = elin + (b*eang)

	error_sum1 += float(evw1)
	error_sum2 += float(evw2)

	pwm.PWM1 = (Kp*evw1) + (Ki*error_sum1)
	pwm.PWM2 = (Kp*evw2) + (Ki*error_sum2)

	pwm.PWM1 = (pwm.PWM1*ticks)/(2*math.pi*r*freq)
	pwm.PWM2 = (pwm.PWM2*ticks)/(2*math.pi*r*freq)

	rospy.loginfo("Publishing PWM: {}, {}".format(pwm.PWM1,pwm.PWM2) )
	pub.publish(pwm)
	rate.sleep()
	

def callback_pwm(data):
	global lind
	global angd

	v = data.linear
	w = data.angular

	lind = v.x
	angd = w.x

	rospy.loginfo("Received Command: {} {}".format(v, w))

	# rospy.loginfo("Publishing PWM: {}, {}".format(pwm.PWM1,pwm.PWM2) ) 
	# pub.publish(pwm)
	# rate.sleep()
	
def listener():
	# rospy.init_node('pwm_listener', anonymous=True)
	rospy.Subscriber("/motor_controller/twist", Twist, callback_pwm)
	rospy.Subscriber("/kobuki/encoders", Encoders, callback_encoder)

	# rospy.spin()

# def keep_moving():
# 	pub = rospy.Publisher('/kobuki/pwm', PWM, queue_size=10)
# 	while not rospy.is_shutdown():
# 		pub.publish(pwm)
# 		rate.sleep()

	# rospy.spin()

if __name__ == '__main__':
	try:
		global rate

		# talker()
		rospy.init_node('controller_node', anonymous=True)
		rate = rospy.Rate(freq)
		
		# initialize()
		listener()

		rospy.spin()

	except rospy.ROSInterruptException:
		pass