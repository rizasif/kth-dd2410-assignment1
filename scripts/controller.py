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
vw1d = 0
vw2d = 0

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
	global vw1d
	global vw2d
	global rate
	global Ki
	global Kp

	# rospy.loginfo(rospy.get_caller_id() + "Encoder Data recieved: {} {}".format(data.delta_encoder1, data.delta_encoder2))
	w1 = (float(2)*math.pi*r*data.delta_encoder1*freq)/ticks
	w2 = (float(2)*math.pi*r*data.delta_encoder2*freq)/ticks

	# lina = (w1+w2)/2.0
	# anga = (w2-w1)/(2.0*b)

	evw1 = vw1d - w1
	evw2 = vw2d - w2

	rospy.loginfo("Error: {}, {}".format(evw1,evw2) )

	# evw1 = ew1 - (b*eang)
	# evw2 = ew2 + (b*eang)

	# evw1 = (evw1*ticks)/(2*math.pi*r*freq)
	# evw2 = (evw2*ticks)/(2*math.pi*r*freq)

	error_sum1 += float(evw1)
	error_sum2 += float(evw2)

	pwm.PWM1 += (Kp*evw1) + (Ki*error_sum1)
	pwm.PWM2 += (Kp*evw2) + (Ki*error_sum2)

	rospy.loginfo("Publishing PWM: {}, {}".format(pwm.PWM1,pwm.PWM2) )
	pub.publish(pwm)
	rate.sleep()
	

def callback_pwm(data):
	global vw1d
	global vw2d

	v = data.linear
	w = data.angular

	vw1 = v - (b*w)
	vw2 = v + (b*w)

	vw1d = vw1
	vw2d = vw2

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