#!/usr/bin/env python
# license removed for brevity
import rospy
from ras_lab1_msgs.msg import PWM
from geometry_msgs.msg import Twist
from ras_lab1_msgs.msg import Encoders
import math

# robot model
ticks = 360
b = 0.115
r = 0.0352
freq = 10

pwm = PWM()

# desired velocity
vw1d = 0
vw2d = 0

# error sum
error_sum1 = 0.0
error_sum2 = 0.0

# controller
Kp = 0.3
Ki = 0.7
Kd = 0.0

# last erros
le1 = 0
le2 = 0

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
	global Kd
	global le1
	global le2

	rospy.loginfo("Encoder Data recieved: {} {}".format(data.delta_encoder1, data.delta_encoder2))
	rospy.loginfo("Desired: {}, {}".format(vw1d,vw2d) )

	evw1 = vw1d - data.delta_encoder1
	evw2 = vw2d - data.delta_encoder2
	
	rospy.loginfo("Error: {}, {}".format(evw1,evw2) )

	error_sum1 += (float(evw1)/freq)
	error_sum2 += (float(evw2)/freq)

	dl1 = evw1-le1
	dl2 = evw2-le2

	rospy.loginfo("DL: {}, {}".format(dl1,dl2) )

	e1 = (Kp*evw1) + (Ki*error_sum1) + (Kd* (dl1*freq) )
	e2 = (Kp*evw2) + (Ki*error_sum2) + (Kd* (dl2*freq) )
	rospy.loginfo("PID: {}, {}".format(e1,e2) )

	pwm.PWM1 += e1
	pwm.PWM2 += e2

	le1 = evw1
	le2 = evw2

	rospy.loginfo("Publishing PWM: {}, {}".format(pwm.PWM1,pwm.PWM2) )
	pub.publish(pwm)
	rate.sleep()
	

def callback_pwm(data):
	global vw1d
	global vw2d

	v = data.linear.x
	w = data.angular.x

	vw1 = v - (b*w)
	vw2 = v + (b*w)

	vw1d = (vw1*ticks)/(2*math.pi*r*freq)
	vw2d = (vw2*ticks)/(2*math.pi*r*freq)

	rospy.loginfo("Received Command: {} {}".format(v, w))
	
def main():
	rospy.Subscriber("/motor_controller/twist", Twist, callback_pwm)
	rospy.Subscriber("/kobuki/encoders", Encoders, callback_encoder)

if __name__ == '__main__':
	try:
		global rate

		# talker()
		rospy.init_node('controller_node', anonymous=True)
		rate = rospy.Rate(freq)
		
		main()

		rospy.spin()

	except rospy.ROSInterruptException:
		pass