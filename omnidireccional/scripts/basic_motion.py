#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

global time
time = 3

if __name__ == "__main__":

	rospy.init_node('basic_motion', anonymous=True)
	pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():

		forward = [1, 0, 0, 0]
		msg = Float32MultiArray(data=forward)
		pub.publish(msg)
		rate.sleep()

"""
	backward = [-1, -1, -1, -1]
	msg = Float32MultiArray(data=backward)
	pub.publish(msg)
	print "Backward"
	rospy.sleep(time)

	left = [-1, 1, -1, 1]
	msg = Float32MultiArray(data=left)
	pub.publish(msg)
	print "Left"
	rospy.sleep(time)

	right = [1, -1, 1, -1]
	msg = Float32MultiArray(data=right)
	pub.publish(msg)
	print "Right"
	rospy.sleep(time)

	counterclockwise = [-1, 1, 1, -1]
	msg = Float32MultiArray(data=counterclockwise)
	pub.publish(msg)
	print "Antihorario"
	rospy.sleep(time)

	clockwise = [1, -1, -1, 1]
	msg = Float32MultiArray(data=clockwise)
	pub.publish(msg)
	print "Horario"
	rospy.sleep(time)

	stop = [0,0,0,0]
	msg = Float32MultiArray(data=stop)
	pub.publish(msg)

"""