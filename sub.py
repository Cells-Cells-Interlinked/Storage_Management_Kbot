#!/usr/bin/python3
import rospy

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

vel = Twist()
pos = Pose()

def callback(data):
	"""Active when receive info from topic"""
	# Using global keyword to edit variable outside function
	global pos
	pos = data

def main():
	rospy.init_node("lab2")
	rate = rospy.Rate(100)	
	# Code to publish
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	sub = rospy.Subscriber('/turtle1/pose', Pose, callback)
	while not rospy.is_shutdown():
		#TODO
		print("Turtle is running at ", pos.x, pos.y)
		vel.linear.x = 1
		if (pos.x >= 10):
			vel.linear.x = 0
			break
		pub.publish(vel)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass


