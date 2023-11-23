#!/usr/bin/python3
"""
http://wiki.ros.org/move_base
http://wiki.ros.org/actionlib
"""
import rospy
import tf.transformations
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import actionlib


def pose_callback(pose_with_covariance):
	# print(pose_with_covariance)
	pose = pose_with_covariance.pose.pose
	print("amcl_pose = {x: %f, y:%f, orientation.z:%f" % (pose.position.x, pose.position.y, pose.orientation.z))

def callback(data):
	"""Active when receive info from topic"""
	# Using global keyword to edit variable outside function
	global pos
	pos = data.pose.pose


def move_base_status_callback(status):
	pass


def move_base_result_callback(result):
	pass


class moveBaseAction():
	def __init__(self):
		self.move_base_action = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
		self.move_base_action.wait_for_server(rospy.Duration(5))

	def createGoal(self, x, y, theta):
		# quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
		quat = tf.transformations.quaternion_from_euler(0, 0, theta)

		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(quat[0], quat[1], quat[2], quat[3]))

		return goal

	def moveToPoint(self, x, y, theta):
		target_point = self.createGoal(x, y, theta)
		self.moveToGoal(target_point)

	def moveToGoal(self, goal):
		self.move_base_action.send_goal(goal)
		success = self.move_base_action.wait_for_result()
		state = self.move_base_action.get_state()
		print ("Move to %f, %f, %f ->" % (
			goal.target_pose.pose.position.x,
			goal.target_pose.pose.position.y,
			goal.target_pose.pose.orientation.z
		))
		if success and state == GoalStatus.SUCCEEDED:
			print(" Complete")
			return True
		else:
			print(" Fail")
			self.move_base_action.cancel_goal()
			return False

class storeMan():
	def __init__(self, pick, drop):
		self.mba = moveBaseAction()
		self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback)
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.r1 = pick[0]
		self.c1 = pick[1]
		self.r2 = drop[0]
		self.c2 = drop[1]
		self.wholePro()
	def transCoor(self):
		# Movement Goal (Center under shelf)
		self.x1 = (self.c1 - 1) * 0.6
		self.y1 = (self.r1 - 1) * 0.6
		# 4 Legs of shelf (The 4 Quadrant)
		self.s1 = [self.x1 + 0.3, self.y1 + 0.3]
		self.s2 = [self.x1 - 0.3, self.y1 + 0.3]
		self.s3 = [self.x1 - 0.3, self.y1 - 0.3]
		self.s4 = [self.x1 + 0.3, self.y1 - 0.3]
		# Dead-zone border (T = Top, B = Bottom, L = Left, R = Right)
		self.Ts1 = [self.s1[0], self.s1[1] + 0.6]
		self.Ts2 = [self.s2[0], self.s2[1] + 0.6]
		self.Ls2 = [self.s2[0] - 0.6, self.s2[1]]
		self.Ls3 = [self.s3[0] - 0.6, self.s3[1]]
		self.Bs3 = [self.s3[0], self.s3[1] - 0.6]
		self.Bs4 = [self.s4[0], self.s4[1] - 0.6]
		self.Rs4 = [self.s4[0] + 0.6, self.s4[1]]
		self.Rs1 = [self.s1[0] + 0.6, self.s1[1]]
		# Simple Dead-Zone
		self.lD = self.s3[0] - 0.6
		self.rD = self.s1[0] + 0.6
		self.tD = self.s1[1] + 0.6
		self.bD = self.s3[1] - 0.6
		# Movement Goal (Drop shelf)
		self.x2 = (self.c2 - 1) * 0.6
		self.y2 = (self.r2 - 1) * 0.6
	def stopIt(self):
		stop_cmd = Twist()
		self.cmd_vel_pub.publish(stop_cmd)
	def wholePro(self):
		self.transCoor()
		self.mba.moveToPoint(self.x1, self.y1, 0.0)
		# 'pos' is current position (x, y theta)
		if (self.lD < pos.position.x < self.s2[0] or
			self.s1[0] < pos.position.x < self.rD or
			self.bD < pos.position.y < self.s3[1] or
			self.s2[1] < pos.position.y < self.tD):
			self.stopIt()
		rospy.sleep(5)
		if (self.lD < pos.position.x < self.s2[0]):
			self.mba.moveToPoint(self.lD + 0.3, self.s2[1] - 0.3, -60.0)
		elif (self.s1[0] < pos.position.x < self.rD):
			self.mba.moveToPoint(self.rD - 0.3, self.s1[1] - 0.3, 60.0)
		elif (self.bD < pos.position.y < self.s3[1]):
			self.mba.moveToPoint(self.s3[0] + 0.3, self.bD + 0.3, 0.0)
		elif (self.s2[1] < pos.position.y < self.tD):
			self.mba.moveToPoint(self.s2[0] + 0.3, self.tD - 0.3, 1.0)
		rospy.sleep(5)
		self.mba.moveToPoint(self.x1, self.y1, 0.0)
		rospy.sleep(5)
		# Extend Jack code here <--------------------------------------------------------------
		self.mba.moveToPoint(self.x2, self.y2, 0.0)
		rospy.sleep(5)
		# Retract Jack code here <--------------------------------------------------------------
		self.mba.moveToPoint(0.0, 0.0, 0.0)

# Main program
def main():
	rospy.init_node('RAI1', anonymous=True)
	publisher_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
	rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
	rospy.Subscriber('/move_base/status', GoalStatusArray, move_base_status_callback)
	rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_result_callback)

	# TODO
	while not rospy.is_shutdown():
		test = storeMan([2,3],[5,7])
		
		break

	rospy.sleep(1)


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass