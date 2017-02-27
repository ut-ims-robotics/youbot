#!/usr/bin/env python

import rospy

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal
from std_msgs.msg import Header
import random

if __name__ == "__main__":
	pub = rospy.Publisher('/arm_1/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size = 10)
	rospy.init_node('fake_traj_publisher')
	rate= rospy.Rate(0.5) # 10 Hz
	d = rospy.Duration
	while not rospy.is_shutdown():
		fake_traj = FollowJointTrajectoryActionGoal()
		fake_traj.header = Header()
		# fake_traj.header.stamp

		fake_traj.goal = FollowJointTrajectoryGoal()

		fake_traj.goal.trajectory = JointTrajectory()
		fake_traj.goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']

		# fake_traj.goal.trajectory.points = JointTrajectoryPoint()[]
		for j in xrange(20):
			point = JointTrajectoryPoint()
			point.positions = [random.random() for i in xrange(5)]
			point.velocities = [random.random() for i in xrange(5)]
			point.time_from_start = rospy.Duration.from_sec(j)
			fake_traj.goal.trajectory.points.append(point)
			

		# fake_traj.goal.trajectory.points.positions = [[random.random() ]
		# fake_traj.goal.trajectory.points.velocities = [[random.random() for i in xrange(5)] for j in xrange(20)]
		# fake_traj.goal.trajectory.points.time_from_start = [d.from_sec(i * 0.1) for i in xrange(20)]
		# fake_traj.goal.trajectory.points.append(positions)
		# fake_traj.goal.trajectory.points.append(velocities)
		# fake_traj.goal.trajectory.points.append(time_from_start)

		pub.publish(fake_traj)
		rate.sleep()

