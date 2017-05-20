#!/usr/bin/env python

import rospy

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal
import matplotlib.pyplot as plt
from matplotlib.pyplot import draw

# plt.ion()

def callback(msg):
	pass
	markers = ['.', '+', '=', '*', '-', '1', '2', '3']	
	nr_joints = len(msg.goal.trajectory.joint_names)
	position = [[] for i in xrange(nr_joints)]
	velocity = [[] for i in xrange(nr_joints)]
	acceleration = [[] for i in xrange(nr_joints)]
	time = []
	for point in msg.goal.trajectory.points:
		time.append(point.time_from_start.to_sec())
		for i, point_position in enumerate(point.positions):
			position[i].append(point_position)
		for i, point_velocity in enumerate(point.velocities):
			velocity[i].append(point_velocity)
		for i, point_acceleration in enumerate(point.accelerations):
			acceleration[i].append(point_acceleration)
		
	plt.close('all')

	fig = plt.figure(1)
	
	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())

	fig.subplots_adjust(left=0.06, bottom=0.07, right =0.97, top=0.96, wspace=0.32, hspace=0.2)

	for i in xrange(nr_joints):
		# plot positions
		plt.subplot(2, nr_joints, i + 1)
		plt.scatter(time, position[i])
		plt.ylabel('position for joint ' + str(i + 1))
		plt.xlabel('time/sec.')
		#plot velocities
		plt.subplot(2, nr_joints, i + nr_joints + 1)
		plt.scatter(time, velocity[i])
		plt.ylabel('velocity for joint' + str(i + 1))
		plt.xlabel('time')


	# plt.tight_layout()
	# plt.draw()
	
	plt.show()

if __name__ == '__main__':
	rospy.init_node("arm1_TrajectoryGoalPloter")

	# edit the topic you are interested
	rospy.Subscriber("/arm_1/arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, callback)
	rospy.spin()