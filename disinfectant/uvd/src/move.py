#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
waypoints = [[(0.0792529,1.03939,0),(0,0,0,1)],
[(6.94202,1.0853,0),(0,0,0,1)],
[(6.9789,-1.34636,0),(0,0,0,1)],
[(-0.433152,1.12214,0),	(0,0,0,1)],
[(-6.51767,1.04845,0),(0,0,0,1)],
[(-3.05134,1.04845,0),(0,0,0,1)],
[(-6.44392,-1.53057,0),(0,0,0,1)]
]

rospy.init_node('patrol')
pub = rospy.Publisher('light', String, queue_size=10)
class Patrol:

    def __init__(self):
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')

    def set_goal_to_point(self, point):

		goal_pose = MoveBaseGoal()
		goal_pose.target_pose.header.frame_id = 'map'
		goal_pose.target_pose.pose.position.x = pose[0][0]
		goal_pose.target_pose.pose.position.y = pose[0][1]
		goal_pose.target_pose.pose.position.z = pose[0][2]
		goal_pose.target_pose.pose.orientation.x = pose[1][0]
		goal_pose.target_pose.pose.orientation.y = pose[1][1]
		goal_pose.target_pose.pose.orientation.z = pose[1][2]
		goal_pose.target_pose.pose.orientation.w = pose[1][3]
		return goal_pose


rate = rospy.Rate(3000)

if __name__ == '__main__':

	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()
	p = Patrol()
	while True:
		for pose in waypoints:
			goal = p.set_goal_to_point(pose)
			client.send_goal(goal)
			client.wait_for_result()
			pub.publish("Turn_on")
			rospy.loginfo("Disinfectant light Turn On")
			rate.sleep()
			pub.publish("Turn_off")
			rospy.loginfo("Disinfectant light Turn Off")
