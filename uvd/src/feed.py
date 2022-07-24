#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt16
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# importing csv module 
import csv 
  
# csv file name 
filename = "point.csv"
  
# initializing the titles and rows list 
fields = [] 
rows = [] 
number = []
i = 0


#waypoints = [[(0.0792529,1.03939,0),(0,0,0,1)],[(6.94202,1.0853,0),(0,0,0,1)],[(6.9789,-1.34636,0),(0,0,0,1)],[(-0.433152,1.12214,0),	(0,0,0,1)],[(-6.51767,1.04845,0),(0,0,0,1)],[(-3.05134,1.04845,0),(0,0,0,1)],[(-6.44392,-1.53057,0),(0,0,0,1)]]

rospy.init_node('patrol')
#pub = rospy.Publisher('light', String, queue_size=10)
pub = rospy.Publisher('toggle_led',UInt16,queue_size=10)
class Patrol:

    def __init__(self):
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')

    def set_goal_to_point(self, point):
		
		print point[0]

		goal_pose = MoveBaseGoal()
		goal_pose.target_pose.header.frame_id = 'map'
		goal_pose.target_pose.pose.position.x = float(point[0])
		goal_pose.target_pose.pose.position.y = float(point[1])
		goal_pose.target_pose.pose.position.z = float(point[2])
		goal_pose.target_pose.pose.orientation.x = float(point[3])
		goal_pose.target_pose.pose.orientation.y = float(point[4])
		goal_pose.target_pose.pose.orientation.z = float(point[5])
		goal_pose.target_pose.pose.orientation.w = float(point[6])
		return goal_pose


rate = rospy.Rate(3000)

if __name__ == '__main__':

	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()
	p = Patrol()
	#while True:
	with open(filename,) as csvfile: 
			# creating a csv reader object 
			csvreader = csv.reader(csvfile) 
      
			# extracting field names through first row 
			#fields = next(csvreader) 
  
			# extracting each data row one by one 
			for row in csvreader:
				rows.append(row)
				#print "rows = " + str(row)
				#print rows[i]
				goal = p.set_goal_to_point(rows[i])
				i+=1
				client.send_goal(goal)
				client.wait_for_result()
				pub.publish(1)
				rospy.loginfo("Disinfectant light Turn On")
				rate.sleep()
				pub.publish(0)
				rospy.loginfo("Disinfectant light Turn Off")				
		
