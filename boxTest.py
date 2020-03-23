# INSTRUCTIONS TO RUN |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#
# Open a new terminal and enter $. roslaunch niryo_one_bringup desktop_rviz_simulation.launch
# Within rviz add the planning scene function to visualize the boxes and barrier
# Edit the below parameters contained within the "CHANGE THESE VALUES" marked region
#
# END INSTRUCTIONS TO RUN |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

import sys
import copy 
import rospy
import numpy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time

#from maths import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from niryo_one_python_api.niryo_one_api import *
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from math import pi

class test:
	def __init__(self):
 		#Initialisation I
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('niryo_one_test', anonymous=True)
		robot = moveit_commander.RobotCommander()

		#Initialisation II
		group_name = "arm"
		group = moveit_commander.MoveGroupCommander(group_name)
		scene = moveit_commander.PlanningSceneInterface()
		display_trajectory_publisher = rospy.Publisher
		moveIntialState = lambda joint_goal: group.go(joint_goal, wait=True)
		planning_frame = group.get_planning_frame()
		eef_link = group.get_end_effector_link()
		group_names = robot.get_group_names()

		#Home - get initial joint values, more accurate than pose.
		self.joint_goal = group.get_current_joint_values()

		#Misc variables
		self.table_name = ''
		self.box_name = ''
		self.boxTwo_name = ''
		self.boxThree_name = ''
		self.robot = robot
		self.scene = scene
		self.group = group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_names

# CHANGE THESE VALUES FOR YOUR SIMULATION ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
		self.barrier_position_x = 0.3
		self.barrier_position_y = 0
		self.barrier_position_z = 0

		self.barrier_size_x = 0.05
		self.barrier_size_y = 0.5
		self.barrier_size_z = 0.05

		self.box_One_position_x = 0.155312
		self.box_One_position_y = 0.0768738
		self.box_One_position_z = 0.00930201

		self.box_Two_position_x = 0.249928
		self.box_Two_position_y = 0.0410179
		self.box_Two_position_z = 0.0118977


		self.box_Three_position_x = 0.252273
		self.box_Three_position_y = -0.0432303
		self.box_Three_position_z = 0.0093861


		self.targetOne_position_x = 0.15
		self.targetOne_position_y = 0.07
		self.targetOne_position_z = 0.025


		self.targetTwo_position_x = 0.15
		self.targetTwo_position_y = 0.07
		self.targetTwo_position_z = 0.06


		self.targetThree_position_x = 0.15
		self.targetThree_position_y = 0.07
		self.targetThree_position_z = 0.095

# END OF "CHANGE THESE VALUES FOR YOUR SIMULATION" ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

	def moveIntialState(self): #Reset to the original state using the joint values found in initialisation II
		group = self.group
		group.go(self.joint_goal, wait=True) #Not necessary to plan
		group.stop() #Reduces residual movement
	
	def add_scene(self, timeout=4): #Create the scene with three boxes and a barrier from variables above
		scene = self.scene
		robot = self.robot
		barx = self.barrier_size_x
		bary = self.barrier_size_y
		barz = self.barrier_size_z

		rospy.sleep(2) #Wait for ros to catch up

		p = PoseStamped() #box one
		p.header.frame_id = robot.get_planning_frame()
		p.pose.position.x = self.box_One_position_x
		p.pose.position.y = self.box_One_position_y
		p.pose.position.z = self.box_One_position_z
		scene.add_box("box_one", p, (0.025, 0.025, 0.025)) #Change the size of the box if necessary

		d = PoseStamped() #box two
		d.header.frame_id = robot.get_planning_frame()
		d.pose.position.x = self.box_Two_position_x
		d.pose.position.y = self.box_Two_position_y
		d.pose.position.z = self.box_Two_position_z
		scene.add_box("box_two", d, (0.025, 0.025, 0.025)) #Change the size of the box if necessary

		g = PoseStamped() #box three
		g.header.frame_id = robot.get_planning_frame()
		g.pose.position.x = self.box_Three_position_x
		g.pose.position.y = self.box_Three_position_y
		g.pose.position.z = self.box_Three_position_z
		scene.add_box("box_three", g, (0.025, 0.025, 0.025)) #Change the size of the box if necessary
		
		b = PoseStamped() #barrier
		b.header.frame_id = robot.get_planning_frame()
		b.pose.position.x = self.barrier_position_x
		b.pose.position.y = self.barrier_position_y
		b.pose.position.z = self.barrier_position_z
		scene.add_box("barrier", b, (barx, bary, barz)) #Size taken from inputs above

	def remove_box(self, timeout=4): #Remove the scene
		scene = self.scene
		robot = self.robot

		rospy.sleep(2)

		scene.remove_world_object("box_one")
		scene.remove_world_object("box_two")
		scene.remove_world_object("box_three")
		scene.remove_world_object("barrier")

	def planPoseGoalOne(self): #Comments applicable to all '#1' 
		group = self.group

		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w =  0.7101854 
		pose_goal.orientation.x = 0.0 
		pose_goal.orientation.y = 0.7040147
		pose_goal.orientation.z = 0.0
		pose_goal.position.x = self.box_One_position_x
		pose_goal.position.y = self.box_One_position_y
		pose_goal.position.z = self.box_One_position_z + 0.05
		group.set_pose_target(pose_goal)

		# Call the planner to compute the plan and execute it.
		plan=group.plan()		
		plan = group.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		group.stop()
	    # Clear targets after planning with poses.
	    # Note: no equivalent function for clear_joint_value_targets()
		group.clear_pose_targets()

	def planPoseGoalTwo(self): #1
		group = self.group
		
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w = 0.7101854
		pose_goal.orientation.x = 0.0 
		pose_goal.orientation.y = 0.7040147
		pose_goal.orientation.z = 0.0 
		pose_goal.position.x = self.box_Two_position_x
		pose_goal.position.y = self.box_Two_position_y
		pose_goal.position.z = self.box_Two_position_z + 0.05
		group.set_pose_target(pose_goal)

		plan=group.plan()		
		plan = group.go(wait=True)

		group.stop()

		group.clear_pose_targets()

	def planPoseGoalThree(self): #1
		group = self.group
		
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w = 0.7101854
		pose_goal.orientation.x = 0.0 
		pose_goal.orientation.y = 0.7040147 
		pose_goal.orientation.z = 0.0  
		pose_goal.position.x = self.box_Three_position_x
		pose_goal.position.y = self.box_Three_position_y
		pose_goal.position.z = self.box_Three_position_z + 0.05
		group.set_pose_target(pose_goal)


		plan=group.plan()		
		plan = group.go(wait=True)

		group.stop()

		group.clear_pose_targets()
	
	def planPoseTargetOne(self): #1
		group = self.group
		
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w = 0.7101854
		pose_goal.orientation.x = 0.0 
		pose_goal.orientation.y = 0.7040147
		pose_goal.orientation.z = 0.0  
		pose_goal.position.x = self.targetOne_position_x
		pose_goal.position.y = self.targetOne_position_y
		pose_goal.position.z = self.targetOne_position_z + 0.05
		group.set_pose_target(pose_goal)

		plan=group.plan()
		plan = group.go(wait=True)

		group.stop()

		group.clear_pose_targets()

	def planPoseTargetTwo(self): #1
		group = self.group
		
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w = 0.7101854
		pose_goal.orientation.x = 0
		pose_goal.orientation.y = 0.7040147 
		pose_goal.orientation.z = 0.0
		pose_goal.position.x = self.targetTwo_position_x
		pose_goal.position.y = self.targetTwo_position_y
		pose_goal.position.z = self.targetTwo_position_z + 0.05
		group.set_pose_target(pose_goal)

		plan=group.plan()
		plan = group.go(wait=True)

		group.stop()

		group.clear_pose_targets()

	def planPoseTargetThree(self): #1
		group = self.group
		
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w = 00.7101854
		pose_goal.orientation.x = 0.0 
		pose_goal.orientation.y = 0.7040147
		pose_goal.orientation.z = 0.0 
		pose_goal.position.x = self.targetThree_position_x
		pose_goal.position.y = self.targetThree_position_y
		pose_goal.position.z = self.targetThree_position_z + 0.05
		group.set_pose_target(pose_goal)

		plan=group.plan()
		plan = group.go(wait=True)

		group.stop()

		group.clear_pose_targets()

	def attach_boxOne(self):
		scene = self.scene
		scene.attach_box("hand_link", "box_one", touch_links="touch_links")#Attatch box to end effector
		rospy.sleep(1)#Sleep thread to allow changes

	def attach_boxTwo(self):
		scene = self.scene
		scene.attach_box("hand_link", "box_two", touch_links="touch_links")#Attatch box to end effector
		rospy.sleep(1)#Sleep thread to allow changes

	def attach_boxThree(self):
		scene = self.scene
		scene.attach_box("hand_link", "box_three", touch_links="touch_links")#Attatch box to end effector
		rospy.sleep(1)#Sleep thread to allow changes

	def remove_boxOne(self):
		scene = self.scene
		scene.remove_attached_object("hand_link", name="box_one")#Remove box as attatched to end effector
		rospy.sleep(1)#Sleep thread to allow changes
	
	def remove_boxTwo(self):
		scene = self.scene
		scene.remove_attached_object("hand_link", name="box_two")#Remove box as attatched to end effector
		rospy.sleep(1)#Sleep thread to allow changes
	
	def remove_boxThree(self):
		scene = self.scene
		scene.remove_attached_object("hand_link", name="box_three")#Remove box as attatched to end effector
		rospy.sleep(1)#Sleep thread to allow changes

		

if __name__ == '__main__':
	try:
		print "-=-=-=- Press 'Enter' to Begin -=-=-=-"
		raw_input()
		test = test()

		test.add_scene()
		time.sleep(5)
		test.planPoseGoalOne()
		test.attach_boxOne()
		test.planPoseTargetOne()
		test.remove_boxOne()

		test.planPoseGoalTwo()
		test.attach_boxTwo()
		test.planPoseTargetTwo()
		test.remove_boxTwo()

		test.planPoseGoalThree()
		test.attach_boxThree()
		test.planPoseTargetThree()
		test.remove_boxThree()

		test.remove_box()
		test.moveIntialState()
   	except rospy.ROSInterruptException:
   		pass