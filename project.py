import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
import whycon.msg
import tf
import roslib

from math import pi
from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Point, Vector3
from niryo_one_python_api.niryo_one_api import *
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import PlaceAction, PlaceGoal
from moveit_msgs.msg import PlaceLocation
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal, GraspGeneratorOptions

# We define a helper Function here to return the robot back to intial pose. Call this function whenever you need to reset the robot's to its initial state
moveIntialState = lambda joint_goal: group.go(joint_goal, wait=True) 

class project:
	def __init__(self):
		#fetching parameters using rospy
		self._table_object_name = rospy.get_param('~table_object_name', 'Grasp_Table')
        self._grasp_object_name = rospy.get_param('~grasp_object_name', 'Grasp_Object')
        self._grasps_ac = SimpleActionClient('/moveit_simple_grasps_server/generate', GenerateGraspsAction)
        
	 	moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('niryo_one_test', anonymous=True)
		robot = moveit_commander.RobotCommander()
		group_name = "arm"
		group = moveit_commander.MoveGroupCommander(group_name)
		scene = moveit_commander.PlanningSceneInterface() 

	 	eef_link = group.get_end_effector_link()
		planning_frame = group.get_planning_frame()
		group_names = robot.get_group_names()
	 	self.group = group

   		print "============ Printing robot state"
    	print robot.get_current_state()
    	print ""

	 	self.pose_table = self._add_table(self._table_object_name)
	 	self.pose_target = self._add_target(self._grasp_object_name)

	 	# Misc variables
    	self.box_name = ''
    	self.robot = robot
    	self.scene = scene
    	self.group = group
    	self.display_trajectory_publisher = display_trajectory_publisher
    	self.planning_frame = planning_frame
    	self.eef_link = eef_link
    	self.group_names = group_names

    	

	def callback(self,data):
		print data
		#print data.markers[1].pose.position.x
		#http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html
	

	def _add_table(self, name):
    	p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = 0.5
        p.pose.position.y = 0.0
        p.pose.position.z = 0.22

        q = quaternion_from_euler(0.0, 0.0, numpy.deg2rad(90.0))
        p.pose.orientation = Quaternion(*q)

        # Table size from ~/.gazebo/models/table/model.sdf, using the values
        # for the surface link.
        self._scene.add_box(name, p, (0.5, 0.4, 0.02))

        return p.pose 

    def _add_target(self, name):
        p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = 0.25   
        p.pose.position.y = 0
        p.pose.position.z = 0.32

        q = quaternion_from_euler(0.0, 0.0, 0.0)
        p.pose.orientation = Quaternion(*q)

        self._scene.add_box(name, p, (0.01, 0.01, 0.005))

        return p.pose


	def _add_box(timeout=100):
		rospy.sleep(2)
	
	
		#### Add an Object
		object_name = "box"
		object_pose = geometry_msgs.msg.PoseStamped()
		#make sure to give a frame id(Frame this data is described with) along with the postions x,y,z.
		object_pose.position.x = 
		object_pose.position.y = 
		object_pose.position.z = 
		object_pose.orientation = 
		scene.add_box(object_name, object_pose, (0.05, 0.1, 0.05)) #adding object to the scene, where you specify the object using name, pose and size arguments.
		rospy.sleep(1) #If the Python node dies before publishing a collision object update message, the message could get lost and the box will not appear.
					#To ensure that the updates are made, we wait for few seconds. This can be done alternatively using get_known_object_names() and get_known_object_names()
					# flags.

	
		#### Plan Without Collision
		planPoseGoal() #Plan with the object in the scene.

	def _generate_grasps(self, pose, width):
		goal = GenerateGraspsGoal()
		goal.pose = pose #Generated from inputs
		goal.width = width #Generated from inputs

		impl = GraspGeneratorOptions()
		impl.grasp_direction = GraspGeneratorOptions.GRASP_DIRECTION_UP
		impl.grasp_rotation  = GraspGeneratorOptions.GRASP_ROTATION_FULL

		state = self._grasps_ac.send_goal_and_wait(goal)
		if state = GoalStatus.SUCCEEDED:
			grasps = self.



			
	def go_to_pose_goal(self):
		group = self.group
		pose_goal = group.get_current_pose()
    		
		#self.group.setPlanningTime(100)
    	pose_goal = geometry_msgs.msg.Pose()
    	pose_goal.orientation.w = 
    	pose_goal.orientation.x = 
    	pose_goal.orientation.y = 
    	pose_goal.orientation.z = 
    	pose_goal.position.x = 
    	pose_goal.position.y = 
    	pose_goal.position.z = 
    	group.set_pose_target(pose_goal)
			
    	plan = group.plan()
    	#plan = self.group.go(wait=True)

    	group.stop()

    	group.clear_pose_targets()
    		

    	#current_pose = self.group.get_current_pose().pose
    	#return all_close(pose_goal, current_pose, 0.01)


def main(args):
	obc = project()
	obc.tf()
	#obc.go_to_pose_goal()
	try:
		rospy.spin()
	except KeyboardInterrupt:
    		print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)




# HONOURABLE MENTIONS -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    #def tfbroad(self):
    	#rospy.init_node('fixed_tf_broadcaster')
    	#br = tf.TransformBroadcaster()
    	#rate = rospy.Rate(10.0)
        #while not rospy.is_shutdown():
        	#br.sendTransform((-0.0999930426478, -0.0210870318115, 0.719462275505),(0.142275213879, 0.011873292114, -0.989174008045, 0.0339347942033),rospy.Time.now(),"box","camera_color_optical_frame")
        #rate.sleep()
    
    #def tflisten(self,data):
    	#listener = tf.TransformListener()
    	#rate = rospy.Rate(10.0)
    	#while not rospy.is_shutdown():
        	#try:
            	#(trans,rot) = listener.lookupTransform("box", "base_link", rospy.Time(0))
        	#except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            	#continue

    #from visualization_msgs.msg import MarkerArray
	#from visualization_msgs.msg import Marker
	#from whycon.msg import detection_results_array

        #init
		#self.sub = rospy.Subscriber("/circle_finder/rviz_marker", MarkerArray, self.callback)
	 	#objectarray = MarkerArray()
	 	#marker = Marker()