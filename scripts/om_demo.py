#! /usr/bin/env python3

#Code taken from Robot Ignite Academy. If perfroms multiple movements in the joint task space. The arm
# will move from the home position to a position above a 40x40mm cube, the pick the cube up, return home, 
# then put the cube bacl where it got it from.

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import time
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler
#from aws_iot_mqtt_bridge.msg import OMPayload

state_msg = ModelState()

###### Functions ########
def open_gripper():
	print("Opening Gripper...")
	gripper_group_variable_values[0] = 0.015
	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go(wait=True)
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	rospy.sleep(1)

def close_gripper():
	print("Closing Gripper...")
	gripper_group_variable_values[0] = -0.003
	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go(wait=True)
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	rospy.sleep(1)

def move_home():
	arm_group.set_named_target("home")
	print("Executing Move: Home")
	plan1 = arm_group.plan()
	arm_group.go(wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print(variable.pose)
	rospy.sleep(1)

def move_zero():
	arm_group.set_named_target("zero")
	print("Executing Move: Zero")
	plan1 = arm_group.plan()
	arm_group.go(wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	gripper_group_variable_values[0] = 0.0
	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go(wait=True)
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	print(variable.pose)
	rospy.sleep(1)

def drop():
	arm_group.set_named_target("drop")
	print("Executing Move: drop")
	plan1 = arm_group.plan()
	arm_group.go(wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print(variable.pose)
	rospy.sleep(1)

def pre_pick():
	arm_group.set_named_target("prepick")
	print("Executing Move: prepick")
	plan1 = arm_group.plan()
	arm_group.go(wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print(variable.pose)
	rospy.sleep(1)

def pick():
	arm_group.set_named_target("pick")
	print("Executing Move: pick")
	plan1 = arm_group.plan()
	arm_group.go(wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print(variable.pose)
	rospy.sleep(1)

def lift():
	arm_group.set_named_target("lift")
	print("Executing Move: lift")
	plan1 = arm_group.plan()
	arm_group.go(wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print(variable.pose)
	rospy.sleep(1)

def reset_world():
	time.sleep(5)
	q = quaternion_from_euler(3.14, 0, 0)
	state_msg.pose.orientation.x = q[0]
	state_msg.pose.orientation.y = q[1]
	state_msg.pose.orientation.z = q[2]
	state_msg.pose.orientation.w = q[3]
	rospy.wait_for_service('/gazebo/set_model_state')
	set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	resp = set_state( state_msg )

def cmd_cb(payload):
	#print(payload.product)
	if(payload == "banana"):
		rotation = 0.3
		state_msg.model_name = 'banana'
		state_msg.pose.position.x = 0.28
		state_msg.pose.position.y = 0.075
		state_msg.pose.position.z = 0.826497
	if(payload == "bean"):
		rotation = -0.3
		state_msg.model_name = 'blackbean'
		state_msg.pose.position.x = 0.28
		state_msg.pose.position.y = -0.075
		state_msg.pose.position.z = 0.826497
	if(payload == "bread"):
		rotation = -0.8
		state_msg.model_name = 'bread'
		state_msg.pose.position.x = 0.20
		state_msg.pose.position.y = -0.20
		state_msg.pose.position.z = 0.826491
	if(payload == "blueberries"):
		rotation = 0.8
		state_msg.model_name = 'blueberries'
		state_msg.pose.position.x = 0.20
		state_msg.pose.position.y = 0.20
		state_msg.pose.position.z = 0.826499
	if(payload == "chicken"):
		rotation = -1.3
		state_msg.model_name = 'chicken'
		state_msg.pose.position.x = 0.070
		state_msg.pose.position.y = -0.25
		state_msg.pose.position.z = 0.826491
	if(payload == "papertowel"):
		rotation = 1.3
		state_msg.model_name = 'papertowels'
		state_msg.pose.position.x = 0.070
		state_msg.pose.position.y = 0.25
		state_msg.pose.position.z = 0.826492
	if(payload == "olive"):
		rotation = -1.9
		state_msg.model_name = 'oliveoil'
		state_msg.pose.position.x = -0.075
		state_msg.pose.position.y = -0.26
		state_msg.pose.position.z = 0.826491
	if(payload == "water"):
		rotation = 1.9
		state_msg.model_name = 'water'
		state_msg.pose.position.x = -0.075
		state_msg.pose.position.y = 0.26
		state_msg.pose.position.z = 0.826494
	if rotation < 0:
		direction = -3.14
	else:
		direction = 3.14
	dropName = 'drop'
	dropCoords = [direction,0.3,-0.9,0.4]
	prepickName = 'prepick'
	prepickCoords = [rotation,0.0,1.3,-1.3]
	pickName = 'pick'
	pickCoords = [rotation,0.6,0.5,-1.1]
	liftName = 'lift'
	liftCoords = [rotation,0,0,0]

	arm_group.remember_joint_values(prepickName, prepickCoords)
	arm_group.remember_joint_values(pickName, pickCoords)
	arm_group.remember_joint_values(dropName, dropCoords)
	arm_group.remember_joint_values(liftName, liftCoords)

	###### Main ########
	move_home()
	open_gripper()
	pre_pick()
	pick()
	close_gripper()
	lift()
	#drop()
	open_gripper()
	move_zero()
	reset_world()

###### Setup ########
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_execute_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

#Had probelms with planner failing, Using this planner now. I believe default is OMPL
arm_group.set_planner_id("RRTConnectkConfigDefault")
#Increased available planning time from 5 to 10 seconds
arm_group.set_planning_time(10);

gripper_group_variable_values = gripper_group.get_current_joint_values()

rospy.loginfo('Waiting for Alexa command')
#rospy.Subscriber('/OMPayload', OMPayload, cmd_cb)
cmd_cb("banana")
rospy.spin()
moveit_commander.roscpp_shutdown()
