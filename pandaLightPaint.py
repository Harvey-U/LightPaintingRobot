#Lighting paint frames with Franka Panda Robot - Main Execution Code
#See documentation and video for detailed executaion instructions

#Import the required modules
#Ensure these are all part of your rospy installation

#!/usr/bin/env python
#ROS specific
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension, Float64
from moveit_commander.conversions import pose_to_list

#Generic python 2
from math import pi
import tf

#Import python frames file from same folder as this code
#Format for this file can be seen in the example butterfly provided or the project documentation
from frames import framelist



#The X,Y,Z coordinates of the centre of the drawing space (M)
oOffCords = [0.5,0,0.5]

#The pitch, roll yaw andgles of the end effector when drawing (radians)
oOffRots = [pi,0,-pi/4]

#The angle of shake
shakeAngle = pi/8

#The frame scale factor, all coordinates , wrt to local origin, will be multiplied by this value
#Change if frame is being drawn too large or small
frameScale = 0.004

#Set the time required for the camera to process the frame
processingTime = 20

#End effector gripper position presets
lightOnDist = 0.03
lightOffDist = 0.05


#This function takes angle and coordiante arguments and will move the end effector to that position and orientation
#Roll is around x axis, pitch is around y axis and yw is around z axis should be supplied in terms of pi
#X, Y and Z are the cartesian coordinates in metres of the end effector, where the x axis faces our from the front of the robot
def goToPose(roll,pitch,yaw,xCord,yCord,zCord):
	group.clear_pose_targets()

	quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
	origin_pose = geometry_msgs.msg.Pose()
	origin_pose.orientation.x = quaternion[0]
	origin_pose.orientation.y = quaternion[1]
	origin_pose.orientation.z = quaternion[2]
	origin_pose.orientation.w = quaternion[3]
	origin_pose.position.x = xCord
	origin_pose.position.y = yCord
	origin_pose.position.z = zCord

	group.set_pose_target(origin_pose)
	plan = group.go(wait=True)


#This function shakes the panda hand by the required number of times in quantity
def shakeHand(quantity):
	quantity = int(quantity)

	#Check if the quanitty value is valid, if not print error message
	if (quantity >=1):
		for i in range (quantity):
			goToPose(oOffRots[0],oOffRots[1],oOffRots[2]-shakeAngle,oOffCords[0],oOffCords[1],oOffCords[2])
			goToPose(oOffRots[0],oOffRots[1],oOffRots[2]+shakeAngle,oOffCords[0],oOffCords[1],oOffCords[2])
			goToPose(oOffRots[0],oOffRots[1],oOffRots[2],oOffCords[0],oOffCords[1],oOffCords[2])
	else:
		print("Shake not executed. Shake quantity value needs to be a positive integer")


#This function takes a frame of coordinates and executes the drawing of the frame
def goOnPath(frame):
	#Clear previous pose targets
	group.clear_pose_targets()

	#Create an empty list to store the frame coordinates
	drawpoints = []

	for coordinate in frame:
		#Calculate the quaternion of the end effector facing forward
		quaternion = tf.transformations.quaternion_from_euler(oOffRots[0],oOffRots[1],oOffRots[2])

		#Initialise the drawpose message 
		drawpose = geometry_msgs.msg.Pose()

		#update the drawpose message base upon the required rotation and coordinate location of the end effector
		drawpose.orientation.x = quaternion[0]
		drawpose.orientation.y = quaternion[1]
		drawpose.orientation.z = quaternion[2]
		drawpose.orientation.w = quaternion[3]
		drawpose.position.x = oOffCords[0] + coordinate[0] *frameScale
		drawpose.position.y = oOffCords[1] + coordinate[1] *frameScale
		drawpose.position.z = oOffCords[2] + coordinate[2] *frameScale

		#Add the drawpose message to the list of drawpoints
		drawpoints.append(copy.deepcopy(drawpose))

	#Calculate motion path base on drawpoints list
	(plan, fraction) = group.compute_cartesian_path(drawpoints, 0.01, 0.0)

	#Execute the calculated motion plan and wait until complete
	group.execute(plan, wait=True) 

#Turn light on with gripper
def lightOn():
	gripper_msg.data = [lightOnDist, lightOnDist]
	gripper_publisher.publish(gripper_msg)	

#Turn light off with gripper
def lightOff():
	gripper_msg.data = [lightOffDist, lightOffDist]
	gripper_publisher.publish(gripper_msg)


#Main code exeution by Panda
if __name__=="__main__":

	#Create rospy node
	rospy.init_node('panda_light_paint_controller', anonymous=True)

	#Initialise motion planning in RVIZ
	moveit_commander.roscpp_initialize(sys.argv)	
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group_name = "panda_arm"
	group = moveit_commander.MoveGroupCommander(group_name)

	#Set up gripper publisher
	gripper_publisher = rospy.Publisher('/franka/gripper_position_controller/command', Float64MultiArray, queue_size=1)
	gripper_msg = Float64MultiArray() #
	gripper_msg.layout.dim = [MultiArrayDimension('', 2, 1)]

    

	#Execute goToPose function to send to starting position
	goToPose(oOffRots[0],oOffRots[1],oOffRots[2],oOffCords[0],oOffCords[1],oOffCords[2])


	#Iterate through each frame in the framlist of the whole animation
	for frame in framelist:

		#User inform message to start photo
		print("Open Shutter")

		#Turn on the light by closing the end effector
		lightOn()

		#Wait 1 second
		rospy.sleep(1)

		#Execute frame motion
		goOnPath(frame)

		#Turn off the light by opening the end effector
		lightOff()

		#User inform message to end photo
		print("Close Shutter")

		#Wait time so that camera has enough time to process image before next frame
		rospy.sleep(processingTime)
