import os

import rospy
import actionlib
#import ig_action_msgs.msg
import move_base_msgs.msg
from std_msgs.msg import (Bool, String)
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ig_action_msgs.msg import InstructionGraphResult
import ig_action_msgs.msg
from actionlib_msgs.msg import *
from map_server import MapServer
from instruction_db import InstructionDB
from kobuki_msgs.msg import BumperEvent

import psutil

import transformations as t
import roslaunch
from gazebo_interface import GazeboInterface
import time

class BaseSystem:

	def __init__(self, map_server, instruction_server, gazebo): 
#		self.ig = actionlib.SimpleActionClient("ig_action_server", ig_action_msgs.msg.InstructionGraphAction)
#		self.ig.wait_for_server()


		self.map_server = map_server
		self.instruction_server = instruction_server
		self.gazebo = gazebo

		self.move_base = None
		self.ig = None

		

	def go_directly(start, target=None):
		if self.move_base is None:
			self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
			self.move_base.wait_for_server()
		if target is None:

			target = start
			start = None
		else:
			start_coords = self.map_server.waypoint_to_coords(start)
			self.gazebo.set_turtlebot_position(start_coords["x"], start_coords["y"], 0)

		target_coords = self.map_server.waypoint_to_coords(target)
		goal = MoveBaseAction()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		quarternion = t.quarternion_from_euler(0,0,0)
		goal.target_pose.pose = Pose(Point(target_coords.x, target_coords.y, 0),
			Quaternion(quarternion[0], quarternion[1], quarternion[2], quarternion[4]))
		self.move_base.send_goal(goal)

		result = self.move_base.wait_for_result()
		state = self.move_base.get_state()
		if result and state == GoalStatus.SUCCEEDED:
			return True
		else:
			return False
	
	def go_instructions(self, start, target, wait, active_cb = None, done_cb = None):
		
		if not wait and (active_cb is None or done_cb is None):
			print("Cannot pass wait=False and no callbacks")
			return False



		# Place the robot at the start
		heading = self.instruction_server.get_start_heading(start, target)
		if heading == -1:
			print("No information for %s to %s" %(start, target))
			return False

		start_coords = self.map_server.waypoint_to_coords(start)

		self.gazebo.set_turtlebot_position(start_coords["x"], start_coords["y"], heading)
		return self.do_instructions(start, target, wait, active_cb, done_cb)

	def do_instructions(self, start, target, wait, active_cb=None, done_cb = None):
		if self.ig is None:
			self.ig = actionlib.SimpleActionClient("ig_action_server", ig_action_msgs.msg.InstructionGraphAction)
			self.ig.wait_for_server()
		igcode = self.instruction_server.get_instructions(start, target)
		goal = ig_action_msgs.msg.InstructionGraphGoal(order=igcode)
		if not wait:
			self.ig.send_goal(goal = goal, done_cb=done_cb, active_cb=active_cb)
		else:
			self.ig.send_goal(goal)
			result = self.ig.wait_for_result()
			state = self.ig.get_state()
			if self.ig.get_result().succeeded and state == GoalStatus.SUCCEEDED:
				return True, None
			else:
				return False, self.ig.get_result().sequence


class ConverterMixin:
	@classmethod
	def convert_to_class(cls, obj):
		obj.__class__ = cls;




class CP3(ConverterMixin,BaseSystem):

	DEFAULT_MAP_FILE = os.path.expanduser("~/catkin_ws/src/cp3_base/maps/cp3.json")
	DEFAULT_INSTRUCTION_FULE = os.path.expanduser("~/catkin_ws/src/cp3_base/instructions/instructions-all.json")

	NODE_MAP = {"aruco" : ["marker_publisher", "marker_manager.py", "marker_pose_publisher.py"],
				"amcl" : ["amcl"],
				"mrpt" : ["mrpt_localization_node"]}

	launch_configs = {'aruco' : 'cp3-aruco-kinect.launch',
             'amcl-kinect' : 'cp3-amcl-kinect.launch',
             'amcl-lidar' : 'cp3-amcl-lidar.launch',
             'mrpt-kinect' : 'cp3-mrpt-kinect.launch',
             'mrpt-lidar' : 'cp3-mrpt-lidar.launch'}

	def __init__(self, gazebo):
		BaseSystem.__init__(self, MapServer(self.DEFAULT_MAP_FILE), InstructionDB(self.DEFAULT_INSTRUCTION_FULE), gazebo)
		self.bump_subscriber = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumped)

	def track_bumps(self):
		self.was_bumped = False
		self.bump_subscriber = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumped)

	def did_bump(self):
		return self.was_bumped;

	def reset_bumps(self):
		self.was_bumped = False

	def bumped(self, be):
		if be.state == BumperEvent.PRESSED:
			self.was_bumped = True
			print("Bumped")

	def kill_node(self,node):
		if node not in ['amcl', 'mrpt', 'aruco']:
			return False, "Illegal node passed in for killing."

		nodes = self.NODE_MAP[node]

		if nodes is None or len(nodes) == 0:
			return False, "Nothing to kill"

		killed = False

		for proc in psutil.process_iter():
			if proc.name() in nodes:
				proc.kill()
				killed = True
			elif proc.name() == "python":
				pynodes = [x for x in nodes if x.endswith("py")]
				for py in pynodes:
					if len(proc.cmdline()) > 1 and proc.cmdline()[1].endswith(py):
						proc.kill()
						killed = True


		return killed, "Could not find a process to kill" if not killed else None

	def get_lights_between_waypoints(self, wp1, wp2):
		return self.map_server.lights_between_waypoints(wp1, wp2);

	def list_lights_on_path(self, waypoints):
		lights = []
		for i in range(len(waypoints)-1):
			l = self.get_lights_between_waypoints(waypoints[i], waypoints[i+1])
			l = [light for light in l if not light in lights]
			lights.extend(l)
		return lights, None

	def launch(self, config, init_node='cp3'):

		launch_file = self.launch_configs[config]
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.expanduser("~/catkin_ws/src/cp3_base/launch/%s" %launch_file)])
		launch.start ()

		if init_node is not None:
			rospy.init_node(init_node)

		self.gazebo = GazeboInterface(0,0)
		time.sleep(10)

		# Check to see if Gazebo came up
		gazebo = False
		for proc in psutil.process_iter():
			if proc.name() == "gzserver":
				gazebo = True

		return launch, gazebo

	def stop(self, launch):
		launch.shutdown()

		# Gotta kill gazebo really
		for proc in psutil.process_iter():
			if proc.name() == "gzserver":
				proc.kill()
		self.move_base = None
		self.ig = None
		self.gazebo = None

if (__name__ == "__main__"):
	for proc in psutil.process_iter():
		print("%s %s" %(proc.name(),proc.cmdline()))