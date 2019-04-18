import os

import rospy
import rosnode
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
from brass_gazebo_plugins.msg import BrassBump
from sensor_msgs.msg import Illuminance
from std_msgs.msg import UInt32MultiArray, Bool, Int8


import psutil

#import tf
import transformations as t
import roslaunch
from gazebo_interface import GazeboInterface
import time
import threading

class BaseSystem:

	def __init__(self, map_server, instruction_server, gazebo, start_configuration): 
#		self.ig = actionlib.SimpleActionClient("ig_action_server", ig_action_msgs.msg.InstructionGraphAction)
#		self.ig.wait_for_server()


		self.map_server = map_server
		self.instruction_server = instruction_server
		self.gazebo = gazebo

		self.move_base = None
		self.ig = None
		self.start_configuration = start_configuration

		

	def go_directly(self, start, target=None):
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
		self.gazebo.set_charging_srv(False)
		self.move_base.send_goal(goal)

		result = self.move_base.wait_for_result()
		state = self.move_base.get_state()
		if result and state == GoalStatus.SUCCEEDED:
			return True
		else:
			return False
	
	def go_instructions(self, start, target, wait, active_cb = None, done_cb = None, sleep_time=None):
		
		if not wait and (active_cb is None or done_cb is None):
			print("Cannot pass wait=False and no callbacks")
			return False



		# Place the robot at the start
		heading = self.instruction_server.get_start_heading(start, target, self.start_configuration)
		if heading == -1:
			print("No information for %s to %s" %(start, target))
			return False

		start_coords = self.map_server.waypoint_to_coords(start)

		self.gazebo.set_turtlebot_position(start_coords["x"], start_coords["y"], heading)
		if sleep_time is not None:
			rospy.sleep(sleep_time)
		return self.do_instructions(start, target, wait, active_cb, done_cb)

	def do_instructions(self, start, target, wait, active_cb=None, done_cb = None, speed=None, single_step=False):
		if self.ig is None:
			self.ig = actionlib.SimpleActionClient("ig_action_server", ig_action_msgs.msg.InstructionGraphAction)
			self.ig.wait_for_server()
		# Check if path is none in this configuration
		path = self.instruction_server.get_path(start, target, self.start_configuration)
		if path is None or len(path) == 0 or (single_step and len(path) > 1):
			igcode = self.instruction_server.get_instructions(start, target, 'favor-timeliness-amcl-kinect')
		else:
			igcode = self.instruction_server.get_instructions(start, target, self.start_configuration)
		if speed is not None:
			igcode = igcode.replace("0.35,", "%s," %speed)
		print("=====> IG for %s to %s for %s is %s" %(start,target, self.start_configuration, igcode))
		goal = ig_action_msgs.msg.InstructionGraphGoal(order=igcode)
		self.gazebo.set_charging_srv(False)
		if not wait:
			self.ig.send_goal(goal = goal, done_cb=done_cb, active_cb=active_cb)
			return True, "Sent goal"
		else:
			self.ig.send_goal(goal)
			result = self.ig.wait_for_result(timeout=rospy.Duration(60*15))
			state = self.ig.get_state()
			if result and state == GoalStatus.SUCCEEDED:
				return True, None
			else:
				if (self.ig.get_result() is not None):
					return False, self.ig.get_result().sequence
				else:
					return False, "IG Failed perhaps because of timeout"

	def execute_instructions(self, igcode, wait=True, active_cb=None, done_cb=None, discharge=True):
		if self.ig is None:
			self.ig = actionlib.SimpleActionClient("ig_action_server", ig_action_msgs.msg.InstructionGraphAction)
			self.ig.wait_for_server()

		goal = ig_action_msgs.msg.InstructionGraphGoal(order=igcode)
		self.gazebo.set_charge_srv(180000)
		if discharge: 
			self.gazebo.set_charging_srv(False)
		if not wait:
			self.ig.send_goal(goal = goal, done_cb=done_cb, active_cb=active_cb)
		else:
			self.ig.send_goal(goal)
			result = self.ig.wait_for_result(timeout=rospy.Duration(15*60))
			state = self.ig.get_state()
			if result and self.ig.get_result().succeeded and state == GoalStatus.SUCCEEDED:
				return True, None
			else:
				if self.ig.get_result() is not None:
					return False, self.ig.get_result().sequence
				else:
					return False, "IG Failed perhaps because of a timeout"

	def wait_for_odom(self, timeout):
#		listener=tf.TransformListener()
		try:
#			listener.waitForTransform("gazebo_world", "odom", rospy.Time(), rospy.Duration(timeout));
			print("cp.wait_for_odom doesn't work because this is python3 and ROS is build with python2")
			return True
		except:
			return False

class ConverterMixin:
	@classmethod
	def convert_to_class(cls, obj):
		obj.__class__ = cls;




class CP3(ConverterMixin,BaseSystem):

	DEFAULT_MAP_FILE = os.path.expanduser("~/catkin_ws/src/cp3_base/maps/cp3.json")
	DEFAULT_INSTRUCTION_FULE = os.path.expanduser("~/catkin_ws/src/cp3_base/instructions/instructions-all.json")

	NODE_MAP = {"aruco" : ["aruco_marker_publisher_front", "aruco_marker_publisher_back", "marker_manager", "marker_pose_publisher"],
				"amcl" : ["amcl"],
				"mrpt" : ["mrpt_localization_node"]}

	launch_configs = {'aruco' : ['cp3-kinect.launch','cp3-aruco.launch'],
             'amcl-kinect' : ['cp3-kinect.launch','cp3-amcl.launch'],
             'amcl-lidar' : ['cp3-lidar.launch', 'cp3-amcl.launch'],
             'mrpt-kinect' : ['cp3-kinect.launch','cp3-mrpt.launch'],
             'mrpt-lidar' : ['cp3-lidar.launch', 'cp3-mrpt.launch']}

	def __init__(self, gazebo, start_configuration=None):
		BaseSystem.__init__(self, MapServer(self.DEFAULT_MAP_FILE), InstructionDB(self.DEFAULT_INSTRUCTION_FULE), gazebo, start_configuration if not None else 'amcl-kinect')
		def default_bump_callback(bumped, velocity, time):
			print("detected bump before start. This should be anomalous")
		self.bump_callback = default_bump_callback
		
		self.bump_subscriber = rospy.Subscriber("/mobile_base/events/brass_bump", BrassBump, self.bumped)
		self.track_aruco = False
		self.track_illuminance = False
		self.marker_subscriber = None
		self.illuminance_subscriber = None
		self.was_bumped = False
		self.bump_speed = -1;

	def init(self):
		off = self.map_server.lights_off()
		if self.gazebo is None:
			return
		for l in off:
			self.gazebo.enable_light(l,False)

	def track_config(self, callback):
		self.report_config = False
		self.sensors = set()
		self.nodes = set()
		self.config_callback = callback
		self.kinect_subscriber = rospy.Subscriber("/mobile_base/kinect/status", Int8, self.update_kinect)
		self.lidar_subscriber = rospy.Subscriber("/mobile_base/lidar/status", Bool, self.update_lidar)
		self.node_thread = threading.Thread(target=self.update_config)
		self.lock = threading.Lock()
		self.node_thread.start()

	def update_kinect(self, mode):
		with self.lock:
			sensors = set(self.sensors)

		to_remove = set()
		to_add = set()
		if mode.data == 2:
			to_remove.add("kinect")
			to_add.add("camera")
		elif mode.data == 1:
			to_remove.add("camera")
			to_add.add("kinect")
		else:
			to_remove.add("kinect")
			to_remove.add("camera")
		sensors = sensors.difference(to_remove)
		sensors = sensors.union(to_add)
		with self.lock:
			if sensors != self.sensors:
				self.sensors = self.sensors.difference(to_remove).union(to_add)
				self.report_config = True


	def update_lidar(self, on):
		with self.lock:
			if on.data and not "lidar" in self.sensors:
				self.sensors.add("lidar")
				self.report_config = True
			elif not on.data and "lidar" in self.sensors:
				self.sensors.remove("lidar")
				self.report_config = True


	def update_config(self):
		while not rospy.is_shutdown():
			n = rosnode.get_node_names()
			n = [x[1:] for x in n]

			has_amcl = "amcl" in n
			has_mrpt = "mrpt_localization_node" in n
			has_aruco = set(self.NODE_MAP["aruco"]).issubset(n)
			rc = False
			if "amcl" in self.nodes and not has_amcl:
				self.nodes.remove("amcl")
				rc = True
			elif has_amcl and not "amcl" in self.nodes:
				self.nodes.add("amcl")
				rc = True
			if "mrpt" in self.nodes and not has_mrpt:
				self.nodes.remove("mrpt")
				rc = True
			elif has_mrpt and not "mrpt" in self.nodes:
				self.nodes.add("mrpt")
				rc = True
			if "aruco" in self.nodes and not has_aruco:
				self.nodes.remove("aruco")
				rc = True
			elif has_aruco and not "aruco" in self.nodes:
				self.nodes.add("aruco")
				rc = True
			if self.config_callback is not None:
				with self.lock:
					self.report_config = False
					n = self.nodes
					s = self.sensors
				self.config_callback(s, n)
			rospy.sleep(1)


	def track_bumps(self, cb):
		self.was_bumped = False
		self.bump_callback = cb

	def did_bump(self):
		return self.was_bumped;

	def reset_bumps(self):
		self.was_bumped = False

	def bumped(self, be):
		if be.state == BrassBump.PRESSED:
			self.was_bumped = True
			self.bump_callback(True, be.velocity, rospy.Time.now())

	def bumpspeed(self, odom):
		if odom.twist.twist.linear.x > self.bump_speed:
			self.bump_speed = odom.twist.twist.linear.x

	def track(self, track_aruco, track_illuminance):
		self.track_aruco = track_aruco
		self.track_illuminance = track_illuminance

	def start_aruco_tracking(self):
		self.min_illuminance = 100000
		self.max_illuminance = -10000
		self.lost_marker = False
		self.marker_last_seen = rospy.Time.now()
		if self.track_aruco:
			self.marker_subscriber = rospy.Subscriber("aruco_marker_publisher_front/markers_list", UInt32MultiArray, self.report_marker)
		if self.track_illuminance:
			self.illuminance_subscriber = rospy.Subscriber("mobile_base/sensors/light_sensor", Illuminance, self.report_illuminance)

	def report_illuminance(self, i):
		if (i.illuminance < self.min_illuminance):
			self.min_illuminance = i.illuminance
		if (i.illuminance > self.max_illuminance):
			self.max_illuminance = i.illuminance

	def report_marker(self, marker_list):
		if len(marker_list.data) > 0:
			now = rospy.Time.now()
			if (now-self.marker_last_seen).to_sec() > 1.5:
				self.lost_marker = True
				print('Lost the marker')

	def teardown_aruco(self):
		if self.marker_subscriber is not None:
			self.marker_subscriber.unregister()
		if self.illuminance_subscriber is not None:
			self.illuminance_subscriber.unregister()
		

	def do_instructions(self, start, target, wait, active_cb = None, done_cb = None, speed=None, single_step=False):

		if wait and (self.track_aruco or self.track_illuminance):
			self.start_aruco_tracking()
		ret = None
		try:
			ret = BaseSystem.do_instructions(self, start, target, wait, active_cb, done_cb, speed=speed, single_step=single_step)
		except:
			ret = False, "Some weird exception";
		if wait and (self.track_aruco or self.track_illuminance):
			self.teardown_aruco()

		return ret

	def kill_node(self,node,specific_node=None):
		if node not in ['amcl', 'mrpt', 'aruco']:
			return False, "Illegal node passed in for killing."

		nodes = self.NODE_MAP[node]

		if nodes is None or len(nodes) == 0:
			return False, "Nothing to kill"

		if specific_node is None:
			rosnode.kill_nodes(nodes)
			return True, None
		elif specific_node in nodes:
			rosnode.kill_nodes(specific_node)
			return True, None
		else:
			return False, "Could not kill %s" %specific_node


	def get_lights_between_waypoints(self, wp1, wp2):
		return self.map_server.lights_between_waypoints(wp1, wp2);

	def list_lights_on_path(self, waypoints):
		lights = []
		for i in range(len(waypoints)-1):
			l = self.get_lights_between_waypoints(waypoints[i], waypoints[i+1])
			l = [light for light in l if not light in lights]
			lights.extend(l)
		return lights, None

	def launch(self, config, init_node='cp3', start_base=True):
		return self.launch_in_parts(config, init_node, start_base)

	def launch_in_parts(self, config, init_node='cp3', start_base=True, additional=[]):

		launch_files = []
		if start_base:
			launch_files.append("cp3-base.launch")

		launch_files.extend(self.launch_configs[config])
		launch_files.extend(additional)

		launches = []
		for l in launch_files:
			uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
			roslaunch.configure_logging(uuid)
			launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.expanduser("~/catkin_ws/src/cp3_base/launch/%s" %l)])
			launch.start ()
			launches.append(launch)

		if init_node is not None:
			rospy.init_node(init_node)

		self.gazebo = GazeboInterface(0,0)
		time.sleep(5)

		gazebo=False
		for proc in psutil.process_iter():
			if proc.name() == "gzserver":
				gazebo=True

		return launches, gazebo

	def stop(self, launch):
		launches = []
		if isinstance(launch, list):
			launches.extend(launch)
		else:
			launches.append(launch)

		for l in launches:
			l.shutdown()

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
