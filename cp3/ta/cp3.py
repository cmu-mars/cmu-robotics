import rospy
import actionlib
#import ig_action_msgs.msg
import move_base_msgs.msg
from std_msgs.msg import (Bool, String)
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import transformations as t

class BaseSystem:

	def __init__(self, map_server): 
#		self.ig = actionlib.SimpleActionClient("ig_action_server", ig_action_msgs.msg.InstructionGraphAction)
#		self.ig.wait_for_server()


		self.map_server = map_server

		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.move_base.wait_for_server()

	def go(start, target=None):
		if target is None:
			target = start
			start = None

			target_coords = self.map_server.waypoint_to_coords(target)
			goal = MoveBaseAction()
			goal.target_pose.header.frame_id = 'map'
			goal.target_pose.header.stamp = rospy.Time.now()
			quarternion = t.quarternion_from_euler(0,0,0)
			goal.target_pose.pose = Pose(Point(target_coords.x, target_coords.y, 0),
				Quaternion(quarternion[0], quarternion[1], quarternion[2], quarternion[4]))
			self.move_base.send_goal(goal)

			result = self.move_base.wait_for_result()
			state = move_base.get_state()
			if result and state == GoalStatus.SUCCEEDED:
				return True
			else:
				return False


class CP3(BaseSystem):

	def __init__(self, map_server):
		super().__init__(map_server)