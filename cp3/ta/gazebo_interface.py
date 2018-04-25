from __future__ import with_statement
from __future__ import print_function
from threading import Lock
import os.path
from std_msgs.msg import ColorRGBA
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from geometry_msgs.msg import *
from brass_gazebo_plugins.srv import *
import transformations as tf # Note that this is copied in locally because the standard Kinetic version does not work
import rospy
import math
import traceback
import time

# This is the model for the obstacle
OBS_MODEL = os.path.expanduser('~/catkin_ws/src/cp-models-p15/models/box')

# These are translation coordinates between the map and gazebo
# The two things should really be syncrhonized
X_MAP_TO_GAZEBO_TRANSLATION = 0
Y_MAP_TO_GAZEBO_TRANSLATION = 0

MARKER_TEMPLATE = '''
<sdf version='1.4'>
<model name="Marker$ID">
  <static>1</static>
  <link name="link">
    <visual name="visual">
      <geometry>
        <mesh>
          <uri>model://marker$ID/meshes/Marker$ID.dae</uri>
        </mesh>
      </geometry>
    </visual>
    <self_collide>0</self_collide>
    <kinematic>0</kinematic>
  </link>
</model>
</sdf>'''


class GazeboExcpetion(Exception):
    pass

class GazeboInterface:
    # Class to manage interaction with Gazebo. This class should be instantiated only once.

    obstacle_names = None # Name of known obstacles
    obstacle_sequence = None # The next number in the sequence
    lock = None # A lock to manage multiple threads
    zero_q = None # A quarternion for zero twist

    def __init__(self, xtrans=X_MAP_TO_GAZEBO_TRANSLATION, ytrans=Y_MAP_TO_GAZEBO_TRANSLATION, timeout=30):
        self.obstacle_names = []
        self.obstacle_sequence = 0
        self.lock = Lock () 
        self.zero_q = tf.quaternion_from_euler(0, 0, 0) # Zero twist obstacle
       
        file_xml = open(OBS_MODEL + ".sdf")
        self.obs_xml = file_xml.read()
        self.obs_xmls = {}

        # Services to gazebo
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_light_properties = rospy.ServiceProxy('/gazebo/set_light_properties', SetLightProperties)
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        self.enable_lights = rospy.ServiceProxy('/gazebo/set_light_properties', SetLightProperties)
        self.set_kinect_srv = rospy.ServiceProxy("/mobile_base/kinect/mode", SetKinectMode)
        self.set_lidar_srv = rospy.ServiceProxy("/mobile_base/lidar/mode", SetLidarMode)
        self.set_headlamp_srv = rospy.ServiceProxy("/mobile_base/headlamp", ToggleHeadlamp)


        self.set_charging_srv = rospy.ServiceProxy("/energy_monitor_client/mobile_base/set_charging", SetCharging)
        self.set_charge_srv = rospy.ServiceProxy("/energy_monitor_client/mobile_base/set_charge", SetCharge)

        self.amcl = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10, latch=True)


        self.X_MAP_TO_GAZEBO_TRANSLATION = xtrans
        self.Y_MAP_TO_GAZEBO_TRANSLATION = ytrans

        try:
            rospy.wait_for_service('/gazebo/get_model_state', timeout=timeout)
            #rospy.wait_for_service('/gazebo/spawn_gazebo_model')
        except rospy.ROSException as e:
            raise GazeboExcpetion("Could not connect to gazebo", e)

    def translateMapToGazebo(self,mx, my):
        # Translate from map coordinates to gazebo coordinates
        # :param mx: x coordinate in map
        # :param my: y coordinate in map
        # :return x, y in gazebo
        return mx - self.X_MAP_TO_GAZEBO_TRANSLATION, my - self.Y_MAP_TO_GAZEBO_TRANSLATION

    def translateGazeboToMap(self, gx, gy):
        # Translate from gazebo coordinates to map coordinates
        # :param mx: x coordinate in gazebo
        # :param my: y coordinate in gazebo
        # :return x, y in map
        return gx + self.X_MAP_TO_GAZEBO_TRANSLATION, gy + self.Y_MAP_TO_GAZEBO_TRANSLATION

    def set_turtlebot_position(self, mx, my, w):
        # Sets the position of the turtlebot in gazebo and AMCL
        # :param mx: x ccoordinate of the robot according to the map
        # :param my: y coordinate of the robot according to the map
        # :param w: rotation of the robot
        try:
            # Start by getting the current state of the robot
            tb = self.get_model_state('mobile_base', '')
            # Check to see if robot is moving. If it is, return an error
            #if (tb.twist.linear.x != 0 or tb.twist.linear.y != 0 or
            #   tb.twist.linear.z != 0 or tb.twist.angular.x!= 0 or
            #   tb.twist.angular.y!= 0 or tb.twist.angular.z != 0):
            #   rospy.logerr('Cannot set the robot position while it is moving')
            #   return False

            # Set the position to the new position
            tb.pose.position.x, tb.pose.position.y = self.translateMapToGazebo(mx, my)
            q = (tb.pose.orientation.x,
                tb.pose.orientation.y,
                tb.pose.orientation.z,
                tb.pose.orientation.w) 
            # Set the angular rotation
            eu = tf.euler_from_quaternion(q)
            eu = eu[0], eu[1], w
            q = tf.quaternion_from_euler(eu[0], eu[1], eu[2])
            tb.pose.orientation.x = q[0]
            tb.pose.orientation.y = q[1]
            tb.pose.orientation.z = q[2]
            tb.pose.orientation.w = q[3]

            # Create the new position message
            ms = ModelState()
            ms.model_name='mobile_base'
            ms.pose = tb.pose
            ms.twist = tb.twist
            res = self.set_model_state(ms)
            if (res.success):
                # Tell the map where it is
                ip = PoseWithCovarianceStamped()
                ip.header.stamp = rospy.Time.now()
                ip.header.frame_id = 'map'
                ip.pose.pose.position.x = mx
                ip.pose.pose.position.y = my
                ip.pose.pose.position.z = 0
                ip.pose.pose.orientation.x = tb.pose.orientation.x
                ip.pose.pose.orientation.y = tb.pose.orientation.y
                ip.pose.pose.orientation.z = tb.pose.orientation.z
                ip.pose.pose.orientation.w = tb.pose.orientation.w
                ip.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
                print ('Publishing amcl location')
                self.amcl.publish(ip)
                time.sleep(2)
                return True
            else:
                rospy.logerr("Failed to set gazebo robot position")
                return False
        except rospy.ServiceException as e:
            rospy.logerr ("Could not set the position of the robot")
            rospy.logerr (traceback.format_exc())
            print(traceback.format_exc())
            return False

    def get_turtlebot_state(self):
        # Queries gazebo to get the current state of the turtlebot
        # :return x,y in map coordinates; w as the yaw
        try:
            tb = self.get_model_state('mobile_base', '')
            q = (tb.pose.orientation.x,
                tb.pose.orientation.y,
                tb.pose.orientation.z,
                tb.pose.orientation.w)
            w = tf.euler_from_quaternion(q)[2]
            v = math.sqrt(tb.twist.linear.x**2+ tb.twist.linear.y**2)
            #print ("accelaration is %s, %s, %s" %(tb.twist.linear.x,tb.twist.linear.y,tb.twist.linear.z))
            x, y =  self.translateGazeboToMap (tb.pose.position.x, tb.pose.position.y)
            return x, y, w, v
        except rospy.ServiceException as e:
            rospy.logerr("Could not get state of robot: %s"%e)
            return None, None, None, None

    def place_new_obstacle(self,x,y, height=None):
        # Place a new obstacl at te map coordinates indicated. The obstacles is
        # given an automatic naming that is returned
        # :param x: x coordinated indicating center of obstacle in the map
        # :param y: y coordinate indicating center of obstacle in map
        # :return the name of the obstacle, or None if failed

        # set up position of the obstacle
        gx, gy = self.translateMapToGazebo(x,y)
        pose = Pose ()
        pose.position.x = gx
        pose.position.y = gy
        pose.position.z = 0.0 # constant because obstacle is 1.5 meters tall
        pose.orientation.x = self.zero_q[0]
        pose.orientation.y = self.zero_q[1]
        pose.orientation.z = self.zero_q[2]
        pose.orientation.w = self.zero_q[3]

        # Generate obstacles name (in threadsafe manner)
        obs_name = ''
        with self.lock:
            obs_name = "Obstacle_%s" %str(self.obstacle_sequence)

        # Create gazebo request and call
        req = SpawnModelRequest()
        req.model_name = obs_name
        if height is None:
            req.model_xml = self.obs_xml
        else:
            if not height in self.obs_xmls:
                file_xml = open(OBS_MODEL + height + ".sdf")
                self.obs_xmls[height] = file_xml.read()
            req.model_xml = self.obs_xmls[height]
        req.initial_pose = pose
        try:
            res = self.spawn_model(req)
            if res.success:
                with self.lock:
                    self.obstacle_sequence = self.obstacle_sequence + 1
                    self.obstacle_names.append(obs_name)
                return obs_name
            else:
                rospy.logerr("Could not place obstacle. Message: %s" %res.status_message)
                return None
        except rospy.ServiceException as e:
            rospy.logerr("Could not place obstacle. Message %s" %e)
            return None

    def delete_obstacle(self, obs_name, check=True):
        # Deletes a model (obstacle) from gazebo. The obs_name must exist

        with self.lock:
            if check and obs_name not in self.obstacle_names:
                rospy.logerr("Nonexistent obstacle [%s]"%obs_name)
                return False
        req = DeleteModelRequest()
        req.model_name = obs_name
        try:
            res = self.delete_model(req)

            if res.success:
                if check:
                    with self.lock:
                        self.obstacle_names.remove(obs_name)
                return True
            else:
                rospy.logerr("Could not remove obstacle. Message: %s" %res.status_message)
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Could not removes obstacle. Message %s"%e)
            return False


    def enable_light(self, light, enablement):
        color = ColorRGBA()
        if enablement:
            color.r = 127
            color.g = 127
            color.b = 127
        else:
            color.r = 0
            color.g = 0
            color.b = 0
        color.a = 255
        resp = self.enable_lights(light, color, 0.25, 0.0, 0.0)
        return resp.success



    def enable_headlamp(self, enablement):
        return self.set_headlamp_srv(enablement)


    def set_kinect_mode(self, mode):
        if mode == 'on':
            mode  = 1
        elif mode == 'off':
            mode = 0
        elif mode == 'image-only':
            mode = 2
        return self.set_kinect_srv(mode)

    def set_lidar_mode(self, mode):
        if mode == 'on':
            mode = True
        elif mode == 'off':
            mode = False
        return self.set_lidar_srv(mode)

    def set_charging(self, charging):
        if mode == 'on':
            mode = True
        elif mode == 'off':
            mode = False
        return self.set_charging_srv(charging)


    def set_charge(self, charge):
        return self.set_charging_srv(charge)

    def place_markers(self, markers):
        mid = 0
        for marker in markers:
            if not self.place_marker(mid, marker):
                print("Could not place Marker%s"% mid)
            mid = mid + 1

    def place_marker(self, id, marker):
        x = marker["x"]
        y = marker["y"]
        on_wall = marker["wall"]
        w = 0;
        translate = "translated" not in marker.keys or not marker["translated"]
        if on_wall == "north":
            if translate:
                y = y + 0.1
            w = -math.pi / 2
        elif on_wall == "south":
            if translate:
                y = y - 0.1
            w = math.pi / 2
        elif on_wall == "east":
            if translate:
                x = x - 0.125
            w = math.pi
        elif on_wall == "west":
            if translate:
                x = x + 0.05
            w = 0
        else:
            print("Marker is not on a wall!?")
         # set up position of the obstacle
        gx, gy = self.translateMapToGazebo(x,y)
        pose = Pose ()
        pose.position.x = gx
        pose.position.y = gy
        pose.position.z = 0.75
        
        q = tf.quaternion_from_euler(0, 0, w)
            

        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        # Generate obstacles name (in threadsafe manner)

        # Create gazebo request and call
        req = SpawnModelRequest()
        req.model_name = "Marker%s" %id
        req.model_xml = MARKER_TEMPLATE.replace("$ID", str(id))
        req.initial_pose = pose
        try:
            res = self.spawn_model(req)
            if res.success:
                return True
            else:
                rospy.logerr("Could not place obstacle. Message: %s" %res.status_message)
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Could not place obstacle. Message %s" %e)
            return False

