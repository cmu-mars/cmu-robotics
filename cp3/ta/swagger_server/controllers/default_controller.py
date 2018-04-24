import connexion
from swagger_server.models.inline_response200 import InlineResponse200
from swagger_server.models.inline_response2001 import InlineResponse2001
from swagger_server.models.inline_response2002 import InlineResponse2002
from swagger_server.models.inline_response2003 import InlineResponse2003
from swagger_server.models.inline_response400 import InlineResponse400
from swagger_server.models.inline_response4001 import InlineResponse4001
from swagger_server.models.inline_response4002 import InlineResponse4002
from swagger_server.models.parameters0 import Parameters0
from swagger_server.models.parameters1 import Parameters1
from swagger_server.models.parameters2 import Parameters2
from datetime import date, datetime
from typing import List, Dict
from six import iteritems
from ..util import deserialize_date, deserialize_datetime

import rospy
import swagger_server.config

def observe_get():
    """
    observe_get
    observe some of the current state of the robot for visualization and invariant checking for perturbation end points. n.b. this information is to be used strictly in a passive way; it is not to be used for evaluation of the test at all.

    :rtype: InlineResponse2003
    """
    x , y = config.cp.gazebo.get_turtlebot_state()

    if x == None or y == None:
        return {} , 400 ## todo: api doesn't specify any payload here

    ret = InlineResponse2003()
    ret.x = x
    ret.y = y
    ret.battery = config.battery
    ret.sim_time = rospy.Time.now().secs
    ret.lights = config.cp.map_server.lights_on()
    return ret

def perturb_light_post(Parameters):
    """
    perturb_light_post
    change the state of a light in the environment
    :param Parameters:
    :type Parameters: dict | bytes

    :rtype: InlineResponse200
    """
    if connexion.request.is_json:
        Parameters = Parameters0.from_dict(connexion.request.get_json())

    ## todo: check if the lights are in LIGHTSET

    if config.cp.gazebo.enable_light(Parameters.id, Parameters.state):
        ret = InlineResponse200()
        ret.sim_time = rospy.Time.now().secs
        return ret
    else:
        ret = InlineResponse4001(rospy.Time.now().secs, "light setting failed on: %s" % Parameters)
        return ret , 400

def perturb_nodefail_post(Parameters):
    """
    perturb_nodefail_post
    cause one of the software nodes to fail
    :param Parameters:
    :type Parameters: dict | bytes

    :rtype: InlineResponse2002
    """
    if connexion.request.is_json:
        Parameters = Parameters2.from_dict(connexion.request.get_json())

    retval , message = config.cp.kill_node(Parameters.id)

    if retval:
        ret = InlineResponse2002()
        ret.sim_time = rospy.Time.now().secs
        return ret
    else:
        ret = InlineResponse4001()
        ret.sim_time = rospy.Time.now().secs
        ret.message = message
        return ret , 400

def perturb_sensor_post(Parameters):
    """
    perturb_sensor_post
    change the state of one of the sensors on the robot
    :param Parameters:
    :type Parameters: dict | bytes

    :rtype: InlineResponse2001
    """
    if connexion.request.is_json:
        Parameters = Parameters1.from_dict(connexion.request.get_json())

    res = None
    if Parameters.id == "kinect" and Parameters.state:
        res = config.cp.gazebo.set_kinect_mode('on')
    elif Parameters.id == "kinect" and not Parameters.state:
        res = config.cp.gazebo.set_kinect_mode('off')

    elif Parameters.id == "lidar" and Parameters.state:
        res = config.cp.gazebo.set_lidar_mode('on')
    elif Parameters.id == "lidar" and not Parameters.state:
        res = config.cp.gazebo.set_lidar_mode('off')

    elif Parameters.id == "camera" and Parameters.state:
        res = config.cp.gazebo.set_kinect_mode('image-only')
    elif Parameters.id == "camera" and not Parameters.state:
        res = config.cp.gazebo.set_kinect_mode('off')

    if res:
        ret = InlineResponse2001()
        ret.sim_time = rospy.Time.now().secs
        return ret
    else:
        ret = InlineResponse4001()
        ret.sim_time = rospy.Time.now.secs()
        ret.message = ("failed to set sensor state: %s" % Parameters)
        return ret , 400

def start_post():
    """
    start_post
    start the turtlebot navigating through the map

    :rtype: None
    """
    if not config.started:
        def active_cb():
            """ callback for when the bot is made active """
            config.logger.debug("received notification that goal is active")

        def done_cb(terminal, result):
            """ callback for when the bot is at the target """
            if 'successfully' in result.sequence:
                config.logger.debug("received notification that the goal has been completed!")
                ## todo: this is one thing that should trigger posting
                ## to /done with a bunch of stuff i don't have yet

        result , msg = config.cp.do_instructions(cp.start, cp.target,
                                                 False, active_cb, done_cb)
        config.started = True
        return {} , 200
    else:
        ret = InlineResponse400("/start called more than once")
        return ret , 400
