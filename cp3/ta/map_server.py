""" utility functions for working with waypoints and maps """
### imports
from __future__ import with_statement
import json

class MapServer():

    def __init__(self, map_file):
        with open(map_file) as wf:
            data = json.load(wp)
            self.waypoint_list = data["map"]


    def waypoint_to_coords(waypoint_id):
        """ given a way point, produce its coordinates """
        if not is_waypoint(waypoint_id):
            raise KeyError('The specified waypointID does not exist')
        waypoint_list = get_waypoint(waypoint_id)
        waypoint = waypoint_list[0]
        return waypoint['coord']

    def is_waypoint(waypoint_id):
        """ given a string, determine if it is actually a waypoint id """
        waypoint_list = get_waypoint(waypoint_id)
        if len(waypoint_list) > 1: 
            raise ValueError('non-unique waypoint identifiers in the map file')
        return len(waypoint_list) == 1

    def get_waypoint(waypoint_id):
        return filter(lambda waypoint: waypoint["node-id"] == waypoint_id, self.waypoint_list)