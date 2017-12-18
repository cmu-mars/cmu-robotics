import argparse
import rospy
from gazebo_interface import GazeboInterface
from cp3 import CP3
from map_server import MapServer
import math
import os
import sys

DEFAULT_MAP_FILE = os.path.expandvars("~/catkin_ws/src/cp_maps/maps/cp3.json")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    commands = ['help', 'enable_light', 'enable_headlamp', 'list_obstacles', 
        'place_obstacle', 'remove_obstacle', 'set_pose', 'kinect', 'lidar', 'where', 'go',
        'voltage', 'charging']
    parser.add_argument('-c', '--challenge', choices={'cp1','cp2','cp3'}, default='cp3', help='The challenge problem context')
    parser.add_argument('command', choices=commands, help='The command to issue to Gazebo')
    parser.add_argument('carg', nargs='*', help='The arguments for the particular command. Use help command to find out more information')

    el_parser = argparse.ArgumentParser(prog=parser.prog + " enable_light")
    el_parser.add_argument('light_id', help='The light id of the light to enable')
    el_parser.add_argument('enablement', choices=['on', 'off'], help='Whether to turn the lights on or off')

    eh_parser = argparse.ArgumentParser(prog=parser.prog + " enable_headlamp")
    eh_parser.add_argument('enablement', choices=['on', 'off'], help='Whether to turn the headlamp on or off')

    po_parser = argparse.ArgumentParser(prog=parser.prog + " place_obstacle")
    po_parser.add_argument('x', type=float, help='The x location relative to the map to place the obstacle')
    po_parser.add_argument('y', type=float, help='The y location relative to the map to place the obstacle')

    do_parser = argparse.ArgumentParser(prog=parser.prog + " remove_obstacle")
    do_parser.add_argument('obstacle_id', help='The id of the obstacle to remove')

    sp_parser = argparse.ArgumentParser(prog=parser.prog + " set_pose")
    sp_parser.add_argument('x', type=float, help='The x location relative to the map to place the robot')
    sp_parser.add_argument('y', type=float, help='The y location relative to the map to place the robot')
    sp_parser.add_argument('w', type=int, help='The rotation (in degrees) that the robot is facing')

    k_parser = argparse.ArgumentParser(prog=parser.prog + "kinect");
    k_parser.add_argument('enablement', choices=['on', 'off', 'image-only'], help='Turn the kinect on or off, or make it only emit rgb image')

    l_parser = argparse.ArgumentParser(prog=parser.prog + " lidar");
    l_parser.add_argument('enablement', choices=['on', 'off'], help='Turn the lidar on or off')

    h_parser = argparse.ArgumentParser ()
    h_parser.add_argument('command', choices = commands[1:], help='The commands where help is available')

    go_parser = argparse.ArgumentParser(prog=parser.prog + " go")
    go_parser.add_argument('start', nargs='?', help='The waypoint label of the start')
    go_parser.add_argument('target', help='The wapoint lable of the target')

    sc_parser = argparse.ArgumentParser(prog=parser.prog + " charging")
    sc_parser.add_argument('enablement', choices=['on', 'off'], help='Turn on/off charging')

    sv_parser = argparse.ArgumentParser(prog=parser.prog + " voltage")
    sv_parser.add_argument('voltage', type=int, help='The voltage to set the charge to')

    args = parser.parse_args()

    map_file = DEFAULT_MAP_FILE
    if args.challenge == 'cp2':
        map_file = os.path.expandvars("~/catkin_ws/src/cp_maps/maps/cp2.json")
    elif args.challenge == 'cp1':
        map_file = os.path.expandvars("~/catkin_ws/src/cp_maps/maps/cp1.json")


    if args.command == 'help':
        hargs = h_parser.parse_args(args.carg)
        if hargs.command == 'enable_light':
            el_parser.print_help()
        elif hargs.command ==  'enable_headlamp':
            print('Turns the headlamp on the TurtleBot on or off')
            eh_parser.print_help()
        elif hargs.command == 'list_obstacles':
            print('Takes no arguments; returns a list of known obstacle ids')
        elif hargs.command == 'place_obstacle':
            print('Places an obstacle in the world')
            po_parser.print_help()
        elif hargs.command == 'remove_obstacle':
            print('Removes an obstacle from the world')
            do_parser.print_help()
        elif hargs.command == 'set_pose':
            print('Sets the pose of the TurtleBot in the world (and optionally the localization)')
            sp_parser.print_help()
        elif hargs.command == 'kinect':
            print('Configures the kinect sensor')
            k_parser.print_help()
        elif hargs.command == 'lidar':
            print('Configures the LIDAR')
            l_parser.print_help()
        elif hargs == 'where':
            print ('Prints where the turtlebot is and its velocity')
        elif hargs == 'go':
            print ('Execute a path. If start is unspecified, then use default navigation')
            go_parser.print_help()
        elif hargs == 'charging':
            print ('Set whether the turtlebot is charging')
            sc_parser.print_help()
        else:
            h_parser.print_help()
        sys.exit()
    else:
        rospy.init_node("cli")
        gazebo = GazeboInterface(0,0)

    if args.command == 'enable_light':
        eargs = el_parser.parse_args(args.carg)
        if eargs.enablement=='on':
            eargs.enablement = True
        elif eargs.enablement=='off':
            eargs.enablement = False
        else:
            el_parser.print_help()
            sys.exit()
        result = gazebo.enable_light(eargs.light_id, eargs.enablement)
        print ('Light was enabled %s' %('successfully' if result else 'unsuccessfully'))
    elif args.command == 'enable_headlamp':
        eargs = eh_parser.parse_args(args.carg)
        if eargs.enablement=='on':
            eargs.enablement = True
        elif eargs.enablement=='off':
            eargs.enablement = False
        else:
            eh_parser.print_help()
            sys.exit()
        result = gazebo.enable_headlamp(eargs.enablement)
        print ('Headlamp was enabled %s' %('successfully' if result else 'unsuccessfully'))
    elif args.command == 'list_obstacles':
        print(gazebo.obstacle_names)
    elif args.command == 'place_obstacle':
        pargs = po_parser.parse_args(args.carg)
        id = gazebo.place_new_obstacle(pargs.x, pargs.y)
        if id is None:
            print ('Could not place an obstacle')
        else:
            print ('Obstacle "%s" placed in the world.'%id)
    elif args.command == 'remove_obstacle':
        rargs = do_parser.parse_args(args.carg)
        result = gazebo.delete_obstacle(rargs.obstacle_id)
        print ('Obstacle was removed %s' %('successfully' if result else 'unsuccessfully'))
    elif args.command == 'set_pose':
        pargs = sp_parser.parse_args(args.carg)
        pargs.w = math.radians(pargs.w)
        result = gazebo.set_turtlebot_position(pargs.x, pargs.y, pargs.w)
        print ('Turtlebot was placed %s' %('successfully' if result else 'unsuccessfully'))
    elif args.command == 'kinect':
        kargs = k_parser.parse_args(args.carg)
        gazebo.set_kinect_mode(kargs.enablement)
    elif args.command == 'lidar':
        largs = l_parser.parse_args(args.carg)
        gazebo.set_lidar_mode(largs.enablement)
    elif args.command == 'where':
        x, y, w, v = gazebo.get_turtlebot_state()
        print("Turtlebot is at (%s, %s), facing %s and going %s ms" %(str(x),str(y),str(w),str(v)))
    elif args.command == 'go':
        gargs = go_parser.parse_args(args.carg)
        maps = MapServer(map_file)
        cp3 = CP3(maps)
        result = cp3.go(gargs.target)
        print('%s moved Turtlebot to %s' %(("Successfully" if result else "Unsuccessfully"), gargs.target))
    elif args.command == 'charging':
        result = gazebo.set_charging(args.enablement is 'on')
        print ("%s set charging" %("Successfully" if result else "Unsuccessfully"))
    elif args.gommand == 'voltage':
        result = gazebo.set_voltage(args.voltage)
        print ("%s set voltage" %("Successfully" if result else "Unsuccessfully"))


#TODO:
# [x] Test CLI to do what we did with GazeboInterface
# [x] Develop map for cp3 and generate worlds, lights, etc
# [x]    Test map and world
# Verify commands in that context
# [x]    Test enable_light
# [x]    Test enable_headlamp
# [x]    Test place_obstacle (place at l1)
# [x]    Test kinect
# [ ]    Test set_pose
# [ ]    Write and test lidar
# [ ]    Test go
# [ ]    Test charging and voltage
# [ ]    Write go with instructions 
