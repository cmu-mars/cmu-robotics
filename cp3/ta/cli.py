import argparse
import rospy
from gazebo_interface import GazeboInterface
from cp3 import CP3
import math
import os
import sys
import json

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    commands = ['help', 'enable_light', 'enable_headlamp', 'list_obstacles', 
        'place_obstacle', 'remove_obstacle', 'set_pose', 'kinect', 'lidar', 'where', 'go',
        'voltage', 'charging', "place_markers", "set_location", "cover", "kill", "list_lights"]
    parser.add_argument('-c', '--challenge', choices={'cp1','cp2','cp3'}, default='cp3', help='The challenge problem context')
    parser.add_argument('command', choices=commands, help='The command to issue to Gazebo')
    parser.add_argument('carg', nargs='*', help='The arguments for the particular command. Use help command to find out more information')

    el_parser = argparse.ArgumentParser(prog=parser.prog + " enable_light")
    el_parser.add_argument('light_id', nargs='+', help='The light id of the light to enable')
    el_parser.add_argument('enablement', choices=['on', 'off'], help='Whether to turn the lights on or off')

    eh_parser = argparse.ArgumentParser(prog=parser.prog + " enable_headlamp")
    eh_parser.add_argument('enablement', choices=['on', 'off'], help='Whether to turn the headlamp on or off')

    po_parser = argparse.ArgumentParser(prog=parser.prog + " place_obstacle")
    po_parser.add_argument('height', nargs='?', type=str, help="The height of the obstacle (must match one of the models)")
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
    go_parser.add_argument("-d", "--direct", action='store_true', help='Go directly there, rather than using instructions')
    go_parser.add_argument('start', nargs='?', help='The waypoint label of the start')
    go_parser.add_argument('target', help='The wapoint lable of the target')

    sc_parser = argparse.ArgumentParser(prog=parser.prog + " charging")
    sc_parser.add_argument('enablement', choices=['on', 'off'], help='Turn on/off charging')

    sv_parser = argparse.ArgumentParser(prog=parser.prog + " voltage")
    sv_parser.add_argument('voltage', type=int, help='The voltage to set the charge to')

    pm_parser = argparse.ArgumentParser(prog=parser.prog + " place_markers")
    pm_parser.add_argument("marker_file", type=str, help="To file containing JSON definition of markers")

    sl_parser = argparse.ArgumentParser(prog=parser.prog + " set_location")
    sl_parser.add_argument("waypoint", type=str, help="The waypoint to move to")

    co_parser = argparse.ArgumentParser(prog=parser.prog + " cover")
    co_parser.add_argument("start_pair", nargs='?', type=str, help='The comma separated pair of waypoints to start with')
    co_parser.add_argument("waypoint_order", type=str, help='File containing the list of waypoints to visit in order')
    co_parser.add_argument('output', type=str, help='File to print statistics for each leg')

    ki_parser = argparse.ArgumentParser(prog=parser.prog + " kill")
    ki_parser.add_argument("node", choices={'amcl', 'mrpt', 'aruco'}, help='The nodes that can be killed')

    ll_parser = argparse.ArgumentParser(prog=parser.prog + " list_lights")
    ll_parser.add_argument("waypoints", type=str, nargs='+', help='A list of connected waypoints')

    args = parser.parse_args()

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
        elif hargs=="place_markers":
            print('Place markers from a file')
            pm_parser.print_help()
        elif hargs=="set_location":
            print("Sets the location of the turtlebot to the named waypoint")
            sl_parser.print_help()
        elif hargs=="cover":
            print("Visits the map in order of waypoints")
            co_praser.print_help()
        elif hargs=="kill":
            print("Kills the provided node")
            ki_parser.print_help()
        elif hargs=="list_lights":
            print("Lists the lights in the map along a path");
            ll_parser.print_help()
        else:
            h_parser.print_help()
        sys.exit()
    else:
        rospy.init_node("cli")
        gazebo = GazeboInterface(0,0)

    cp = CP3(gazebo)
    if args.challenge == "cp1":
        # do something with cp1
        print("CP1 not currently supported")

    if args.command == 'enable_light':
        eargs = el_parser.parse_args(args.carg)
        if eargs.enablement=='on':
            eargs.enablement = True
        elif eargs.enablement=='off':
            eargs.enablement = False
        else:
            el_parser.print_help()
            sys.exit()
        for id in eargs.light_id:
            result = gazebo.enable_light(id, eargs.enablement)
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
        id = gazebo.place_new_obstacle(pargs.x, pargs.y, pargs.height)
        if id is None:
            print ('Could not place an obstacle')
        else:
            print ('Obstacle "%s" placed in the world.'%id)
    elif args.command == 'remove_obstacle':
        rargs = do_parser.parse_args(args.carg)
        result = gazebo.delete_obstacle(rargs.obstacle_id, False)
        print ('Obstacle was removed %s' %('successfully' if result else 'unsuccessfully'))
    elif args.command == 'set_pose':
        pargs = sp_parser.parse_args(args.carg)
        pargs.w = math.radians(pargs.w)
        result = gazebo.set_turtlebot_position(pargs.x, pargs.y, pargs.w)
        print ('Turtlebot was placed %s' %('successfully' if result else 'unsuccessfully'))
    elif args.command == 'set_location':
        largs = sl_parser.parse_args(args.carg)
        location = cp.map_server.waypoint_to_coords(largs.waypoint)
        result = gazebo.set_turtlebot_position(location["x"], location["y"], 0)
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
        if gargs.direct or gargs.start is None:
            result = cp.go_directly(gargs.target)
            print('%s moved Turtlebot to %s' %(("Successfully" if result else "Unsuccessfully"), gargs.target))
        elif gargs.direct:
            result = cp.go_directly(gargs.start, gargs.target)
            print('%s moved Turtlebot to %s' %(("Successfully" if result else "Unsuccessfully"), gargs.target))
        else:
            result, message = cp.go_instructions(gargs.start, gargs.target, wait=True)
            print('%s moved Turtlebot from %s to %s%s' %(("Successfully" if result else "Unsuccessfully"), 
                        gargs.start, 
                        gargs.target, ((": %s" %reason) if not result else "")))
    elif args.command == 'charging':
        result = gazebo.set_charging(args.enablement is 'on')
        print ("%s set charging" %("Successfully" if result else "Unsuccessfully"))
    elif args.command == 'voltage':
        result = gazebo.set_voltage(args.voltage)
        print ("%s set voltage" %("Successfully" if result else "Unsuccessfully"))
    elif args.command == "place_markers":
        pargs = pm_parser.parse_args(args.carg)
        pargs.marker_file = os.path.expandvars(pargs.marker_file)
        f = open(pargs.marker_file)
        s = f.read()
        markers = json.loads(s)
        result = gazebo.place_markers(markers)
        print ("Created markers")
    elif args.command == "kill":
        kargs = ki_parser.parse_args(args.carg)
        CP3.convert_to_class(cp)
        result, reason = cp.kill_node(kargs.node)
        print ('%s killed %s%s' %(("Successfully" if result else "Unsuccessfully"), kargs.node, ("" if reason is None else (":%s" %reason))))
    elif args.command == "list_lights":
        largs = ll_parser.parse_args(args.carg)
        CP3.convert_to_class(cp)
        result, reason = cp.list_lights_on_path(largs.waypoints)
        print result
    elif args.command == "cover":
        cargs = co_parser.parse_args(args.carg)
        with open(cargs.waypoint_order) as f:
            waypoints = f.readlines()
        waypoints = [x.strip() for x in waypoints]
        waypoints = [x if x.startswith("l") else "l%s" %x for x in waypoints]

        start_wp = -1
        if cargs.start_pair is not None:
            s = cargs.start_pair.split(',')
            w1 = s[0]
            w2 = s[1]

            for i in range(1, len(waypoints)-1):
                if waypoints[i-1] == w1 and waypoints[i] == w2:
                    start_wp = i
            if start_wp == -1:
                print("Could not find a coverage from %s to %s. Exiting." %(w1,w2))
                sys.exit()
        else:
            start_wp = 1
        location = cp.map_server.waypoint_to_coords(waypoints[start_wp-1])
        result = gazebo.set_turtlebot_position(location["x"], location["y"], 0)
        if not result:
            print("Failed to start -- could not move robot to " + waypoints[start_wp-1])
            sys.exit()

        for i in range(start_wp,len(waypoints)-1):
            s = waypoints[i-1]
            t = waypoints[i]
            print ("Going from %s to %s" %(s,t))
            retries = 0
            result = False
            while retries < 3 and not result:
                if retries > 0:
                    location = cp.map_server.waypoint_to_coords(s)
                    result = gazebo.set_turtlebot_position(location["x"], location["y"], 0)
                    if not result:
                        print("Failed to start -- could not move robot to " + s)
                        sys.exit() 
                start = rospy.Time.now()
                result, msg = cp.do_instructions(s, t, True)
                end = rospy.Time.now()
                with open(cargs.output, 'a') as f:
                    f.write("%s,%s,%s,%s\n" %(s,t,result,(end-start).to_sec()))
                retries = retries + 1



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
# [x]    Write and test lidar
# [ ]    Test go
# [ ]    Test charging and voltage
# [x]    Write go with instructions 
