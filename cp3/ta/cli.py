from __future__ import with_statement 
import argparse
import rospy
from gazebo_interface import GazeboInterface
from track_instructions import InstructionTracker
from cp3 import CP3
import math
import os
import sys
import json
import subprocess
import time
import psutil
import os
import signal
import traceback

def kill_launch(cmd):
    for proc in psutil.process_iter():
        s = [str(item) for item in proc.cmdline()]
        j = " ".join(s)
        if len(proc.cmdline()) > 0 and j.endswith(cmd):
            proc.terminate()
        elif proc.name() == "gzserver":
            proc.terminate()


def distance(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

if __name__ == "__main__":


    parser = argparse.ArgumentParser()
    commands = ['help', 'enable_light', 'enable_headlamp', 'list_obstacles', 
        'place_obstacle', 'remove_obstacle', 'set_pose', 'kinect', 'lidar', 'where', 'go',
        'voltage', 'charging', "place_markers", "set_location", "cover", "kill", "list_lights", "safety_test", 'execute', 'validate_coverage']
    configs = ['amcl-kinect', 'mrpt-kinect', 'amcl-lidar', 'mrpt-lidar', 'aruco']
    parser.add_argument('--challenge', choices={'cp1','cp2','cp3'}, default='cp3', help='The challenge problem context')
    parser.add_argument('command', choices=commands, help='The command to issue to Gazebo')
    #parser.add_argument('carg', nargs='*', help='The arguments for the particular command. Use help command to find out more information')

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
    go_parser.add_argument("-c", "--config", choices=configs, help="The configuration to start")
    go_parser.add_argument("-a", "--aruco", action="store_true", help="Track visual marker visibility")
    go_parser.add_argument('-w', "--write", type=str, help="Where to write information to")
    go_parser.add_argument('-l', '--lights', type=str, help="Comma separated list of lights to turn off")
    go_parser.add_argument('-i', '--illuminance', action='store_true', help="Track illuminance and report max and min")
    go_parser.add_argument('-u', '--launch', action='store_true', help="Attempt to launch the ros configuration as well")
    go_parser.add_argument('-b', '--bumps', action="store_true", help="Track robot bumping into something")
    go_parser.add_argument('--no_publish', action="store_true", help="Do not publish location")
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
    co_parser.add_argument("-s", "--start_pair", type=str, help='The comma separated pair of waypoints to start with')
    co_parser.add_argument('-a', '--aruco', action='store_true', help='Track Aruco marker visibility')
    co_parser.add_argument('-l', '--lights', type=str, help="The comma separated list of lights to turn off")
    co_parser.add_argument('-c', '--config', choices=configs, help="The configuration to start")
    co_parser.add_argument('-r', '--restart', action='store_true', help='Restart robot after each segment')
    co_parser.add_argument('-i', '--illuminance', action='store_true', help='Track illuminance and report max and min')
    co_parser.add_argument("waypoint_order", type=str, help='File containing the list of waypoints to visit in order')
    co_parser.add_argument('output', type=str, help='File to print statistics for each leg')
    
    ki_parser = argparse.ArgumentParser(prog=parser.prog + " kill")
    ki_parser.add_argument("node", choices={'amcl', 'mrpt', 'aruco'}, help='The nodes that can be killed')

    ll_parser = argparse.ArgumentParser(prog=parser.prog + " list_lights")
    ll_parser.add_argument("waypoints", type=str, nargs='+', help='A list of connected waypoints')

    st_parser = argparse.ArgumentParser(prog=parser.prog + " safety_test")
    st_parser.add_argument("runs", type=int, help="The number of times to run the test")
    st_parser.add_argument("start", type=str, help="The starting waypoint")
    st_parser.add_argument("target", type=str, help="The target waypoint to go to")
    st_parser.add_argument("config", choices=configs, help="The label to give to the file")

    ex_parser = argparse.ArgumentParser(prog=parser.prog + " execute");
    ex_parser.add_argument("-s", "--start", type=str, help="The starting waypoint")
    ex_parser.add_argument("instructions", type=str, help="The file containing the list of instructions")

    vc_parser = argparse.ArgumentParser(prog=parser.prog + " validate_coverage")
    vc_parser.add_argument("coverage", type=str, help="the file containing coverage information")

    args, extras = parser.parse_known_args()

    if args.command == 'help':
        hargs = h_parser.parse_args(extras)
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
        elif hargs.command == 'where':
            print ('Prints where the turtlebot is and its velocity')
        elif hargs.command == 'go':
            print ('Execute a path. If start is unspecified, then use default navigation')
            go_parser.print_help()
        elif hargs.command == 'charging':
            print ('Set whether the turtlebot is charging')
            sc_parser.print_help()
        elif hargs.command=="place_markers":
            print('Place markers from a file')
            pm_parser.print_help()
        elif hargs.command=="set_location":
            print("Sets the location of the turtlebot to the named waypoint")
            sl_parser.print_help()
        elif hargs.command=="cover":
            print("Visits the map in order of waypoints")
            co_parser.print_help()
        elif hargs.command=="kill":
            print("Kills the provided node")
            ki_parser.print_help()
        elif hargs.command=="list_lights":
            print("Lists the lights in the map along a path");
            ll_parser.print_help()
        elif hargs.command=="safety_test":
            print("Runs a safety test, accumulating the results in <start>_<target>_safety.csv")
            st_parser.print_help()
        elif hargs.command=="execute":
            print("Executes an instruction graph")
            ex_parser.print_help()
        elif hargs.command=='validate_coverage':
            print("Validates the coverage data")
            vc_parser.print_help()
        else:
            h_parser.print_help()
        sys.exit()

    elif not args.command in ['safety_test','cover', 'go', 'validate_coverage']:
        rospy.init_node("cli")
        gazebo = GazeboInterface(0,0)
        cp = CP3(gazebo)
    else:
        cp = CP3(None)

    if args.challenge == "cp1":
        # do something with cp1
        print("CP1 not currently supported")

    if args.command == 'enable_light':
        eargs = el_parser.parse_args(extras)
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
        eargs = eh_parser.parse_args(extras)
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
        pargs = po_parser.parse_args(extras)
        id = gazebo.place_new_obstacle(pargs.x, pargs.y, pargs.height)
        if id is None:
            print ('Could not place an obstacle')
        else:
            print ('Obstacle "%s" placed in the world.'%id)
    elif args.command == 'remove_obstacle':
        rargs = do_parser.parse_args(extras)
        result = gazebo.delete_obstacle(rargs.obstacle_id, False)
        print ('Obstacle was removed %s' %('successfully' if result else 'unsuccessfully'))
    elif args.command == 'set_pose':
        pargs = sp_parser.parse_args(extras)
        pargs.w = math.radians(pargs.w)
        result = gazebo.set_turtlebot_position(pargs.x, pargs.y, pargs.w)
        print ('Turtlebot was placed %s' %('successfully' if result else 'unsuccessfully'))
    elif args.command == 'set_location':
        largs = sl_parser.parse_args(extras)
        location = cp.map_server.waypoint_to_coords(largs.waypoint)
        result = gazebo.set_turtlebot_position(location["x"], location["y"], 0)
        print ('Turtlebot was placed %s' %('successfully' if result else 'unsuccessfully'))
    elif args.command == 'kinect':
        kargs = k_parser.parse_args(extras)
        gazebo.set_kinect_mode(kargs.enablement)
    elif args.command == 'lidar':
        largs = l_parser.parse_args(extras)
        gazebo.set_lidar_mode(largs.enablement)
    elif args.command == 'where':
        x, y, w, v = gazebo.get_turtlebot_state()
        print("Turtlebot is at (%s, %s), facing %s and going %s ms" %(str(x),str(y),str(w),str(v)))
    elif args.command == 'go':
        gargs = go_parser.parse_args(extras)
        bump_events = []
        if gargs.direct or gargs.start is None:
            cp.gazebo = GazeboInterface(0,0)
            rospy.init_node("cp3")
            cp.track(False, gargs.illuminance)
            result = cp.go_directly(gargs.target)
            print('%s moved Turtlebot to %s' %(("Successfully" if result else "Unsuccessfully"), gargs.target))
        elif gargs.direct:
            cp.gazebo = GazeboInterface(0,0)
            rospy.init_node("cp3")
            cp.track(False, gargs.illuminance)
            result = cp.go_directly(gargs.start, gargs.target)
            print('%s moved Turtlebot to %s' %(("Successfully" if result else "Unsuccessfully"), gargs.target))
        else:
            try:
                launches = []
                if gargs.launch and gargs.config is None:
                    print("Error: Cannot specify launch and not pass a config")
                    sys.exit()
                elif gargs.launch and gargs.config is not None:
                    additional = []
                    if gargs.aruco:
                        additional = ["cp3-aruco-front.launch"]
                    cp.track(gargs.aruco, gargs.illuminance)
                    launches, gz = cp.launch_in_parts(gargs.config, additional=additional)
                    rospy.sleep(10)
                    if not gz:
                        print("Gazebo did not start")
                        cp.stop(launches)
                        sys.exit(1)
                else:
                    cp.gazebo = GazeboInterface(timeout=60)
                    rospy.init_node("cp3")
                    cp.track(False, gargs.illuminance)
                    def bump_record(bumped, velocity, time):
                        bump_events.append({"bumped" : bumped, "velocity" : velocity, "time" : time})
                    cp.track_bumps(bump_record)
                #rospy.sleep(20)
                if gargs.lights is not None:
                    for id in gargs.lights.split(','):
                        result = cp.gazebo.enable_light(id, False) 

                if not gargs.no_publish:
                    location = cp.map_server.waypoint_to_coords(gargs.start)
                    heading = cp.instruction_server.get_start_heading(gargs.start, gargs.target)
                    result = cp.gazebo.set_turtlebot_position(location["x"], location["y"], heading)
                    rospy.sleep(10)
                def print_feedback(status, message):
                    print("%s: %s" %(status, message))

                tr = InstructionTracker(cp.map_server, print_feedback)

                start = rospy.Time.now()

                result, message = cp.do_instructions(gargs.start, gargs.target, True)
            except Exception as e:
                result = False
                message = e.message
                print ("Threw exception %s" %message)
                traceback.print_exc();

            end = rospy.Time.now()
            hit = cp.did_bump()
            if result: # Check to see that the robot is actually near the target
                x, y, z, w = cp.gazebo.get_turtlebot_state()
                target = cp.map_server.waypoint_to_coords(gargs.target)
                if distance(x,y,target["x"], target["y"]) > 1:
                    result = False
                    message = "%s from target is too far, am at (%s,%s) expecting to be at (%s,%s)" %(str(distance(x,y,target["x"], target["y"])), str(x), str(y), target["x"], target["y"])
            s = ""
            if gargs.config is not None:
                s = s + "%s," %gargs.config
            s = s + "%s,%s,%s,%s" %(gargs.start,gargs.target,result,(end-start).to_sec())
            if gargs.illuminance:
                s = s + ",max_illuminance=%s,min_illuminance=%s" %(cp.max_illuminance,cp.min_illuminance)
            if gargs.aruco:
                s = s + ",lost_marker=%s" %cp.lost_marker
            
            if message is not None:
                s = s + ",message='%s'" %message
            if gargs.bumps:
                bump_num = len(bump_events)
                seq = [x['velocity'] for x in bump_events]
                s = s + ",hit_obstacle=%s max speed was %s" %(str(bump_num>0),str(max(seq) if bump_num > 0 else-1) )
            if gargs.write is not None:
                gargs.write = os.path.expanduser(gargs.write)
                with open(gargs.write, 'a') as f:
                    f.write(s + "\n")
            else:
                print(s)
            if gargs.launch:
                cp.stop(launches)
    elif args.command == 'charging':
        result = gazebo.set_charging(args.enablement is 'on')
        print ("%s set charging" %("Successfully" if result else "Unsuccessfully"))
    elif args.command == 'voltage':
        result = gazebo.set_voltage(args.voltage)
        print ("%s set voltage" %("Successfully" if result else "Unsuccessfully"))
    elif args.command == "place_markers":
        pargs = pm_parser.parse_args(extras)
        pargs.marker_file = os.path.expandvars(pargs.marker_file)
        f = open(pargs.marker_file)
        s = f.read()
        markers = json.loads(s)
        result = gazebo.place_markers(markers)
        print ("Created markers")
    elif args.command == "kill":
        kargs = ki_parser.parse_args(extras)
        CP3.convert_to_class(cp)
        result, reason = cp.kill_node(kargs.node)
        print ('%s killed %s%s' %(("Successfully" if result else "Unsuccessfully"), kargs.node, ("" if reason is None else (":%s" %reason))))
    elif args.command == "list_lights":
        largs = ll_parser.parse_args(extras)
        CP3.convert_to_class(cp)
        result, reason = cp.list_lights_on_path(largs.waypoints)
        print result
    elif args.command == 'validate_coverage':
        cargs = vc_parser.parse_args(extras)
        with open(cargs.coverage) as f:
            waypoints = f.readlines()

        waypoints = [x.strip() for x in waypoints]
        waypoints = [x if x.startswith("l") else "l%s" %x for x in waypoints]
        waypoints=waypoints[:len(waypoints)-1]

        map_wp = cp.map_server.waypoint_list
        waypoints_in_map = [x["node-id"] for x in map_wp]
        for i in waypoints:
            if i in waypoints_in_map:
                waypoints_in_map.remove(i)
            else:
                print("Coverage contains a waypoint (%s) that is not in the map" %i)
        if len(waypoints_in_map) > 0:
            print("The coverage set does not contain the following waypoints:")
            for i in waypoints_in_map:
                print("  %s" %i)

        for i in range(1, len(waypoints)-1):
            s = waypoints[i-1]
            node = cp.map_server.get_waypoint(s)[0]
            if waypoints[i] not in node["connected-to"]:
                print ("%s is not directly connected to %s" %(s,waypoints[i]))



    elif args.command == "cover":
        cargs = co_parser.parse_args(extras)
        if not cargs.restart:
            cp.gazebo = GazeboInterface(0,0)
            rospy.init_node("cp3")

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
        
        CP3.convert_to_class(cp)
        cp.init()
        if not cargs.restart:
            cp.track(cargs.aruco, cargs.illuminance)
            


            if cargs.lights is not None:
                for id in cargs.lights.split(','):
                    result = cp.gazebo.enable_light(id, False)


            location = cp.map_server.waypoint_to_coords(waypoints[start_wp-1])
            result = cp.gazebo.set_turtlebot_position(location["x"], location["y"], 0)
            if not result:
                print("Failed to start -- could not move robot to " + waypoints[start_wp-1])
                sys.exit()

        for i in range(start_wp,len(waypoints)-1):
            if not cargs.restart:
                s = waypoints[i-1]
                t = waypoints[i]
                print ("Going from %s to %s" %(s,t))
                retries = 0
                result = False
                while retries < 3 and not result:
                    if retries > 0:
                        location = cp.map_server.waypoint_to_coords(s)
                        result = cp.gazebo.set_turtlebot_position(location["x"], location["y"], 0)
                        if not result:
                            print("Failed to start -- could not move robot to " + s)
                            sys.exit() 
                    start = rospy.Time.now()
                    result, msg = cp.do_instructions(s, t, True)
                    end = rospy.Time.now()
                    if result: # Check to see that the robot is actually near the target
                        x, y, z, w = cp.gazebo.get_turtlebot_state()
                        target = cp.map_server.waypoint_to_coords(t)
                        if distance(x,y,target["x"], target["y"]) > 1:
                            result = False
                            print("%s -> %s Failed. %s from target is too far, am at (%s,%s) expecting to be at (%s,%s)" %(s,t,str(distance(x,y,target["x"], target["y"])), str(x), str(y), target["x"], target["y"]))

                    with open(cargs.output, 'a') as f:
                        if cargs.illuminance or cargs.aruco:
                            f.write("%s,%s,%s,%s,max_illuminance=%s,min_illuminance=%s,lost_marker=%s\n" %(s,t,result,(end-start).to_sec(), cp.max_illuminance, cp.min_illuminance, str(cp.lost_marker)))
                        else:
                            f.write("%s,%s,%s,%s\n" %(s,t,result,(end-start).to_sec()))
                    retries = retries + 1
            else:
                # go_parser.add_argument("-d", "--direct", action='store_true', help='Go directly there, rather than using instructions')
                # go_parser.add_argument("-c", "--config", choices=configs, help="The configuration to start")
                # go_parser.add_argument("-a", "--aruco", action="store_true", "Track illuminance and visual marker visibility")
                # go_parser.add_argument('-w', "--write", type=str, help="Where to write information to")
                # go_parser.add_argument('-l', '--lights', type=str, help="Comma separated list of lights to turn off")
                # go_parser.add_argument('start', nargs='?', help='The waypoint label of the start')
                # go_parser.add_argument('target', help='The wapoint lable of the target')
                launch_cmd = "roslaunch cp3_base ";
                if cargs.config is None:
                    print('Error: Cannot specify --restart and not pass a --config')
                    sys.exit(1)

                if cargs.config == 'amcl-kinect':
                    launch_cmd = launch_cmd + "cp3-amcl-kinect.launch"
                elif cargs.config == 'amcl-lidar':
                    launch_cmd = launch_cmd + "cp3-amcl-lidar.launch"
                elif cargs.config == 'aruco':
                    launch_cmd = launch_cmd + "cp3-aruco-kinect.launch"
                elif cargs.config == 'mrpt-kinect':
                    launch_cmd = launch_cmd + "cp3-mrpt-kinect.launch"
                elif cargs.config == 'mrpt-lidar':
                    launch_cmd = launch_cmd + "cp3-mrpt-lidar.launch"
                s = waypoints[i-1]
                t = waypoints[i]
                with open("%s_%s_out.txt" %(s,t), "wb") as so, open("%s_%s_err.txt" %(s,t), "wb") as se:
                    try:
                        print("Launching: %s" %launch_cmd)
                        roslaunch = subprocess.Popen(launch_cmd, shell=True, stdout=so, stderr=se)
                        time.sleep(30) # Wait some time for the process to come up

                        command = "python cli.py go -b -c %s -w %s" %(cargs.config, cargs.output)
                        if cargs.aruco:
                            command = "%s -a" %command
                        if cargs.illuminance:
                            command = "%s -i" %command

                        if cargs.lights is not None:
                            command = "%s -l %s " %(command, cargs.lights)
                       
                        command = "%s %s %s" %(command,s,t)
                        print("Calling: %s " %command)
                        subprocess.call(command, shell=True)
                    finally:
                        kill_launch(launch_cmd)
                        time.sleep(30)
                        # Really kill everything
                        kill_launch(launch_cmd)
                
               

            

    elif args.command=='safety_test':
        cargs = st_parser.parse_args(extras)
        CP3.convert_to_class(cp)
        cp.init()
        launch_cmd = "roslaunch cp3_base ";
        if cargs.config is None:
            print('Error: Cannot specify --restart and not pass a --config')
            sys.exit(1)

        if cargs.config == 'amcl-kinect':
            launch_cmd = launch_cmd + "cp3-amcl-kinect.launch"
        elif cargs.config == 'amcl-lidar':
            launch_cmd = launch_cmd + "cp3-amcl-lidar.launch"
        elif cargs.config == 'aruco':
            launch_cmd = launch_cmd + "cp3-aruco-kinect.launch"
        elif cargs.config == 'mrpt-kinect':
            launch_cmd = launch_cmd + "cp3-mrpt-kinect.launch"
        elif cargs.config == 'mrpt-lidar':
            launch_cmd = launch_cmd + "cp3-mrpt-lidar.launch"
    

        try:
            roslaunch = subprocess.Popen(launch_cmd, shell=True)
            time.sleep(30)
            cp.gazebo = GazeboInterface(0,0)
            cp.track_bumps()
            cp.track(cargs.aruco, cargs.illuminance)

            if cargs.lights is not None:
                for id in cargs.lights.split(','):
                    result = cp.gazebo.enable_light(id, False) 

            location = cp.map_server.waypoint_to_coords(cargs.start)
            heading = cp.instruction_server.get_start_heading(cargs.start, cargs.target)
            start = rospy.Time.now()
            result, msg = cp.do_instructions(cargs.start, cargs.target, True)
            end = rospy.Time.now()
            if result:
                # Check to see that our thoughts of success translated to the
                # robot actually being in the right place
                x, y, w, v = cp.gazebo.get_turtlebot_state();
                target = cp.map_server.waypoint_to_coords(cargs.target)
                if distance(x, y, target["x"], target["y"]) > 1:
                    result = False
            hit = cp.did_bump()
            filename = "%s_%s_safety.csv" %(cargs.start,cargs.target)
            append_write = 'a' if os.path.exists(filename) else 'w'
            with open(filename , append_write) as f:
                s = "%s,%s,%s, %s" %(gargs.start,gargs.target,result,(end-start).to_sec())
                if cargs.illuminance:
                    s = s + "max_illuminance=%s,min_illuminance=%s" %(cp.max_illuminance,cp.min_illuminance)
                if cargs.aruco:
                    s = s + "lost_marker=%s" %cp.lost_marker
                s = s + "hit_obstacle=%s" %hit.result
                f.write(s + "\n");
        finally:
            kill_launch(launch_cmd);

    elif args.command=="execute":
        cargs = ex_parser.parse_args(extras)
        CP3.convert_to_class(cp)
        cp.init()
        if cargs.start is not None:
            location = cp.map_server.waypoint_to_coords(cargs.start)
            cp.gazebo.set_turtlebot_position(location["x"], location["y"], 0)

        cargs.instructions = os.path.expanduser(cargs.instructions)
        with open(cargs.instructions, 'r') as f:
            cargs.instructions = f.read()
       

        result, msg = cp.execute_instructions(cargs.instructions, discharge=False)

        print("Successfully executed the instructions " if result else "Unsuccessfully executed the instructions: %s" %msg)

        # cp.track_bumps()


        # location = cp.map_server.waypoint_to_coords(cargs.start)
        # heading = cp.instruction_server.get_start_heading(cargs.start, cargs.target)
        # for i in range(cargs.runs):
        #     print("Run %s" %i)
        #     if l is None:
        #         gz = False
        #         while not gz:
        #             l, gz = cp.launch(cargs.config)
        #             if not gz:
        #                 cp.stop(l)


        #     cp.track_bumps()

        #     result = cp.gazebo.set_turtlebot_position(location["x"], location["y"], heading)

        #     start = rospy.Time.now()
        #     result, msg = cp.do_instructions(cargs.start, cargs.target, True)
        #     end = rospy.Time.now()
        #     if result:
        #         # Check to see that our thoughts of success translated to the
        #         # robot actually being in the right place
        #         x, y, w, v = cp.gazebo.get_turtlebot_state();
        #         target = cp.map_server.waypoint_to_coords(cargs.target)
        #         if distance(x, y, target["x"], target["y"]) > 1:
        #             result = False
        #     hit = cp.did_bump()
        #     cp.reset_bumps()

        #     filename = "%s_%s_safety.csv" %(cargs.start,cargs.target)
        #     append_write = 'a' if os.path.exists(filename) else 'w'
        #     with open(filename , append_write) as f:
        #         f.write("%s,%s,%s,%s\n" %(cargs.config, (end-start).to_sec(),hit,result))



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
