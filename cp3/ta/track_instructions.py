import rospy
from ig_action_msgs.msg import InstructionGraphActionFeedback, InstructionGraphActionResult

import re

class InstructionTracker():

    NEW_IG = "New Instruction Graph"
    AT_WAYPOINT = "At Waypont"
    FAILED_TO_REACH_WAYPOINT = "Failed Waypoint"
    COMPLETED_IG = "Completed Instruction Graph"
    CANCELED_IG = "Instruction graph canceled"
    FAILED_IG = "Instruction graph failed"

    def __init__(self, map_server, callback):
        self.callback = callback
        self.map_server = map_server
        self.ig_feedback_topic = rospy.Subscriber("/ig_action_server/feedback", InstructionGraphActionFeedback, self.ig_step)
        self.ig_result_topic = rospy.Subscriber("/ig_action_server/result", InstructionGraphActionResult, self.ig_result)

    def ig_step(self, feedback):
        if "Received new valid IG" in feedback.feedback.sequence:
            self.callback(self.NEW_IG, feedback.feedback.sequence)
        elif "MoveAbsH:" in feedback.feedback.sequence:
            match=re.match(r'MoveAbsH\s*\((.*),\s*(.*),\s*.*,\s*.*\s*\): (.*)', feedback.feedback.sequence)
            if match is not None:
                x = float(match.group(1))
                y = float(match.group(2))
                status = match.group(3)
                if "SUCCESS" in status:
                    wp = self.map_server.coords_to_waypoint(x,y)
                    if wp is not None:
                        self.callback(self.AT_WAYPOINT, wp)
                    else:
                        self.callback(self.AT_WAYPOINT, "unknown")
                elif "FAILED" in status or "CANCELED" in status:
                    wp = self.map_server.coords_to_waypoint(x,y)
                    if wp is not None:
                        self.callback(self.FAILED_TO_REACH_WAYPOINT, wp)
                    else:
                        self.callback(self.FAILED_TO_REACH_WAYPOINT, "unknown")
            else:
                print("DId not match %s" %feedback.sequence.sequence)

    def ig_result(self, result):
        if "successfully" in result.result.sequence:
            self.callback(self.COMPLETED_IG, result.result.sequence)
        elif "canceled" in result.result.sequence:
            self.callback(self.CANCELED_IG, result.result.sequence)
        elif "failed" in result.result.sequence:
            self.callback(self.FAILED_IG, result.result.sequence)