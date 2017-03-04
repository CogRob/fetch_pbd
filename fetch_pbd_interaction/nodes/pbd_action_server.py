#!/usr/bin/env python
'''This runs the action server for the PbD backend.
It handles the interactions between the an action client and recorded actions.
It coordinates execution of leaned actions with the robot on the client's behalf.
'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# ROS builtins
# from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleActionClient, SimpleActionServer
# from visualization_msgs.msg import MarkerArray
from interactive_markers.interactive_marker_server import \
     InteractiveMarkerServer
from tf import TransformListener
# import rospkg

# Local
# from fetch_arm_control.msg import GripperState
from fetch_pbd_interaction.session import Session
# from fetch_pbd_interaction.msg import ExecutionStatus
from fetch_pbd_interaction.msg import ExecuteAction, ExecuteResult, ExecuteFeedback
# from fetch_pbd_interaction.msg import RobotSound, WorldState
# from fetch_pbd_interaction.srv import Ping, PingResponse, GetObjectList
from fetch_pbd_interaction.robot import Robot
# from std_msgs.msg import String
# from std_srvs.srv import Empty

# ######################################################################
# Module level constants
# ######################################################################

EXECUTION_Z_OFFSET = -0.00
BASE_LINK = 'base_link'
TOPIC_IM_SERVER = 'programmed_actions'


class PBDAction(object):
    '''Handles what leanred action to execute'''

    def __init__(self, action_server_name):
        self.action_server_name = action_server_name

        # Run the system
        self.tf_listener = TransformListener()
        self.robot = Robot(tf_listener)
        self.im_server = InteractiveMarkerServer(TOPIC_IM_SERVER)
        # Get path to example json file
        file_path = rospy.get_param("from_file")
        self.session = Session(robot, tf_listener,
                      im_server, from_file=file_path)

        if session.n_actions() > 0:
            rospy.loginfo("There are {} actions.".format(session.n_actions()))

        self.action_server = actionlib.SimpleActionServer(
            self.action_server_name,
            ExecuteAction,
            auto_start=False)
        self.action_server.register_goal_callback(self.__goal_callback)

    def start(self):
        self.action_server.start()

    def __goal_callback(self):
        target_goal = self.action_server.accept_new_goal()
        rospy.loginfo("Target goal received: " + str(target_goal))
        self.execute(target_goal)

    def execute(self, target_goal):
        if self.session.switch_to_action_by_name(target_goal.action_name):
            n_primitives = self.session.n_primitives()
            rospy.loginfo("Executing action %s, " +
                          "with {} primitives.".format(
                          target_goal.action_name, n_primitives))
            # if self.session.execute_current_action():
            #     self.action_server.set_succeeded()

            primitives_executed = 0
            for primitive in range(n_primitives):
                if self.session.execute_primitive(primitive):
                    self.send_feedback(target_goal, n_primitives, primitive)
                    primitives_executed += 1
            self.send_result(target_goal, n_primitives, primitives_executed)
        else:
            self.send_abort(target_goal, 0, 0)
            rospy.logwarn("No actions available!")

    def send_feedback(self, target_goal, n_primitives, primitive_executed):
        """ Call this method to send feedback about the progress of the action """

        feedback = ExecuteFeedback()
        feedback.action_name = target_goal.action_name
        feedback.n_primitives = n_primitives
        feedback.primitive_executed = primitive_executed
        self.action_server.publish_feedback(feedback)
        rospy.loginfo("Execute feedback. action_name: {0} " +
                        "n_primitives: {1} " +
                        "primitive_executed: {2}".format(
                            feedback.action_name,
                            feedback.n_primitives,
                            feedback.primitive_executed))

    def send_result(self, target_goal, n_primitives, primitives_executed):
        """ Call this method to send resutls about of the action """

        result = ExecuteResult()
        result.action_name = target_goal.action_name
        result.n_primitives = n_primitives
        result.primitives_executed = primitives_executed
        result.action_completed = n_primitives == primitives_executed
        self.action_server.set_succeeded(feedback)
        rospy.loginfo("Execute result. action_name: {0} " +
                        "n_primitives: {1} " +
                        "primitives_executed: {2} " +
                        "action_completed: {3}".format(
                            result.action_name,
                            result.n_primitives,
                            result.primitives_executed,
                            result.action_completed))

    def send_abort(self, target_goal, n_primitives, primitives_executed):
        """ Call this method to send abort info about of the action """

        result = ExecuteResult()
        result.action_name = target_goal.action_name
        result.n_primitives = n_primitives
        result.primitives_executed = primitives_executed
        result.action_completed = False
        self.action_server.set_aborted(feedback)
        rospy.loginfo("Execute abort. action_name: {0} " +
                        "n_primitives: {1} " +
                        "primitives_executed: {2} " +
                        "action_completed: {3}".format(
                            result.action_name,
                            result.n_primitives,
                            result.primitives_executed,
                            result.action_completed))

# ######################################################################
# Main Function
# ######################################################################

if __name__ == '__main__':
    rospy.init_node('fetch_pbd_action_server')
    server = PBDAction(rospy.get_name()))
    rospy.spin()
