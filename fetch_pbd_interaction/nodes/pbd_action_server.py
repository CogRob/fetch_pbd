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
from actionlib import SimpleActionClient, SimpleActionServer
from interactive_markers.interactive_marker_server import \
     InteractiveMarkerServer
from tf import TransformListener
# import rospkg

# Local
from fetch_pbd_interaction.session import Session
from fetch_pbd_interaction.msg import ExecuteAction, ExecuteResult, ExecuteFeedback
from fetch_pbd_interaction.robot import Robot

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
        self.robot = Robot(self.tf_listener)
        self.im_server = InteractiveMarkerServer(TOPIC_IM_SERVER)
        # Get path to example json file
        file_path = rospy.get_param("from_file")
        self.session = Session(self.robot, self.tf_listener,
                      self.im_server, from_file=file_path)

        n_actions = self.session.n_actions()
        if n_actions > 0:
            rospy.loginfo("There are {} actions.".format(n_actions))

        self.action_server = SimpleActionServer(
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
        action_names = target_goal.action_names
        # continue_on_failure = target_goal.continue_on_failure
        actions_completed = [False] * len(action_names)
        success = True
        for index, action_name in enumerate(action_names):
            if self.session.switch_to_action_by_name(action_name):
                completed = self.session.execute_current_action()
                actions_completed[index] = completed
                success = success and completed
                self.send_feedback(target_goal, action_name, index, actions_completed)
        self.send_result(target_goal, actions_completed, success)

    def send_feedback(self, target_goal,
                            current_action_executed_name,
                            current_action_executed_index,
                            current_actions_completed):
        """ Call this method to send feedback about the progress of the action """

        feedback = ExecuteFeedback()
        feedback.action_names = target_goal.action_names
        feedback.current_action_executed_name = current_action_executed_name
        feedback.current_action_executed_index = current_action_executed_index
        feedback.current_actions_completed = current_actions_completed
        self.action_server.publish_feedback(feedback)
        rospy.loginfo("Execute feedback.\n{0}".format(str(feedback)))

    def send_result(self, target_goal, actions_completed, success):
        """ Call this method to send resutls about of the action """

        result = ExecuteResult()
        result.action_names = target_goal.action_names
        result.actions_completed = actions_completed
        result.success = success
        self.action_server.set_succeeded(result)
        rospy.loginfo("Execute result.\n{0}".format(str(result)))

    def send_abort(self, target_goal, actions_completed, success):
        """ Call this method to send abort info about of the action """

        result = ExecuteResult()
        result.action_names = target_goal.action_names
        result.actions_completed = actions_completed
        result.success = success
        self.action_server.set_aborted(result)
        rospy.loginfo("Execute abort.\n{0}".format(str(result)))

# ######################################################################
# Main Function
# ######################################################################

if __name__ == '__main__':
    rospy.init_node('fetch_pbd_action_server')
    server = PBDAction('/pbd_action_exicute')
    server.start()
    rospy.spin()
