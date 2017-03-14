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
        '''Initilize PbD Action Server'''
        # Initilize PbD Session
        self.action_server_name = action_server_name
        self.tf_listener = TransformListener()
        self.robot = Robot(self.tf_listener)
        self.im_server = InteractiveMarkerServer(TOPIC_IM_SERVER)
        self.session = Session(self.robot, self.tf_listener,
                      self.im_server, from_file=rospy.get_param("from_file"))
        n_actions = self.session.n_actions()
        rospy.loginfo("There are {} actions avalable.".format(n_actions))
        if n_actions > 0:
            actions = self.session._get_action_names()
            rospy.loginfo("List of avalable actions includes: {}".format(actions))

        # Initilize Simple Action Server
        self.action_server = SimpleActionServer(
            self.action_server_name,
            ExecuteAction,
            auto_start=False)
        self.action_server.register_goal_callback(self.__goal_callback)

    def start(self):
        '''Start PbD Action Server'''
        self.action_server.start()

    def __goal_callback(self):
        '''Goal Callback for PbD Action requests from clinets'''
        target_goal = self.action_server.accept_new_goal()
        rospy.loginfo("Target goal received: " + str(target_goal))
        self.execute(target_goal)

    def switch_to_action_by_name(self, name):
        '''Switches to action with name

            Args:
                name (str): The action name to switch to.

            Returns:
                bool: Whether successfully switched to index action.
        '''
        names = self.session._get_action_names()
        if name in names:
            index = names.index(name)
            return self.switch_to_action_by_index(index)
        else:
            return False

    def switch_to_action_by_index(self, index):
        '''Switches to action with index

        Args:
            index (int): The action id to switch to.

        Returns:
            bool: Whether successfully switched to index action.
        '''
        self.session._lock.acquire()
        self.session._selected_primitive = -1

        if index < 0 or index >= len(self.session._actions):
            rospy.logwarn("Index out of bounds: {}".format(index))
            return False

        if not index in self.session._actions:
            rospy.logwarn(
                "Can't switch actions: failed to load action {}".format(
                    index))
            return False

        self.session._current_action_id = index
        self.session._clear_world_objects_srv()
        self.session._lock.release()
        # self.session.get_current_action().initialize_viz()
        self.session._update_session_state()
        self.session.publish_primitive_tf()
        # action = self.session._actions[self.session._current_action_id]
        # for i in range(self.session.n_primitives()):
        #     try:
        #         self.session._tf_listener.waitForTransform("base_link",
        #                      "primitive_" + str(i),
        #                      rospy.Time.now(),
        #                      rospy.Duration(5.0))
        #         action.get_primitive(i).update_viz(False)
        #     except Exception, e:
        #         rospy.loginfo("Frame primitive_" + str(i) +
        #                       " is not available.")
        return True

    def execute(self, target_goal):
        '''Execute list of goal PbD Action in order'''
        action_names = target_goal.action_names
        # continue_on_failure = target_goal.continue_on_failure
        actions_completed = [False] * len(action_names)
        success = True
        for action_index, action_name in enumerate(action_names): # Loop through each action
            if self.switch_to_action_by_name(action_name): # Switch session to action
                self.session.get_current_action()._reachable_override = True # Avoide checking all premetive are reachalbe
                completed = self.session.execute_current_action() # If action exists in session, then execute
                actions_completed[action_index] = completed # Record progress
                success = success and completed # Diliberate continued success
                self.send_feedback(target_goal, action_name, action_index, actions_completed) # Update clients with feeback
        self.send_result(target_goal, actions_completed, success) # Send final results

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
