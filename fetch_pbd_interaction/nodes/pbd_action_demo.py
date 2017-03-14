#!/usr/bin/env python
'''This runs the demo client for action server.
'''

# ######################################################################
# Imports
# ######################################################################
from __future__ import print_function

# Core ROS imports come first.
import rospy

# ROS builtins
# import rospkg

# Local
import actionlib
import fetch_pbd_interaction.msg

# ######################################################################
# Module level constants
# ######################################################################

ACTION_SERVER_NAME = 'pbd_action_exicute'

class PBDClient():

    def __init__(self):
        # Creates the SimpleActionClient, passing the type of the action
        self.client = actionlib.SimpleActionClient(
            ACTION_SERVER_NAME,
            fetch_pbd_interaction.msg.ExecuteAction
        )
        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()

    def set_goal(self, action_names, continue_on_failure = False):
        # Creates a goal to send to the action server.
        self.action_goal = fetch_pbd_interaction.msg.ExecuteActionGoal().goal
        self.action_goal.action_names = action_names
        self.action_goal.continue_on_failure = continue_on_failure

    def send_goal(self):
        # Sends the goal to the action server.
        self.client.send_goal(self.action_goal, done_cb=self.done_cb, feedback_cb=self.feedback_cb)
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()

    def feedback_cb(self, feedback):
        print("Feedback.\n{0}".format(str(feedback)))

    def done_cb(self, terminal_state, result):
        print("Result.\n{0}".format(str(result)))


# ######################################################################
# Main Function
# ######################################################################

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fetch_pbd_action_client')
        client = PBDClient()
        action_names = [
            'small_wave_hello',
            'selfie_engage',
            'selfie_disengage'
        ]
        client.set_goal(action_names)
        client.send_goal()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
