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

action_server_name = 'pbd_action_exicute'

def PBDClient():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient(
        action_server_name,
        fetch_pbd_interaction.msg.ExecuteAction
    )

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    action_goal = fetch_pbd_interaction.msg.ExecuteActionGoal()
    action_goal.goal.action_names = action_names = [
        'small_wave_hello',
        'large_wave_hello',
        'selfie_engage',
        'selfie_disengage'
    ]
    action_goal.goal.continue_on_failure = False

    # Sends the goal to the action server.
    client.send_goal(action_goal.goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

# ######################################################################
# Main Function
# ######################################################################

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fetch_pbd_action_client')
        result = PBDClient()
        print("Result.\n{0}".format(str(result)))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
