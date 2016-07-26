'''Robot class that provides an interface to arm and head (maybe base later)'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy
import roslib

# System builtins
import os

# ROS builtins
from sound_play.libsoundplay import SoundClient
from std_srvs.srv import Empty
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

# Local
from fetch_pbd_interaction.srv import MoveArm, GetGripperState, \
                                      SetGripperState, GetArmMovement, \
                                      GetEEPose, GetJointStates, GetGazeGoal, \
                                      GetNearestObject
from fetch_social_gaze.msg import GazeGoal, GazeAction
from fetch_pbd_interaction.msg import ArmState, Landmark

# ######################################################################
# Module level constants
# ######################################################################

# String for base link name
BASE_LINK = 'base_link'

PKG = 'fetch_pbd_interaction'

# Directory sounds are in, within the package.
SOUNDS_DIR = os.path.join(roslib.packages.get_pkg_dir(PKG), 'sounds', '')

# Common postfix for sound files.
SOUND_FILEFORMAT = '.wav'

# ######################################################################
# Classes
# ######################################################################

class Robot:
    '''Robot class that provides an interface to arm
    and head (maybe base later)'''

    def __init__(self, tf_listener):
        '''
        Args:
            tf_listener (TransformListener)
        '''

        self._tf_listener = tf_listener
        # arm services

        self.move_arm_to_joints_srv = rospy.ServiceProxy(
                                        'move_arm_to_joints', MoveArm)
        rospy.wait_for_service('move_arm_to_joints')

        self.move_arm_to_pose_srv = rospy.ServiceProxy(
                                        'move_arm_to_pose', MoveArm)
        rospy.wait_for_service('move_arm_to_pose')

        self.start_move_arm_to_pose_srv = rospy.ServiceProxy(
                                            'start_move_arm_to_pose', MoveArm)
        rospy.wait_for_service('start_move_arm_to_pose')

        self.is_reachable_srv = rospy.ServiceProxy('is_reachable', MoveArm)
        rospy.wait_for_service('is_reachable')

        self.is_arm_moving_srv = rospy.ServiceProxy(
                                    'is_arm_moving', GetArmMovement)
        rospy.wait_for_service('is_arm_moving')

        self.relax_arm_srv = rospy.ServiceProxy('relax_arm', Empty)
        rospy.wait_for_service('relax_arm')

        self.reset_arm_movement_history_srv = rospy.ServiceProxy(
                                        'reset_arm_movement_history', Empty)
        rospy.wait_for_service('reset_arm_movement_history')

        self.get_gripper_state_srv = rospy.ServiceProxy(
                                        'get_gripper_state', GetGripperState)
        rospy.wait_for_service('get_gripper_state')

        self.get_joint_states_srv = rospy.ServiceProxy(
                                        'get_joint_states', GetJointStates)
        rospy.wait_for_service('get_joint_states')

        self.get_ee_pose_srv = rospy.ServiceProxy('get_ee_pose', GetEEPose)
        rospy.wait_for_service('get_ee_pose')

        self.set_gripper_state_srv = rospy.ServiceProxy(
                                        'set_gripper_state', SetGripperState)
        rospy.wait_for_service('set_gripper_state')

        rospy.loginfo("Got all arm services")


        # head services

        self.gaze_client = SimpleActionClient('gaze_action', GazeAction)

        self.gaze_client.wait_for_server(rospy.Duration(5))

        self.current_gaze_goal_srv = rospy.ServiceProxy(
                                        'get_current_gaze_goal', GetGazeGoal)
        rospy.wait_for_service('get_current_gaze_goal')

        rospy.loginfo("Got all head services")

        # world services

        self._get_nearest_object_srv = rospy.ServiceProxy(
                                        'get_nearest_object', GetNearestObject)
        rospy.wait_for_service('get_nearest_object')

        rospy.loginfo("Got all world services")


        # sound stuff
        self._sound_client = SoundClient()

    # arm stuff

    def move_arm_to_pose(self, arm_state):
        '''Move robot's arm to pose

        Args:
            arm_state (ArmState) : contains pose info
        Return:
            bool : success
        '''
        return self.move_arm_to_pose_srv(arm_state).success

    def start_move_arm_to_pose(self, arm_state):
        '''Start thread to move robot's arm to pose

        Args:
            arm_state (ArmState) : contains pose info
        Return:
            bool : success
        '''
        # TODO(sarah): Is this really necessary? Investigate.
        return self.start_move_arm_to_pose_srv(arm_state).success

    def move_arm_to_joints(self, arm_state):
        '''Move robot's arm to joints positions from arm_state

        Args:
            arm_state (ArmState) : contains joint position info
        Return:
            bool : success
        '''
        return self.move_arm_to_joints_srv(arm_state).success

    def can_reach(self, arm_state):
        '''Returns whether arm can reach pose

        Args:
            arm_state (ArmState) : contains pose info
        Return:
            bool : success
        '''
        return self.is_reachable_srv(arm_state).success

    def reset_arm_movement_history(self):
        '''Resets movement history of arm'''
        self.reset_arm_movement_history_srv()

    def get_gripper_state(self):
        '''Returns state of gripper

        Returns:
            GripperState.OPEN|GripperState.CLOSED
        '''
        return self.get_gripper_state_srv().gripper_state

    def set_gripper_state(self, gripper_state):
        '''Sets state of gripper. Assumed to succeed

        Args:
            gripper_state (GripperState.OPEN|GripperState.CLOSED)
        '''
        self.set_gripper_state_srv(gripper_state)

    def get_arm_state(self):
        '''Returns current state of arm

        Returns:
            ArmState
        '''
        abs_ee_pose = self.get_ee_pose_srv().ee_pose  # (PoseStamped)
        joint_pose = self.get_joint_states_srv().joints  # ([float64])

        state = None
        rel_ee_pose = None

        resp = self._get_nearest_object_srv(
            abs_ee_pose)
        has_nearest = resp.has_nearest
        if not has_nearest:
            # Arm state is absolute (relative to robot's base_link).
            state = ArmState(
                ArmState.ROBOT_BASE,  # ref_frame (uint8)
                abs_ee_pose,  # ee_pose (PoseStamped)
                joint_pose,  # joint_pose ([float64])
                Landmark()  # ref_frame_landmark (Landmark)
            )
        else:
            nearest_obj = resp.nearest_object
            # Arm state is relative (to some object in the world).
            # rospy.loginfo("Relative to: {}".format(nearest_obj.name))

            rel_ee_pose = self._tf_listener.transformPose(
                            nearest_obj.name, abs_ee_pose)

            state = ArmState(
                ArmState.OBJECT,  # ref_frame (uint8)
                rel_ee_pose,  # ee_pose (PoseStamped)
                joint_pose,  # joint_pose [float64]
                nearest_obj  # ref_frameLandmark (Landmark)
            )

        return state

    def relax_arm(self):
        '''Make sure gravity compensation controller is on and other
        controllers are off
        '''
        self.relax_arm_srv()

    def is_arm_moving(self):
        '''Check if arm is currently moving

        Returns:
            bool : True if arm is moving, else False
        '''
        return self.is_arm_moving_srv().moving

    # Head stuff

    def shake_head(self, num=5):
        '''Shakes robot's head

        Args:
            num (int) : number of times to perform action
        '''
        goal = GazeGoal()
        goal.action = GazeGoal.SHAKE
        goal.repeat = num
        if goal.action != self.current_gaze_goal_srv().gaze_goal:
            self.gaze_client.send_goal(goal)

    def nod_head(self, num=5):
        '''Nods robot's head

        Args:
            num (int) : number of times to perform action
        '''
        goal = GazeGoal()
        goal.action = GazeGoal.NOD
        goal.repeat = num
        if goal.action != self.current_gaze_goal_srv().gaze_goal:
            self.gaze_client.send_goal(goal)

    def look_at_point(self, point):
        '''Points robot's head at point

        Args:
            point (Point)
        '''
        goal = GazeGoal()
        goal.action = GazeGoal.LOOK_AT_POINT
        goal.point = point
        if goal.action != self.current_gaze_goal_srv().gaze_goal:
            self.gaze_client.send_goal(goal)

    def look_at_ee(self, follow=True):
        '''Makes head look at (or follow) end effector position

        Args:
            follow (bool, optional) : If True, follow end effector,
                                      else, just glance at end effector
        '''
        rospy.loginfo("Look at ee")
        goal = GazeGoal()
        if follow:
            goal.action = GazeGoal.FOLLOW_EE
        else:
            goal.action = GazeGoal.GLANCE_EE
        if goal.action != self.current_gaze_goal_srv().gaze_goal:
            self.gaze_client.send_goal(goal)

    def look_forward(self):
        '''Point head forward'''
        goal = GazeGoal()
        goal.action = GazeGoal.LOOK_FORWARD
        if goal.action != self.current_gaze_goal_srv().gaze_goal:
            self.gaze_client.send_goal(goal)

    def look_down(self):
        '''Point head down at table'''
        # TODO(sarah): maybe actually scan for table instead of
        # looking at static point
        goal = GazeGoal()
        goal.action = GazeGoal.LOOK_DOWN
        if goal.action != self.current_gaze_goal_srv().gaze_goal:
            self.gaze_client.send_goal(goal)
        while (self.gaze_client.get_state() == GoalStatus.PENDING or
               self.gaze_client.get_state() == GoalStatus.ACTIVE):
            rospy.sleep(0.2)

    # Sound stuff

    def play_sound(self, requested_sound):
        '''Play sound that is requested

        Args:
            requested_sound (RobotSound.ERROR|etc...) : see RobotSound.msg
        '''
        self._sound_client.playWave(
                os.path.join(SOUNDS_DIR, requested_sound + SOUND_FILEFORMAT))