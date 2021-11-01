#! /usr/bin/env python
# coding: utf-8

"""
Copyright [2019] [RT Corporation]

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import math
import roslib
import rospy
import foodly_move_msgs
import actionlib
import geometry_msgs
from foodly_move_msgs.msg import Camera
from foodly_move_msgs.msg import Indicator
from foodly_move_msgs.msg import CancelPlan
from foodly_move_msgs.msg import Pose
from foodly_move_msgs.msg import EndeffectorAction
from foodly_move_msgs.msg import EndeffectorGoal
from foodly_move_msgs.msg import EndeffectorFeedback
from foodly_move_msgs.msg import EndeffectorResult
from foodly_move_msgs.msg import WaypointsAction
from foodly_move_msgs.msg import WaypointsGoal
from foodly_move_msgs.msg import WaypointsFeedback
from foodly_move_msgs.msg import WaypointsResult
from foodly_move_msgs.msg import MotionAction
from foodly_move_msgs.msg import MotionGoal
from foodly_move_msgs.msg import MotionResult
from foodly_move_msgs.msg import MotionFeedback
from foodly_move_msgs.srv import EefConfig, EefConfigRequest


def calc_camdeg2pos( degree ):
    CAMERA_BASE_X = 0.08195
    CAMERA_BASE_Z = 0.094
    NECK_BASE_Z = 0.474
    NECK_BASE_X = 0.105

    rad = -math.radians( degree )
    temp_x = CAMERA_BASE_X + 0.2
    temp_z = CAMERA_BASE_Z
    cam_x = ( temp_x * math.cos(rad) - temp_z * math.sin(rad) ) + NECK_BASE_X
    cam_z = ( temp_x * math.sin(rad) + temp_z * math.cos(rad) ) + NECK_BASE_Z
    return cam_x, cam_z


if __name__ == '__main__':


    rospy.init_node('move_api_example')

    motion_client = actionlib.SimpleActionClient('/foodly/dev_api/move/motion', MotionAction)
    motion_client.wait_for_server()

    way_client = [ actionlib.SimpleActionClient('/foodly/dev_api/move/waypoints/right', WaypointsAction), \
                            actionlib.SimpleActionClient('/foodly/dev_api/move/waypoints/left', WaypointsAction) ]
    way_client[0].wait_for_server()
    way_client[1].wait_for_server()

    eef_client = [ actionlib.SimpleActionClient('/foodly/dev_api/move/endeffector/right', EndeffectorAction), \
                            actionlib.SimpleActionClient('/foodly/dev_api/move/endeffector/left', EndeffectorAction) ]
    eef_client[0].wait_for_server()
    eef_client[1].wait_for_server()

    camera_pub = rospy.Publisher('/foodly/dev_api/move/camera', Camera, queue_size=1)
    indicator_pub = rospy.Publisher('/foodly/dev_api/move/indicator', Indicator, queue_size=1)
    cancel_plan_pub = rospy.Publisher('/foodly/dev_api/move/cancel_plan', CancelPlan, queue_size=1)

    eef_config_srv = rospy.ServiceProxy('/foodly/dev_api/move/eefconfig', EefConfig)
    eef_config_srv.wait_for_service()

    # Home motion
    motion_req = MotionGoal()
    motion_req.motion_id = 0
    motion_client.send_goal( motion_req )
    motion_client.wait_for_result(timeout=rospy.Duration(10.0))

    # Remove magnet (Right)
    motion_req = MotionGoal()
    motion_req.motion_id = 1
    motion_client.send_goal( motion_req )
    motion_client.wait_for_result(timeout=rospy.Duration(3.0))
    # Remove magnet (Left)
    motion_req = MotionGoal()
    motion_req.motion_id = 2
    motion_client.send_goal( motion_req )
    motion_client.wait_for_result(timeout=rospy.Duration(3.0))

    # Move camera
    cam_msg = Camera()
    cam_msg.position.y = 0.0
    cam_msg.position.x, cam_msg.position.z = calc_camdeg2pos(45)
    camera_pub.publish( cam_msg )
    rospy.sleep(2)

    # Indicator(Yellow blink) 
    indicator_msg = Indicator()
    indicator_msg.code = 138
    indicator_pub.publish(indicator_msg)

    # Constant values
    EEF_OPEN = 0
    EEF_CLOSE = 1
    RIGHT_ARM = 0
    LEFT_ARM = 1

    # EndeffectorConfig
    eefconfig_req = EefConfigRequest()
    eefconfig_req.part = RIGHT_ARM
    eefconfig_req.eef_length = 130 # [mm]
    eefconfig_req.food_size = 100  # [mm]
    eefconfig_req.close_size = 80  # [mm]
    eefconfig_req.grasp_torque = 3.0 # [N]
    eefconfig_req.grasp_threshold = 2.0 # [N]
    eef_config_srv(eefconfig_req)

    # Waypoints(Right)
    waypoints = []
    way = Pose()
    way.position.x = 0.4
    way.position.y = -0.160
    way.position.z = -0.14#0.0
    way.direction.x = 0.0
    way.direction.y = 0.0#-45.0
    way.direction.z = 0.0
    way.sec.data = rospy.Duration( 2.0 )
    way.eef = EEF_CLOSE
    waypoints.append( way )
    action_goal = WaypointsGoal()
    action_goal.waypoint = waypoints
    action_goal.part = RIGHT_ARM
    way_client[RIGHT_ARM].send_goal( action_goal )
    way_client[RIGHT_ARM].wait_for_result( rospy.Duration.from_sec(3.0) )
    if way_client[RIGHT_ARM].get_state() == actionlib.GoalStatus.SUCCEEDED:
        print("Succeeded!")
        rospy.sleep(2)#
    else:
        print("Aborted..")

    waypoints = []
    way = Pose()
    way.position.x = 0.4
    way.position.y = -0.1
    way.position.z = 0.3
    way.direction.x = 0.0
    way.direction.y = 0.0
    way.direction.z = 0.0
    way.sec.data = rospy.Duration( 2.0 )
    way.eef = EEF_OPEN
    waypoints.append( way )
    action_goal = WaypointsGoal()
    action_goal.waypoint = waypoints
    action_goal.part = RIGHT_ARM
    way_client[RIGHT_ARM].send_goal( action_goal )
    way_client[RIGHT_ARM].wait_for_result( rospy.Duration.from_sec(3.0) )
    if way_client[RIGHT_ARM].get_state() == actionlib.GoalStatus.SUCCEEDED:
        print("Succeeded!")
    else:
        print("Aborted..")

    # EndeffectorConfig
    eefconfig_req = EefConfigRequest()
    eefconfig_req.part = LEFT_ARM
    eefconfig_req.eef_length = 130 # [mm]
    eefconfig_req.food_size = 100  # [mm]
    eefconfig_req.close_size = 80  # [mm]
    eefconfig_req.grasp_torque = 3.0 # [N]
    eefconfig_req.grasp_threshold = 2.0 # [N]
    eef_config_srv(eefconfig_req)

    # Waypoints(Left)
    waypoints = []
    way = Pose()
    way.position.x = 0.4
    way.position.y = 0.160
    way.position.z = 0.0
    way.direction.x = 0.0
    way.direction.y = -45.0
    way.direction.z = 0.0
    way.sec.data = rospy.Duration( 2.0 )
    way.eef = EEF_CLOSE
    waypoints.append( way )
    action_goal = WaypointsGoal()
    action_goal.waypoint = waypoints
    action_goal.part = LEFT_ARM
    way_client[LEFT_ARM].send_goal( action_goal )
    way_client[LEFT_ARM].wait_for_result( rospy.Duration.from_sec(3.0) )
    if way_client[LEFT_ARM].get_state() == actionlib.GoalStatus.SUCCEEDED:
        print("Succeeded!")
    else:
        print("Aborted..")

    waypoints = []
    way = Pose()
    way.position.x = 0.4
    way.position.y = 0.1
    way.position.z = 0.3
    way.direction.x = 0.0
    way.direction.y = 0.0
    way.direction.z = 0.0
    way.sec.data = rospy.Duration( 2.0 )
    way.eef = EEF_OPEN
    waypoints.append( way )
    action_goal = WaypointsGoal()
    action_goal.waypoint = waypoints
    action_goal.part = LEFT_ARM
    way_client[LEFT_ARM].send_goal( action_goal )
    way_client[LEFT_ARM].wait_for_result( rospy.Duration.from_sec(3.0) )
    if way_client[LEFT_ARM].get_state() == actionlib.GoalStatus.SUCCEEDED:
        print("Succeeded!")
    else:
        print("Aborted..")


    # Home motion
    motion_req = MotionGoal()
    motion_req.motion_id = 0
    motion_client.send_goal( motion_req )
    motion_client.wait_for_result(timeout=rospy.Duration(10.0))

    # Move camera
    cam_msg = Camera()
    cam_msg.position.y = 0.0
    cam_msg.position.x, cam_msg.position.z = calc_camdeg2pos(0)
    camera_pub.publish( cam_msg )
    rospy.sleep(2)

    # Indicator(Green blink)
    indicator_msg = Indicator()
    indicator_msg.code = 73
    indicator_pub.publish(indicator_msg)



