#! /usr/bin/env python
# coding: utf-8

import math
import roslib
import rospy
import foodly_move_msgs
import actionlib
import geometry_msgs
import numpy as np
import os, sys

from std_msgs.msg import Header
import tf2_ros
import tf2_py as tf2

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


class MotionControl:
    def __init__(self):
        self.motion_client = actionlib.SimpleActionClient('/foodly/dev_api/move/motion', MotionAction)
        self.motion_client.wait_for_server()

        self.way_client = [ actionlib.SimpleActionClient('/foodly/dev_api/move/waypoints/right', WaypointsAction), \
                                actionlib.SimpleActionClient('/foodly/dev_api/move/waypoints/left', WaypointsAction) ]
        self.way_client[0].wait_for_server()
        self.way_client[1].wait_for_server()

        self.eef_client = [ actionlib.SimpleActionClient('/foodly/dev_api/move/endeffector/right', EndeffectorAction), \
                                actionlib.SimpleActionClient('/foodly/dev_api/move/endeffector/left', EndeffectorAction) ]
        self.eef_client[0].wait_for_server()
        self.eef_client[1].wait_for_server()

        self.camera_pub = rospy.Publisher('/foodly/dev_api/move/camera', Camera, queue_size=1)
        self.indicator_pub = rospy.Publisher('/foodly/dev_api/move/indicator', Indicator, queue_size=1)
        self.cancel_plan_pub = rospy.Publisher('/foodly/dev_api/move/cancel_plan', CancelPlan, queue_size=1)
        
        self.eef_config_srv = rospy.ServiceProxy('/foodly/dev_api/move/eefconfig', EefConfig)
        self.eef_config_srv.wait_for_service()
        
        self.prepare_arms()
        self.move_camera(75)
        self.indicator(138)
        rospy.sleep(2)
        print("Startup complete")
        
    def indicator(self, code):
        # Indicator Codes: 138 = Yellow blink, 73 = Green blink 
        indicator_msg = Indicator()
        indicator_msg.code = code
        self.indicator_pub.publish(indicator_msg)
    
    def home_position(self):
        # Home motion
        self.move_camera(0)
        motion_req = MotionGoal()
        motion_req.motion_id = 0
        self.motion_client.send_goal( motion_req )
        self.motion_client.wait_for_result(timeout=rospy.Duration(10.0))
    
    def prepare_arms(self):
        RIGHT_ARM = 0
        LEFT_ARM = 1
        EEF_LENGTH = 130    # [mm]
        FOOD_SIZE = 70      # [mm]
        CLOSE_SIZE = 60     # [mm]
        GRASP_TORQUE = 3.0  # [N]
        GRASP_THRESH = 2.0  # [N]
    
        self.home_position()
        
        # EndeffectorConfig
        eefconfig_req = EefConfigRequest()
        eefconfig_req.part = RIGHT_ARM
        eefconfig_req.eef_length = EEF_LENGTH
        eefconfig_req.food_size = FOOD_SIZE
        eefconfig_req.close_size = CLOSE_SIZE
        eefconfig_req.grasp_torque = GRASP_TORQUE
        eefconfig_req.grasp_threshold = GRASP_THRESH
        self.eef_config_srv(eefconfig_req)
    
        # EndeffectorConfig
        eefconfig_req = EefConfigRequest()
        eefconfig_req.part = LEFT_ARM
        eefconfig_req.eef_length = EEF_LENGTH
        eefconfig_req.food_size = FOOD_SIZE
        eefconfig_req.close_size = CLOSE_SIZE
        eefconfig_req.grasp_torque = GRASP_TORQUE
        eefconfig_req.grasp_threshold = GRASP_THRESH
        self.eef_config_srv(eefconfig_req)
    
        # Remove magnet (Right)
        motion_req = MotionGoal()
        motion_req.motion_id = 1
        self.motion_client.send_goal( motion_req )
        self.motion_client.wait_for_result(timeout=rospy.Duration(3.0))
        
        # Remove magnet (Left)
        motion_req = MotionGoal()
        motion_req.motion_id = 2
        self.motion_client.send_goal( motion_req )
        self.motion_client.wait_for_result(timeout=rospy.Duration(3.0))
    
    def _calc_camdeg2pos(self, degree):
        CAMERA_BASE_X = 0.08195
        CAMERA_BASE_Z = 0.094
        NECK_BASE_Z = 0.474
        NECK_BASE_X = 0.105

        rad = -math.radians(degree)
        temp_x = CAMERA_BASE_X + 0.2
        temp_z = CAMERA_BASE_Z
        cam_x = (temp_x * math.cos(rad) - temp_z * math.sin(rad)) + NECK_BASE_X
        cam_z = (temp_x * math.sin(rad) + temp_z * math.cos(rad)) + NECK_BASE_Z
        return cam_x, cam_z
    
    def move_camera(self, cam_angle):
        # Move camera
        cam_msg = Camera()
        cam_msg.position.y = 0.0
        cam_msg.position.x, cam_msg.position.z = self._calc_camdeg2pos(cam_angle)
        self.camera_pub.publish( cam_msg )
        
    def _create_way(self, eef_pos, eef_dir, eef_state):
        # Fill in all the data for each waypoint Pose
        way = Pose()
        way.position.x = eef_pos[0]
        way.position.y = eef_pos[1]
        way.position.z = eef_pos[2]
        way.direction.x = eef_dir[0]
        way.direction.y = eef_dir[1]
        way.direction.z = eef_dir[2]
        way.sec.data = rospy.Duration(2.0)
        way.eef = eef_state
        
        return way
        
    def pick_place(self, arm, eef_pos, eef_dir, action):
        if action != 1 and action != 0:
            print("Unrecognised Action")
            return
            
        EEF_OPEN = 0
        EEF_CLOSE = 1
        PICK = 0
        PLACE = 1
             
        waypoints = []
        
        # move over block
        eef_above_pos = eef_pos[:]
        eef_above_pos[2] += 0.1
        way = self._create_way(eef_above_pos, eef_dir, action)
        waypoints.append( way )
        
        # lower pincer around block
        way = self._create_way(eef_pos, eef_dir, action)
        waypoints.append( way )

        action = (action + 1) % 2
        
        rospy.sleep(1)
        way = self._create_way(eef_pos, eef_dir, action)
        waypoints.append( way )
        
        # raise above where block was
        way = self._create_way(eef_above_pos, eef_dir, action)
        waypoints.append( way )
        
        action_goal = WaypointsGoal()
        action_goal.waypoint = waypoints
        action_goal.part = arm
        self.way_client[arm].send_goal( action_goal )
        self.way_client[arm].wait_for_result( rospy.Duration.from_sec(20.0) )
        if self.way_client[arm].get_state() != actionlib.GoalStatus.SUCCEEDED:
            print("Aborted..")
            
    def drop(self, arm, eef_pos):
        EEF_OPEN = 0
        EEF_CLOSE = 1
        waypoints = []
        way = self._create_way(eef_pos, [0.0, 0.0, 0.0], EEF_OPEN)
        waypoints.append( way )
        
        action_goal = WaypointsGoal()
        action_goal.waypoint = waypoints
        action_goal.part = arm
        self.way_client[arm].send_goal( action_goal )
        self.way_client[arm].wait_for_result( rospy.Duration.from_sec(10.0) )
        if self.way_client[arm].get_state() == actionlib.GoalStatus.SUCCEEDED:
            print("Succeeded!")
        else:
            print("Aborted..")


if __name__ == '__main__':
    EEF_OPEN = 0
    EEF_CLOSE = 1
    RIGHT_ARM = 0
    LEFT_ARM = 1
    PICK = 0
    PLACE = 1

    rospy.init_node('block_picker')
    
    motion_controller = MotionControl()
    for i in range(10):
        motion_controller.pick_place(LEFT_ARM, [0.45, 0.205, -0.155], [0.0, 0.0, 0.0], PICK)
        motion_controller.pick_place(LEFT_ARM, [0.35, 0.0, -0.155], [0.0, 0.0, 0.0], PLACE)
        motion_controller.pick_place(LEFT_ARM, [0.35, 0.0, -0.155], [0.0, 0.0, 90.0], PICK)
        motion_controller.pick_place(LEFT_ARM, [0.45, 0.205, -0.155], [0.0, 0.0, 90.0], PLACE)
        print("Completed iteration: {}".format(i))
        
    motion_controller.home_position()
    motion_controller.indicator(73)
    




