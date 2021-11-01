#! /usr/bin/env python
# coding: utf-8

import math
import roslib
import rospy
import foodly_move_msgs
import actionlib
import geometry_msgs
import cv2
import numpy as np
import os, sys


import tensorflow as tf

from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge, CvBridgeError
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

# not required since this program does not need machine learning, it is only collecting data
# sys.path.append('./src/foodly_felix/Mask_RCNN/Mask_RCNN')
# from mrcnn import utils
# import mrcnn.model as modellib
# from mrcnn.model import log

# sys.path.append('./samples/blocks')
# import Blocks


def calc_camdeg2pos( degree ):
    # function provided by RT, uesd for conversion of angle when giving end rotations to robot head
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
        
        # used to estimate time it will take arm to get into position
        self.ARM_TIME = rospy.Duration(3.0)
        
        # used to estimate time it takes arm to descend and grab object
        self.ARM_OFFSET = rospy.Duration(0.5)
        
        # used to estimate position of object in space on a conveyor belt
        self.OBJ_VELOCITY = 0.01
        
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
        FOOD_SIZE = 100     # [mm]
        CLOSE_SIZE = 80     # [mm]
        GRASP_TORQUE = 3.0  # [N]
        GRASP_THRESH = 2.0  # [N]
    
        self.home_position
        
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
        
    def move_camera(self, cam_angle):
        # Move camera
        cam_msg = Camera()
        cam_msg.position.y = 0.0
        cam_msg.position.x, cam_msg.position.z = calc_camdeg2pos(cam_angle)
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
        way.sec.data = rospy.Duration( 1.0 )
        way.eef = eef_state
        
        return way
        
    def intercept(self, arm, obj_info):
        EEF_OPEN = 0
        EEF_CLOSE = 1
        
        scan_time, eef_pos, eef_dir = obj_info
        
        start_time = rospy.get_rostime()
        y_offset = ((start_time - scan_time + self.ARM_TIME + self.ARM_OFFSET) * self.OBJ_VELOCITY).to_sec()
        
        # move to above intercept position
        waypoints = []
        eef_intercept_pos = eef_pos
        eef_intercept_pos[1] += y_offset
        eef_intercept_pos[2] += 0.1
        
        way = self._create_way(eef_intercept_pos, eef_dir, EEF_OPEN)
           
        waypoints.append( way )
        
        action_goal = WaypointsGoal()
        action_goal.waypoint = waypoints
        action_goal.part = arm
        self.way_client[arm].send_goal( action_goal )
        self.way_client[arm].wait_for_result( self.ARM_TIME )
        if not self.way_client[arm].get_state() == actionlib.GoalStatus.SUCCEEDED:
            return False
            
        while rospy.get_rostime() < start_time + self.ARM_TIME:
            pass
        
        # lower arm around object
        waypoints = []
        eef_intercept_pos[2] -= 0.05
        way = self._create_way(eef_intercept_pos, eef_dir, EEF_CLOSE)
            
        waypoints.append( way )
        
        # raise grasped object up
        eef_intercept_pos[2] += 0.05
        way = self._create_way(eef_intercept_pos, eef_dir, EEF_CLOSE)
        
        waypoints.append( way )
        
        action_goal = WaypointsGoal()
        action_goal.waypoint = waypoints
        action_goal.part = arm
        self.way_client[arm].send_goal( action_goal )
        self.way_client[arm].wait_for_result( self.ARM_TIME )
        
        return True
        
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
        
        action = (action + 1) % 2
        
        # lower pincer around block
        way = self._create_way(eef_pos, eef_dir, action)
        waypoints.append( way )
        
        # raise above where block was
        way = self._create_way(eef_above_pos, eef_dir, action)
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

# unnecessary since we are only collecting data
# config = Blocks.BlocksConfig()
# class InferenceConfig(config.__class__):
#     GPU_COUNT = 1
#     IMAGES_PER_GPU = 1
#     DETECTION_MIN_CONFIDENCE = 0.9


class ProcessImages:
    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_subscriber = rospy.Subscriber('camera/color/image_raw', Image, self.img_callback, queue_size = 1)
        
        # we only need the rgb images as data
        # self.registered_pc_subscriber = rospy.Subscriber('camera/depth_registered/points', PointCloud2, self.pc_callback, queue_size = 1)
        # self.marker_publisher = rospy.Publisher("detected_centers", Marker, queue_size = 1)
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.last_img = None
        self.last_pc = None
        self.block_list = []
        self.id_counter = 0
        
        # unnecessary since we are only collecting data
        # self.model = None
        # self.load_model()
    
    def load_model(self):
        weights_path = './src/foodly_felix/Mask_RCNN/Mask_RCNN/logs/blocks20210810T1500/mask_rcnn_blocks_0004.h5'
        config = InferenceConfig()
        config.display()
        DEVICE = '/cpu:0'
        MODEL_DIR = './src/foodly_felix/Mask_RCNN/Mask_RCNN/logs'
        
        with tf.device(DEVICE):
            self.model = modellib.MaskRCNN(mode = "inference", model_dir = MODEL_DIR, config = config)
        self.model.load_weights(weights_path, by_name = True)
        self.model.keras_model._make_predict_function()
        
    def img_callback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("input image", img)
            # convert bgr image to rgb
            self.last_img = img[:, :, ::-1].copy()
            key = cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)
            
    def pc_callback(self, data):
        self.last_pc = data
    
    def create_marker(self, x, y, z):
        pos = PointStamped()
        pos.point.x = x
        pos.point.y = y
        pos.point.z = z

        obj_marker = Marker()
        obj_marker.header.frame_id = "base_link"
        obj_marker.header.stamp    = rospy.get_rostime()
        obj_marker.ns = "block"
        obj_marker.id = self.id_counter
        self.id_counter += 1
        
        obj_marker.type = 2 # sphere
        obj_marker.action = 0
        obj_marker.pose.position = pos.point
        obj_marker.pose.orientation.x = 0
        obj_marker.pose.orientation.y = 0
        obj_marker.pose.orientation.z = 0
        obj_marker.pose.orientation.w = 1.0
        obj_marker.scale.x = 0.01
        obj_marker.scale.y = 0.01
        obj_marker.scale.z = 0.01

        obj_marker.color.r = 0.0
        obj_marker.color.g = 1.0
        obj_marker.color.b = 0.0
        obj_marker.color.a = 1.0

        obj_marker.lifetime = rospy.Duration(0)
        
        self.marker_publisher.publish(obj_marker)

        
    def find_blocks(self):
        if self.model is not None and self.last_pc is not None and self.last_img is not None:
            print("entered find blocks")
            start_time = rospy.get_rostime()
            try:
                trans = self.tf_buffer.lookup_transform("base_link", self.last_pc.header.frame_id, self.last_pc.header.stamp, rospy.Duration(10.0))
            except tf2.LookupException as ex:
                rospy.logwarn(ex)
                return
            except tf2.LookupException as ex:
                rospy.logwarn(ex)
                return
                
            trans_cloud = do_transform_cloud(self.last_pc, trans)
            pc_gen = pc2.read_points(trans_cloud, skip_nans=False)
            np_scene = np.zeros((360, 640, 3), dtype=np.float32)
            
            for i, point in enumerate(pc_gen):
                x, y, z, _ = point
                if not np.isnan(x):
                    np_scene[i // 640, i % 640] = x, y, z
                    
            output_img = self.last_img.copy()
            output_img2 = self.last_img.copy()
            results = self.model.detect([self.last_img])[0]
            rois = results["rois"]
            masks = results["masks"]
            class_ids = results["class_ids"]
            scores = results["scores"]
            
            for mask_idx in range(masks.shape[-1]):
                # draw mask prediction from Mask R-CNN on output image
                output_img2 = np.uint8(np.where(masks[:, :, mask_idx, None], np.random.randint(256, size=3), output_img2))

                # associate pixels in mask to their 3D positions (N, 3)
                points = np.where(masks[:, :, mask_idx])
                points_space = np_scene[points]
                points_space = points_space[~np.all(points_space == 0, axis=1)]
                points_space = points_space[np.all((-0.6 < points_space) & (points_space < 0.6), axis=1)]
                
                # find minimum bounding box for each mask on rgb image
                points_pairs = np.column_stack(points)
                rect_img = cv2.minAreaRect(points_pairs[:, ::-1])
                (x, y), (w, h), angle = rect_img
                x = int(x)
                y = int(y)
                # print(angle, w, h)
                if w > h:
                    angle += 90
                
                # draw minimum bounding box on output image
                box = cv2.boxPoints(rect_img)
                box = np.int32(box)
                cv2.drawContours(output_img, [box], 0, (0, 255, 0), 2)
                cv2.circle(output_img, (x, y), 3, (0, 0, 255), 1)
                
                # find minimum bounding box for points in space
                if len(points_space) > 0:
                    scene_x = np.average(points_space, axis = 0)[0]
                    scene_y = np.average(points_space, axis = 0)[1]
                    scene_z = np.average(points_space, axis = 0)[2]
                    if scene_z < 0.2:
                        self.block_list.append((start_time, [scene_x, scene_y, scene_z], [0.0, 0.0, -angle]))
                        self.create_marker(scene_x, scene_y, scene_z)

            cv2.imshow("processed image", output_img[:, :, ::-1])
            cv2.imshow("masks image", output_img2[:, :, ::-1])



if __name__ == '__main__':
    RIGHT_ARM = 0
    LEFT_ARM = 1

    rospy.init_node('block_picker')
    
    image_processor = ProcessImages()
    motion_controller = MotionControl()
    
    # move the arms out of view of the Realsense
    # remove this if you want them present in data, as if they aren't they may be
    # misidentified later on
    motion_controller.drop(RIGHT_ARM, [0.35, -0.3, 0.1])
    motion_controller.drop(LEFT_ARM, [0.35, 0.3, 0.1])

    print("while OpenCV image window is selected")
    print("press a to save image")
    print("press q to quit")
    
    i = 0
    while True:
        key = cv2.waitKey(10)
        if key == ord('a'):
           print("image_{} saved".format(i))
           i += 1
           img = image_processor.last_img[:, :, ::-1].copy()
           cv2.imwrite("image_{}.jpg".format(i), img)
        if key == ord('q'):
            break
                   

    motion_controller.home_position()
    motion_controller.indicator(73)
    

    cv2.destroyAllWindows()




