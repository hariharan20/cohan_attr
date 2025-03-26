#! /usr/bin/env python
import rospy
from cohan_msgs.msg import Trajectory, AgentTrajectoryArray, TrackedAgents
import numpy as np
from nav_msgs.msg import OccupancyGrid
import scipy
from sensor_msgs.msg import Image
import tf
from geometry_msgs.msg import Quaternion
from rosgraph_msgs.msg import Clock
import math
from std_msgs.msg import Float64, String
import json
import rospkg
import time 
from ultralytics import YOLO
# from mediapipe import *
import mediapipe as mp
from cohan_attr.msg import attr
from cohan_msgs.msg import TrackedAgents , AgentPathArray, CrossingInfo
from geometry_msgs.msg import Pose, PoseArray , PoseStamped
import time
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from scipy.spatial import KDTree
import copy


LEN_OF_HUMAN_TRAJ = 10

fd = mp.solutions.face_detection
from cv_bridge import CvBridge
bridge = CvBridge()


angle_dict ={
    'follow_and_by_l' : [30 , 'left' , 'behind'],
    'follow_and_by_r' : [330, 'right' , 'behind'],
    'by_r' : [210 , 'right' , 'front'],
    'by_l' : [150 , 'left' , 'front'],
    'cross_behind_from_l' : [240 , 'left' , 'behind'], 
    'cross_behind_from_r' : [120 , 'right' , 'behind'],
    'cross_in_front_from_l' : [300 , 'left' , 'front'],
    'cross_in_front_from_r' : [60 , 'right' , 'front'],
    # 'collision_left' : [270 , 'left' , 'front'],
    # 'collision_right' : [90 , 'right' , 'front'],
    # 'collision_behind' : [180 , 'left' , 'front'],
    # 'collision_front' : [0 , 'right' , 'front'],
    # 'avoid_left' : [270 , 'left', 'behind'],
    # 'avoid_right' : [90 , 'right', 'behind'],
    # 'avoid_behind' : [180 , 'left', 'behind'],
    # 'avoid_front' : [0 , 'right', 'behind'],
}

position_dict = {
    ('left' , 'behind') : {
        1 : 'follow_and_by_l',
        90 : 'cross_behind_from_l',
        180 : 'by_l',
        270 : 'cross_behind_from_r',
        359 : 'follow_and_by_l',
    },
    ('right' , 'behind') : {
        1 : 'follow_and_by_r',
        90 : 'cross_behind_from_l',
        180 : 'by_r',
        270 : 'cross_behind_from_r',
        359 : 'follow_and_by_r',
    },
    ('left' , 'front') : {
        1 : 'follow_and_by_l',
        90 : 'cross_in_front_from_l',
        180 : 'by_l',
        270 : 'cross_in_front_from_r',
        359 : 'follow_and_by_l',

    },
    ('right' , 'front') : {
        0 : 'follow_and_by_r',
        90 : 'crossin_front_from_l',
        180 : 'by_r',
        270 : 'cross_in_front_from_r',
        359 : 'follow_and_by_r',
    },
}

# position_dict_keys = list(position_dict.keys())

def rad_to_deg2(rad) : 
    if rad < 0 : 
        return 360 + (rad *180 / math.pi)
    return rad * 180 / math.pi

def get_angle_condition(behind_or_front , left_or_right):
    list_of_angle = []
    list_of_condition = []
    for key , value in angle_dict.items():
        if value[1] == left_or_right and value[2] == behind_or_front:
            list_of_angle.append(value[0]) 
            list_of_condition.append(key)
    return list_of_angle , list_of_condition

def get_direction_from_position_dict(behind_or_front , left_or_right , angle_of_robot_wrt_human):
    angle_dict = position_dict[(left_or_right , behind_or_front)]
    angle_dict_keys = list(angle_dict.keys())
    angle_dict_keys = np.array(angle_dict_keys)
    angle_difference = np.abs(angle_dict_keys - angle_of_robot_wrt_human)
    min_index = np.argmin(angle_difference)
    return angle_dict[angle_dict_keys[min_index]]


def get_pose_wrt_human(robot_position , human_position , human_heading_dx , human_heading_dy):
    d1 = (robot_position[0] - human_position[0]) * (human_heading_dy) - (robot_position[1] - human_position[1]) * (human_heading_dx)
    d1_orthogonal = (robot_position[0] - human_position[0]) * (-human_heading_dx) - (robot_position[1] - human_position[1]) * (human_heading_dy)
    if d1 > 0:
        left_or_right = 'right'
    else:
        left_or_right = 'left'
    if d1_orthogonal > 0:
        front_or_behind = 'behind'
    else:
        front_or_behind = 'front'
    return left_or_right, front_or_behind


# import ros_numpy 
def quat_to_euler(w , z):
    euler_angles = tf.transformations.euler_from_quaternion([0 , 0  , z , w])
    return euler_angles[2]

def rad_to_deg(theta):
    return ((180/3.14)*theta)
import cv2
class cohan_attr:
    def __init__(self):
        self.last_agent_data = rospy.Time.now()
        self.map =None
        self.trigger_time = rospy.get_param("robot_trigger_time" , 4.0)
        self.trigger_distance_to_door = rospy.get_param("robot_convo_trigger_distance" , 2.0)
        self.grid_half_size = 30
        ros_pack = rospkg.RosPack()
        self.attr_msg = attr()
        self.attr_pub = rospy.Publisher('/cohan_attr/attr' , attr , queue_size= 1)
        # self.alert_pub = rospy.Publisher('cohan_attr/attr' , attr , queue_size= 1 , latch=True)

        # self.img_pub = rospy.Publisher('/map_image' , Image , queue_size =10, latch=True)
        self.angle_pub = rospy.Publisher('/angle', Float64 , queue_size=10, latch=True)
        self.clock_flag = False
        self.door_centers  =[]
        rospy.set_param('start_convo' , False)
        # self.start_convo = False
        locations = json.load(open(ros_pack.get_path('cohan_attr')  + '/config/locations.json'))
        for location in locations['map']: 
            if ('enter' in location['name']) or ('exit' in location['name']):
                self.door_centers.append(location['pose']['center'])
        yolo_model = 'yolov8s.pt'
        # yolo_model = 'yolov8n.pt'
        self.yolo = YOLO(yolo_model)
        # rospy.set_param('check_for_humans' ,False)
        self.mp_face_detection = fd
        self.last_image_sent = time.time()
        self.publish_image = True
        self.last_start_convo = time.time()
        self.img_pub = rospy.Publisher('/cohan_attr/human_image'  , Image , queue_size=1, latch=False)
        self.pub = rospy.Publisher('/tracked_agents_pose' , PoseStamped   , queue_size=10 )
        self.global_poly_model = None
        self.robot_global_poly_model = None
        self.local_poly_model = None
        self.tree = None
        self.human_tree = None
        self.attr_image_np = np.ones((500 , 500 , 3) , dtype=np.uint8) * 255
        cv2.putText(self.attr_image_np , 'Attributes' , (100 , 250) , cv2.FONT_HERSHEY_SIMPLEX , 1 , (0,0,0) , 2)
        self.image_msg = bridge.cv2_to_imgmsg(self.attr_image_np ,  encoding="rgb8")
        self.image_pub = rospy.Publisher('attributes_from_cohan' , Image , queue_size=1 , latch=True)
        self.image_pub.publish(self.image_msg)

        rospy.set_param('reset_human_traj_record' , True)
        rospy.Subscriber('move_base/HATebLocalPlannerROS/agents_local_trajs' , AgentTrajectoryArray, self.agent_cb )
        rospy.Subscriber('move_base/HATebLocalPlannerROS/crossing_info' , CrossingInfo, self.crossing_cb )
        rospy.Subscriber('/tracked_agents' , TrackedAgents , self.tracked_agents_cb)    
        rospy.Subscriber('/l515/color/image_raw' , Image , self.image_cb)
        # rospy.Subscriber('/move_base/HATebLocalPlannerROS/local_traj' , Trajectory , self.robot_cb)
        rospy.Subscriber('/move_base/HATebLocalPlannerROS/local_traj' , Trajectory , self.ecohan_cb)
        # rospy.Subscriber('/move_base/HATebLocalPlannerROS/local_plan' , Path , self.robot_plan_cb)
        rospy.Subscriber('/move_base/global_costmap/costmap' , OccupancyGrid , self.obs_cb)
        # rospy.Subscriber('/move_base/HATebLocalPlannerROS/agents_global_plans' , AgentPathArray , self.agents_global_plans_cb) 
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_cb)
        rospy.Subscriber('/clock' , Clock , self.flag_checker)
        rospy.Timer(rospy.Duration(0.1), self.check_the_plans)
        self.pose_msg = PoseStamped()
        self.tracked_agent_data = None
        self.goal_set = False
        self.publish_analysis = False
        self.pose_msg_array = []
        self.crossing_info = CrossingInfo()
        self.crossing_points_array = []
        self.human_crossing_points_array = []   
        self.last_crossing_check = time.time()
        self.agent_radius = rospy.get_param('/move_base/HATebLocalPlannerROS/agent_radius')
        self.robot_radius = rospy.get_param('/move_base/HATebLocalPlannerROS/robot_radius')
        self.marker_pub = rospy.Publisher('/cylinder_marker', Marker, queue_size=10)

        # rospy.Subscriber('/clock' , Clock , self.clock_cb )
        
    def crossing_cb(self, msg):
        self.crossing_info = msg

    def goal_cb(self, msg):
        self.goal_set = True
        
    def agents_global_plans_cb(self , data):
        self.human_global_plan = self.human_path_extractor(data)
        self.human_tree = KDTree(self.human_global_plan)
        # self.human_global_plan = self.path_extractor(data)
        # print(np.array(self.human_global_plan)[: , 1].shape)
        human_plan_np = np.array(self.human_global_plan)
        self.global_poly_model, residuals , _ ,  _ , _ = np.polyfit(human_plan_np[: , 0] , human_plan_np[: , 1] , 3 , full=True)
    def publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"  # Change to "odom" or "base_link" as needed
        marker.header.stamp = rospy.Time.now()
        marker.ns = "cylinder"
        marker.id = 0
        marker.type = Marker.CYLINDER  # Cylinder shape
        marker.action = Marker.ADD

        # Position & Orientation
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.0  # Center of the cylinder
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Cylinder scale (diameter and height)
        marker.scale.x = 0.2 # Diameter
        marker.scale.y = 0.2  # Diameter
        marker.scale.z = 1.0  # Height

        # Color (RGBA)
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 1.0  # Fully opaque

        marker.lifetime = rospy.Duration(1)  # Permanent
        self.marker_pub.publish(marker)
    
    def path_extractor(self, plan):
        x = []
        y = []
        # print(len(plan.paths))
        for pose in plan.paths[0].path.poses : 
            x.append(pose.pose.position.x)
            y.append(pose.pose.position.y)
        return x , y
    
    def human_path_extractor(self , plan):
        poses = []
        for pose in plan.paths[0].path.poses :
            poses.append([pose.pose.position.x, pose.pose.position.y])
        return poses

    def robot_path_extractor(self, plan):
        x = []
        y = []
        poses = []
        # print(len(plan.paths))
        for pose in plan.poses : 
            x.append(pose.pose.position.x)
            y.append(pose.pose.position.y)
            poses.append([pose.pose.position.x, pose.pose.position.y])
        return poses
    
    def detect_outliers_iqr(self, data):
        Q1 = np.percentile(data, 25)
        Q3 = np.percentile(data, 75)
        IQR = Q3 - Q1
        lower_bound = Q1 - 1.5 * IQR
        upper_bound = Q3 + 1.5 * IQR
        return [x for x in data if x < lower_bound or x > upper_bound]



    def human_contribution(self ):
        # print('INSIDE HUMAN CONTRIBUTION')
        if self.crossing_info.indices:
            cp = np.array([self.human_local_plan[0][self.crossing_info.indices[0]] , self.human_local_plan[1][self.crossing_info.indices[0]]])
            distance, index = self.human_tree.query((cp[0], cp[1]))
            gp = np.array(self.human_global_plan[index])
            # print(cp , gp)
            self.human_crossing_points_array.append(np.linalg.norm(gp-cp))

            # print(self.human_crossing_points_array)
            if len(self.human_crossing_points_array) == 12: 
                human_crossing_point_copy = copy.deepcopy(self.human_crossing_points_array)
                outliers = self.detect_outliers_iqr(human_crossing_point_copy)
                print(outliers)
                for outlier in outliers : 
                    outlier_index = human_crossing_point_copy.index(outlier)
                    human_crossing_point_copy.pop(outlier_index)            
                diff = max(human_crossing_point_copy[3 : ]) - min(human_crossing_point_copy[3 : ])
                if diff > 0.3:
                    print("The Human needs to contribute more")
                    print(diff)
                elif diff <= 0.3:
                    print("The Human has contributed as planned")
                    print(diff)
                
    def tracked_agents_cb(self , data):
        # self.pose_msg_array = []
        self.tracked_agent_data = data 
        # if rospy.get_param('reset_human_traj_record' , False) and 
        if self.goal_set:
            self.human_crossing_points_array = []   
            self.pose_msg_array = []
            self.crossing_points_array = []
            self.robot_global_plan = []
            self.tree = None
            # print(np.sqrt(residuals)/len(self.human_global_plan[0]))
            # print(np.array(self.human_global_plan).shape)/
            self.robot_global_plan = self.robot_path_extractor(rospy.wait_for_message('/move_base/HATebLocalPlannerROS/global_plan', Path ))
            # self.robot_global_poly_model, residuals , _ ,  _ , _ = np.polyfit(self.robot_global_plan[0] , self.robot_global_plan[1] , 3 , full=True)
            self.tree = KDTree(self.robot_global_plan)
            # print(np.array(self.robot_global_plan).shape)
            # print(np.sqrt(residuals)/len(self.robot_global_plan[0]))
            
            time.sleep(1)
            self.initial_time = time.time()
            # print(np.sqrt(residuals)/len(self.human_local_plan[0]))
            # rospy.set_param('reset_human_traj_record' , False)
            # print(self.human_tree) 
            self.goal_set = False
            self.publish_analysis = True
            time.sleep(1)
        if self.publish_analysis :
            self.human_global_plan = self.human_path_extractor(rospy.wait_for_message('/move_base/HATebLocalPlannerROS/agents_global_plans' , AgentPathArray))
            self.human_tree = KDTree(self.human_global_plan)
            human_plan_np = np.array(self.human_global_plan)
            self.global_poly_model, residuals , _ ,  _ , _ = np.polyfit(human_plan_np[: , 0] , human_plan_np[: , 1] , 3 , full=True)
            self.human_local_plan = self.path_extractor(rospy.wait_for_message('/move_base/HATebLocalPlannerROS/agents_local_plans' , AgentPathArray ))
            self.local_poly_model  , residuals , _ ,  _ , _= np.polyfit(self.human_local_plan[0] , self.human_local_plan[1] , 3 , full=True)
            self.human_contribution()
            
        if len(self.pose_msg_array) == LEN_OF_HUMAN_TRAJ:
            self.pose_msg_array.pop(0)
        self.pose_msg.header.stamp = rospy.Time.now()
        self.pose_msg.header.frame_id = 'map'
        self.pose_msg.pose.position.x = data.agents[1].segments[0].pose.pose.position.x
        self.pose_msg.pose.position.y = data.agents[1].segments[0].pose.pose.position.y
        self.pose_msg.pose.orientation.z = data.agents[1].segments[0].pose.pose.orientation.z
        self.pose_msg.pose.orientation.w = data.agents[1].segments[0].pose.pose.orientation.w
        # self.pub.publish(self.pose_msg)
        self.pose_msg_array.append([data.agents[1].segments[0].pose.pose.position.x , data.agents[1].segments[0].pose.pose.position.y])

    def robot_plan_cb(self, msg):
        if self.crossing_info.indices:
            if self.crossing_info.indices[0] < 10 and not self.goal_set:
                cp = np.array([msg.poses[0].pose.position.x, msg.poses[0].pose.position.y])
                # print(cp)
                distance, index = self.tree.query((cp[0], cp[1]))
                gp = np.array(self.robot_global_plan[index])
                self.crossing_points_array.append(np.linalg.norm(gp-cp))
                self.last_crossing_check = time.time()
        else:
            if time.time() - self.last_crossing_check > 2.0 and self.crossing_points_array:
               diff = max(self.crossing_points_array) - min(self.crossing_points_array)
               if diff > 0.3:
                   print("The robot has contributed more than planned")
                   print(diff)
               elif diff <= 0.3:
                   print("The robot moved as planned!")
                   print(diff)
                   
               self.crossing_points_array = []
        

    def check_the_plans(self , _):
        try :
            if len(np.array(self.pose_msg_array).shape) == 2: 
                recorded_path = np.array(self.pose_msg_array)
                y_global_pred = np.polyval(self.global_poly_model , recorded_path[:,0])
                y_local_pred = np.polyval(self.local_poly_model , recorded_path[:,0])
                error_global = np.mean(np.abs(y_global_pred - recorded_path[:,1]))
                error_local = np.mean(np.abs(y_local_pred - recorded_path[:,1]))
                # print("Global Error : " , error_global)
                # print("Local Error : " , error_local)
                compliant_human = False
                if error_local < 0.7 and error_global > 0.7 :
                    compliant_human = True 
                    # rospy.set_param('compliant_human' , True)
                # elif error_local < 0.7: 
                elif error_local > 0.7 and error_global < 0.7 :
                    compliant_human = False 
                    # rospy.set_param('compliant_human' , False)
                if self.publish_analysis and (time.time() - self.initial_time > 2):
                    self.publish_analysis = False
                    self.attr_msg.compliant_human = compliant_human
                    # print(self.attr_msg)
                    self.attr_pub.publish(self.attr_msg)
                    # rospy.set_param('reset_human_traj_record' , True)
        except :
            pass


    def is_face_visible(self  , input_image) :
        with self.mp_face_detection.FaceDetection(model_selection=1 , min_detection_confidence=0.5 ) as face_detection : 
            results = face_detection.process(np.ascontiguousarray(input_image))
        face_detected = False
        if results.detections : 
            face_detected = True
        return face_detected 
    
    def flag_checker(self , _ ):
        if (time.time() - self.last_image_sent> 5.0) or rospy.get_param('check_for_humans' , False) : 
            self.publish_image = True
            # print('PUBLISH IMAGE SET TO TRUE')
        else : 
            self.publish_image = False 

    def image_cb(self , data):
        img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        # if rospy.get_param('check_for_humans' ,False):
            # if True : 
        result =  self.yolo(img , show= False , verbose=False)
        boxes = result[0].boxes.xyxy.tolist()
        classes = result[0].boxes.cls.tolist()
        names = result[0].names
        confidences = result[0].boxes.conf.tolist()
        human_bbs = []
        human_confs = []
        # Iterate through the results
        for box, cls, conf in zip(boxes, classes, confidences):
            if cls == 0 : 
                human_bbs.append(box)
                human_confs.append(conf)
        if len(human_confs) > 0 :
            confi_id = np.argmin(human_confs)
            # print( human_bbs[confi_id])
            [x_min , y_min , x_max , y_max] = human_bbs[confi_id]
            # print(img.shape)
            # print(img[math.floor(y_min) : math.floor(y_max) , math.floor(x_min) : math.floor(x_max),  : ].shape)
            # print(x_max - x_min , y_max - y_min )
            cropped_image = img[math.floor(y_min) : math.floor(y_max) , math.floor(x_min) : math.floor(x_max)]
            # rospy.logerr('CROPPED IMAGE')
            if (x_max - x_min) > 250 and (y_max - y_min) > 600: 
                if self.publish_image: 
                    if self.is_face_visible(cropped_image):
                        rospy.logerr('FACE VISIBLE')
                        img_msg = bridge.cv2_to_imgmsg(cropped_image ,  encoding="rgb8")
                        self.img_pub.publish(img_msg)
                        # self.alert_pub.publish(String("{data: '{\"bottleneck\": true, \"dialogue\": \"I will pass closely on your left\"}'}"))
                        self.publish_image = False
                        print('published image')
                        rospy.set_param('human_detected' , True)
                        self.last_image_sent = time.time()
                        rospy.set_param('check_for_humans' , False)


    def obs_cb(self, data):
        print(data.info)
        print(np.unique(self.map))
        self.resolution  = data.info.resolution
        self.map_width = data.info.width
        self.map_height = data.info.height
    
    def clock_cb(self , clock_data):
        self.clock_flag = True
        rospy.sleep(2)
        self.clock_flag = False

    def agent_cb(self, data):
        self.last_agent_data =  data.header.stamp
        agent_trajs = data.trajectories
        self.agent_trajs_arr = []
        for agent_traj in agent_trajs :
            self.agent_tfs_arr = []
            self.agent_pts_arr = []
            self.agent_orientation_arr = []
            for i , points in enumerate(agent_traj.trajectory.points ):
                if points.time_from_start > rospy.Duration(0.0):
                    self.agent_tfs_arr.append(points.time_from_start.to_sec())
                    agent_orientation = quat_to_euler(points.transform.rotation.z , points.transform.rotation.w)
                    self.agent_pts_arr.append([points.transform.translation.x , points.transform.translation.y ])
                    self.agent_orientation_arr.append(agent_orientation)
            self.agent_trajs_arr.append([self.agent_tfs_arr, self.agent_pts_arr , self.agent_orientation_arr])
        # print(len(self.agent_trajs_arr))
    
    # def min_distance_calc(self , arr1 , arr2) : 
    #     arr1_np =  np.array(arr1)
    #     arr2_np =  np.array(arr2)
    #     min_index = 1000
    #     min_distance = 1000
    #     id_ = len(arr2_np.shape) -1
    #     for i , arr1_np_ in enumerate(arr1_np):
    #         distance = np.linalg.norm(arr1_np_ - arr2_np , axis = id_)
    #         if np.min(distance) < min_distance :
    #             min_index = i
    #             min_agent_index = np.argmin(distance)
    #             min_distance = np.min(distance)
    #     return min_index , min_distance , min_agent_index
        
    # def avg_slope(self, pts):
    #     pts = np.array(pts)
    #     if not len(pts.shape) == 2:
    #         return None 
    #     # print(pts[: , 1].shape)
    #     result = scipy.stats.linregress(pts[:,0] , pts[:,1])
    #     return result.slope

    # def direction_of_crossing_static(self , robot_pts_arr , robot_index , human_pose , time_to_nearest_pose , min_distance):
    #     robot_pts_slice = robot_pts_arr[robot_index-10 : robot_index]
    #     robot_pts_slice_np = np.array(robot_pts_slice)
    #     robot_slope = self.avg_slope(robot_pts_slice)
    #     if not type(robot_slope) == type(None):
    #         if not np.isnan(robot_slope): 
    #             robot_pts_wrt_human = robot_pts_slice_np - np.array([human_pose[0] , human_pose[1]])
    #             human_slope = human_pose[2]
    #             # print(human_slope)
                
    #             slope_difference = rad_to_euler(math.atan(robot_slope) - human_slope) 
    #             # slope_difference = math.atan(robot_slope) - human_slope
    #             self.angle_pub.publish(Float64(rad_to_deg(slope_difference)))
    #             # text = self.slope_conditions(rad_to_deg(slope_difference) , robot_pts_wrt_human)
    #             self.attr_msg.direction_of_crossing = text
    #             if self.crossing_info:
    #                 if self.crossing_info.times:
    #                     self.attr_msg.distance_while_crossing = self.crossing_info.distances[0] - (self.agent_radius + self.robot_radius)
    #                     self.attr_msg.time_to_cross = self.crossing_info.times[0]
    #             # self.attr_pub.publish(self.attr_msg)
    #             full_text = str(round(time_to_nearest_pose , 2)) + " secs | " + str(round(min_distance , 2)) + "m | " + text
    #             # if round(time_to_nearest_pose , 0) == self.trigger_time : 
    #             #     nothing = 0

    def distance_to_nearest_door(self, robot_point , min_distance , agent_pose , robot_current_pose , robot_pts_arr):
        dis_to_door_list = np.linalg.norm(np.array(self.door_centers) - np.array(robot_point) , axis=1)
        dis_to_human = np.linalg.norm( np.array(agent_pose)- np.array(robot_current_pose))
        closest_door_to_traj_dist = 2.0
        if (np.min(dis_to_door_list) < self.trigger_distance_to_door ) :  
            closest_door_centre = self.door_centers[np.argmin(dis_to_door_list)]
            closest_door_to_traj_dist = np.linalg.norm(np.array(robot_pts_arr) - np.array(closest_door_centre) , axis=1)
        if (np.min(closest_door_to_traj_dist) < 0.2 ) or ((dis_to_human < 5.0) and min_distance < 2.0):
            if (not rospy.get_param('start_convo' , False)) and (time.time() - self.last_start_convo > 10.0): 
                rospy.set_param('start_convo',  True)
                self.last_start_convo = time.time()
                print('Convo Started !!')
                # time.sleep(20)
                # self.start_convo = True

    # def slope_conditions(self, slope_difference , robot_pts_wrt_human):
    #     if slope_difference < 90 and slope_difference > 45 :
    #         text = 'Moving in Front of the human '
    #     elif slope_difference >90 :
    #         text ='Moving to the Left of' 
    #     elif slope_difference < 45 and slope_difference >=0 :
    #         text ='Moving to the Right of '
    #     elif slope_difference > -90 and slope_difference<-45 :
    #         text ='Moving behind of the human'
    #     elif slope_difference < -90 : 
    #         text ='Following the human and moving to the left of'
    #     elif slope_difference >-45 and slope_difference <0:
    #         text ='Following the human and moving to the right of'
        
    #     return text
    
    # def crossing_point_calc(self , robot_pts_arr , robot_tfs_arr , human_pts_arr , human_tfs_arr):
    #     robot_pts_arr = np.array(robot_pts_arr)
    #     human_pts_arr = np.array(human_pts_arr)
    #     # human_tfs_arr = np.array(human_tfs_arr)
    #     # robot_tfs_arr = np.array(robot_tfs_arr)
    #     min_distance = 1000000
    #     for j  ,[robot_tfs , robot_pts] in enumerate(zip(robot_tfs_arr , robot_pts_arr)):
    #         for  i , human_tfs in enumerate(human_tfs_arr): 
    #             if human_tfs - robot_tfs > 0.1 : 
    #                 if human_tfs - robot_tfs < 0.5:
    #                     distance = np.linalg.norm(robot_pts - human_pts_arr[i])
    #                     if distance < min_distance:
    #                         min_distance = distance
    #                         # min_index = np.where(robot_pts_arr == robot_pts)[0][0]
    #                         min_index = j
    #                         agent_index = i
    #     return min_index , min_distance , agent_index
    def ecohan_cb (self , data)  :
        if rospy.get_param('check_for_crossing' , False) :
            time.sleep(0.2)
            robot_crossing_point_list = rospy.wait_for_message('/move_base/HATebLocalPlannerROS/crossing_points' , PoseArray )
            # print(robot_crossing_point_list)
            if len(robot_crossing_point_list.poses) > 0 :
                if len(self.agent_trajs_arr) > 0 : 
                    try : 
                        robot_crossing_point =  robot_crossing_point_list.poses[0] 
                        robot_heading_angle = quat_to_euler(robot_crossing_point.orientation.z , robot_crossing_point.orientation.w)
                        robot_pose = [robot_crossing_point.position.x , robot_crossing_point.position.y]
                        crossing_info = rospy.wait_for_message('/move_base/HATebLocalPlannerROS/crossing_info' , CrossingInfo)
                        agent_crossing_pose = self.agent_trajs_arr[0][1][crossing_info.indices[0]]
                        agent_heading_angle = self.agent_trajs_arr[0][2][crossing_info.indices[0]]
                        agent_next_pose_after_crossing_point = self.agent_trajs_arr[0][1][crossing_info.indices[0] + 1]
                        agent_dx = agent_next_pose_after_crossing_point[0] - agent_crossing_pose[0]
                        agent_dy = agent_next_pose_after_crossing_point[1] - agent_crossing_pose[1]
                        agent_pose = [agent_crossing_pose[0] , agent_crossing_pose[1]]
                        left_or_right , front_or_behind = get_pose_wrt_human(robot_pose , agent_pose , agent_dx , agent_dy)
                        angle_of_robot_wrt_human = rad_to_deg2(robot_heading_angle - agent_heading_angle)
                        direction = get_direction_from_position_dict(front_or_behind , left_or_right , angle_of_robot_wrt_human)
                        self.attr_msg.direction_of_crossing = direction
                        self.attr_msg.distance_while_crossing = self.crossing_info.distances[0] - (self.agent_radius + self.robot_radius)
                        self.attr_msg.time_to_cross = self.crossing_info.times[0]
                        self.attr_pub.publish(self.attr_msg)
                        print(direction)
                        print(self.attr_msg.distance_while_crossing)
                        print(self.attr_msg.time_to_cross)
                        self.image = np.ones((500 , 500 , 3) , dtype=np.uint8) * 255
                        cv2.putText(self.image , str(angle_of_robot_wrt_human), (30 , 350) , cv2.FONT_HERSHEY_SIMPLEX , 1 , (0,0,0) , 2)
                        cv2.putText(self.image , str(round(robot_heading_angle , 3)) + ' ' + str(round(agent_heading_angle , 3)) , (30 , 300) , cv2.FONT_HERSHEY_SIMPLEX , 1 , (0,0,0) , 2)
                        cv2.putText(self.image , left_or_right + front_or_behind , (30 , 150) , cv2.FONT_HERSHEY_SIMPLEX , 1 , (0,0,0) , 2)
                        cv2.putText(self.image , direction , (30 , 250) , cv2.FONT_HERSHEY_SIMPLEX , 1 , (0,0,0) , 2)
                        self.image_msg = bridge.cv2_to_imgmsg(self.image ,  encoding="rgb8")
                        self.image_pub.publish(self.image_msg)
                        rospy.set_param('check_for_crossing' , False)
                    except :
                        pass
            else : 
                pass


    def robot_cb(self , data):
        self.robot_pts_arr = []
        self.robot_tfs_arr = []
        sync_error = False
        for i , points in enumerate(data.points):
            if points.time_from_start > rospy.Duration(0.0):
                self.robot_tfs_arr.append(points.time_from_start.to_sec())
                self.robot_pts_arr.append([points.transform.translation.x , points.transform.translation.y])
        min_distance = 1000000
        min_time = 100
        if self.last_agent_data < (data.header.stamp - rospy.Duration(1)) :
            # tracked_agent_data = rospy.wait_for_message('/tracked_agents' , TrackedAgents , timeout=4.0)
            tracked_agent_data = self.tracked_agent_data
            nearest_agent_id = "Not yet initialized ----"
            for k , agent in enumerate(tracked_agent_data.agents):
                agent_pose = [agent.segments[0].pose.pose.position.x , agent.segments[0].pose.pose.position.y]
                index, distance , _ = self.min_distance_calc(self.robot_pts_arr , agent_pose)
                if distance < min_distance:
                    min_distance = distance 
                    min_index = index
                    nearest_agent_id  = k
            agent_pose = [tracked_agent_data.agents[nearest_agent_id].segments[0].pose.pose.position.x , tracked_agent_data.agents[nearest_agent_id].segments[0].pose.pose.position.y ]
            agent_angle = quat_to_euler(tracked_agent_data.agents[nearest_agent_id].segments[0].pose.pose.orientation.z , tracked_agent_data.agents[nearest_agent_id].segments[0].pose.pose.orientation.w)

        else:    
            try : 
                min_distance = 10000
                for z , agent_traj in enumerate(self.agent_trajs_arr):
                    if len(agent_traj[1]) == 0 :
                        continue
                    # index , distance , agent_index = self.min_distance_calc(self.robot_pts_arr , agent_traj[1])
                    index , distance , agent_index = self.crossing_point_calc(self.robot_pts_arr , self.robot_tfs_arr , agent_traj[1] , agent_traj[0])
                    if distance < min_distance :
                        min_distance = distance
                        min_index = index 
                        nearest_agent_id = z 
                        nearest_agent_traj_index = agent_index
                agent_pose= self.agent_trajs_arr[nearest_agent_id][1][nearest_agent_traj_index]
                agent_angle= self.agent_trajs_arr[nearest_agent_id][2][nearest_agent_traj_index]
            except : 
                sync_error = True
        if not sync_error :
            # self.direction_of_crossing_static(self.robot_pts_arr, min_index , [agent_pose[0] , agent_pose[1] , agent_angle]  , self.robot_tfs_arr[min_index] , min_distance)
            time.sleep(0.1)
            self.distance_to_nearest_door(self.robot_pts_arr[min_index] , min_distance , agent_pose , self.robot_pts_arr[0] , self.robot_pts_arr)
            self.publish_marker(self.robot_pts_arr[min_index][0], self.robot_pts_arr[min_index][1])


if __name__ == "__main__":
    rospy.init_node("cohan_attr")
    obj = cohan_attr()
    rospy.spin()
