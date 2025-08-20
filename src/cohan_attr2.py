#! /usr/bin/env python
import rospy 
import message_filters
import numpy as np
import tf
from cohan_msgs.msg import AgentPathArray, CrossingInfo
from std_msgs.msg import String
from nav_msgs.msg import Path
import math
import mediapipe as mp
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cohan_attr.msg import attr
import time
from nav_msgs.msg import OccupancyGrid
from scipy.spatial import KDTree
import dynamic_reconfigure.client
from move_base_msgs.msg import MoveBaseActionGoal
import json


LEN_OF_HUMAN_TRAJ = 10

fd = mp.solutions.face_detection
from cv_bridge import CvBridge
bridge = CvBridge()

position_dict =  {
    ('left' , 'behind') : {
        1 : 'follow and pass by left,move right',
        90 : 'cross behind from left,move forward',
        180 : 'pass by left,move right',
        270 : 'cross behind from right,move forward',
        359 : 'follow and pass by left,move right',
    },
    ('right' , 'behind') : {
        1 : 'follow and pass by right,move left',
        90 : 'cross behind from left,move forward',
        180 : 'pass by right,move left',
        270 : 'cross behind from right,move forward',
        359 : 'follow and pass by right,move left',
    },
    ('left' , 'front') : {
        1 : 'follow and pass by left,move right',
        90 : 'cross in front from left,move back',
        180 : 'pass by left,move right',
        270 : 'cross in front from right,move back',
        359 : 'follow and pass by left,move right',

    },
    ('right' , 'front') : {
        0 : 'follow and pass by right,move left',
        90 : 'crossing in front from left,move back',
        180 : 'pass by right,move left',
        270 : 'crossing in front from right,move back',
        359 : 'follow and pass by right,move left',
    },
}

def quat_to_euler(w , z):
    euler_angles = tf.transformations.euler_from_quaternion([0 , 0  , z , w])
    return euler_angles[2]


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


def get_direction(robot_position , human_position , human_dx , human_dy , robot_heading_angle , human_heading_angle):
    angle_of_robot_wrt_human = rad_to_deg2(robot_heading_angle - human_heading_angle)
    l_or_r , f_or_b = get_pose_wrt_human(robot_position , human_position , human_dx , human_dy)
    angle_dict = position_dict[(l_or_r , f_or_b)]
    angle_dict_keys = list(angle_dict.keys())
    angle_dict_keys = np.array(angle_dict_keys)
    angle_difference = np.abs(angle_dict_keys - angle_of_robot_wrt_human)
    min_index = np.argmin(angle_difference)
    return  angle_dict[angle_dict_keys[min_index]]


def rad_to_deg2(rad) : 
    if rad < 0 : 
        return 360 + (rad *180 / math.pi)
    return rad * 180 / math.pi


class SituationMonitor:
    def __init__(self):
        agent_local_plans = message_filters.Subscriber('/move_base/HATebLocalPlannerROS/agents_local_plans' , AgentPathArray )
        reconf_client= dynamic_reconfigure.client.Client("/move_base/HATebLocalPlannerROS")
        reconf_client.update_configuration({'weight_agent_viapoint' : 10.0 , 'weight_viapoint' : 0.5 })
        robot_local_plans = message_filters.Subscriber('/move_base/HATebLocalPlannerROS/local_plan' , Path )
        agent_global_paths = message_filters.Subscriber('/move_base/HATebLocalPlannerROS/agents_global_plans' , AgentPathArray )
        crossing_info = message_filters.Subscriber('/move_base/HATebLocalPlannerROS/crossing_info' , CrossingInfo)
        self.img_pub = rospy.Publisher('/cohan_attr/human_image'  , Image , queue_size=1, latch=True)
        self.attr_pub = rospy.Publisher('/cohan_attr/attributes' , attr , queue_size=1, latch=False)
        self.analysis_pub = rospy.Publisher('/robot_analysis' ,  String, queue_size=1, latch=False)
        rospy.Subscriber('/camera/color/image_raw' , Image , self.image_cb)
        mf_sub = message_filters.ApproximateTimeSynchronizer([agent_local_plans , agent_global_paths, robot_local_plans , crossing_info] , 10 , 0.1 , allow_headerless=True)
        rospy.Subscriber('/map' , OccupancyGrid , self.obs_cb)
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_cb)

        mf_sub.registerCallback(self.mf_sub_cb)
        self.planning_cycles = 0
        rospy.Timer(rospy.Duration(0.1), self.reset_callback) 
        self.mp_face_detection = fd
        yolo_model = 'yolov8s.pt'
        self.last_image_sent = time.time()
        self.send_data = True
        self.send_image = True
        self.yolo = YOLO(yolo_model)

    def obs_cb(self , msg) :
        self.costmap_data = msg
        
    def goal_cb(self, msg):
        rospy.set_param('reset_attr' , True)

    def reset_callback(self, _):
        if rospy.get_param('reset_attr' , False):
            self.planning_cycle = 0
            self.send_data = True
            self.send_image = True
            rospy.set_param('reset_attr' , False)

    def nearest_obstacele_w_sim_laser(self, x, y, theta, fov_deg=90, angle_resolution_deg=0.5, max_range=8.0, for_robot=False):
        map_info = self.costmap_data.info
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        width = map_info.width
        height = map_info.height

        fov_rad = np.deg2rad(fov_deg)
        angle_increment = np.deg2rad(angle_resolution_deg)
        angles = np.arange(-fov_rad / 2, fov_rad / 2 + angle_increment, angle_increment)

        ranges = []
        list_of_points = []

        for angle in angles:
            hit = False
            for r in np.arange(0, max_range, resolution):
                beam_x = x + r * np.cos(theta + angle)
                beam_y = y + r * np.sin(theta + angle)

                grid_x = int((beam_x - origin_x) / resolution)
                grid_y = int((beam_y - origin_y) / resolution)

                if 0 <= grid_x < width and 0 <= grid_y < height:
                    idx = grid_y * width + grid_x
                    if self.costmap_data.data[idx] > 90:
                        ranges.append(r)
                        hit = True
                        list_of_points.append((beam_x, beam_y))  
                        break
            if not hit:
                ranges.append(max_range)

        min_distance = min(ranges)
        obstacle_detected = min_distance < max_range

        return min_distance, obstacle_detected

    def mf_sub_cb(self, agent_local_plan , agent_global_paths,  robot_local_plan , crossing_info):
        
            
        if len(crossing_info.indices) > 0 and self.send_data: 
            self.planning_cycles += 1
            NUMBER_OF_PLANNING_CYCLES_TO_SKIP = 4


            if self.planning_cycles  > NUMBER_OF_PLANNING_CYCLES_TO_SKIP:
                
                poses = []
                for pose in agent_global_paths.paths[0].path.poses :
                    poses.append([pose.pose.position.x , pose.pose.position.y])
                human_global_data_tree = KDTree(poses)
            


                human_poses = []
                for pose in agent_local_plan.paths[0].path.poses :
                    human_poses.append([pose.pose.position.x , pose.pose.position.y , pose.pose.orientation.z , pose.pose.orientation.w])
                human_crossing_point = human_poses[crossing_info.indices[0]]
                human_post_crossing_point = human_poses[crossing_info.indices[0] + 1]

                robot_poses = []
                for pose in robot_local_plan.poses :
                    robot_poses.append([pose.pose.position.x , pose.pose.position.y , pose.pose.orientation.z , pose.pose.orientation.w])
                robot_crossing_point = robot_poses[crossing_info.indices[0]]
            
                robot_theta = math.atan2(robot_crossing_point[1] - human_crossing_point[1] , robot_crossing_point[0] - human_crossing_point[0])
                robot_distance_to_obstacle , _ = self.nearest_obstacele_w_sim_laser(robot_crossing_point[0] , robot_crossing_point[1] , robot_theta , for_robot=True)
            
                human_theta = math.atan2(human_crossing_point[1] - robot_crossing_point[1] , human_crossing_point[0] - robot_crossing_point[0])
                human_distance_to_obstacle , _ = self.nearest_obstacele_w_sim_laser(human_crossing_point[0] , human_crossing_point[1] , human_theta )


                human_heading_angle = quat_to_euler(human_crossing_point[3] , human_crossing_point[2])
                robot_heading_angle = quat_to_euler(robot_crossing_point[3] , robot_crossing_point[2])
                
                human_dx = human_post_crossing_point[0] - human_crossing_point[0]
                human_dy = human_post_crossing_point[1] - human_crossing_point[1]

                dist_human_robot_at_crossing_point = math.sqrt((human_crossing_point[0] - robot_crossing_point[0])**2 + (human_crossing_point[1] - robot_crossing_point[1])**2)
                direction_of_crossing = get_direction(
                    robot_crossing_point[:2],
                    human_crossing_point[:2],
                    human_dx,
                    human_dy,
                    robot_heading_angle,
                    human_heading_angle
                )


                time_to_cross = crossing_info.times[0]
                human_poses_np = np.array(human_poses)
                points_on_local_plan = human_poses_np[crossing_info.indices[0]-5 : crossing_info.indices[0]+5]
                distance = 0
                for points_ in points_on_local_plan: 
                    distance_to_global  = human_global_data_tree.query(points_[:2])[0] 
                    if distance_to_global > distance :
                        distance = distance_to_global

                                    


                human_needs_to_contribute = [True if distance < 2.0 else False ][0]
                human_can_contribute =[True if human_distance_to_obstacle > 2.0 else False ][0]
                robot_can_contribute = [True if robot_distance_to_obstacle > 2.0 and dist_human_robot_at_crossing_point > 1.0 else False ][0]
                dialogue = ''
                bottleneck = False
                if time_to_cross < 4.0 :
                    if human_needs_to_contribute : 
                        if robot_can_contribute : 
                            rospy.loginfo('Speak the Direction of Crossing')
                            dialogue = 'I will ' + direction_of_crossing.split(',')[0] + ' of you. Please be aware of my presence.'
                        else : 
                            if human_can_contribute :
                                rospy.loginfo('Speak the Situation and Ask to Move ') 
                                dialogue = 'I see we will be constraint, could you please ' + direction_of_crossing.split(',')[1] + ' ?'
                            else :
                                rospy.loginfo('Say, the robot will dock')
                                dialogue = 'I will dock if needed.'
                                bottleneck = True
                    else : 
                        if robot_can_contribute : 
                            rospy.loginfo('No Speech Needed')
                            dialogue = ''
                        else :
                            rospy.loginfo('Say the direction of crossing') 
                            dialogue = 'I will ' + direction_of_crossing.split(',')[0] + ' of you.'
                    if dialogue != '' :
                        robot_response = {'bottleneck' : bottleneck  ,'dialogue' : dialogue}
                        self.analysis_pub.publish(String(json.dumps(robot_response)))

                    self.send_data = False



    def is_face_visible(self  , input_image) :
        with self.mp_face_detection.FaceDetection(model_selection=1 , min_detection_confidence=0.5 ) as face_detection : 
            results = face_detection.process(np.ascontiguousarray(input_image))
        face_detected = False
        if results.detections : 
            face_detected = True
        return face_detected 



    def image_cb(self, data):
        # rospy.loginfo('Image received') 
        if time.time() - self.last_image_sent > 3.0 and self.send_image : 
            rospy.loginfo('Processing image for face detection')
            img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
            result =  self.yolo(img , show= False , verbose=False)
            boxes = result[0].boxes.xyxy.tolist()
            classes = result[0].boxes.cls.tolist()
            confidences = result[0].boxes.conf.tolist()
            human_bbs = []
            human_confs = []
            for box, cls, conf in zip(boxes, classes, confidences):
                if cls == 0 : 
                    human_bbs.append(box)
                    human_confs.append(conf)
            if len(human_confs) > 0 :
                rospy.loginfo('Found humans in the image')
                confi_id = np.argmax(human_confs)
                [x_min , y_min , x_max , y_max] = human_bbs[confi_id]
                cropped_image = img[math.floor(y_min) : math.floor(y_max) , math.floor(x_min) : math.floor(x_max)]
                # if (x_max - x_min) > 100 and (y_max - y_min) > 100: 
                if self.is_face_visible(cropped_image):
                            rospy.logerr('FACE VISIBLE')
                            img_msg = bridge.cv2_to_imgmsg(cropped_image ,  encoding="rgb8")
                            self.img_pub.publish(img_msg)
                            self.publish_image = False
                            print('published image')
                            self.last_image_sent = time.time()
                            self.send_image = False


if __name__ == '__main__':
    rospy.init_node('situation_monitor')
    situation_monitor = SituationMonitor()
    rospy.spin()    
