#! /usr/bin/env python
import rospy 
from cohan_msgs.msg import TrackedAgents , AgentPathArray
from scipy.spatial import KDTree
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped , PoseArray
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import time 
import cv2
import numpy as np
from cv_bridge import CvBridge
bridge = CvBridge()
from sensor_msgs.msg import Image
class contribution_monitor:
    def __init__(self):
        self.human_contribution_array = []
        self.human_check_time = 0.3
        self.robot_current_pose = None
        self.robot_crossing_pose = None
        self.distance_to_crossing = None
        self.start_analysis = False
        # self.fig, self.ax = plt.subplots()
        # initial_text = 'Contribution Monitor'
        # plt.ion()
        # self.fig = plt.figure()
        # plt.text(0.5, 0.5, initial_text, ha='center', va='center')
        # self.fig, self.ax = plt.subplots()
        # self.ax.set_xlim(0, 1)
        # self.ax.set_ylim(0, 1)
        # self.ax.axis("off")  # Hide axes

        # Text Object
        # self.text = self.ax.text(0.5, 0.5, "Waiting for data...", 
                                #  ha='center', va='center', fontsize=14, color='black')
        self.get_new_goal = True
        self.get_new_local = True
        self.image = np.ones((500 , 500 , 3) , dtype=np.uint8) * 255
        cv2.putText(self.image , 'Contribution Monitor' , (100 , 250) , cv2.FONT_HERSHEY_SIMPLEX , 1 , (0,0,0) , 2)
        self.local_comparison_pub = rospy.Publisher('local_comparison' , AgentPathArray , queue_size=1 , latch=True)
        self.local_comparison_msg = AgentPathArray()
        self.local_comparison_msg.header.frame_id = 'map'
        self.global_comparison_pub = rospy.Publisher('global_comparison' , AgentPathArray , queue_size=1 , latch=True)
        self.global_comparison_msg = AgentPathArray()
        self.global_comparison_msg.header.frame_id = 'map'
        self.image_msg = bridge.cv2_to_imgmsg(self.image ,  encoding="rgb8")
        self.image_pub = rospy.Publisher('contribution_monitor_image' , Image , queue_size=1 , latch=True)
        self.image_pub.publish(self.image_msg)
        rospy.Subscriber('move_base/HATebLocalPlannerROS/agents_local_plans' , AgentPathArray , self.agent_cb)
        rospy.Subscriber('/move_base/HATebLocalPlannerROS/agents_global_plans' , AgentPathArray , self.agent_global_cb)
        # rospy.Subscriber('/odom' , Odometry , self.robot_cb)
        # rospy.Subscriber('move_base/')
        self.robot_crossing_point_wrt_agent_global_plan = None
        self.human_global_data_tree = None
        self.previous_l_or_r = None
        rospy.Subscriber('/move_base/HATebLocalPlannerROS/local_plan' , Path , self.robot_plan_cb)
        rospy.Timer(rospy.Duration(0.1) , self.timer_cb)
        # plt.ion()
        # plt.show()
    def get_pose_wrt_human(self, robot_position , human_position , human_heading_dx , human_heading_dy):
        # print('INSIDE THE GET POSE WRT HUMAN')
        d1 = (robot_position[0] - human_position[0]) * (human_heading_dy) - (robot_position[1] - human_position[1]) * (human_heading_dx)
        if d1 > 0:
            left_or_right = 'right'
        else:
            left_or_right = 'left'
        return left_or_right


    def robot_plan_cb(self, data):
        robot_current_pose = [data.poses[0].pose.position.x , data.poses[0].pose.position.y]
        try :
            self.crossing_point = rospy.wait_for_message('/move_base/HATebLocalPlannerROS/crossing_points' , PoseArray , timeout = 0.2)
            # print(crossing_point)
            self.robot_crossing_pose = [self.crossing_point.poses[0].position.x , self.crossing_point.poses[0].position.y]
            # print(self.robot_crossing_pose, robot_current_pose)
            if self.human_global_data_tree : 
                # print('INSIDE THE ROBOT PLAN CB')
                l_or_r = self.get_pose_wrt_human(self.robot_crossing_pose , self.human_global_path_point_1 , self.human_global_path_dx , self.human_global_path_dy)
                if not self.previous_l_or_r :
                    self.previous_l_or_r = l_or_r
                    self.get_new_local = True
                    print(l_or_r)
                self.current_l_or_r = l_or_r
                if self.previous_l_or_r != self.current_l_or_r:
                    print('CHANGE IN THE POSE')
                    print(l_or_r)
                    self.get_new_local = True
                self.previous_l_or_r = l_or_r
            self.distance_to_crossing = ((robot_current_pose[0] - self.robot_crossing_pose[0])**2 + (robot_current_pose[1] - self.robot_crossing_pose[1])**2)**0.5
        except :
            self.distance_to_crossing = None
            pass

    def timer_cb(self, _):
        if rospy.get_param('reset_sim' , False):
            self.human_contribution_array = []
            # self.human_check_time = 0.2
            self.robot_current_pose = None
            self.robot_crossing_pose = None
            self.distance_to_crossing = None
            self.start_analysis = False
            self.human_global_data_tree = None
            self.previous_l_or_r = None
            self.robot_crossing_point_wrt_agent_global_plan = None
            self.image = np.ones((500 , 500 , 3) , dtype=np.uint8) * 255
            cv2.putText(self.image , 'Contribution Monitor' , (100 , 250) , cv2.FONT_HERSHEY_SIMPLEX , 1 , (0,0,0) , 2)
            self.image_msg = bridge.cv2_to_imgmsg(self.image ,  encoding="rgb8")
            self.image_pub.publish(self.image_msg)
            rospy.set_param('reset_sim' , False)
            self.get_new_goal = True
            self.get_new_local = True
            print('Resetting the simulation')

    def agent_cb(self, data ):
        if self.distance_to_crossing : 
            # print(self.distance_to_crossing)
            if self.distance_to_crossing < 5.0:
                # print(self.distance_to_crossing)
                self.start_analysis = True
            # if self.distance_to_crossing < 4.0:
                # print(self.distance_to_crossing)
                # self.start_analysis = True
            if self.get_new_local : # REMOVE THE COMMENT
            # if False: # COMMENT THIS LINE
                print('INSIDE THE LOCAL TREE BUILDING : ) ')
                self.local_comparison_msg = data
                self.local_comparison_pub.publish(self.local_comparison_msg)
                poses = []
                for pose in data.paths[0].path.poses : 
                    poses.append([pose.pose.position.x , pose.pose.position.y])
                self.human_local_data_tree = KDTree(poses)
                self.get_new_local = False
        if self.start_analysis:
            self.last_agent_plan = data.header.stamp
            ############### TO BE COMMENTED OUT ################
            # print('INSIDE THE LOCAL TREE BUILDING : ) ')
            # self.local_comparison_msg = data
            # self.local_comparison_pub.publish(self.local_comparison_msg)
            # poses = []
            # for pose in data.paths[0].path.poses : 
            #     poses.append([pose.pose.position.x , pose.pose.position.y])
            # self.human_local_data_tree = KDTree(poses)
            ################TILL HERE ############################
            tracked_agent_position_found =  False
            while not tracked_agent_position_found : 
                tracked_agents_data = rospy.wait_for_message('tracked_agents' , TrackedAgents)
                if tracked_agents_data.header.stamp.to_sec() - self.last_agent_plan.to_sec()  > self.human_check_time : 
                    tracked_agent_position_found = True
            current_position = [tracked_agents_data.agents[1].segments[0].pose.pose.position.x , tracked_agents_data.agents[1].segments[0].pose.pose.position.y]
            # print(current_position)
            distance_to_global , _ = self.human_global_data_tree.query([current_position])
            distance_to_local , _ = self.human_local_data_tree.query([current_position])
            # print(distance_to_global , distance_to_local)
            if distance_to_local > distance_to_global :
                self.human_contribution_array.append(0)
                # print('Human is not contributing')
            else : 
                self.human_contribution_array.append(1)
                # print('Human is contributing')
            # print(len(self.human_contribution_array))
            if len(self.human_contribution_array) == 2:
                self.human_contribution_array.pop(0)

                if sum(self.human_contribution_array) > 0:
                    new_text = 'Human is contributing'
                else :
                    new_text = 'Human is not contributing'
                # print(new_text)
                self.image = np.ones((500 , 500 , 3) , dtype=np.uint8) * 255
                cv2.putText(self.image , new_text , (30 , 250) , cv2.FONT_HERSHEY_SIMPLEX , 1 , (0,0,0) , 2)
                self.image_msg = bridge.cv2_to_imgmsg(self.image ,  encoding="rgb8")
                self.image_pub.publish(self.image_msg)
                # self.human_contribution_array = []
                # time.sleep(10)
 # Allow UI to update
        
        
    def agent_global_cb(self, data):
        
        self.agent_global_data = data

        
        
        if self.get_new_goal : 
            self.global_comparison_msg = data
            self.global_comparison_pub.publish(self.global_comparison_msg)
            poses = [] 
            for pose in data.paths[0].path.poses : 
                poses.append([pose.pose.position.x , pose.pose.position.y])
            self.human_global_path_point_1 = [poses[0][0] , poses[0][1]]
            human_global_path_point_2 = [poses[-1][0] , poses[-1][1]]
            self.human_global_path_dx = human_global_path_point_2[0] - self.human_global_path_point_1[0]
            self.human_global_path_dy = human_global_path_point_2[1] - self.human_global_path_point_1[1]
            self.human_global_data_tree = KDTree(poses)
            self.get_new_goal = False
            # print('Global plan received')
        # print('TREE BUILT')

        
if __name__ == "__main__" : 
    rospy.init_node('contribution_monitor')
    contribution_monitor()
    rospy.spin()