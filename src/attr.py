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
from std_msgs.msg import Float64
import json
import rospkg
import time 
from cohan_attr.msg import attr


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
        self.img_pub = rospy.Publisher('/map_image' , Image , queue_size =10, latch=True)
        self.angle_pub = rospy.Publisher('/angle', Float64 , queue_size=10, latch=True)
        self.attr_msg = attr()
        self.attr_pub = rospy.Publisher('cohan_attr/attr' , attr , queue_size= 1 , latch=True)
        self.clock_flag = False
        self.door_centers  =[]
        rospy.set_param('start_convo' , False)
        # self.start_convo = False
        locations = json.load(open(ros_pack.get_path('cohan_attr')  + '/config/locations.json'))
        for location in locations['map']: 
            if ('enter' in location['name']) or ('exit' in location['name']):
                self.door_centers.append(location['pose']['center'])
        rospy.Subscriber('move_base/HATebLocalPlannerROS/agents_local_trajs' , AgentTrajectoryArray, self.agent_cb )

        rospy.Subscriber('/move_base/HATebLocalPlannerROS/local_traj' , Trajectory , self.robot_cb)
        rospy.Subscriber('/move_base/global_costmap/costmap' , OccupancyGrid , self.obs_cb)
        rospy.Subscriber('/clock' , Clock , self.clock_cb )

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
                    self.agent_tfs_arr.append(points.time_from_start)
                    agent_orientation = quat_to_euler(points.transform.rotation.z , points.transform.rotation.w)
                    self.agent_pts_arr.append([points.transform.translation.x , points.transform.translation.y ])
                    self.agent_orientation_arr.append(agent_orientation)
            self.agent_trajs_arr.append([self.agent_tfs_arr, self.agent_pts_arr , self.agent_orientation_arr])
    
    def min_distance_calc(self , arr1 , arr2) : 
        arr1_np =  np.array(arr1)
        arr2_np =  np.array(arr2)
        min_index = 1000
        min_distance = 1000
        id_ = len(arr2_np.shape) -1
        for i , arr1_np_ in enumerate(arr1_np):
            distance = np.linalg.norm(arr1_np_ - arr2_np , axis = id_)
            if np.min(distance) < min_distance :
                min_index = i
                min_agent_index = np.argmin(distance)
                min_distance = np.min(distance)
        return min_index , min_distance , min_agent_index
        
    def avg_slope(self, pts):
        pts = np.array(pts)
        if not len(pts.shape) == 2:
            return None 
        result = scipy.stats.linregress(pts[:,0] , pts[:,1])
        return result.slope

    def direction_of_crossing_static(self , robot_pts_arr , robot_index , human_pose , time_to_nearest_pose , min_distance):
        robot_pts_slice = robot_pts_arr[robot_index-10 : robot_index]
        robot_pts_slice_np = np.array(robot_pts_slice)
        robot_slope = self.avg_slope(robot_pts_slice)
        if not type(robot_slope) == type(None):
            if not np.isnan(robot_slope): 
                robot_pts_wrt_human = robot_pts_slice_np - np.array([human_pose[0] , human_pose[1]])
                human_slope = human_pose[2]
                slope_difference = math.atan(robot_slope) - human_slope
                self.angle_pub.publish(Float64(rad_to_deg(slope_difference)))
                text = self.slope_conditions(rad_to_deg(slope_difference) , robot_pts_wrt_human)
                self.attr_msg.distance_while_crossing = min_distance
                self.attr_msg.direction_of_crossing = text
                self.attr_msg.time_to_cross = time_to_nearest_pose
                self.attr_pub.publish(self.attr_msg)
                full_text = str(round(time_to_nearest_pose , 2)) + " secs | " + str(round(min_distance , 2)) + "m | " + text
                if round(time_to_nearest_pose , 0) == self.trigger_time : 
                    nothing = 0

    def distance_to_nearest_door(self, robot_point , min_distance , agent_pose , robot_current_pose):
        dis_to_door_list = np.linalg.norm(np.array(self.door_centers) - np.array(robot_point) , axis=1)
        dis_to_human = np.linalg.norm( np.array(agent_pose)- np.array(robot_current_pose))
        if (np.min(dis_to_door_list) < self.trigger_distance_to_door ) or (dis_to_human < 5.0) or min_distance < 2.0:
            if not rospy.get_param('start_convo' , False) : 
                rospy.set_param('start_convo',  True)
                print('Convo Started !!')
                time.sleep(20)
                # self.start_convo = True

    def slope_conditions(self, slope_difference , robot_pts_wrt_human):
        if slope_difference < 90 and slope_difference > 45 :
            text = 'Moving in Front of the human '
        elif slope_difference >90 :
            text ='Moving to the Left of' 
        elif slope_difference < 45 and slope_difference >=0 :
            text ='Moving to the Right of '
        elif slope_difference > -90 and slope_difference<-45 :
            text ='Moving behind of the human'
        elif slope_difference < -90 : 
            text ='Following the human and moving to the left of'
        elif slope_difference >-45 and slope_difference <0:
            text ='Following the human and moving to the right of'
        
        return text

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
            tracked_agent_data = rospy.wait_for_message('/tracked_agents' , TrackedAgents , timeout=4.0)
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
            min_distance = 10000
            for z , agent_traj in enumerate(self.agent_trajs_arr):
                if len(agent_traj[1]) == 0 :
                    continue
                index , distance , agent_index = self.min_distance_calc(self.robot_pts_arr , agent_traj[1])
                if distance < min_distance :
                    min_distance = distance
                    min_index = index 
                    nearest_agent_id = z 
                    nearest_agent_traj_index = agent_index
            try : 
                agent_pose= self.agent_trajs_arr[nearest_agent_id][1][nearest_agent_traj_index]
                agent_angle= self.agent_trajs_arr[nearest_agent_id][2][nearest_agent_traj_index]
            except : 
                sync_error = True
        if not sync_error :
            self.direction_of_crossing_static(self.robot_pts_arr, min_index , [agent_pose[0] , agent_pose[1] , agent_angle]  , self.robot_tfs_arr[min_index] , min_distance)
            time.sleep(0.5)
            self.distance_to_nearest_door(self.robot_pts_arr[min_index] , min_distance , agent_pose , self.robot_pts_arr[0])


if __name__ == "__main__":
    rospy.init_node("cohan_attr")
    obj = cohan_attr()
    rospy.spin()