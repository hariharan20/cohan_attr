#! /usr/bin/env python
import rospy
from cohan_msgs.msg import Trajectory, AgentTrajectoryArray, TrackedAgents
import numpy as np
from nav_msgs.msg import OccupancyGrid
import ros_numpy
import matplotlib.pyplot as plt
import scipy
from sensor_msgs.msg import Image
import tf
from geometry_msgs.msg import Quaternion
from rosgraph_msgs.msg import Clock
tf_listener = tf.TransformListener()

def quat_to_euler(w , z):
    # quaternion = Quaternion(0 , 0 ,z , w)
    euler_angles = tf.transformations.euler_from_quaternion([0 , 0  , z , w])
    return euler_angles[2]

import cv2
class cohan_attr:
    def __init__(self):
        self.last_agent_data = rospy.Time.now()
        self.map =None
        self.grid_half_size = 30
        self.img_pub = rospy.Publisher('/map_image' , Image , queue_size =10, latch=True)
        rospy.Subscriber('move_base/HATebLocalPlannerROS/agents_local_trajs' , AgentTrajectoryArray, self.agent_cb )
        rospy.Subscriber('/move_base/HATebLocalPlannerROS/local_traj' , Trajectory , self.robot_cb)
        rospy.Subscriber('/move_base/global_costmap/costmap' , OccupancyGrid , self.obs_cb)
        rospy.Subscriber('/clock' , Clock , self.clock_cb )

    def obs_cb(self, data):
        print(data.info)
        self.map = (np.array(data.data).reshape(data.info.height , data.info.width) > 0.3).astype(np.uint8)*255
        print(np.unique(self.map))
        self.resolution  = data.info.resolution
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.img_pub.publish(ros_numpy.msgify(Image , self.map , encoding='mono8'))
    
    def clock_cb(self , clock_data):
        self.map_image = np.array([self.map , self.map, self.map])
        self.img_pub.publish(ros_numpy.msgify(Image,  self.map_image , encoding="rgb8"))

    def agent_cb(self, data):
        self.agent_trajs_arr = []
        self.last_agent_data =  data.header.stamp 
        agent_trajs = data.trajectories
        for agent_traj in agent_trajs :
            self.agent_tfs_arr = []
            self.agent_pts_arr = []
            for i , points in enumerate(agent_traj.trajectory.points ):
                if points.time_from_start > rospy.Duration(0.0):
                    self.agent_tfs_arr.append(points.time_from_start)
                    self.agent_pts_arr.append([points.transform.translation.x , points.transform.translation.y])
            self.agent_trajs_arr.append([self.agent_tfs_arr, self.agent_pts_arr])
    
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

    def nearest_obstacle(self , agent_arr):
        agent_arr_np = np.array(agent_arr)
        agent_arr_np_grid = agent_arr_np/self.resolution
        min_distances = []
        for pose in agent_arr_np_grid :
            x , y  = pose # Correct it
            search_grid = self.map[x-self.grid_half_size:x+self.grid_half_size , y-self.grid_half_size:y+self.grid_half_size]
            distances = []
            for i , point_x in enumerate(search_grid) :
                y_points = search_grid[point_x]
                for j, point_xy in enumerate(y_points):
                    if point_xy :
                        distances.append(np.linalg.norm([i-x , j-y]))
            if len(distances) >0: 
                min_distances.append(np.min(distances)*self.resolution)
            else : 
                min_distances.append(1000)
        print(min_distances)
        
    def avg_slope(self, pts):
        result = scipy.stats.linregress(pts[:,0] , pts[:,1])

        return result.slope

    def direction_of_crossing_static(self , robot_pts_arr , robot_index , human_pose):
        robot_pts_slice = robot_pts_arr[robot_index-10 : robot_index]
        robot_pts_slice_np = np.array(robot_pts_slice)
        robot_slope = self.avg_slope(robot_pts_slice)
        robot_pts_wrt_human = robot_pts_slice_np - np.array(human_pose[0] , human_pose[1])
        human_slope = human_pose[2]
        slope_difference = robot_slope - human_slope 
        
        self.slope_conditions(slope_difference , robot_pts_wrt_human)


    def direction_of_crossing(self, robot_pts_arr , robot_index, human_index,  human_pts_arr):
        robot_pts_slice = robot_pts_arr[robot_index-10 : robot_index]
        robot_pts_slice_np = np.array(robot_pts_slice)
        robot_slope = self.avg_slope(robot_pts_slice)
        robot_pts_wrt_human = robot_pts_slice_np - human_pts_arr[human_index]
        human_slope = self.avg_slope(human_pts_arr[human_index - 10 : human_index])        
        slope_difference = human_slope - robot_slope
        
        self.slope_conditions(slope_difference , robot_pts_wrt_human)

    def slope_conditions(self, slope_difference , robot_pts_wrt_human):
        if self.avg_slope(robot_pts_wrt_human) < 0 :
            print('Crossing Behind or Left of the Human')
            direction_tag = True
        elif self.avg_slope(robot_pts_wrt_human) > 0 : 
            print('Crossing Infront of Right of Human')
            direction_tag = False
        if slope_difference < 45 :
            if direction_tag :
                text = "I will be following you crossing you by left"
            else :
                text = "I will be following you crossing you by right"
        elif slope_difference > 45 :
            if direction_tag :
                text = "I will cross behind you"
            else :
                text = "I will cross infront of you"
        print(text)        
        #### MORE CONDITIONS IF NEEDED 
    def nearest_obstacle_to_point(self , agent_coor):
        agent_x = int((agent_coor[0]+ 20)/self.resolution) 
        agent_y = int((agent_coor[1]+ 20)/self.resolution) 
        print(agent_x , agent_y)
        x_min_index = agent_x-self.grid_half_size if agent_x-self.grid_half_size > 0 else 0
        x_max_index = agent_x+self.grid_half_size if agent_x+self.grid_half_size < self.map_width else self.map_width
        y_min_index = agent_y-self.grid_half_size if agent_y-self.grid_half_size > 0 else 0
        y_max_index = agent_y+self.grid_half_size if agent_y+self.grid_half_size < self.map_height else self.map_height
        print(x_min_index , x_max_index , y_min_index , y_max_index)
        search_grid = self.map[ y_min_index : y_max_index , x_min_index :x_max_index ]
        self.map[ y_min_index : y_max_index , x_min_index :x_max_index ] = self.map[ y_min_index : y_max_index , x_min_index :x_max_index ] * (100/255)
        # self.img_pub.publish(ros_numpy.msgify(Image , self.map , encoding='mono8'))
        self.map[ y_min_index : y_max_index , x_min_index :x_max_index ] = self.map[ y_min_index : y_max_index , x_min_index :x_max_index ] * (255/100)
        # print(search_grid.shape)
        distances = []
        for i , point_x in enumerate(search_grid) :
            y_points = search_grid[i]
            for j, point_xy in enumerate(y_points):
                if point_xy :
                    distances.append(np.linalg.norm([i-agent_x , j-agent_y]))
        if len(distances) > 0 :
            return np.min(distances)*self.resolution
        else :
            return "No Near Obstacles Detected"

    def robot_cb(self , data):
        self.robot_pts_arr = []
        self.robot_tfs_arr = []
        for i , points in enumerate(data.points):
            if points.time_from_start > rospy.Duration(0.0):
                self.robot_tfs_arr.append(points.time_from_start.to_sec())
                self.robot_pts_arr.append([points.transform.translation.x , points.transform.translation.y])
        min_distance = 100
        min_time = 100
        if self.last_agent_data < (data.header.stamp - rospy.Duration(1)) :
            print('No moving humans detected')
            tracked_agent_data = rospy.wait_for_message('/tracked_agents' , TrackedAgents , timeout=4.0)
            for k , agent in enumerate(tracked_agent_data.agents):
                agent_pose = [agent.segments[0].pose.pose.position.x , agent.segments[0].pose.pose.position.y]
                index, distance , _ = self.min_distance_calc(self.robot_pts_arr , agent_pose)
                if distance < min_distance:
                    min_distance = distance 
                    min_index = index
                    nearest_agent_id  = k
            agent_pose = [tracked_agent_data.agents[nearest_agent_id].segments[0].pose.pose.position.x , tracked_agent_data.agents[nearest_agent_id].segments[0].pose.pose.position.y ]
            agent_angle = quat_to_euler(tracked_agent_data.agents[nearest_agent_id].segments[0].pose.pose.orientation.z , tracked_agent_data.agents[nearest_agent_id].segments[0].pose.pose.orientation.w)
            self.direction_of_crossing_static(self.robot_pts_arr, min_index , [agent_pose[0] , agent_pose[1] , agent_angle] )
        else : 
            self.robot_pts_arr = []
            self.robot_tfs_arr = []
            for i , points in enumerate(data.points):
                if points.time_from_start > rospy.Duration(0.0):
                    self.robot_tfs_arr.append(points.time_from_start.to_sec())
                    self.robot_pts_arr.append([points.pose.position.x , points.pose.position.y])
            for z , agent_traj in enumerate(self.agent_trajs_arr):
                index , distance , agent_index = self.min_distance_calc(self.robot_pts_arr , agent_traj[1])
                if distance < min_distance :
                    min_distance = distance
                    min_index = index 
                    nearest_agent_id = z 
                    nearest_agent_traj_index = agent_index
            agent_pose= self.agent_trajs_arr[nearest_agent_id][1][nearest_agent_traj_index]
            # static_human = False
            self.direction_of_crossing(self.robot_pts_arr ,min_index , nearest_agent_traj_index, self.agent_trajs_arr[nearest_agent_id][1][nearest_agent_traj_index] )
            # agent_angle = 
        # print(self.robot_tfs_arr[min_index])
        # print(min_distance)
        # self.direction_of_crossing(self.robot_pts_arr , min_index,nearest_agent_traj_index , self.agent_trajs_arr[nearest_agent_id][1] , agent_pose , static_human  )
        print(self.nearest_obstacle_to_point(agent_pose))
        print(self.nearest_obstacle_to_point(self.robot_pts_arr[min_index]))
        # text = "Hi Human, I will be closest to you in " + str(self.robot_tfs_arr[min_index]) + " seconds, within a distance of " + str(min_distance) + " meters"
        # print(text)  

if __name__ == "__main__":
    rospy.init_node("cohan_attr")
    obj = cohan_attr()
    rospy.spin()

