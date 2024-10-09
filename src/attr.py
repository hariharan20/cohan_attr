#! /usr/bin/env python
import rospy
from cohan_msgs.msg import TrajectoryStamped, AgentTrajectoryArray, TrackedAgents
import numpy as np

class cohan_attr:
    def __init__(self):
        self.last_agent_data = rospy.Time.now()
        rospy.Subscriber('move_base/HATebLocalPlannerROS/agents_local_trajs' , AgentTrajectoryArray, self.agent_cb )
        rospy.Subscriber('/move_base/HATebLocalPlannerROS/local_traj' , TrajectoryStamped , self.robot_cb)

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
                    self.agent_pts_arr.append([points.pose.position.x , points.pose.position.y])
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
                min_distance = np.min(distance)
        return min_index , min_distance

        
    def robot_cb(self , data):
        self.robot_pts_arr = []
        self.robot_tfs_arr = []
        for i , points in enumerate(data.points):
            if points.time_from_start > rospy.Duration(0.0):
                self.robot_tfs_arr.append(points.time_from_start.to_sec())
                self.robot_pts_arr.append([points.pose.position.x , points.pose.position.y])
        min_distance = 100
        min_time = 100
        # if self.last_agent_data :
        # print("I am in")
        if self.last_agent_data < (data.header.stamp - rospy.Duration(1)) :
            print('No moving humans detected')
            tracked_agent_data = rospy.wait_for_message('/tracked_agents' , TrackedAgents , timeout=4.0)
            for agent in tracked_agent_data.agents:
                agent_pose = [agent.segments[0].pose.pose.position.x , agent.segments[0].pose.pose.position.y]
                index, distance = self.min_distance_calc(self.robot_pts_arr , agent_pose)
                if distance < min_distance:
                    min_distance = distance 
                    min_index = index
        else : 
            self.robot_pts_arr = []
            self.robot_tfs_arr = []
            for i , points in enumerate(data.points):
                if points.time_from_start > rospy.Duration(0.0):
                    self.robot_tfs_arr.append(points.time_from_start.to_sec())
                    self.robot_pts_arr.append([points.pose.position.x , points.pose.position.y])
            for agent_traj in self.agent_trajs_arr:
                index , distance = self.min_distance_calc(self.robot_pts_arr , agent_traj[1])
                if distance < min_distance :
                    min_distance = distance
                    min_index = index 
        # print(self.robot_tfs_arr[min_index])
        # print(min_distance)
        text = "Hi Human, I will be closest to you in " + str(self.robot_tfs_arr[min_index]) + " seconds, within a distance of " + str(min_distance) + " meters"
        print(text)  

if __name__ == "__main__":
    rospy.init_node("cohan_attr")
    obj = cohan_attr()
    rospy.spin()

