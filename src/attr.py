#! /usr/bin/env python
import rospy
from cohan_msgs.msg import TrajectoryStamped, AgentTrajectoryArray
import numpy as np

class cohan_attr:
    def __init__(self):
        # self.pub = rospy.Publisher('/cohan_attributes' , ) #TODO: define if needed
        rospy.Subscriber('move_base/HATebLocalPlannerROS/agents_local_trajs' , AgentTrajectoryArray, self.agent_cb )
        rospy.Subscriber('/move_base/HATebLocalPlannerROS/local_traj' , TrajectoryStamped , self.robot_cb)

    def agent_cb(self, data):
        self.agent_trajs_arr = []
        self.last_agent_data =  data.header.stamp 
        agent_trajs = data.trajectories
        for agent_traj in agent_trajs :
            self.agent_tfs_arr = []
            self.agent_pts_arr = []
            for i , points in enumerate(agent_traj.points ):
                self.agent_tfs_arr.append(points.time_from_start)
                self.agent_pts_arr.append([points.pose.position.x , points.pose.position.y])
            self.agent_trajs_arr.append([self.agent_tfs_arr, self.agent_pts_arr])
    
    def min_distance_calc(self , arr1 , arr2) : 
        arr1_np =  np.array(arr1)
        arr2_np =  np.array(arr2)
        distance = np.linalg.norm(arr1_np - arr2_np , axis = 0)
        min_index = np.argmin(distance)
        if type(min_index) == np.int64:
            return min_index , distance[min_index]
        else :
            return min_index[0] , distance[min_index[0]]

    def robot_cb(self , data):
        if self.last_agent_data < (data.header.stamp - rospy.Duration(1)) :
            #TODO: conditions when the agent data was published in the very past ;)
            print('')
        else : 
            min_distance = 100
            min_time = 100
            self.robot_pts_arr = []
            self.robot_tfs_arr = []
            for i , points in enumerate(data.points):
                self.robot_tfs_arr.append(points.time_from_start)
                self.robot_pts_arr.append([points.pose.position.x , points.pose.position.y])
            
            for agent_traj in self.agent_trajs_arr:
                index , distance = self.min_distance_calc(self.robot_pts_arr , agent_traj[1])
                if distance < min_distance :
                    min_distance = distance
                    min_index = index 
            print(self.robot_tfs_arr[index])
            print(min_distance)
            
if __name__ == "__main__":
    rospy.init_node("cohan_attr")
    obj = cohan_attr()
    rospy.spin()

