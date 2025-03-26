#! /usr/bin/env python
import rospy 
from cohan_msgs.msg import TrackedAgents 
from nav_msgs.msg import OccupancyGrid
import numpy as np


class human_goal_estimator:
    def __init__(self, human_id ):
        rospy.Subscriber('tracked_agents' , TrackedAgents , self.agent_cb)
        self.human_id = human_id
        self.human_goal_x = None
        self.human_goal_y = None
        self.human_x_array = []
        self.human_y_array = []
        rospy.Subscriber('/move_base/global_costmap/costmap' , OccupancyGrid , self.costmap_cb)
        # rospy.Timer(rospy.Duration(0.1) , self.goal_updater)
        

    def costmap_cb(self , data ):
        self.costmap = data.data
        print(np.array(self.costmap).shape)
    def agent_cb(self, data):
        tracked_agent_data = data
        self.human_x_array.append(tracked_agent_data.agents[self.human_id].segments[0].pose.pose.position.x)
        self.human_y_array.append(tracked_agent_data.agents[self.human_id].segments[0].pose.pose.position.y)


if __name__ == '__main__':
    rospy.init_node('human_goal_estimator')
    human_goal_estimator(0)
    rospy.spin()