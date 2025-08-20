#! /usr/bin/env python
import rospy  
import numpy as np
import math 
import time
from cohan_msgs.msg import TrackedAgents , AgentPathArray , AgentTrajectoryArray , CrossingInfo
import dynamic_reconfigure.client
from std_msgs.msg import String
import message_filters
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray , Marker
from jsk_rviz_plugins.msg import OverlayText
from scipy.spatial import KDTree
import tf
from contri_decision_tree import DecisionTree
from std_msgs.msg import Float32
def quat_to_euler(w , z):
    euler_angles = tf.transformations.euler_from_quaternion([0 , 0  , z , w])
    return euler_angles[2]

GAMMA = 0.98


def discounted_average(x):
    T = len(x)
    weights = [GAMMA**(T - t - 1) for t in range(T)]
    weighted_sum = sum(w * x_t for w, x_t in zip(weights, x))
    # print(weights)
    total_weight = sum(weights)
    return weighted_sum / total_weight


class ECoHAN : 
    def __init__(self) :
        self.reconf_client= dynamic_reconfigure.client.Client("/move_base/HATebLocalPlannerROS")
        self.update_weights(reset=True)
        self.update_weights(10)
        self.initial_comparison_done = False
        self.global_comparison_pub = rospy.Publisher('global_comparison' , AgentPathArray , queue_size=1 , latch=True)
        self.tight_band_pub = rospy.Publisher('tight_band' , AgentPathArray , queue_size=1 , latch=True)
        self.human_crossing_point_pub = rospy.Publisher('human_crossing_point' , Marker , queue_size=1 , latch=True)
        self.robot_crossing_point_pub= rospy.Publisher('robot_crossing_point' , Marker , queue_size=1 , latch=True)
        self.over_text_pub = rospy.Publisher('/overlay_text' , OverlayText , queue_size=1 , latch=True)
        self.contri_text_pub= rospy.Publisher('/contri_text' , OverlayText , queue_size=1 , latch=True)

        self.over_text_pub.publish(OverlayText(text='ROBOT SPEECH' , width=700, height=300 , left=0 , top=0))
        self.robot_crossing_point_laser_pub = rospy.Publisher('/robot_crossing_point_laser' , MarkerArray , queue_size=10 , latch=True)
        self.human_crossing_point_laser_pub = rospy.Publisher('/human_crossing_point_laser' , MarkerArray , queue_size=10, latch=True)
        rospy.Subscriber('human_speech_listener' , String , self.human_speech_listener_cb)
        self.decision_tree = DecisionTree()
        self.planning_cycles = 0
        self.get_new_global = True
        self.got_new_global_tree = False
        self.tight_band_comparison_done = False
        self.start_final_comparison = False
        self.final_comparison_done = False


        self.human_crossing_point_msg = Marker()
        self.human_crossing_point_msg.header.frame_id = 'map'
        self.human_crossing_point_msg.type = Marker.SPHERE  
        self.human_crossing_point_msg.action = Marker.ADD  
        self.human_crossing_point_msg.color.a = 1.0  
        self.human_crossing_point_msg.color.r = 1.0  
        self.human_crossing_point_msg.scale.x = 0.1  
        self.human_crossing_point_msg.scale.y = 0.1  
        self.human_crossing_point_msg.scale.z = 0.1  


        self.robot_crossing_point_msg = Marker()
        self.robot_crossing_point_msg.header.frame_id = 'map'
        self.robot_crossing_point_msg.type = Marker.SPHERE  
        self.robot_crossing_point_msg.action = Marker.ADD  
        self.robot_crossing_point_msg.color.a = 1.0  
        self.robot_crossing_point_msg.color.b = 1.0  
        self.robot_crossing_point_msg.scale.x = 0.1  
        self.robot_crossing_point_msg.scale.y = 0.1  
        self.robot_crossing_point_msg.scale.z = 0.1  

        self.tracked_postion_msg = Marker()


        self.human_path_monitor_array = []
        self.tracked_position_array_msg = MarkerArray()
        self.check_for_monitoring = False


        self.tracked_postion_pub = rospy.Publisher('tracked_position' , MarkerArray , queue_size =1 , latch = True)
        self.tracked_postion_msg.header.frame_id = 'map'
        self.tracked_postion_msg.type = Marker.SPHERE  # Use SPHERE to represent a single point
        self.tracked_postion_msg.action = Marker.ADD  # The action to add the marker
        self.tracked_postion_msg.color.a = 1.0  # Fully opaque
        self.tracked_postion_msg.color.g = 0.5  # Red color (you can adjust the color as needed)
        self.tracked_postion_msg.color.b = 0.5  # Red color (you can adjust the color as needed)
        self.tracked_postion_msg.scale.x = 0.1  # Size of the sphere
        self.tracked_postion_msg.scale.y = 0.1  # Size of the sphere
        self.tracked_postion_msg.scale.z = 0.1  # Size of the sphere
        self.tracked_position_array_msg = MarkerArray()
        self.tracked_postion_pub.publish(self.tracked_position_array_msg)

        rospy.Subscriber('/move_base/HATebLocalPlannerROS/agents_global_plans' , AgentPathArray , self.agent_global_cb)
        rospy.Subscriber('/map' , OccupancyGrid , self.obs_cb)
        rospy.Timer(rospy.Duration(0.1), self.resetter)
        rospy.Timer(rospy.Duration(0.2) , self.contribution_monitor)
        agent_local_plans = message_filters.Subscriber('/move_base/HATebLocalPlannerROS/agents_local_plans' , AgentPathArray )
        robot_local_plans = message_filters.Subscriber('/move_base/HATebLocalPlannerROS/local_plan' , Path )
        crossing_info = message_filters.Subscriber('/move_base/HATebLocalPlannerROS/crossing_info' , CrossingInfo)
        mf_sub = message_filters.ApproximateTimeSynchronizer([agent_local_plans , robot_local_plans , crossing_info] , 10 , 0.1 , allow_headerless=True)
        mf_sub.registerCallback(self.mf_sub_cb)
        self.human_points = []
        self.human_can_contribute , self.human_needs_to_contribute , self.human_is_contributing = False , False , False
        
        self.contrib_pub = rospy.Publisher('contribution' , Float32 , queue_size=1 , latch=True)
        rospy.Timer(rospy.Duration(0.1) , self.pipeline)
        self.got_tight_band = False
        self.human_contribution = 0.0
        self.human_pose_array = MarkerArray()
        # self.human_pose_array.header.frame_id = 'map'
        self.human_pose_array_pub = rospy.Publisher('human_pose_array' , MarkerArray , queue_size=1 , latch=True)
        self.start_dist_data_recording = False
        print('Inside ECoHAN')

    ## Human Speech Listener Callback

    def human_speech_listener_cb(self , msg) :
        llm_output = human_speech_listener(msg.data)
        if llm_output['mode'] == 'dock' :
            self.robot_nav_utils.dock()
            rospy.loginfo('Robot is docking')
        elif llm_output['mode'] == 'continue' :
            rospy.loginfo('Robot is continuing')
            pass
        else :
            rospy.loginfo('Unknown mode from LLM : ' + llm_output['mode'])

    ## Costmap Callback

    def obs_cb(self , msg) :
        self.costmap_data = msg


            
    ## Resetting the parameters

    def resetter(self , _) : 
        if rospy.get_param('reset_sim' , False) : 
            rospy.set_param('reset_sim' , False)
            self.human_contribution = 0.0
            self.planning_cycles = 0
            self.over_text_pub.publish(OverlayText(text='ROBOT SPEECH' , width=700, height=300 , left=0 , top=0))
            self.get_new_global = True
            self.got_new_global_tree = False
            self.tight_band_comparison_done = False
            self.start_final_comparison = False
            self.final_comparison_done = False
            self.update_weights(10.0)
            self.human_path_monitor_array = []
            self.start_dist_data_recording = False
            self.tracked_position_array_msg = MarkerArray()
            self.check_for_monitoring = False
            self.human_points = []
            self.initial_comparison_done = False
            self.decision_tree.reset()
            self.human_can_contribute , self.human_needs_to_contribute , self.human_is_contributing = False , False , False
            self.human_pose_array = MarkerArray()
            # self.human_pose_array.header.frame_id = 'map'
            self.got_tight_band = False
    ## Updating the weights of the agent and robot

    def update_weights(self, weight_to_be_adjusted=0 , reset = False):
        if reset :
            new_weight_agent_viapoint = 0.2
            new_weight_viapoint = 0.1
        else : 
            weight_viapoint = self.current_weight_viapoint
            weight_agent_viapoint = self.current_weight_agent_viapoint
            new_weight_agent_viapoint = weight_agent_viapoint + weight_to_be_adjusted
            if new_weight_agent_viapoint < 0.1 : 
                new_weight_agent_viapoint = 0.1
            elif new_weight_agent_viapoint > 10.0 : 
                new_weight_agent_viapoint = 10.0
            new_weight_viapoint = weight_viapoint - weight_to_be_adjusted
            if new_weight_viapoint < 0.05 : 
                new_weight_viapoint = 0.05
            elif new_weight_viapoint > 10.0 : 
                new_weight_viapoint = 10.0
        self.reconf_client.update_configuration({'weight_agent_viapoint' : new_weight_agent_viapoint , 'weight_viapoint' : new_weight_viapoint })
        self.current_weight_agent_viapoint = new_weight_agent_viapoint
        self.current_weight_viapoint = new_weight_viapoint
        # rospy.loginfo('Updated agnet weights to : ' + str(new_weight_agent_viapoint) + ' , robot weight :  ' + str(new_weight_viapoint))

    ## Agent Global Callback

    def agent_global_cb(self, msg):
        if self.get_new_global : 
            self.get_new_global = False
            self.global_comparison_pub.publish(msg)
            poses = []
            for pose in msg.paths[0].path.poses :
                poses.append([pose.pose.position.x , pose.pose.position.y])
            self.global_points =poses
            self.human_global_path_point_1 = [poses[0][0] , poses[0][1]]
            human_global_path_point_2 = [poses[-1][0] , poses[-1][1]]
            self.human_global_path_dx = human_global_path_point_2[0] - self.human_global_path_point_1[0]
            self.human_global_path_dy = human_global_path_point_2[1] - self.human_global_path_point_1[1]
            self.human_global_data_tree = KDTree(poses)
            self.got_new_global_tree = True
    

    ## Nearest Obstacle with Simulated Laser

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

        if list_of_points:
            marker_array = MarkerArray()
            for i, (wx, wy) in enumerate(list_of_points):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = rospy.Time.now()
                marker.ns = "laser_obstacles"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = wx
                marker.pose.position.y = wy
                marker.pose.position.z = 0.0
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0 if for_robot else 0.0
                marker.color.b = 0.0 if for_robot else 1.0
                marker_array.markers.append(marker)

            if for_robot:
                self.robot_crossing_point_laser_pub.publish(marker_array)
            else:
                self.human_crossing_point_laser_pub.publish(marker_array)

        return min_distance, obstacle_detected
    
    def contribution_monitor(self, _ ):
        if self.got_new_global_tree :
            tracked_agents_data =  rospy.wait_for_message('tracked_agents' , TrackedAgents , timeout = 0.5)
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time.now()
            marker.ns = "human_pose_array"
            marker.id = len(self.human_pose_array.markers) + 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = tracked_agents_data.agents[1].segments[0].pose.pose.position.x
            marker.pose.position.y = tracked_agents_data.agents[1].segments[0].pose.pose.position.y
            marker.pose.position.z = 0.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.5
            marker.color.b = 1.0
            self.human_pose_array.markers.append(marker)


            # human_pose = Pose()
            # human_pose.position.x = tracked_agents_data.agents[1].segments[0].pose.pose.position.x
            # human_pose.position.y = tracked_agents_data.agents[1].segments[0].pose.pose.position.y
            # self.human_pose_array.poses.append(human_pose)
            self.human_pose_array_pub.publish(self.human_pose_array)
            robot_odom_data = rospy.wait_for_message('/base_pose_ground_truth'  , Odometry)
            robot_current_position = [robot_odom_data.pose.pose.position.x , robot_odom_data.pose.pose.position.y]
            current_position = [tracked_agents_data.agents[1].segments[0].pose.pose.position.x , tracked_agents_data.agents[1].segments[0].pose.pose.position.y]

            # dist_to_global = self.human_global_data_tree.query(current_position)[0]
            index_of_closest_point_on_global_path = self.human_global_data_tree.query(current_position)[1]
            closest_point_on_global_path = self.global_points[index_of_closest_point_on_global_path]
            self.human_points.append(current_position)
            distance_robot_and_point_of_global_path = math.sqrt((robot_current_position[0] - closest_point_on_global_path[0])**2 + (robot_current_position[1] - closest_point_on_global_path[1])**2)
            distance_robot_and_current_point =   math.sqrt((robot_current_position[0] - current_position[0])**2 + (robot_current_position[1] - current_position[1])**2)
            # if np.abs(distance_robot_and_current_point - distance_robot_and_point_of_global_path ) < 0.1 :
                # self.human_path_monitor_array.append(0.0)
            # else :
            distance_current_and_point_of_global_path = math.sqrt((current_position[0] - closest_point_on_global_path[0])**2 + (current_position[1] - closest_point_on_global_path[1])**2)
            if distance_robot_and_current_point - distance_robot_and_point_of_global_path < 0 :
                distance_current_and_point_of_global_path = -distance_current_and_point_of_global_path
            info = round(distance_current_and_point_of_global_path , 3)
            # rospy.loginfo('Distance to global path : ' + str(info))
            self.contrib_pub.publish(Float32(data=info))
            if self.start_dist_data_recording : 
                self.human_path_monitor_array.append(distance_current_and_point_of_global_path)
            # self.human_path_monitor_array.append(distance_current_and_point_of_global_path)
            # if len(self.human_path_monitor_array) == 10 :
            #     marker_array = MarkerArray()
            #     for  i , point in enumerate(self.human_points) :
            #         marker = Marker()
            #         marker.header.frame_id = 'map'
            #         marker.header.stamp = rospy.Time.now()
            #         marker.ns = "human_path_monitor"
            #         marker.id = i
            #         marker.type = Marker.SPHERE
            #         marker.action = Marker.ADD
            #         marker.scale.x = 0.1
            #         marker.scale.y = 0.1
            #         marker.scale.z = 0.1
            #         marker.color.a = 1.0
            #         marker.color.r = 1.0
            #         marker.color.g = 0.5
            #         marker.pose.position.x = point[0]
            #         marker.pose.position.y = point[1]
            #         marker_array.markers.append(marker)
            #     self.tracked_postion_pub.publish(marker_array)


                # contribution = discounted_average(self.human_path_monitor_array)
                # self.human_path_monitor_array.pop(0)    
                # self.contrib_pub.publish(Float32(data=contribution))
                # self.human_points.pop(0)
                # if contribution > 0.01  :
                    # robot_text = 'Human is Contributing'
                    # is_contributing = True
                # else :
                    # robot_text = 'Human is not Contributing'
                    # is_contributing = False
                # self.human_is_contributing = is_contributing
                # self.human_contribution = contribution - 0.1
                # self.over_text_pub.publish(OverlayText(text=robot_text , width=700, height=300 , left=0 , top=0))





    def mf_sub_cb(self , agent_local_plan, robot_local_plan  ,crossing_info) :         
        try  :
            if self.got_new_global_tree :

                self.crossing_info =crossing_info
                self.planning_cycles += 1
                human_poses = []
                for pose in agent_local_plan.paths[0].path.poses :
                    human_poses.append([pose.pose.position.x , pose.pose.position.y , pose.pose.orientation.z , pose.pose.orientation.w])
                human_crossing_point = human_poses[crossing_info.indices[0]]
                human_post_crossing_point = human_poses[crossing_info.indices[0] + 1]
                self.agent_local_plan = agent_local_plan   
                
                robot_poses = []
                for pose in robot_local_plan.poses :
                    robot_poses.append([pose.pose.position.x , pose.pose.position.y , pose.pose.orientation.z , pose.pose.orientation.w])
                robot_crossing_point = robot_poses[crossing_info.indices[0]]
                
                robot_theta = math.atan2(robot_crossing_point[1] - human_crossing_point[1] , robot_crossing_point[0] - human_crossing_point[0])
                self.robot_distance_to_obstacle , _ = self.nearest_obstacele_w_sim_laser(robot_crossing_point[0] , robot_crossing_point[1] , robot_theta , for_robot=True)
                
                human_theta = math.atan2(human_crossing_point[1] - robot_crossing_point[1] , human_crossing_point[0] - robot_crossing_point[0])
                self.human_distance_to_obstacle , _ = self.nearest_obstacele_w_sim_laser(human_crossing_point[0] , human_crossing_point[1] , human_theta )
                
                self.human_crossing_point = human_crossing_point
                self.human_heading_angle = quat_to_euler(human_crossing_point[3] , human_crossing_point[2])
                self.robot_heading_angle = quat_to_euler(robot_crossing_point[3] , robot_crossing_point[2])
                self.robot_crossing_point = robot_crossing_point
                self.human_dx = human_post_crossing_point[0] - human_crossing_point[0]
                self.human_dy = human_post_crossing_point[1] - human_crossing_point[1]
                self.dist_human_robot_at_crossing_point = math.sqrt((human_crossing_point[0] - robot_crossing_point[0])**2 + (human_crossing_point[1] - robot_crossing_point[1])**2)


                human_poses_np = np.array(human_poses)
                points_on_local_plan = human_poses_np[crossing_info.indices[0]-5 : crossing_info.indices[0]+5]
                distance  = 0
                for points_ in points_on_local_plan: 
                    distance_to_global  = self.human_global_data_tree.query(points_[:2])[0] 
                    if distance_to_global > distance :
                        distance = distance_to_global
                        human_crossing_point = points_
                human_crossing_point_on_global_path = self.global_points[self.human_global_data_tree.query(human_crossing_point[:2])[1]]
                dist_robot_and_human_crossing_point = math.sqrt((human_crossing_point[0] - robot_crossing_point[0])**2 + (human_crossing_point[1] - robot_crossing_point[1])**2)
                dist_robot_and_human_crossing_point_on_global_path = math.sqrt((human_crossing_point_on_global_path[0] - robot_crossing_point[0])**2 + (human_crossing_point_on_global_path[1] - robot_crossing_point[1])**2)
                if dist_robot_and_human_crossing_point - dist_robot_and_human_crossing_point_on_global_path < 0 :
                    distance = -distance
                self.distance_tight_band = distance


                NUMBER_OF_PLANNING_CYCLES_TO_SKIP = 4
                if self.got_new_global_tree and self.planning_cycles > NUMBER_OF_PLANNING_CYCLES_TO_SKIP:
                    self.got_tight_band = True
                    # rospy.loginfo('Got tight band')




        except : 
            pass


    def pipeline(self , _):
        try :
            if self.got_tight_band :  
                if float(math.floor(self.crossing_info.times[0])) == 7.0  and not self.initial_comparison_done :

                    self.start_dist_data_recording = True
                    self.human_crossing_point_msg.pose.position.x = self.human_crossing_point[0]
                    self.human_crossing_point_msg.pose.position.y = self.human_crossing_point[1]
                    self.human_crossing_point_pub.publish(self.human_crossing_point_msg)
                    self.robot_crossing_point_msg.pose.position.x = self.robot_crossing_point[0]
                    self.robot_crossing_point_msg.pose.position.y = self.robot_crossing_point[1]
                    self.robot_crossing_point_pub.publish(self.robot_crossing_point_msg)
                    self.tight_band_pub.publish(self.agent_local_plan)
                    self.decision_tree.update_initial(self.human_distance_to_obstacle , self.distance_tight_band , self.dist_human_robot_at_crossing_point , self.robot_distance_to_obstacle)
                    self.decision_tree.update_initial_direction(self.robot_crossing_point[:2] , self.human_crossing_point[:2] , self.human_dx , self.human_dy , self.robot_heading_angle , self.human_heading_angle)
                    verbal_text = self.decision_tree.get_decision(initial=True)
                    if not verbal_text : 
                        verbal_text = 'No Speech Needed'
                    self.over_text_pub.publish(OverlayText(text=verbal_text , width=700, height=300 , left=0 , top=0))
                    # time.sleep(1.5)
                    self.initial_comparison_done = True

                if float(math.floor(self.crossing_info.times[0])) == 4.0 and not self.final_comparison_done :
                    # self.start_dist_data_recording = False
                    monitor_data = self.human_path_monitor_array
                    avg_of_monitor_data = discounted_average(monitor_data)
                    rospy.loginfo('Average of monitor data : ' + str(avg_of_monitor_data))
                    self.human_path_monitor_array = []
                    # monitor_data_ratio = sum
                    # self.human_is_contributing
                    self.decision_tree.update_final(self.human_distance_to_obstacle , self.distance_tight_band , self.dist_human_robot_at_crossing_point , self.human_is_contributing , self.robot_distance_to_obstacle , self.human_contribution)
                    self.decision_tree.update_final_direction(self.robot_crossing_point[:2] , self.human_crossing_point[:2] , self.human_dx , self.human_dy , self.robot_heading_angle , self.human_heading_angle)
                    verbal_text , to_dock  , direction_to_dock = self.decision_tree.get_decision(initial=False)
                    if not verbal_text : 
                        verbal_text = 'No Speech Needed'
                    self.over_text_pub.publish(OverlayText(text=verbal_text , width=700, height=300 , left=0 , top=0))
                    if to_dock :
                        rospy.loginfo('Robot is docking')
                        # self.robot_nav_utils.dock(direction=direction_to_dock)
                    # time.sleep(4.0)
                    else :
                        time.sleep(5.0)

                    monitor_data = self.human_path_monitor_array
                    avg_of_monitor_data = discounted_average(monitor_data)
                    rospy.loginfo('Average of monitor data after crossing : ' + str(avg_of_monitor_data))
                    if avg_of_monitor_data > 0.2 : 
                        is_human_contributing = True
                    else : 
                        is_human_contributing = False


                    if is_human_contributing :
                        verbal_text = 'Thanks for Contributing'
                        self.over_text_pub.publish(OverlayText(text=verbal_text , width=700, height=300 , left=0 , top=0))
                    else :
                        verbal_text = 'ROBOT INTERNAL VOICE : THE HUMAN DID NOT CONTRIBUTE'
                        self.over_text_pub.publish(OverlayText(text=verbal_text , width=700, height=300 , left=0 , top=0))

                    self.final_comparison_done = True
        except :
            pass

if __name__ == '__main__':
    rospy.init_node('ecohan', anonymous=True)
    e_cohan = ECoHAN()
    rospy.spin()