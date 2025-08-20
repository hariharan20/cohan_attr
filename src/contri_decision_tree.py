#! /usr/bin/env python
import rospy
import numpy as np
import math


class DecisionTree:
    def __init__(self , speak_anyways = False) :
        self.position_dict =  {
            ('left' , 'behind') : {
                1 : 'follow and pass by left ',
                90 : 'cross behind from left',
                180 : 'pass by left,move right',
                270 : 'cross behind from right',
                359 : 'follow and pass by left',
            },
            ('right' , 'behind') : {
                1 : 'follow and pass by right',
                90 : 'cross behind from left',
                180 : 'pass by right,move left',
                270 : 'cross behind from right',
                359 : 'follow and pass by right',
            },
            ('left' , 'front') : {
                1 : 'follow and pass by left',
                90 : 'cross in front from left',
                180 : 'pass by left,move right',
                270 : 'cross in front from right',
                359 : 'follow and pass by left',

            },
            ('right' , 'front') : {
                0 : 'follow and pass by right',
                90 : 'crossing in front from left',
                180 : 'pass by right,move left',
                270 : 'crossing in front from right',
                359 : 'follow and pass by right',
            },
        }


        self.initially_contri_needed = None
        self.initially_contri_can = None
        self.initially_crossing_point_too_tight = None
        self.finally_contri_needed = None
        self.finally_contri_can = None
        self.finally_crossing_point_too_tight = None
        self.human_is_contributing = None
        self.intially_robot_can_contribute = None
        self.finally_robot_can_Contribute = None
        self.THRESHOLD_DIST_TO_GLOBAL = 0.2
        self.HUMAN_THRESHOLD_TO_OBSTACLE = 1.0
        self.HUMAN_ROBOT_DISTANCE_THRESHOLD = 1.0
        self.ROBOT_THRESHOLD_TO_OBSTACLE = 0.3
        self.left_or_right = None
        self.front_or_behind = None
        self.crossing_direction = None


    def update_pose_wrt_human(self , robot_position , human_position , human_heading_dx , human_heading_dy):
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
        self.left_or_right = left_or_right
        self.front_or_behind = front_or_behind


    def rad_to_deg2(self, rad) : 
        if rad < 0 : 
            return 360 + (rad *180 / math.pi)
        return rad * 180 / math.pi


    def reset(self)  :
        self.initially_contri_needed = None
        self.initially_contri_can = None
        self.initially_crossing_point_too_tight = None
        self.finally_contri_needed = None
        self.finally_contri_can = None
        self.finally_crossing_point_too_tight = None
        self.human_is_contributing = None
        self.left_or_right = None
        self.front_or_behind = None
        self.crossing_direction = None



    def update_initial(self , dist_to_obstacle , dist_to_global , human_robot_distance  , robot_dist_to_obstacle)  :
        self.initial_human_need = [True if dist_to_global > self.THRESHOLD_DIST_TO_GLOBAL else False][0]
        self.initial_human_needed_contri = dist_to_global
        self.initial_human_comfortably_can = [True if dist_to_obstacle > self.HUMAN_THRESHOLD_TO_OBSTACLE else False][0]
        self.initial_crossing_point_comfortable = [True if (((robot_dist_to_obstacle - self.ROBOT_THRESHOLD_TO_OBSTACLE) + (human_robot_distance - self.HUMAN_ROBOT_DISTANCE_THRESHOLD)) > 0) else False][0]         
    
    
    def update_final(self , dist_to_obstacle , dist_to_global , human_robot_distance  , is_contributing , robot_dist_to_obstacle , contribution)  :
        self.final_human_need = [True if self.initial_human_needed_contri - contribution > 0 else False][0]

        self.final_human_comfortably_can = [True if dist_to_obstacle > self.HUMAN_THRESHOLD_TO_OBSTACLE else False][0]
        self.final_crossing_point_comfortable = [True if (((robot_dist_to_obstacle - self.ROBOT_THRESHOLD_TO_OBSTACLE) + (human_robot_distance - self.HUMAN_ROBOT_DISTANCE_THRESHOLD)) > 0) else False][0]
        self.human_contribution = contribution
        self.final_human_need_contri = self.initial_human_needed_contri - contribution
        rospy.loginfo(str(self.initial_human_needed_contri) + str(' , ') + str(contribution) )

    def update_initial_direction(self, robot_position , human_position , human_dx , human_dy , robot_heading_angle , human_heading_angle):
        angle_of_robot_wrt_human = self.rad_to_deg2(robot_heading_angle - human_heading_angle)
        self.update_pose_wrt_human(robot_position , human_position , human_dx , human_dy)
        angle_dict = self.position_dict[(self.left_or_right ,self.front_or_behind)]
        angle_dict_keys = list(angle_dict.keys())
        angle_dict_keys = np.array(angle_dict_keys)
        angle_difference = np.abs(angle_dict_keys - angle_of_robot_wrt_human)
        min_index = np.argmin(angle_difference)
        self.initial_crossing_direction = angle_dict[angle_dict_keys[min_index]]


    def update_final_direction(self, robot_position , human_position , human_dx , human_dy , robot_heading_angle , human_heading_angle):
        angle_of_robot_wrt_human = self.rad_to_deg2(robot_heading_angle - human_heading_angle)
        self.update_pose_wrt_human(robot_position , human_position , human_dx , human_dy)
        angle_dict = self.position_dict[(self.left_or_right ,self.front_or_behind)]
        angle_dict_keys = list(angle_dict.keys())
        angle_dict_keys = np.array(angle_dict_keys)
        angle_difference = np.abs(angle_dict_keys - angle_of_robot_wrt_human)
        min_index = np.argmin(angle_difference)
        self.crossing_direction = angle_dict[angle_dict_keys[min_index]]
 

    def get_decision(self , initial=False) :
        verbal_text = None 
        to_dock = False
        direction_to_dock = None

        if initial : 
            rospy.loginfo('Initial Human Need : ' + str(self.initial_human_need))
            rospy.loginfo('Initial Human Comfortably Can : ' + str(self.initial_human_comfortably_can))
            rospy.loginfo('Initial Crossing Point Comfortably : ' + str(self.initial_crossing_point_comfortable))
            if self.initial_human_need and (not self.initial_crossing_point_comfortable) :
                verbal_text = 'The crossing will be tight '
                if self.initial_human_comfortably_can : 
                    verbal_text += 'Could you please ' + self.initial_crossing_direction.split(',')[1]  #### CASE 1 # Open Space, Less Space for Robot, Human is Near Robot # RECORDED
                else : 
                    verbal_text += 'I will dock if needed' ####### CASE 2 # Tigh Space , Less Space for Robot, Human is Near Robot ## RECORDED
            elif  self.initial_human_need and self.initial_crossing_point_comfortable :
                verbal_text = 'I will ' + self.initial_crossing_direction.split(',')[0] + 'of you' ##### CASE 3 # Open Space, More Space for Robot, Human is Near Robot #MAY NOT HAPPEND
            return verbal_text 
        else: 
            rospy.loginfo('Final Human Need : ' + str(self.final_human_need))
            rospy.loginfo('Final Human Comfortably Can : ' + str(self.final_human_comfortably_can))
            rospy.loginfo('Final Crossing Point Comfortably : ' + str(self.final_crossing_point_comfortable))
            if np.abs(self.human_contribution) < 0.1 : 
                self.human_contribution = 0
            rospy.loginfo('Human Contribution : ' + str(self.human_contribution))
            if self.final_human_need and (not self.final_crossing_point_comfortable) : 
                if self.human_contribution > 0 :
                    verbal_text = 'Could you please move a bit more' ##### CASE 4 # Cooperative Human, Less Space for Robot, Human is Near Robot, 
                else : 
                    verbal_text = 'I will dock, Could you please ' + self.crossing_direction.split(',')[1] + ' of you' ##### CASE 5 # Non Cooperative Human, Less Space for Robot, Human is Near Robot , ## RECORDED
                    to_dock = True 
            elif self.final_human_need and self.final_crossing_point_comfortable :
                verbal_text = 'I will ' + self.crossing_direction.split(',')[0] + ' of you' ##### CASE 6 # More Space for Robot, Human is Near Robot
            elif (not self.final_human_need) and (not self.final_crossing_point_comfortable) :
                verbal_text = 'I will ' + self.crossing_direction.split(',')[0] + ' of you' ##### CASE 7 # Less Space for Robot, Human is Far from Robot
            

            if to_dock : 
                direction_to_dock = self.crossing_direction.split(',')[0]
                if 'left' in direction_to_dock :
                    direction_to_dock = 'right'
                elif 'right' in direction_to_dock :
                    direction_to_dock = 'left'
            return verbal_text , to_dock , direction_to_dock