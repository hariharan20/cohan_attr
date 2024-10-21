#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseActionResult
from gazebo_msgs.msg import ModelState , ModelStates

class sim_env:
    def __init__(self):
        self.pub = rospy.Publisher('/human2/cmd_vel' , Twist, queue_size=10, latch=True)
        self.msg = Twist()
        self.state_pub = rospy.Publisher('/gazebo/set_model_state' , ModelState , queue_size=10 , latch=True)
        self.human_state_msg = ModelState()
        self.human_state_msg.model_name = 'human2'
        self.human_state_msg.reference_frame = 'map'
        model_stats = rospy.wait_for_message('/gazebo/model_states' , ModelStates , timeout=4.0 )
        self.human_state_msg.pose= model_stats.pose[3] # States of human2 in the scene
        self.msg.linear.x = 0.5
        self.pub.publish(self.msg)
        sub=rospy.Subscriber('move_base/result' , MoveBaseActionResult , self.end_cb)

    def reset_sim(self):
        self.msg.linear.x = 0
        self.pub.publish(self.msg)
        rospy.wait_for_service('/gazebo/reset_world')
        try :
            reset_world_service = rospy.ServiceProxy('/gazebo/reset_world' , Empty)
            reset_world_service()
            self.state_pub.publish(self.human_state_msg)
            rospy.sleep(1.0)
            self.msg.linear.x = 0.5
            self.pub.publish(self.msg)
        except rospy.ServiceException as e :
            rospy.logerr(f"Service call failed: {e}")

    def end_cb(self, data):
        self.reset_sim()
        print("Reset Done")
if __name__ == "__main__":
    rospy.init_node('human_cmd_pub')
    obj = sim_env()
    rospy.spin()