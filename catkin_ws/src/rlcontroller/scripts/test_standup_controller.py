#!/usr/bin/env python
from ambot_msgs.msg import RobotState 
from ambot_msgs.msg import RobotAction
from ambot_msgs.msg import MotorAction
import rospy
import ros_numpy
import time
class StandUp():
    def __init__(self, args,
            namespace="/ambot_v1", 
            action_topic="/actions", 
            state_topic="/states",
            ):
        """
        Init ros node and topic

        """
        self.namespace = namespace
        self.action_topic = action_topic
        self.state_topic = state_topic

        rospy.init_node("ambot_test_controller", log_level= rospy.DEBUG)
        #1) declare an action topic/action
        self.action_publisher = rospy.Publisher(
             self.namespace + self.action_topic,
             RobotAction,
             queue_size= 1,
         )
        #2) declare a subscriable topic/state
        self.state_subscriber = rospy.Subscriber(
             self.namespace + self.state_topic,
             RobotState, # message type 
             self.state_callback, # process the received message
             queue_size= 1,
            )

        self.state_get_time = rospy.Time.now()
        self.rosrate = rospy.Rate(100)
        self.dof_map= [ # from isaacgym simulation joint order to URDF order
                2, 3, 5,
                6, 7, 9,
                10, 11, 13,
                14, 15, 17,
            ], # real_joint_idx = dof_map[sim_joint_idx]
        self.default_dof_pos = [0, 0, 1.57, 0, 0, -1.57]*2
        self.wait_untill_ros_working()

    def wait_untill_ros_working(self):
        rospy.loginfo("Controller is waiting for getting robot states from robot nodes.")
        while not hasattr(self, "state_buffer"):
            self.rosrate.sleep()
        rospy.loginfo("State_buffer acquired, stop waiting.")

    def state_callback(self, msg):
        self.state_buffer = msg
        self.state_dt = rospy.Time.now() - self.state_get_time
        self.state_get_time = rospy.Time.now()
        self.dof_pos = [self.state_buffer.motorState[self.dof_map[i]].pos for i in range(self.num_dof)]

    def pub_actions(self, actions):

        joints_cmd = RobotAction()
        joints_cmd.motorAction=[MotorAction()]*24
        joints_cmd.motor_num = 24

        # using ambot_msgs.RobotAction type
        for sim_joint_idx in range(len(self.dof_map)):
            real_joint_idx = self.dof_map[sim_joint_idx]
            tmp = MotorAction() 
            tmp.pos = actions[sim_joint_idx]
            tmp.kp = self.p_gains[sim_joint_idx]
            tmp.kd = self.d_gains[sim_joint_idx]
            joints_cmd.motorAction[real_joint_idx] = tmp

        self.action_publisher.publish(joints_cmd)

    def standup_procedure(self, 
            angle_tolerance= 0.1,
            num_actions=12,
            ):
        """
        Args:
            warmup_timesteps: the number of timesteps to linearly increase the target position
        """
        rospy.loginfo("Robot standing up, please wait ...")

        target_pos = [0]*num_actions
        relative_default_target_pos = [0]*num_actions

        while not rospy.is_shutdown():
            dof_pos = [self.state_buffer.motorState[env.dof_map[i]].pos for i in range(num_actions)]
            diff = [dof_pos[i] - self.default_dof_pos[i] for i in range(num_actions)]
            direction = [1 if i > 0 else -1 for i in diff]
            transition_diff = [ angle_tolerance* i for i in direction]

            rospy.loginfo(" max joint error (rad): {:.2f}".format(max([abs(i) for i in diff])))
            if all([abs(i) < angle_tolerance for i in diff]):
                break

            for i in range(num_actions):
                target_pos[0, i] = transition_diff[i] if abs(transition_diff[i]) > angle_tolerance else relative_default_target_pos[i]
            
            self.send_action(target_pos) # send_action set increasement of control
            ros_rate.sleep()

        rospy.loginfo("Robot stood up! set motion mode to be 1 by the remote control/rosparam to continue ...")

if __name__ == "__main__":
    """ The script to run the Ambot script in ROS.
    It's designed as a main function and not designed to be a scalable code.
    """
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace",
        type= str,
        default= "/ambot_v1",                    
    )
    parser.add_argument("--logdir",
        type= str,
        help= "The log directory of the trained model",
        default= None,
    )
    parser.add_argument("--walkdir",
        type= str,
        help= "The log directory of the walking model, not for the skills.",
        default= None,
    )
    parser.add_argument("--mode",
        type= str,
        help= "The mode to determine which computer to run on.",
        choices= ["jetson", "upboard", "full"],                
    )
    parser.add_argument("--debug",
        action= "store_true",
    )

    #import pdb;pdb.set_trace()
    args, unknown = parser.parse_known_args()
    
    print(args)
    standup = StandUp(args)
    
    standup.standup_procedure()

