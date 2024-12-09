import numpy as np
import torch
import torch.nn.functional as F
from torch.autograd import Variable
import json
import os
import os.path as osp
from collections import OrderedDict
from typing import Tuple

import rospy
from ambot_msgs.msg import RobotState 
from ambot_msgs.msg import RobotAction
from ambot_msgs.msg import MotorAction
#from unitree_legged_msgs.msg import LowState
#from unitree_legged_msgs.msg import LegsCmd

#from unitree_legged_msgs.msg import Float32MultiArrayStamped
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from rosgraph_msgs.msg import Clock

import threading
import time
import datetime

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from multiprocessing import Process, Queue

import ros_numpy
import numpy as np

#print("cuda gpu architecture:")
#print(torch.cuda.get_arch_list())  # 返回['sm_37', 'sm_50', 'sm_60', 'sm_61', 'sm_70', 'sm_75', 'sm_80', 'sm_86', 'compute_37']
#print(torch.cuda.get_device_capability(0))  # 返回(8, 9)，代表sm_89


log_level = rospy.DEBUG
rospy.init_node("ambot_legged_gym", log_level=log_level)

@torch.no_grad()
def resize2d(img, size):
    return (F.adaptive_avg_pool2d(Variable(img), size)).data

@torch.jit.script
def quat_rotate_inverse(q, v):
    shape = q.shape
    q_w = q[:, -1]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * \
        torch.bmm(q_vec.view(shape[0], 1, 3), v.view(
            shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c

#@torch.jit.script
def quat_rotate(q, v):
    shape = q.shape
    q_w = q[:, -1]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * \
        torch.bmm(q_vec.view(shape[0], 1, 3), v.view(
            shape[0], 3, 1)).squeeze(-1) * 2.0
    return a + b + c

@torch.jit.script
def copysign(a, b):
    # type: (float, Tensor) -> Tensor
    a = torch.tensor(a, device=b.device, dtype=torch.float).repeat(b.shape[0])
    return torch.abs(a) * torch.sign(b)

@torch.jit.script
def get_euler_xyz(q):
    qx, qy, qz, qw = 0, 1, 2, 3
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (q[:, qw] * q[:, qx] + q[:, qy] * q[:, qz])
    cosr_cosp = q[:, qw] * q[:, qw] - q[:, qx] * \
        q[:, qx] - q[:, qy] * q[:, qy] + q[:, qz] * q[:, qz]
    roll = torch.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (q[:, qw] * q[:, qy] - q[:, qz] * q[:, qx])
    pitch = torch.where(torch.abs(sinp) >= 1, copysign(
        np.pi / 2.0, sinp), torch.asin(sinp))

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (q[:, qw] * q[:, qz] + q[:, qx] * q[:, qy])
    cosy_cosp = q[:, qw] * q[:, qw] + q[:, qx] * \
        q[:, qx] - q[:, qy] * q[:, qy] - q[:, qz] * q[:, qz]
    yaw = torch.atan2(siny_cosp, cosy_cosp)

    return roll % (2*np.pi), pitch % (2*np.pi), yaw % (2*np.pi)

@torch.jit.script
def normalize(x, eps: float = 1e-9):
    return x / x.norm(p=2, dim=-1).clamp(min=eps, max=None).unsqueeze(-1)

@torch.jit.script
def quat_unit(a):
    return normalize(a)


class AmbotReal:
    """ This is the handler that works for ROS 1 on ambot. """
    def __init__(self,
            extra_cfg= dict(),
            device= torch.device("cpu"),
            cfg= dict(),
        ):
        """
        NOTE:
            * Must call start_ros() before using this class's get_obs() and send_action()
            * Joint order of simulation and of real A1 protocol are different, see dof_names
            * We store all joints values in the order of simulation in this class
        Args:
            forward_depth_embedding_dims: If a real number, the obs will not be built as a normal env.
                The segment of obs will be subsituted by the embedding of forward depth image from the
                ROS topic.
            cfg: same config from amot_config but a dict object.
            extra_cfg: some other configs that is hard to load from file.
        """
        self.device = device
        self.num_envs = 1
        self.cfg = cfg
        self.dt = cfg["control"]["decimation"]*cfg["sim"]["dt"] # in sec
        self.base_lin_vel = torch.zeros((self.num_envs,3), dtype=torch.float32).to(self.device)
        self.num_dof=cfg["env"]["num_actions"]
        self.extra_cfg = dict(
            # TODO, should read torque limits from urdf file
            torque_limits= torch.tensor([10, 10, 10] * 4, dtype= torch.float32, device= self.device, requires_grad= False), # Nm
            dof_names= [ # NOTE: order matters. This list is the order in simulation.
                "FL_joint1",
                "FL_joint2",
                "FL_joint3",

                "FR_joint1",
                "FR_joint2",
                "FR_joint3",

                "HL_joint1",
                "HL_joint2",
                "HL_joint3",

                "HR_joint1",
                "HR_joint2",
                "HR_joint3",
            ],
            # motor strength is multiplied directly to the action.
            motor_strength= torch.ones(cfg["env"]["num_actions"], dtype= torch.float32, device= self.device, requires_grad= False),
        ); self.extra_cfg.update(extra_cfg)
        if "torque_limits" in self.cfg["control"]: # update torque_limits from conf file of legged_robot projects
            if isinstance(self.cfg["control"]["torque_limits"], (tuple, list)):
                for i in range(len(self.cfg["control"]["torque_limits"])):
                    self.extra_cfg["torque_limits"][i] = self.cfg["control"]["torque_limits"][i]
            else:
                self.extra_cfg["torque_limits"][:] = self.cfg["control"]["torque_limits"]
        self.commands = torch.zeros((self.num_envs, cfg['commands']['num_commands'],), 
                device= self.device, dtype= torch.float32) # zeros for initialization
        self.actions = torch.zeros((self.num_envs, cfg["env"]["num_actions"]), device= device, dtype= torch.float32)
        self.dof_pos = torch.zeros((self.num_envs, cfg["env"]["num_actions"]), device= device, dtype= torch.float32)
        self.dof_vel = torch.zeros((self.num_envs, cfg["env"]["num_actions"]), device= device, dtype= torch.float32)
        self.torques = torch.zeros((self.num_envs, cfg["env"]["num_actions"]), device= device, dtype= torch.float32)
        self.process_configs()
    
        
    def process_configs(self):
        self.up_axis_idx = 2 # 2 for z, 1 for y -> adapt gravity accordingly
        self.projected_gravity_vec = torch.zeros((self.num_envs, 3), device=self.device, dtype= torch.float32,requires_grad=False)
        self.projected_gravity_vec[:, self.up_axis_idx] = -1

        self.obs_scales = self.cfg["normalization"]["obs_scales"]
        self.obs_scales["dof_pos"] = torch.tensor(self.obs_scales["dof_pos"], device= self.device, dtype= torch.float32,requires_grad=False)

        # control config
        self.p_gains = torch.zeros(self.num_dof, device= self.device, dtype= torch.float32,requires_grad=False)
        self.d_gains = torch.zeros(self.num_dof, device= self.device, dtype= torch.float32,requires_grad=False)
        self.default_dof_pos = torch.zeros(self.num_dof, device= self.device, dtype= torch.float32, requires_grad=False)

        # NOTE, pleas do not change kp and kd if the model was trained use a actuator_net.  keep the kp and kd are same with one when collect training datat in real world
        for i in range(self.num_dof):
            name = self.extra_cfg["dof_names"][i]
            self.default_dof_pos[i] = self.cfg["init_state"]["default_joint_angles"][name] * 0.0
            for dof_name in self.cfg["control"]["stiffness"].keys():
                if dof_name in name:
                    self.p_gains[i] = self.cfg["control"]["stiffness"][dof_name]
            for dof_name in self.cfg["control"]["damping"].keys():
                if dof_name in name:
                    self.d_gains[i] = self.cfg["control"]["damping"][dof_name]

        self.torque_limits = self.extra_cfg["torque_limits"]

        # Print few basic cfg info
        rospy.loginfo("[Env] torque limit: {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f}".format(*self.torque_limits))
        rospy.loginfo("[Env] dof default pos: {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f}".format(*self.default_dof_pos))

        rospy.loginfo("[Env] dt: {}, motor Kp: {}, motor Kd: {}".format(
        self.cfg["sim"]["dt"],
        self.cfg["control"]["stiffness"],
        self.cfg["control"]["damping"],
            ))
        rospy.loginfo("[Env] commands vel deadband: {}, lin ang deadbad: {}, lin_vel_x range: {}, lin_vel_y range: {}, lin_vel_z range: {}".format(
            self.cfg["commands"]["lin_vel_deadband"],
            self.cfg["commands"]["ang_vel_deadband"],
            self.cfg["commands"]["ranges"]["lin_vel_x"],
            self.cfg["commands"]["ranges"]["lin_vel_y"],
            self.cfg["commands"]["ranges"]["ang_vel_yaw"],
        ))
        self.commands_scale = torch.tensor([self.obs_scales["lin_vel"],self.obs_scales["lin_vel"],self.obs_scales["ang_vel"]],
                                           device=self.device, requires_grad=False, )[:self.cfg["commands"]["num_commands"]]

        self.obs_scales["gait_commands"] = torch.tensor([
                                            self.obs_scales["body_height_cmd"],self.obs_scales["gait_freq_cmd"],
                                            self.obs_scales["gait_phase_cmd"],self.obs_scales["gait_phase_cmd"],
                                            self.obs_scales["gait_phase_cmd"],self.obs_scales["gait_phase_cmd"],
                                            self.obs_scales["footswing_height_cmd"],self.obs_scales["body_pitch_cmd"],
                                            self.obs_scales["body_roll_cmd"],self.obs_scales["stance_width_cmd"],
                                           self.obs_scales["stance_length_cmd"]],
                                           device=self.device, requires_grad=False,)
        #skill_vel_range= config_dict["commands"]["ranges"]["lin_vel_x"],
        self.lin_vel_deadband= torch.tensor(self.cfg["commands"]["lin_vel_deadband"],dtype=torch.float32, device=self.device)
        self.ang_vel_deadband= torch.tensor(self.cfg["commands"]["ang_vel_deadband"],dtype=torch.float32, device=self.device)

        # get obs config
        self.obs_segments = self.get_obs_segment_from_components(self.cfg["env"]["obs_components"])
        self.num_obs = self.get_num_obs_from_components(self.cfg["env"]["obs_components"])
        components = self.cfg["env"].get("privileged_obs_components", None)
        self.privileged_obs_segments = None if components is None else self.get_num_obs_from_components(components)
        self.num_privileged_obs = None if components is None else self.get_num_obs_from_components(components)
        self.all_obs_components = self.cfg["env"]["obs_components"] + (self.cfg["env"].get("privileged_obs_components", []) if components is not None else [])

        # store config values to attributes to improve speed
        self.clip_obs = self.cfg["normalization"]["clip_observations"]
        self.control_type = self.cfg["control"]["control_type"]
        self.action_scale = self.cfg["control"]["action_scale"]
        rospy.loginfo("[Env] action scale: {:.1f}".format(self.action_scale))
        self.clip_actions = self.cfg["normalization"]["clip_actions"]
        if self.cfg["normalization"].get("clip_actions_method", None) == "hard":
            rospy.loginfo("clip_actions_method with hard mode")
            rospy.loginfo("clip_actions_high: " + str(self.cfg["normalization"]["clip_actions_high"]))
            rospy.loginfo("clip_actions_low: " + str(self.cfg["normalization"]["clip_actions_low"]))
            self.clip_actions_method = "hard"
            self.clip_actions_low = torch.tensor(self.cfg["normalization"]["clip_actions_low"], 
                    device=self.device, dtype=torch.float32,requires_grad=False)
            self.clip_actions_high = torch.tensor(self.cfg["normalization"]["clip_actions_high"], 
                    device= self.device, dtype= torch.float32,requires_grad=False)
        else:
            rospy.loginfo("clip_actions_method is " + str(self.cfg["normalization"].get("clip_actions_method", None)))



        if "gait_clock" in self.all_obs_components:
            self.num_leg = 4
            self.gait_indices = torch.zeros(self.num_envs, dtype=torch.float, 
                    device=self.device, requires_grad=False)
            self.clock_inputs = torch.zeros(self.num_envs, self.num_leg, dtype=torch.float, 
                    device=self.device,requires_grad=False)

        if "gait_commands" in self.all_obs_components:
            self.gait_commands = torch.zeros(self.num_envs, cfg["commands"]["num_gait_commands"], dtype=torch.float, device=self.device, requires_grad=False) # x vel, y vel, yaw vel, heading

        if "forward_depth" in self.all_obs_components:
            resolution = self.cfg["sensor"]["forward_camera"].get(
                "output_resolution",
                self.cfg["sensor"]["forward_camera"]["resolution"],
            )
            if not self.forward_depth_embedding_dims:
                self.forward_depth_buf = torch.zeros(
                    (self.num_envs, *resolution),
                    device= self.device,
                    dtype= torch.float32,
                    requires_grad=False
                )
            else:
                self.forward_depth_embedding_buf = torch.zeros(
                    (1, self.forward_depth_embedding_dims),
                    device= self.device,
                    dtype= torch.float32,
                    requires_grad=False
                )


    # get commands from ros
    def get_commands(self, commands):
        self.commands = commands
        # remap the commands from -1 to 1 to commands_rang
        rng = self.cfg["commands"]["ranges"]["lin_vel_x"]
        self.commands[:,0] = (rng[1]-rng[0])/2*(self.commands[:,0]-1)+rng[1]
        rng = self.cfg["commands"]["ranges"]["lin_vel_y"]
        self.commands[:,1] = (rng[1]-rng[0])/2*(self.commands[:,1]-1)+rng[1]
        rng = self.cfg["commands"]["ranges"]["ang_vel_yaw"]
        self.commands[:,2] = (rng[1]-rng[0])/2*(self.commands[:,2]-1)+rng[1]
        # set small commands to zero, is 
        # TODO comment these to enable low walking speed
        self.commands[0, :2] *= (torch.norm(self.commands[0, :2]) > self.lin_vel_deadband)
        self.commands[0, 2] *=  (torch.abs(self.commands[0, 2]) > self.ang_vel_deadband)

    
    # Get state buffer from ros message
    def get_state(self, state_buffer):
        self.state_buffer = state_buffer


    def _init_height_points(self):
        """ Returns points at which the height measurments are sampled (in base frame)

        Returns:
            [torch.Tensor]: Tensor of shape (num_envs, self.num_height_points, 3)
        """
        return None
    
    def _get_heights(self):
        """ TODO: get estimated terrain heights around the robot base """
        # currently return a zero tensor with valid size
        return torch.zeros(self.num_envs, 187, device= self.device, requires_grad= False)
    

    """ Get obs components and cat to a single obs input """
    def _get_proprioception_obs(self):
        # base_ang_vel = quat_rotate_inverse(
        #     torch.tensor(self.state_buffer.imu.quaternion).unsqueeze(0),
        #     torch.tensor(self.state_buffer.imu.gyroscope).unsqueeze(0),
        # ).to(self.device)
        # NOTE: Different from the isaacgym. 

        # NOTE: Mujoco linvear velocity of free joints are in global frame, the orientation of free joints are also in global frame,
        # but the rotational velocities of a free joint are in the local frame. 
        # NOTE: How about the VN100 coordinate of gyroscope, 

        # The anglar velocity is already in base frame, no need to rotate
        self.base_lin_accel = torch.tensor([
            self.state_buffer.imu.acceleration.x, 
            self.state_buffer.imu.acceleration.y, 
            self.state_buffer.imu.acceleration.z], 
            device=self.device,requires_grad=False).unsqueeze(0)

        self.base_ang_vel = torch.tensor([
            self.state_buffer.imu.gyroscope.x, 
            self.state_buffer.imu.gyroscope.y, 
            self.state_buffer.imu.gyroscope.z], 
            device= self.device,requires_grad=False).unsqueeze(0)

        base_quat = torch.tensor([
            self.state_buffer.imu.quaternion.x, 
            self.state_buffer.imu.quaternion.y, 
            self.state_buffer.imu.quaternion.z, 
            self.state_buffer.imu.quaternion.w, 
            ], device=self.device,requires_grad=False).unsqueeze(0)
        self.base_quat = quat_unit(base_quat.cpu())

        self.projected_gravity = quat_rotate_inverse(self.base_quat, self.projected_gravity_vec.cpu()).to(self.device)
        self.base_lin_vel = torch.tensor([
            self.state_buffer.imu.linvel.x, 
            self.state_buffer.imu.linvel.y, 
            self.state_buffer.imu.linvel.z], 
            device=self.device,requires_grad=False).unsqueeze(0)

        # disbale lin vel since they are not used in training and also not accessiable in real robot
        if not self.cfg["env"]["use_lin_vel"]:
            self.base_lin_vel[:] = 0.
        
        # get dof pos
        self.dof_pos = torch.tensor([
            self.state_buffer.motorState[i].pos for i in range(self.num_dof)
        ], dtype= torch.float32, device= self.device,requires_grad=False).unsqueeze(0)
        # get dof vel
        self.dof_vel = torch.tensor([
            self.state_buffer.motorState[i].vel for i in range(self.num_dof)
        ], dtype= torch.float32, device= self.device, requires_grad=False).unsqueeze(0)
        
        return torch.cat([
            self.base_lin_vel * self.obs_scales["lin_vel"],
            self.base_ang_vel * self.obs_scales["ang_vel"],
            self.projected_gravity,
            self.commands * self.commands_scale,
            (self.dof_pos - self.default_dof_pos) * self.obs_scales["dof_pos"],
             self.dof_vel * self.obs_scales["dof_vel"],
             self.actions
            ], dim= -1)

    def _get_forward_depth_obs(self):
        if not self.forward_depth_embedding_dims:
            return self.forward_depth_buf.flatten(start_dim= 1)
        else:
            if self.low_state_get_time.to_sec() - self.forward_depth_embedding_stamp.to_sec() > 0.4:
                rospy.logerr("Getting depth embedding later than low_state later than 0.4s")
            return self.forward_depth_embedding_buf.flatten(start_dim= 1)

    def _get_gait_clock_obs(self):
        frequencies = self.commands[:, 4]
        phases = self.commands[:, 5]
        offsets = self.commands[:, 6]
        bounds = self.commands[:, 7]
        durations = self.commands[:, 8]
        self.gait_indices = torch.remainder(self.gait_indices + self.dt * frequencies, 1.0)


        if getattr(self.cfg["commands"], "pacing_offset", False):
            foot_indices = [self.gait_indices + phases + offsets + bounds,
                            self.gait_indices + bounds,
                            self.gait_indices + offsets,
                            self.gait_indices + phases]
        else:
            foot_indices = [self.gait_indices + phases + offsets + bounds,
                            self.gait_indices + offsets,
                            self.gait_indices + bounds,
                            self.gait_indices + phases]

        # if self.cfg.commands.durations_warp_clock_inputs:
        for idx in range(self.num_leg):
            self.clock_inputs[:, idx] = torch.sin(2 * np.pi * foot_indices[idx]) # time reference variables

        return self.clock_inputs

    def _get_gait_commands_obs(self, privileged=False):
        return  self.gait_commands


    def compute_observation(self):
        """ use the updated state_buffer to compute observation vector """
        obs_segments = self.obs_segments
        obs = []
        for k, v in obs_segments.items():
            obs.append(
                getattr(self, "_get_" + k + "_obs")() * \
                self.obs_scales.get(k, 1.)
            )
        obs = torch.cat(obs, dim= 1)
        self.obs_buf = obs


    def get_obs(self):
        """ The function that refreshes the buffer and return the observation vector.
        """
        self.compute_observation()
        self.obs_buf = torch.clip(self.obs_buf, -self.clip_obs, self.clip_obs)
        return self.obs_buf.to(self.device)



    """ Copied from legged_robot_field. Please check whether these are consistent. """
    def get_obs_segment_from_components(self, components):
        segments = OrderedDict()
        if "proprioception" in components:
            segments["proprioception"] = (3+3+3+self.cfg["commands"]["num_commands"]+3*self.cfg['env']['num_actions'],)
        if "height_measurements" in components:
            segments["height_measurements"] = (187,)
        if "forward_depth" in components:
            resolution = self.cfg["sensor"]["forward_camera"].get(
                "output_resolution",
                self.cfg["sensor"]["forward_camera"]["resolution"],
            )
            segments["forward_depth"] = (1, *resolution)
        # The following components are only for rebuilding the non-actor module.
        # DO NOT use these in actor network and check consistency with simulator implementation.
        if "base_pose" in components:
            segments["base_pose"] = (6,) # xyz + rpy
        if "robot_config" in components:
            segments["robot_config"] = (1 + 3 + 1,)
        if "motor_config" in components:
            """ Related to robot_config_buffer attribute, Be careful to change. """
            # motor offset for each joint
            segments["motor_config"] = (self.cfg['env']['num_actions']*4,)
        if "engaging_block" in components:
            # This could be wrong, please check the implementation of BarrierTrack
            segments["engaging_block"] = (1 + (4 + 1) + 2,)
        if "sidewall_distance" in components:
            segments["sidewall_distance"] = (2,)
        if "gait_clock" in components:
            segments["gait_clock"] = (4,)
        return segments

        
    def get_num_obs_from_components(self, components):
        obs_segments = self.get_obs_segment_from_components(components)
        num_obs = 0
        for k, v in obs_segments.items():
            num_obs += np.prod(v)
        return num_obs



    def clip_action_by_torque_limit(self, actions):
        """ For position control, scaled actions should be in the coordinate of robot default dof pos
        """
        dof_vel = self.dof_vel
        dof_pos_ = self.dof_pos - self.default_dof_pos
        p_limits_low = (-self.torque_limits) + self.d_gains*dof_vel
        p_limits_high = (self.torque_limits) + self.d_gains*dof_vel
        actions_low = ((p_limits_low/self.p_gains) + dof_pos_)/(self.action_scale+1e-9)
        actions_high = ((p_limits_high/self.p_gains) + dof_pos_)/(self.action_scale+1e-9)
        actions_torque_clipped = torch.clip(actions, actions_low, actions_high)
        return actions_torque_clipped


    """ The methods combined with outer model forms the step function
    NOTE: the outer user handles the loop frequency.
    """
    def process_action(self, actions,**kwargs):
        """ The function that send commands to the real robot.

        previous process action, inlcuding clip, scale, and add default pos
        """
        clip_actions = self.cfg["normalization"]["clip_actions"]
        self.actions = torch.clip(actions, -clip_actions, clip_actions).to(self.device).unsqueeze(0)

        if self.cfg["control"]["computer_clip_torque"]:
            self.actions = self.clip_action_by_torque_limit(self.actions)

        actions_scaled =  self.actions * self.action_scale
        robot_coordinates_action =  actions_scaled + self.default_dof_pos.unsqueeze(0)

        return robot_coordinates_action



import rospy
import time 
import pandas as pd
from collections import OrderedDict
import csv
def record_params(q_rosparams):
    keys = q_rosparams.get()[0]
    params = q_rosparams.get()[1]

    data_file = os.path.join(os.getenv("AMBOT"),"AmbotRL/legged_gym/logs/checking_obs",
            datetime.datetime.now().strftime("%m_%d_%H_%M")+"_rosparams.csv")
    print(f"rosparams saving at: {data_file}")
    with open(data_file, "w",newline="\n") as fd:
        fd.write(f"\t".join(["time"] + keys))
        fd.write(f"\n")

    while not rospy.is_shutdown():
        parms = q_rosparams.get()
        with open(data_file, "a", newline='\n' ) as fd:
            fd = csv.writer(fd,delimiter='\t')
            for i in range(len(params["time"])):
                fd.writerow([params[key][i] for key in keys])

        time.sleep(0.2)  # Adjust the sleep duration as needed


class RosInterface():
    def __init__(self,
            env,
            rate,
            namespace="ambotv2",
            forward_depth_topic="abc",
            forward_depth_embedding_dims="abcd",
            odom_topic="abce",
            subscribe_move_cmd=True,
            ):


        # ROS params
        #log_level = rospy.DEBUG
        #rospy.init_node("ambot_legged_gym", log_level=log_level)
        self.robot_namespace = namespace
        subscribeTopics = rospy.get_param(self.robot_namespace + "/subscribeTopics")
        advertiseTopics = rospy.get_param(self.robot_namespace + "/advertiseTopics")
        self.state_topic = self.robot_namespace+"/"+subscribeTopics[0]
        self.action_topic = self.robot_namespace+"/"+advertiseTopics[0]
        rospy.loginfo(f"state topic: {self.state_topic}")
        rospy.loginfo(f"action topic: {self.action_topic}")

        self.subscribe_move_cmd = subscribe_move_cmd
        self.rate = rospy.Rate(rate)

        # env
        self.env=env
        self.device = self.env.device
        self.num_dof = self.env.num_dof
        self.commands = self.env.commands.clone()

        self.forward_depth_topic = forward_depth_topic
        self.forward_depth_embedding_dims = forward_depth_embedding_dims
        self.odom_topic = odom_topic

        self.low_state_dt = 0.0
        self.low_state_get_time = rospy.Time.now()


    def start_ros(self):

        # Ros topics
        self.action_publisher = rospy.Publisher(
            self.action_topic,
            RobotAction,
            queue_size= 1,
        )
        #This launches the subscriber callback function
        name = self.state_topic
        #if name == rospy.remap_name(name):
        #    rospy.logwarn("topic '%s' is not remapped" % name)
        self.state_subscriber = rospy.Subscriber(
            self.state_topic,
            RobotState, # message type 
            self.update_low_state, # process the received message
            queue_size= 1,
        )

        self.odom_subscriber = rospy.Subscriber(
            self.robot_namespace + self.odom_topic,
            Odometry,
            self.update_base_pose,
            queue_size= 1,
        )
        if(self.subscribe_move_cmd):
            self.move_cmd_subscriber = rospy.Subscriber(
                self.robot_namespace + "/cmd_vel",
                Twist,
                self.update_move_cmd,
                queue_size= 1,
                )
        if "forward_depth" in self.env.all_obs_components:
            if not self.forward_depth_embedding_dims:
                self.forward_depth_subscriber = rospy.Subscriber(
                    self.robot_namespace + self.forward_depth_topic,
                    Image,
                    self.update_forward_depth,
                    queue_size= 1,
                )
            else:
                self.forward_depth_subscriber = rospy.Subscriber(
                    self.robot_namespace + self.forward_depth_topic,
                    Float32MultiArrayStamped,
                    self.update_forward_depth_embedding,
                    queue_size= 1,
                )
        if "base_config" in self.env.all_obs_components:
            self.pose_cmd_subscriber = rospy.Subscriber(
                "/body_pose",
                Pose,
                self.dummy_handler,
                queue_size= 1,
            )



        # initialze several buffers so that the system works even without message update.
        self.base_position_buffer = torch.zeros((self.env.num_envs, 3), device= self.device, requires_grad= False)


        # get ROS params for hardware configs, consider to remove this instead of using urdf limits
        leg_name =  rospy.get_param(self.robot_namespace + "/leg_name")
        joint_name =  rospy.get_param(self.robot_namespace + "/joint_name")
        self.joint_limits_high = torch.tensor([
            rospy.get_param(self.robot_namespace + "/max_joint_limits/{}".format(leg+"_"+joint))/180*np.pi \
                    for leg in leg_name for joint in joint_name
        ]).to(self.device)
        self.joint_limits_low = torch.tensor([
            rospy.get_param(self.robot_namespace + "/min_joint_limits/{}".format(leg+"_"+joint))/180*np.pi \
                    for leg in leg_name for joint in joint_name
        ]).to(self.device)
    
    def wait_untill_ros_working(self):
        rospy.loginfo("Controller is waiting for getting robot states from robot nodes.")
        while not hasattr(self, "state_buffer"):
            self.rate.sleep()
        self.sleep_dof_pos = torch.tensor([self.state_buffer.motorState[i].pos for i in range(self.num_dof)]).to(self.device).unsqueeze(0)
        rospy.loginfo("AmbotReal.state_buffer acquired, stop waiting.")

        # start a process to fetch ros parameter server
        self.rosparam_server()
        
        # record rosparams
        self.q_rosparams = Queue()
        time.sleep(1)  # Adjust the sleep duration as needed
        record_params_p = Process(target=record_params, args=(self.q_rosparams,))
        record_params_p.start()
        self.q_rosparams.put([self.rosparam_keys, self.rosparams])

    # Ros params
    def rosparam_server(self):
        self.lock  =  threading.Lock()
        thread = threading.Thread(target=self._rosparam_server, daemon=True)
        thread.start()

    def _rosparam_server(self):
        # ros params
        self.motion_mode=0
        self.gaits = OrderedDict({"pronking": [0, 0, 0],
             "trotting": [0.5, 0, 0],
             "bounding": [0, 0.5, 0],
             "pacing": [0, 0, 0.5]})
        
        all_names = rospy.get_param_names()
        names = []
        self.rosparam_keys = []
        for name in all_names:
            if "v2/operation_cmd" in name:
                names.append(name)
                self.rosparam_keys.append(name.split("/")[-1])

       
        self.rosparams = {key: [] for key in self.rosparam_keys}
        self.rosparams["time"] = []

        while not rospy.is_shutdown():
            self.lock.acquire()
            self.motion_mode = int(rospy.get_param(self.robot_namespace + "/operation_cmd/motion_mode"))
            gait = int(rospy.get_param(self.robot_namespace + "/operation_cmd/gait"))
            if(not self.subscribe_move_cmd):
                self.commands[:, 0] = float(rospy.get_param(self.robot_namespace + "/operation_cmd/Vx"))
                self.commands[:, 1] = float(rospy.get_param(self.robot_namespace + "/operation_cmd/Vy"))
                self.commands[:, 2] = float(rospy.get_param(self.robot_namespace + "/operation_cmd/Rz"))
            if "gait_commands" in self.env.all_obs_components:
                self.gait_commands[:, 0] = float(rospy.get_param(self.robot_namespace + "/operation_cmd/body_height_cmd"))
                self.gait_commands[:, 1]= float(rospy.get_param(self.robot_namespace + "/operation_cmd/gait_frequency_cmd"))
                self.gait_commands[:, 2:5] = torch.tensor(self.gaits[list(self.gaits.keys())[gait]])
                self.gait_commands[:, 5] = float(rospy.get_param(self.robot_namespace + "/operation_cmd/gait_duration"))
                self.gait_commands[:, 6] = float(rospy.get_param(self.robot_namespace + "/operation_cmd/feetswing_height_cmd"))
                self.gait_commands[:, 7] = float(rospy.get_param(self.robot_namespace + "/operation_cmd/body_pitch"))
                self.gait_commands[:, 8] = float(rospy.get_param(self.robot_namespace + "/operation_cmd/body_roll"))
                self.gait_commands[:, 9] = float(rospy.get_param(self.robot_namespace + "/operation_cmd/stance_width_cmd"))
                self.gait_commands[:, 10] = float(rospy.get_param(self.robot_namespace + "/operation_cmd/stance_length_cmd"))
            self.env.get_commands(self.commands)

            for key, name in zip(self.rosparam_keys, names):
                value = rospy.get_param(name, '0.0')
                self.rosparams[key].append(value)
            
            self.rosparams["time"].append(rospy.get_time())


            # send out rosparams buf
            self.q_rosparams.put([self.rosparam_keys, self.rosparams])

            # clear rosparams buf
            self.rosparams = {key: [] for key in self.rosparam_keys}
            self.rosparams["time"] = []

            self.lock.release()
            time.sleep(0.2)


    #from multiprocessing import Process, Queue
    #q_actions = Queue()
    #q_observations = Queue()




    def publish_joints_cmd(self, robot_coordinates_action):
        """ publish the joint position directly to the robot. NOTE: The joint order from input should
        be in simulation order. The value should be absolute value rather than related to dof_pos.
        """

        # NOTE, This clip desired joint position. It is not action.
        # NOTE, for PD controller, there is a large joint tracking error, so, maybe disbale this clip.
        #robot_coordinates_action = torch.clip(
        #    robot_coordinates_action,
        #    self.joint_limits_low,
        #    self.joint_limits_high,
        #)
        
        #tmp_torque= self.p_gains * (robot_coordinates_action - self.dof_pos) -  self.d_gains * self.dof_vel
        #if(any(abs(max(tmp_torque))> self.torque_limits)):
        #    print("Warning exceed max torques, the torques are:",tmp_torque)

        joints_cmd = RobotAction()
        joints_cmd.motorAction=[MotorAction()]*self.num_dof
        joints_cmd.motor_num = self.num_dof

        # using ambot_msgs.RobotAction type
        for idx in range(self.num_dof):
            tmp = MotorAction() 
            tmp.q = robot_coordinates_action[0, idx]
            tmp.dq = 0
            tmp.Kp = self.env.p_gains[idx]
            tmp.Kd = self.env.d_gains[idx]
            tmp.tor = 0
            joints_cmd.motorAction[idx] = tmp

        self.action_publisher.publish(joints_cmd)


    def send_action(self,actions, **kwargs):
        # convert actions generated by policy to desired joint position
        robot_coordinates_action = self.env.process_action(actions, **kwargs)
        # wrap the message and publish
        self.publish_joints_cmd(robot_coordinates_action)


    """ ROS callbacks and handlers that update the buffer """
    def update_low_state(self, ros_msg):
        self.state_buffer = ros_msg
        self.low_state_dt = rospy.Time.now() - self.low_state_get_time
        self.low_state_get_time = rospy.Time.now()
        self.env.get_state(self.state_buffer)

    """ ROS callbacks and handlers that update the buffer """
    def update_debug_state(self, ros_msg):
        self.debug_state_buffer = ros_msg
        self.low_state_dt = rospy.Time.now() - self.low_state_get_time
        self.low_state_get_time = rospy.Time.now()

    def update_base_pose(self, ros_msg):
        """ update robot odometry for position """
        self.base_position_buffer[0, 0] = ros_msg.pose.pose.position.x
        self.base_position_buffer[0, 1] = ros_msg.pose.pose.position.y
        self.base_position_buffer[0, 2] = ros_msg.pose.pose.position.z

    def update_move_cmd(self, ros_msg): # by a cmd topic, rather than joystick 
        self.commands[0, 0] = ros_msg.linear.x
        self.commands[0, 1] = ros_msg.linear.y
        self.commands[0, 2] = ros_msg.angular.z
        
        self.env.get_commands(self.commands)
        

    def update_forward_depth(self, ros_msg):
        # TODO not checked.
        self.forward_depth_header = ros_msg.header
        buf = ros_numpy.numpify(ros_msg)

        self.env.forward_depth_buf = resize2d(
            torch.from_numpy(buf.astype(np.float32)).unsqueeze(0).unsqueeze(0).to(self.device),
            self.forward_depth_buf.shape[-2:],
        )

    def update_forward_depth_embedding(self, ros_msg):
        rospy.loginfo_once("ambot_ros_run recieved forward depth embedding.")
        self.forward_depth_embedding_stamp = ros_msg.header.stamp

        self.env.forward_depth_embedding_buf[:] = torch.tensor(ros_msg.data).unsqueeze(0) # (1, d)

    def dummy_handler(self, ros_msg):
        """ To meet the need of teleop-legged-robots requirements """
        pass




