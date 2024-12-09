#!/home/ubuntu/.pyenv/versions/machinelearning/bin/python3
#!/home/cc/.pyenv/versions/machinelearning/bin/python3
#!/home/suntao/.pyenv/versions/parkour/bin/python3
#!/usr/bin/env python3
import os
import os.path as osp
import json
import numpy as np
import torch
from collections import OrderedDict
from functools import partial
from typing import Tuple

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import rospy
import ros_numpy
import time
import datetime
import time
from multiprocessing import Process, Queue

from ambot_real import AmbotReal, RosInterface, resize2d, get_euler_xyz
from logger import Logger
from rsl_rl import modules
from rsl_rl.utils.utils import get_obs_slice
import copy
from load_wtw_policy import *

@torch.no_grad()
def handle_forward_depth(ros_msg, model, publisher, output_resolution, device):
    """ The callback function to handle the forward depth and send the embedding through ROS topic """
    buf = ros_numpy.numpify(ros_msg).astype(np.float32)
    forward_depth_buf = resize2d(
        torch.from_numpy(buf).unsqueeze(0).unsqueeze(0).to(device),
        output_resolution,
    )
    embedding = model(forward_depth_buf)
    ros_data = embedding.reshape(-1).cpu().numpy().astype(np.float32)
    publisher.publish(Float32MultiArray(data= ros_data.tolist()))

# recoding obs data
@torch.no_grad()
def recording_obs(q_obs, q_idx):
    recording_obs_buf=[]
    obs_buf_len=100
    old_idx = -1
    while(1):
        obs = q_obs.get()
        idx = q_idx.get()
        if idx <= obs_buf_len:
            if idx > old_idx:
                recording_obs_buf.append(obs.detach())
                old_idx = idx
        
        if idx==obs_buf_len:
            import pandas as pd
            np_array = [tensor.numpy() for tensor in recording_obs_buf]
            np_array_stacked = np.vstack(np_array)
            df = pd.DataFrame(np_array_stacked)
            header= [prefix+str(idx) for prefix in ["base_linvel_", "base_angvel_", "gravity_"] for idx in range(3)] + \
                    ["command_"+str(idx) for idx in range(3)] + \
                    ["pos_"+str(idx) for idx in range(12)] + \
                    ["vel_"+str(idx) for idx in range(12)] + \
                    ["action_"+str(idx) for idx in range(12)]
            header+=["gait_clock_"+str(idx) for idx in range(4)] if obs.shape[1]>48 else []
            header+=["gait_command_"+str(idx) for idx in range(11)] if obs.shape[1]>52 else []
            base_path=os.path.join(os.getenv("AMBOT"),"AmbotRL/legged_gym")
            folder = os.path.join(base_path, "logs/checking_obs", datetime.datetime.now().strftime("%m_%d_%H_%M"))
            os.makedirs(folder, exist_ok=True)
            file_path=os.path.join(folder,"cnt_obs_buf.csv")
            df.to_csv(file_path, index=False, header=header)
            print(f"recoding data at: {folder}")
            return 
    print("exit recoding obs process")

class SkilledAmbotReal(AmbotReal):
    """ Some additional methods to help the execution of skill policy """
    def __init__(self, 
            *args, 
            **kwargs
        ):

        super().__init__(*args, **kwargs)


def load_skill_policy(args, env, config_dict):

    model = getattr(modules, config_dict["runner"]["policy_class_name"])(
        num_actions= config_dict["env"]["num_actions"],
        obs_segments= env.obs_segments,
        privileged_obs_segments= env.privileged_obs_segments,
        **config_dict["policy"]
    )

    # load the model with the latest checkpoint
    model_names = [i for i in os.listdir(args.logdir) if i.startswith("model_")]
    model_names.sort(key= lambda x: int(x.split("_")[-1].split(".")[0]))
    model_path = osp.join(args.logdir, model_names[-1])
    print("model path:", model_path)
    state_dict = torch.load(model_path, map_location= "cpu")
    model.load_state_dict(state_dict["model_state_dict"])
    model.to(args.device)

    # policy 
    if hasattr(model,"memory_a"):
        memory_module = model.memory_a
    elif hasattr(model,"encoder_a"):
        memory_module = model.encoder_a
    actor_mlp = model.actor
    #@torch.jit.script
    def policy(obs):
        if hasattr(model,"memory_a"):
            recurrent_embedding = memory_module(obs)
            actions = actor_mlp(recurrent_embedding.squeeze(0))
        if hasattr(model, "encoder_a"):
            latent_state = memory_module(obs)
            actions = actor_mlp(torch.cat([latent_state, obs],dim=-1).squeeze(0))
        else:
            actions = actor_mlp(obs)
        return actions

    return model, policy



def switch_posture(
        ros,
        target_dof_pos=None,
        angle_tolerance=0.1,
        num_actions=12,
    ):
    """
    Args:
        warmup_timesteps: the number of timesteps to linearly increase the target position
    """
    rospy.loginfo("Robot standing up, please wait ...")
    motion_mode = ros.motion_mode
    if target_dof_pos is None:
        target_dof_pos = copy.copy(ros.env.default_dof_pos)
    intermediate_target_dof_pos = torch.zeros((1, num_actions), device=ros.device, dtype= torch.float32,requires_grad=False)
    directions = num_actions * [0]
    while not rospy.is_shutdown():
        dof_pos = [ros.env.state_buffer.motorState[i].pos for i in range(num_actions)]
        diffs = [target_dof_pos[i]- dof_pos[i] for i in range(num_actions)]
        for idx, value in enumerate(diffs):
            if value > angle_tolerance:
                intermediate_target_dof_pos[0,idx] = angle_tolerance + dof_pos[idx]
            elif value < -angle_tolerance:
                intermediate_target_dof_pos[0,idx] = -angle_tolerance + dof_pos[idx]
            else:
                intermediate_target_dof_pos[0,idx] = target_dof_pos[idx]
            
        rospy.loginfo(" max joint error (rad): {:.4f}, tolerance: {:.4f}".format(max([abs(i) for i in diffs]), angle_tolerance))
        if all([abs(i) <= angle_tolerance for i in diffs]):
            break

        ros.env.get_obs()
        ros.publish_joints_cmd(intermediate_target_dof_pos)
        ros.rate.sleep()

    rospy.loginfo("Robot behavior target dof psoture! set motion mode to be 1 by the remote control/rosparam to continue ...")
    # robot stand up
    while not rospy.is_shutdown():
        if ros.motion_mode!=motion_mode: # start to standuping
            break
        ros.env.get_obs()
        ros.publish_joints_cmd(intermediate_target_dof_pos)
        ros.rate.sleep()
    rospy.loginfo("Robot standing up procedure finished!")





def main(args):

    # config files
    assert args.logdir is not None
    with open(osp.join(args.logdir, "config.json"), "r") as f:
        config_dict = json.load(f, object_pairs_hook= OrderedDict)
    
    # override come configs since in real world
    config_dict["terrain"]["measure_heights"] = False
    print("env cfg info:")

    # create env 
    env = SkilledAmbotReal(
        device= args.device,
        cfg= config_dict,
    )
    # load policy model
    model, policy = load_skill_policy(args, env,config_dict)
    model.eval()

    # create ros 
    ros = RosInterface(
            env,
            rate=args.ros_rate,
            namespace=args.namespace,
            subscribe_move_cmd=False,
            )

    # loading standuping policy
    ros.start_ros()
    ros.wait_untill_ros_working()

    with torch.no_grad():
        #2) skill and standuping modes
        start_time=time.time_ns()

        obs=env.get_obs()

        # recoding obs data
        q_obs = Queue()
        q_idx = Queue()
        recording_obs_p = Process(target=recording_obs,args=(q_obs,q_idx,))
        recording_obs_p.start()
        idx=1

        while not rospy.is_shutdown():
            if ros.motion_mode==1:# standup
                #1) performing standup procedure
                switch_posture(
                    ros,
                    num_actions=12
                )
            elif ros.motion_mode==2:# skill 
                obs = ros.env.get_obs()
                actions = policy(obs)
                # send actions
                ros.send_action(actions) # sent actions
                # logging
                rospy.loginfo_throttle(5, "skilled policy")
                rospy.loginfo_throttle(1,"commands:{:}".format(env.commands))
            elif ros.motion_mode==3:# reset model
                rospy.loginfo_throttle(1, "model reset, robot sleep")
                switch_posture(
                    ros,
                    target_dof_pos = ros.sleep_dof_pos.squeeze(0),
                    num_actions=12
                )
                model.reset()
            elif ros.motion_mode==4:# close 
                rospy.signal_shutdown("Controller send stop signal, exiting")
                exit(0)
            elif ros.motion_mode==0:
                continue
            
            # pipeline data to other process
            idx+=1
            q_obs.put(obs.cpu())
            q_idx.put(idx)

            ros.rate.sleep()

            rospy.loginfo_throttle(1,"update fre: {:}".format(1.0/(time.time_ns()-start_time)*1e9))
            start_time = time.time_ns()




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
    parser.add_argument("--mode",
        type= str,
        help= "The mode to determine which computer to run on.",
        choices= ["jetson", "upboard", "full"],                
    )
    parser.add_argument("--ros_rate",
        type= int,
        default=100,
        help= "The ros node update rate.",
    )
    parser.add_argument("--debug",
        action= "store_true",
    )
    parser.add_argument("--device",
            type=str,
            default= "cuda:0" , #torch.cuda.get_device_name(0),
    )

    args, unknown = parser.parse_known_args()
    print("args:", args)
    main(args)
