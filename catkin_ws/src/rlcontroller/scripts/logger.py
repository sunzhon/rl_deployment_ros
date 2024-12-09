# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
import collections
from multiprocessing import Process, Value
import pandas as pd
import os

class Logger:
    def __init__(self, dt):
        self.state_log = defaultdict(list)
        self.rew_log = defaultdict(list)
        self.dt = dt
        self.num_episodes = 0
        self.plot_process = None

    def log_state(self, key, value):
        self.state_log[key].append(value)

    def log_states(self, dict):
        for key, value in dict.items():
            self.log_state(key, value)

    def log_rewards(self, dict, num_episodes):
        for key, value in dict.items():
            if 'rew' in key:
                self.rew_log[key].append(value.item() * num_episodes)
        self.num_episodes += num_episodes

    def reset(self):
        self.state_log.clear()
        self.rew_log.clear()

    def store_states(self, file_path=os.path.split(os.path.realpath(__file__))[0]):
        print("save data")
        self.store_process = Process(target=self._store, args=(file_path,))
        self.store_process.start()


    def _store(self, file_path):
        pd_data = pd.DataFrame(self.state_log)
        file_path = os.path.join(file_path,"states.csv")
        print("file path", file_path)
        pd_data.to_csv(file_path)
        print("save data succssfully!")

    def plot_states(self):
        self.plot_process = Process(target=self._plot)
        self.plot_process.start()

    def _plot(self):
        nb_rows = 4
        nb_cols = 4
        fig, axs = plt.subplots(nb_rows, nb_cols)
        plt.subplots_adjust(top=0.94, bottom=0.05, left=0.06, right=0.95, hspace=0.38,
                    wspace=0.27)
        for key, value in self.state_log.items():
            time = np.linspace(0, len(value)*self.dt, len(value))
            break
        log= self.state_log
        colors_map=collections.OrderedDict({
                "measured_x": "r-", "command_x": "r-.",
                "measured_y": "g-", "command_y": "g-.",
                "measured_z": "b-", "command_z": "b-.",
                })
        # plot base vel x
        a = axs[0, 0]
        if log["base_lin_vel_x"]: a.plot(time, log["base_lin_vel_x"], colors_map["measured_x"], label='measured_x')
        if log["command_x"]: a.plot(time, log["command_x"], colors_map["command_x"], label='command_x')
        if log["base_lin_vel_y"]: a.plot(time, log["base_lin_vel_y"], colors_map["measured_y"], label='measured_y')
        if log["command_y"]: a.plot(time, log["command_y"], colors_map["command_y"],label='command_y')
        if log["base_lin_vel_z"]: a.plot(time, log["base_lin_vel_z"], colors_map["measured_z"], label='measured_z')
        if log["command_z"]: a.plot(time, log["command_z"], colors_map["command_z"], label='command_z')
        a.set(xlabel='time [s]', ylabel='base lin vel [m/s]', title='Base velocity')
        a.legend()


        # plot base orientation
        a = axs[0, 1]
        if log["base_pitch"]:
            a.plot(time, log["base_pitch"], colors_map["measured_x"],label='measured_picth')
            a.plot(time, [-0.75] * len(time), label= 'thresh')
        if log["base_roll"]: a.plot(time, log["base_roll"], colors_map["measured_y"], label='measured_roll')
        if log["base_yaw"]:  a.plot(time, log["base_yaw"], colors_map["measured_z"], label='measured_yaw')
        # if log["command_yaw"]: a.plot(time, log["command_yaw"], label='commanded')
        a.set(xlabel='time [s]', ylabel='base ang [rad]', title='Base orientation')
        a.legend()


        # plot base ang vel x, y, and z
        a = axs[0, 2]
        if log["base_ang_vel_x"]: a.plot(time, log["base_ang_vel_x"], colors_map["measured_x"], label='ang_vel_x')
        if log["base_ang_vel_y"]: a.plot(time, log["base_ang_vel_y"], colors_map["measured_y"], label='ang_vel_y')
        if log["base_ang_vel_z"]: a.plot(time, log["base_ang_vel_z"], colors_map["measured_z"], label='ang_vel_z')
        a.set(xlabel='time [s]', ylabel='base ang lin vel [rad/s]', title='Base ang velocity')
        a.legend()


        # plot base projected gravity x, y, and z
        a = axs[0, 3]
        if log["base_pro_gravity_x"]: a.plot(time, log["base_pro_gravity_x"], colors_map["measured_x"], label='gravity_x')
        if log["base_pro_gravity_y"]: a.plot(time, log["base_pro_gravity_y"], colors_map["measured_y"], label='gravity_y')
        if log["base_pro_gravity_z"]: a.plot(time, log["base_pro_gravity_z"], colors_map["measured_z"], label='gravity_z')
        a.set(xlabel='time [s]', ylabel='base projected gravity [m/s2]', title='Base pro gravity ')
        a.legend()


        # plot joint targets and measured positions
        a = axs[1, 0]
        for idx in range(3):
            if log["dof_pos_"+str(idx)]: a.plot(time, 
                    log["dof_pos_"+str(idx)], 
                    colors_map[list(colors_map.keys())[2*idx]], 
                    label='measured_'+str(idx))
            if log["dof_pos_target_"+str(idx)]: a.plot(time, 
                    log["dof_pos_target_"+str(idx)], 
                    colors_map[list(colors_map.keys())[2*idx+1]], 
                    label='target_'+str(idx))
        a.set(xlabel='time [s]', ylabel='Position [rad]', title='DOF Position')
        a.legend()

        a = axs[1, 1]
        for idx in range(3,6):
            if log["dof_pos_"+str(idx)]: a.plot(time, log["dof_pos_"+str(idx)], 
                    colors_map[list(colors_map.keys())[2*(idx-3)]], 
                    label='measured_'+str(idx))
            if log["dof_pos_target_"+str(idx)]: a.plot(time, log["dof_pos_target_"+str(idx)], 
                    colors_map[list(colors_map.keys())[2*(idx-3)+1]], 
                    label='target_'+str(idx))
        a.set(xlabel='time [s]', ylabel='Position [rad]', title='DOF Position')
        a.legend()

        a = axs[1, 2]
        for idx in range(6,9):
            if log["dof_pos_"+str(idx)]: a.plot(time, 
                    log["dof_pos_"+str(idx)], 
                    colors_map[list(colors_map.keys())[2*(idx-6)]], 
                    label='measured_'+str(idx))
            if log["dof_pos_target_"+str(idx)]: a.plot(time, 
                    log["dof_pos_target_"+str(idx)], 
                    colors_map[list(colors_map.keys())[2*(idx-6)+1]], 
                    label='target_'+str(idx))
        a.set(xlabel='time [s]', ylabel='Position [rad]', title='DOF Position')
        a.legend()

        a = axs[1, 3]
        for idx in range(9,12):
            if log["dof_pos_"+str(idx)]: a.plot(time, log["dof_pos_"+str(idx)], 
                    colors_map[list(colors_map.keys())[2*(idx-9)]], 
                    label='measured_'+str(idx))
            if log["dof_pos_target_"+str(idx)]: a.plot(time, log["dof_pos_target_"+str(idx)], 
                    colors_map[list(colors_map.keys())[2*(idx-9)+1]], 
                    label='target_'+str(idx))
        a.set(xlabel='time [s]', ylabel='Position [rad]', title='DOF Position')
        a.legend()

        # plot joint velocity
        a = axs[2, 0]
        for idx in range(3):
            if log["dof_vel_"+str(idx)]: a.plot(time, log["dof_vel_"+str(idx)], 
                    label='measured_'+str(idx))
        a.set(xlabel='time [s]', ylabel='Velocity [rad/s]', title='Joint Velocity')
        a.legend()

        a = axs[2, 1]
        for idx in range(3,6):
            if log["dof_vel_"+str(idx)]: a.plot(time, log["dof_vel_"+str(idx)], 
                    label='measured_'+str(idx))
        a.set(xlabel='time [s]', ylabel='Velocity [rad/s]', title='Joint Velocity')
        a.legend()

        a = axs[2, 2]
        for idx in range(6,9):
            if log["dof_vel_"+str(idx)]: a.plot(time, log["dof_vel_"+str(idx)], 
                    label='measured_'+str(idx))
        a.set(xlabel='time [s]', ylabel='Velocity [rad/s]', title='Joint Velocity')
        a.legend()

        a = axs[2, 3]
        for idx in range(9,12):
            if log["dof_vel_"+str(idx)]: a.plot(time, log["dof_vel_"+str(idx)], 
                    label='measured_'+str(idx))
        a.set(xlabel='time [s]', ylabel='Velocity [rad/s]', title='Joint Velocity')
        a.legend()

        # plot dof torques
        a = axs[3, 0]
        for idx in range(3):
            if log["torques_"+str(idx)]: a.plot(time, log["torques_"+str(idx)], 
                    label='measured_'+str(idx))
        a.set(xlabel='time [s]', ylabel='Joint Torque [Nm]', title='Torque')
        a.legend()

        a = axs[3, 1]
        for idx in range(3,6):
            if log["torques_"+str(idx)]: a.plot(time, log["torques_"+str(idx)], 
                    label='measured_'+str(idx))
        a.set(xlabel='time [s]', ylabel='Joint Torque [Nm]', title='Torque')
        a.legend()

        # plot contact forces
        a = axs[3, 3]
        if log["contact_forces_z"]:
            forces = np.array(log["contact_forces_z"])
            for i in range(forces.shape[1]):
                a.plot(time, forces[:, i], label=f'force {i}')
        a.set(xlabel='time [s]', ylabel='Forces z [N]', title='Vertical Contact forces')
        a.legend()
        # # plot torque/vel curves
        # a = axs[2, 1]
        # if log["dof_vel"]!=[] and log["dof_torque"]!=[]: a.plot(log["dof_vel"], log["dof_torque"], 'x', label='measured')
        # a.set(xlabel='Joint vel [rad/s]', ylabel='Joint Torque [Nm]', title='Torque/velocity curves')
        # a.legend()
        # plot power curves
        """
        a = axs[3, 3]
        if log["power"]!=[]: a.plot(time, log["power"], label='power [W]')
        a.set(xlabel='time [s]', ylabel='Power [W]', title='Power')
        
        # plot rewards
        a = axs[3, 0]
        if log["max_torques"]: a.plot(time, log["max_torques"], label='max_torques')
        if log["max_torque_motor"]: a.plot(time, log["max_torque_motor"], label='max_torque_motor')
        if log["max_torque_leg"]: a.plot(time, log["max_torque_leg"], label='max_torque_leg')
        a.set(xlabel='time [s]', ylabel='max_torques [Nm]', title='max_torques')
        a.legend(fontsize= 5)
        # plot customed data
        a = axs[3, 1]
        if log["student_action"]:
            a.plot(time, log["student_action"], label='s')
            a.plot(time, log["teacher_action"], label='t')
        a.legend()
        a.set(xlabel='time [s]', ylabel='value before step()', title='student/teacher action')
        a = axs[3, 2]
        a.plot(time, log["reward"], label='rewards')
        for i in log["mark"]:
            if i > 0:
                a.plot(time, log["mark"], label='user mark')
                break
        for key in log.keys():
            if "reward_removed_" in key:
                a.plot(time, log[key], label= key)
        a.set(xlabel='time [s]', ylabel='', title='rewards')
        # a.set_ylim([-0.12, 0.1])
        """
        a.legend(fontsize = 3)
        plt.show()

    def print_rewards(self):
        print("Average rewards per second:")
        for key, values in self.rew_log.items():
            mean = np.sum(np.array(values)) / self.num_episodes
            print(f" - {key}: {mean}")
        print(f"Total number of episodes: {self.num_episodes}")
    
    def __del__(self):
        if self.plot_process is not None:
            self.plot_process.kill()
