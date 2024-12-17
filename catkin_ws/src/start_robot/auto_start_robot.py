#! /usr/bin/python3

import os
import sys
import time
from multiprocessing import Process


def open_bt(bluetooth):
    '''
    open bluetooth 
    '''
    not_find_bluetooth=True
    while not_find_bluetooth:
        if os.path.exists(bluetooth):
            not_find_bluetooth=False
        else:
            time.sleep(2)
            print("Did not find joystick, please connect joystick")

def launch_ros(bluetooth):
    '''
    launch ros node by joy stick
    '''
    print(os.environ['AMBOT'])
    startpath = os.environ['AMBOT'] + "/projects/catkin_ws/src/start_robot/start_robot"
    data=[]
    if os.path.exists(bluetooth):
        with open(bluetooth, 'rb') as f:
            while True:
                try:
                    a=f.readline(1)
                except:
                    break
                data.append(ord(a))
                if len(data)==16:
                    if (data[-2] == 2) and (data[-1]==0) : # start robot
                        print("Attention, ambot will move")
                        time.sleep(3)
                        os.system(startpath)
                        # os.system("roslaunch ambot_controller ambot_controller.launch")
                        # print(data)
                        break
                    if((data[-3]==0) and (data[-2]==1) and (data[-1]==8)): # turn off computer
                        print("shutdown down the robot computer!")
                        # os.system("sudo shutdown now")
                        break
                    if((data[-3]==0) and (data[-2]==1) and (data[-1]==9)): # kill ambot
                        print("kill ambot!")
                        os.system("rosnode kill /ambot_controller")
                        break
                    data.clear()

if __name__=="__main__":
    bluetooth="/dev/input/js0"
    # os.system("source /home/cc/.zshrc")
    # open_bt(bluetooth)
    # time.sleep(4)
    while True:
        open_bt(bluetooth)
        time.sleep(3)
        p=Process(target=launch_ros,args=(bluetooth,))
        # print("please UP or Down  joystick")
        print("please push right button to start!!")
        p.start()
        p.join()

