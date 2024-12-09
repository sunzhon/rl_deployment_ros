#!/usr/bin/env python
# Software License Agreement (BSD License)
import sys
import time
from std_msgs.msg import Bool
#sys.path.append("/home/suntao/PycharmProjects/Reflex")
import attitudeReflex
import simRos
import rospy


if __name__ == '__main__':
    time.sleep(3) 
    arg = sys.argv[1:len(sys.argv)]
    print(arg)
    try:
        rosNode = simRos.simRosClass(arg)
        attitude = attitudeReflex.attitudeReflexClass()
        while not rospy.is_shutdown():
            termiState = rosNode.paraDir[rosNode.terminateNodeTopic]==Bool(True)
            if termiState:
                break
            attitude.setInput(rosNode.paraDir[rosNode.simTimeTopic], rosNode.paraDir[rosNode.sensorValueTopic])
            attitude.step()
            data = attitude.getOutput()
            rosNode.setMotorPosition(data)
            rosNode.rosSpinOnce()

    except rospy.ROSInterruptException:
        pass
