#!/usr/bin/env python3
import rospy
from jetmax_control.srv import IK, IKRequest


if __name__ == "__main__":
    rospy.init_node("go_home")
    moving = rospy.ServiceProxy("/jetmax_control/inverse_kinematics", IK)
    x, y, z = [0, 150, 50]
    while z < 200:
        z += 1
        moving(IKRequest(x, y, z))
        rospy.sleep(0.05)
