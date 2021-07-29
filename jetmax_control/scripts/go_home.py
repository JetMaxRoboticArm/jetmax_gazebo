#!/usr/bin/env python3
import rospy
from jetmax_control.srv import FK, FKRequest, FKResponse

if __name__ == "__main__":
    rospy.init_node("go_home")
    go_home = rospy.ServiceProxy("/jetmax_control/forward_kinematics", FK)
    ret = go_home(FKRequest(90, 90, 0))
    rospy.logdebug(ret)