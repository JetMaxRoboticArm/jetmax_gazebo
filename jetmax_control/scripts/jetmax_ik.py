#!/usr/bin/env python3
import sys
import math
import rospy
from rospy.topics import Publisher
from std_msgs.msg import Float64
import jetmax_kinematics
from jetmax_control.srv import IK, IKRequest, IKResponse

NUM_OF_JOINTS = 9
joints_publishers = []
ik_service = None


# This callback function executes whenever a inverse_kinematics service is requested
def ik_callback(req):
    global joints_publishers
    position = req.x, req.y, req.z
    ik_result= jetmax_kinematics.inverse_kinematics(position)
    if ik_result:
        fk_result = jetmax_kinematics.forward_kinematics(ik_result)
        if fk_result:
            joint_angles = [deg * math.pi / 180 for deg in fk_result]
            # Set the arm joint angles
            for i in range(NUM_OF_JOINTS):
                joints_publishers[i].publish(Float64(joint_angles[i]))
            return [True, ]
    return [False, ]

def main():
    global joints_publishers, fk_service
    rospy.init_node('ik_jetmax', log_level=rospy.DEBUG)
    for i in range(NUM_OF_JOINTS):
        pub = rospy.Publisher("/jetmax/joint{}_position_controller/command".format(i + 1),Float64, queue_size=10)
        joints_publishers.append(pub)

    fk_service = rospy.Service("/jetmax_control/inverse_kinematics", IK, ik_callback)
    rospy.loginfo("Ready to send joint angles")
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(e)
        sys.exit(-1)
    
if __name__ == "__main__":
    main()