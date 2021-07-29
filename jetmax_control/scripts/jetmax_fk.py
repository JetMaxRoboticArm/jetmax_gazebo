#!/usr/bin/env python3
import sys
import math
import rospy
from rospy.topics import Publisher
from std_msgs.msg import Float64
import jetmax_kinematics
from jetmax_control.srv import FK, FKRequest, FKResponse

NUM_OF_JOINTS = 9
joints_publishers = []
fk_service = None


def fk_callback(req):
    """
    This callback function executes whenever a forward_kinematics service is requested
    @param req: FKRequest
    @return: FKResponse
    """
    global joints_publishers
    angles = [req.angle_rotate, req.angle_left, req.angle_right]
    fk_result = jetmax_kinematics.forward_kinematics(angles)
    if fk_result:
        joint_angles = [deg * math.pi / 180 for deg in fk_result]
        for i in range(NUM_OF_JOINTS):
            joints_publishers[i].publish(Float64(joint_angles[i]))
        return [True, ]
    return [False, ]


if __name__ == "__main__":
    rospy.init_node('fk_jetmax', log_level=rospy.DEBUG)
    for i in range(NUM_OF_JOINTS):
        pub = rospy.Publisher(
            "/jetmax/joint{}_position_controller/command".format(i + 1), Float64, queue_size=10)
        joints_publishers.append(pub)

    fk_service = rospy.Service(
        "/jetmax_control/forward_kinematics", FK, fk_callback)
    rospy.loginfo("Ready to send joint angles")
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(e)
        sys.exit(-1)
