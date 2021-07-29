import rospy
import math

AngleRotateRange = 0, 240
AngleLeftRange = 0, 180
AngleRightRange = -20, 160
L0 = 84.4
L1 = 8.14
L2 = 128.41
L3 = 138.0


def forward_kinematics(angle):
    """
    Jetmax forward kinematics
    @param angle: active angles [rotate, angle left, angle right]
    @return: joint angles list
    """
    alpha1, alpha2, alpha3 = angle
    if (AngleRotateRange[0] <= alpha1 <= AngleRotateRange[1] and
        AngleLeftRange[0] <= alpha2 <= AngleLeftRange[1] and
            AngleRightRange[0] <= alpha2 <= AngleRightRange[1]):

        alpha3 = -alpha3
        joint_angle = [0] * 9

        # 3 active joints
        joint_angle[0] = alpha1 - 90  # joint1
        joint_angle[1] = 90 - alpha2  # joint2
        joint_angle[5] = alpha3  # joint6

        # 6 passive joints for display
        joint_angle[2] = 90 - (alpha2 + alpha3)
        joint_angle[3] = 90 - alpha2
        joint_angle[4] = joint_angle[1]
        joint_angle[6] = joint_angle[2]
        joint_angle[7] = alpha3
        joint_angle[8] = alpha3
        print(joint_angle)
        return joint_angle
    else:
        rospy.logerr("Infeasible angle values!, feasible range is AngleRotate: ({}), AngleLeft: ({}), AngleRight: ({})".format(
            AngleRotateRange, AngleLeftRange, AngleRightRange))
        rospy.logwarn("Requested angles are; angleRotate: {:.2f}, angleLeft: {:.2f}, angleRight: {:.2f}".format(
            alpha1, alpha2, alpha3))
        return None


def inverse_kinematics(position):
    """
    Jetmax inverse kinematics
    @param position: position [x, y , z]
    @return: active joint angles
    """
    x, y, z = position
    y = -y
    if y == 0:
        if x < 0:
            theta1 = -90
        elif x > 0:
            theta1 = 90
        else:
            rospy.logerr('Invalid coordinate x:{} y:{} z:{}'.format(x, y, z))
            return None
    else:
        theta1 = math.atan(x / y)
        theta1 = 0.0 if x == 0 else theta1 * 180.0 / math.pi
        if y < 0:
            if theta1 < 0 < x:
                theta1 = 90 + theta1
                print("A", theta1)
            elif theta1 > 0 > x:
                theta1 = 90 + (90 - (90 - theta1))
                print("B", theta1)
            else:
                pass

        x = math.sqrt(x * x + y * y) - L1
        z = z - L0

        if math.sqrt(x * x + z * z) > L2 + L3:
            return None

        alpha = math.atan(z / x) * 180.0 / math.pi
        beta = math.acos((L2 * L2 + L3 * L3 - (x * x + z * z)
                          ) / (2 * L2 * L3)) * 180.0 / math.pi
        gama = math.acos((L2 * L2 - L3 * L3 + (x * x + z * z)) /
                         (2 * L2 * math.sqrt(x * x + z * z))) * 180.0 / math.pi

        pos1 = theta1 + 90
        theta2 = alpha + gama
        pos2 = 180 - theta2
        theta3 = beta + theta2
        pos3 = 180 - theta3
        return pos1, pos2, pos3
