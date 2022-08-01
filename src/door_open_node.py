#!/usr/bin/python3

import rospy
from husky import Robot
import numpy as np
import math
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
from param_provider import ParamProvider

def ManualPose(robot):
    robot.ActivateTeachMode()
    print(robot.GetActualTCPPose())
    input("set init pose and press enter")
    robot.DeactivateTeachMode()


def YoloIntegration():
    from doorHandle import DoorHandle
    force_limits = [0.5, 0.5, 0.5, math.pi/2, math.pi/2, math.pi/2]
    deltaUnblokDRfromSteady = np.array([0, 0, 0.03])

    rospy.init_node("tests")
    pub_start_move = rospy.Publisher(ParamProvider.base_controller_topic, Bool, queue_size=10)
    robot = Robot(ParamProvider.ur_ip)
    robot._rtde_c.forceModeStop()
    door_handle = DoorHandle(ParamProvider.yolo_topic, "ur_arm_base")

    while not rospy.is_shutdown():

        robot.ActivateTeachMode()
        robot.OpenGripper()
        ManualPose(robot)
        robot.AlignXZtoXYPlane()
        rospy.sleep(2)

        door_handle.StopUpdateHandle()
        handle_basis = door_handle.GetActualCoordSystem()
        point_for_grip = door_handle.GetActualMiddlePoint()
        if (handle_basis is not None) and (point_for_grip is not None):
            robot.OpenGripper()

            # grip handle
            commandForUR = list(point_for_grip) + list(robot.RotvecFromBasis(handle_basis))
            robot.MoveL(commandForUR, 0.5, 0.1)
            robot._rtde_c.forceMode(robot.GetActualTCPPose(), [0, 0, 0, 1, 0, 1], [
                               0, 0, 0, 0, 0, 0], 2, force_limits)
            robot.CloseGripper()

            # rotate handle
            gripped_frame = robot.GetActualTCPPose()
            robot.PushUntilForce([1, 0, 0, 0, 0, 1],
                                 [-150, 0, 0, 0, 0, -50], force_limits)

            # try open
            start_opening_frame = np.matmul(
                Rotation.from_rotvec(robot.GetActualTCPPose()[3:]).as_matrix(),
                robot.GetActualTCPPose()[0:3]
            )
            actual_opening_frame = np.matmul(
                Rotation.from_rotvec(robot.GetActualTCPPose()[3:]).as_matrix(),
                robot.GetActualTCPPose()[0:3]
            )

            while (not rospy.is_shutdown()) and (not all(np.fabs(actual_opening_frame - start_opening_frame) >= deltaUnblokDRfromSteady)):
                robot.ForceMode(robot.GetActualTCPPose(), [
                                1, 0, 1, 0, 0, 1], [-150, 0, -150, 0, 0, -50], 2, force_limits)
                actual_opening_frame = np.matmul(
                    Rotation.from_rotvec(
                        robot.GetActualTCPPose()[3:]).as_matrix(),
                    robot.GetActualTCPPose()[0:3]
                )

            robot._rtde_c.forceModeStop()

            # align with xy plane
            while (not rospy.is_shutdown()) and not robot.IfAlignedYZtoXYPlane(0.15):
                robot.ForceMode(robot.GetActualTCPPose(), [1, 0, 1, 0, 0, 1], [
                                50, 0, 0, 0, 0, 30], 2, force_limits)
            robot._rtde_c.forceModeStop()

            #open in place
            while (not rospy.is_shutdown()) and np.linalg.norm(np.array(robot.GetActualTCPPose()[:2])) > 0.5:
                rot = Rotation.from_rotvec(
                    robot.GetActualTCPPose()[3:]).as_matrix()
                f_result = np.matmul(rot, np.array(
                    robot.GetActualTCPForce()[:3])) - np.array([0, 0, 100])
                rf = np.matmul(rot, np.array(robot.GetActualTCPForce()[3:]))
                robot._rtde_c.forceMode(robot.GetActualTCPPose(), [0, 1, 1, 1, 0, 0], [
                                        0, -f_result[1], -150, -8*robot.GetActualTCPForce()[3], 0, 0], 2, force_limits)
            robot._rtde_c.forceModeStop()
            robot.ActivateTeachMode()
            pub_start_move.publish(Bool(True))
            robot.MoveBaseX(1.0, -0.05)
            input()
        
        door_handle.StartUpdateHandle()

def main():
    YoloIntegration()


if __name__ == "__main__":
    main()
