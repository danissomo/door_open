#!/usr/bin/python3

import rospy
from husky import Robot
import numpy as np
import math
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
from param_provider import ParamProvider
from doorHandle import DoorHandle
from enum import Enum, auto

from geometry_msgs.msg import PoseArray
class DoorOpen:
    
    class RsultEnum(Enum):
        SUCCESS = auto()
        NO_OBJECTS_FOUND = auto()
        UNREACHABLE = auto()
        SINGULARITY = auto()
        GRIPER_DETACHES = auto()
        GRIPPER_STUCK = auto()

    def __init__(self) -> None:
        rospy.init_node("~")
        self.force_limits = [0.5, 0.5, 0.5, math.pi/2, math.pi/2, math.pi/2]
        self.deltaUnblokDRfromSteady = np.array([0, 0, 0.03])
        self.robot = Robot(ParamProvider.ur_ip)
        self.pub_start_move = rospy.Publisher(ParamProvider.base_controller_topic, Bool, queue_size=10)
        self.door_handle_handler =  DoorHandle(ParamProvider.yolo_topic, "ur_arm_base")
        
        self.robot.ForceModeStop()
        self.robot.ActivateTeachMode()
        self.robot.OpenGripper()

        self.sub = rospy.Subscriber(ParamProvider.door_hinge_topic, PoseArray, callback=self.YoloIntegration, queue_size=1)

    def YoloIntegration(self, doorHinge : PoseArray):
        self.robot.ForceModeStop()
        self.robot.ActivateTeachMode()
        self.robot.OpenGripper()
        self.robot.ManualPose()
        self.robot.AlignXZtoXYPlane()
        rospy.sleep(2)

        self.door_handle_handler.StopUpdateHandle()
        handle_basis = self.door_handle_handler.GetActualCoordSystem()
        point_for_grip = self.door_handle_handler.GetActualMiddlePoint()
        self.door_handle.StartUpdateHandle()
        if (handle_basis is not None) and (point_for_grip is not None):
            self.robot.OpenGripper()

            # grip handle
            self.robot.MoveL_point_rot(point_for_grip, self.robot.RotvecFromBasis(handle_basis), 0.5, 0.1)
            self.robot._rtde_c.forceMode(self.robot.GetActualTCPPose(), [0, 0, 0, 1, 0, 1], [
                            0, 0, 0, 0, 0, 0], 2, self.force_limits)
            self.robot.CloseGripper()

            # rotate handle
            gripped_frame = self.robot.GetActualTCPPose()
            self.robot.PushUntilForce([1, 0, 0, 0, 0, 1],
                                [-150, 0, 0, 0, 0, -50], self.force_limits)

            # try open
            start_opening_frame = np.matmul(
                Rotation.from_rotvec(self.robot.GetActualTCPPose()[3:]).as_matrix(),
                self.robot.GetActualTCPPose()[0:3]
            )
            actual_opening_frame = np.matmul(
                Rotation.from_rotvec(self.robot.GetActualTCPPose()[3:]).as_matrix(),
                self.robot.GetActualTCPPose()[0:3]
            )

            while (not rospy.is_shutdown()) and (not all(np.fabs(actual_opening_frame - start_opening_frame) >= self.deltaUnblokDRfromSteady)):
                self.robot.ForceMode(self.robot.GetActualTCPPose(), [
                                1, 0, 1, 0, 0, 1], [-150, 0, -150, 0, 0, -50], 2, self.force_limits)
                actual_opening_frame = np.matmul(
                    Rotation.from_rotvec(
                        self.robot.GetActualTCPPose()[3:]).as_matrix(),
                    self.robot.GetActualTCPPose()[0:3]
                )

            self.robot._rtde_c.forceModeStop()

            # align with xy plane
            while (not rospy.is_shutdown()) and not self.robot.IfAlignedYZtoXYPlane(0.15):
                self.robot.ForceMode(self.robot.GetActualTCPPose(), [1, 0, 1, 0, 0, 1], [
                                50, 0, 0, 0, 0, 30], 2, self.force_limits)
            self.robot._rtde_c.forceModeStop()

            #open in place
            while (not rospy.is_shutdown()) and np.linalg.norm(np.array(self.robot.GetActualTCPPose()[:2])) > 0.5:
                rot = Rotation.from_rotvec(
                    self.robot.GetActualTCPPose()[3:]).as_matrix()
                f_result = np.matmul(rot, np.array(
                    self.robot.GetActualTCPForce()[:3])) - np.array([0, 0, 100])
                rf = np.matmul(rot, np.array(self.robot.GetActualTCPForce()[3:]))
                self.robot._rtde_c.forceMode(self.robot.GetActualTCPPose(), [0, 1, 1, 1, 0, 0], [
                                        0, -f_result[1], -150, -8*self.robot.GetActualTCPForce()[3], 0, 0], 2, self.force_limits)
            self.robot._rtde_c.forceModeStop()
            self.robot.ActivateTeachMode()
            self.pub_start_move.publish(Bool(True))      
    
    def Init():
        pass

    def FindDoorHandle():
        pass


    def GripHandle():
        pass


    def RotateHandle():
        pass


    def OpenDoor():
        pass


    def Cancel():
        pass

    def DetachFromDoor():
        pass
    
def main():
    d = DoorOpen()
    rospy.spin()


if __name__ == "__main__":
    main()
