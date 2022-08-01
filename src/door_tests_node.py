#!/usr/bin/python3

import rospy
from aruco import Aruco, ArucoTimeStampedContainer
from husky import PositionHystrory, Robot
from ur_ar_definer import ArucoRelManipulatorNode, StampedArucoRelManipulator
import numpy as np
import math
from scipy.spatial.transform import Rotation
import copy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

import tf

class DoorPositionPublisher:
    def __init__(self, poseHyst: PositionHystrory, arContainer: ArucoTimeStampedContainer) -> None:
        self.poseHystory = poseHyst
        self.arucoContainer = arContainer
        self.tfListen = tf.TransformListener()
        self.timer = rospy.Timer(rospy.Duration(nsecs=100), self.Callback)
        self.publisher = rospy.Publisher(
            "door_handle", PointStamped, queue_size=10)

    def Callback(self, args):
        if self.arucoContainer.Count() > 0:
            arRelCam , t = self.arucoContainer.GetArucoByIndex(-1)
            ar: ArucoRelManipulatorNode = StampedArucoRelManipulator.TransformAndGetByIndex(
                self.arucoContainer, self.poseHystory, -1)
            if ar is None:
                return
            handle_pos = ar.aruco.GetHandlePose()
            ret = PointStamped()
            ret.header.frame_id = "ur_arm_base"
            ret.header.stamp = ar.time.data
            ret.point.x = handle_pos[0]
            ret.point.y = handle_pos[1]
            ret.point.z = handle_pos[2]

            
            
            if self.tfListen.frameExists("ur_arm_base") and self.tfListen.frameExists("rs_camera"):
                
                arRelCam : Aruco =arRelCam
                p = PointStamped()
                p.point.x = arRelCam.center[0]
                p.point.y = arRelCam.center[1]
                p.point.z = arRelCam.center[2]
                p.header.frame_id = "rs_camera"
                p.header.stamp = t.data
                pointRelBase = self.tfListen.transformPoint("ur_arm_base", p)
                print("my", ar.aruco.center)
                print("tf", pointRelBase.point)

            self.publisher.publish(ret)


def TestPointPublisher():
    rospy.init_node("tests")
    robot = Robot("192.168.131.40")
    rospy.sleep(1)
    arcontainer = ArucoTimeStampedContainer("/aruco_localizator/objects")
    doorPublisher = DoorPositionPublisher(
        robot.GetManipulatorHystory(), arcontainer)
    robot.ActivateTeachMode()
    rospy.spin()


def ManualPose(robot):
    robot.ActivateTeachMode()
    print(robot.GetActualTCPPose())
    input("set init pose and press enter")
    robot.DeactivateTeachMode()


def OffsetGet():
    try:
        fileOffsets = open("offsets.txt", "a")
    except:
        fileOffsets = open("offsets.txt", "w")
    rospy.init_node("tests")
    arcontainer = ArucoTimeStampedContainer("/aruco_localizator/objects")
    robot = Robot("192.168.131.40")
    print(robot.IfAlignedYZtoXYPlane())
    while not rospy.is_shutdown():
        rospy.sleep(1)
        ManualPose(robot)

        if arcontainer.Count() > 0:
            arContainerRelBase = StampedArucoRelManipulator.Transform(
                arcontainer, robot._eefHystory)
            if(len(arContainerRelBase) == 0):
                continue
            ar: ArucoRelManipulatorNode = arContainerRelBase[-1]

            print("move robot to handle")
            ManualPose(robot)

            actualTCP = robot.GetActualTCPPose()
            cmd = [actualTCP[0], actualTCP[1], actualTCP[2], ar.rotvecAlignedNormal[0],
                   ar.rotvecAlignedNormal[1], ar.rotvecAlignedNormal[2]]
            robot.MoveL([actualTCP[0], actualTCP[1], actualTCP[2], ar.rotvecAlignedNormal[0],
                        ar.rotvecAlignedNormal[1], ar.rotvecAlignedNormal[2]], 0.2, 0.2)

            actualTCP = robot.GetActualTCPPose()
            actual = np.array(actualTCP[0:3] + [1])
            offset = np.matmul(
                ar.aruco.toArSpaceR_Matrix,
                np.matmul(ar.aruco.toArSpaceT_Matrix, actual)
            )
            fileOffsets.write(np.array2string(offset) + "\n")

            arcontainer._buffer.clear()


def TestLocalization():
    rospy.init_node("tests")
    arcontainer = ArucoTimeStampedContainer("/aruco_localizator/objects")
    robot = Robot("192.168.131.40")
    while not rospy.is_shutdown():

        # robot.OpenGripper()
        robot.ActivateTeachMode()
        rospy.sleep(2)
        if arcontainer.Count() > 0:
            # robot.OpenGripper()
            arContainerRelBase = StampedArucoRelManipulator.Transform(
                arcontainer, robot._eefHystory)
            if(len(arContainerRelBase) == 0):
                continue
            ar: ArucoRelManipulatorNode = arContainerRelBase[-1]

            actualTCP = robot.GetActualTCPPose()
            cmd = [actualTCP[0], actualTCP[1], actualTCP[2], ar.rotvecAlignedNormal[0],
                   ar.rotvecAlignedNormal[1], ar.rotvecAlignedNormal[2]]
            input("aruco found")
            print("diff ", np.array(
                robot.GetActualTCPPose()[:3]) - ar.aruco.center)
            print("actual ", robot.GetActualTCPPose())
            arcontainer._buffer.clear()


def ToTestRandomShit():
    rospy.init_node("tests")
    robot = Robot("192.168.131.40")
    ManualPose(robot)
    rospy.sleep(1)
    arcontainer = ArucoTimeStampedContainer("/aruco_localizator/objects")
    ManualPose(robot)

    while rospy.is_shutdown():
        print(1)
        if arcontainer.Count() > 0:
            print(1)
            ar: ArucoRelManipulatorNode = StampedArucoRelManipulator.TransformAndGetByIndex(
                arcontainer, robot.GetManipulatorHystory(), -1)
            rotvec = robot.RotvecFromBasis(ar.aruco.Yaxis, ar.aruco.Xaxis, -ar.aruco.Zaxis)
            robot.ChangeOrientation(rotvec, 0.1)

    rospy.spin()


def GripHandle2():
    safe_distFdoor = 1.4
    force_limits = [0.5, 0.5, 0.5, math.pi/2, math.pi/2, math.pi/2]
    offsetFOpenedDoor = 0.15
    deltaUnblokDRfromSteady = np.array([0, 0, 0.03])

    rospy.init_node("tests")
    pub_start_move = rospy.Publisher("some_topic", Bool, queue_size=10)
    arcontainer = ArucoTimeStampedContainer("/aruco_localizator/objects")
    robot = Robot("192.168.131.40")
    robot._rtde_c.forceModeStop()
    while not rospy.is_shutdown():

        robot.ActivateTeachMode()
        robot.OpenGripper()
        ManualPose(robot)
        robot.AlignXZtoXYPlane()
        rospy.sleep(2)

        if arcontainer.Count() > 0:
            initPose = robot.GetActualQ()
            robot.OpenGripper()
            # get Actual aruco
            ar: ArucoRelManipulatorNode = StampedArucoRelManipulator.TransformAndGetByIndex(
                arcontainer, robot.GetManipulatorHystory(), -1)
            if ar is None:
                continue
            doorPositionRelBase = copy.copy(ar.aruco.center)

            # grip handle
            robot.GripHandle(ar, force_limits)

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
            print(np.fabs(actual_opening_frame - start_opening_frame)
                  >= deltaUnblokDRfromSteady)

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

            robot.ActivateTeachMode()
            pub_start_move.publish(Bool(True))
            input()
            # #variant with avoid singularity
            # while not rospy.is_shutdown():
            #     robot.ForceModeWithSingularityAwoid([0, 0, 0, 0, 0, 0], [1, 1, 1, 0, 0, 0], [2,2,2, math.pi/2, math.pi/2, math.pi/2], R2 = 0.8)
            #     rospy.sleep(0.1)

            # z = ar.aruco.FromArucoBasis([-1, 0, 0]) - ar.aruco.center
            # x = ar.aruco.FromArucoBasis([0, 0, 1]) - ar.aruco.center
            # y = ar.aruco.FromArucoBasis([0, 1, 0]) - ar.aruco.center
            # robot.OrientationByForceMode([x,y,z])
            # while (not rospy.is_shutdown()):
            #     robot.ForceMode(robot.GetActualTCPPose(), [1, 1, 1, 0, 0, 1], [0]*6, 2, [2, 2, 2, math.pi/2, math.pi/2, math.pi/2])


def GripHandle():
    safe_distFdoor = 1.4
    force_limits = [0.5, 0.5, 0.5, math.pi/2, math.pi/2, math.pi/2]
    offsetFOpenedDoor = 0.15
    deltaUnblokDRfromSteady = np.array([0, 0, 0.03])

    rospy.init_node("tests")
    arcontainer = ArucoTimeStampedContainer("/aruco_localizator/objects")
    robot = Robot("192.168.131.40")
    robot._rtde_c.forceModeStop()
    while not rospy.is_shutdown():

        robot.ActivateTeachMode()
        robot.OpenGripper()
        ManualPose(robot)
        robot.AlignXZtoXYPlane()
        rospy.sleep(2)

        if arcontainer.Count() > 0:
            initPose = robot.GetActualQ()
            robot.OpenGripper()
            # get Actual aruco
            ar: ArucoRelManipulatorNode = StampedArucoRelManipulator.TransformAndGetByIndex(
                arcontainer, robot.GetManipulatorHystory(), -1)
            if ar is None:
                continue
            doorPositionRelBase = copy.copy(ar.aruco.center)

            # grip handle
            robot.GripHandle(ar, force_limits)

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
            print(np.fabs(actual_opening_frame - start_opening_frame)
                  >= deltaUnblokDRfromSteady)

            while (not rospy.is_shutdown()) and (not all(np.fabs(actual_opening_frame - start_opening_frame) >= deltaUnblokDRfromSteady)):
                robot.ForceMode(robot.GetActualTCPPose(), [
                                1, 0, 1, 0, 0, 1], [-150, 0, -150, 0, 0, -50], 2, force_limits)
                actual_opening_frame = np.matmul(
                    Rotation.from_rotvec(
                        robot.GetActualTCPPose()[3:]).as_matrix(),
                    robot.GetActualTCPPose()[0:3]
                )
                print(np.fabs(actual_opening_frame - start_opening_frame))
                print(np.fabs(actual_opening_frame - start_opening_frame)
                      >= deltaUnblokDRfromSteady)
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

            # variant with avoid singularity
            while not rospy.is_shutdown():
                robot.ForceModeWithSingularityAwoid([0, 0, 0, 0, 0, 0], [1, 1, 1, 0, 0, 0], [
                                                    2, 2, 2, math.pi/2, math.pi/2, math.pi/2], R2=0.8)
                rospy.sleep(0.1)

            # disconnect from door
            # robot.SoftDetachFromObj()

            # robot.MoveL(robot._rtde_c.poseTrans(robot.GetActualTCPPose(), [0, 0, -offsetFOpenedDoor, 0, 0, 0]), 0.1, 0.1)

            # robot._rtde_c.speedStop()

            # robot.AlignXZtoXYPlane()
            # robot.AlignXZtoXYPlane()

            # arcontainer.Clean()
            # startJoggingPos = np.array(robot.GetActualTCPPose()[:3])

            # while (not rospy.is_shutdown()) and ( arcontainer.Count() == 0 ) and (np.linalg.norm( startJoggingPos - np.array( robot.GetActualTCPPose()[:3] )) < 0.5 ):
            #     robot.MoveL(robot._rtde_c.poseTrans(robot.GetActualTCPPose(), [0, 0.05, 0, 0, 0, 0]), 0.2, 0.1)

            # arcontainer.Clean()
            # rospy.sleep(3)

            # #get Actual aruco
            # ar  : ArucoRelManipulatorNode = StampedArucoRelManipulator.TransformAndGetByIndex(arcontainer, robot._eefHystory, -1)
            # if ar is None:
            #     continue

            # #move to side of the door
            # pointOnSide = ar.aruco.FromArucoBasis([0.4, 0.20, 0])
            # z = ar.aruco.FromArucoBasis([0, -1, 0]) - ar.aruco.center
            # x = ar.aruco.FromArucoBasis([0, 0, -1]) - ar.aruco.center
            # y = ar.aruco.FromArucoBasis([1, 0, 0]) - ar.aruco.center

            # cmdOnSide = list(pointOnSide) + list(robot.RotvecFromBasis([x, y, z]))
            # print(cmdOnSide)

            # Ik_ = robot._rtde_c.getInverseKinematics(cmdOnSide, [0.7215346097946167, -1.4592064062701624, 1.5015535354614258, -3.2460129896747034, -0.12220603624452764, 0.032231949269771576])
            # robot.MoveJ([0, -math.pi/2, 0, -math.pi/2, 0, 0], 0.5, 0.1)
            # robot.MoveJ(Ik_, 0.5, 0.1)

            # rot = Rotation.from_rotvec(robot.GetActualTCPPose()[3:]).as_matrix()

            # robot._rtde_c.speedL(list(np.matmul(rot, [0, 0, 0.1])) + [0, 0, 0])

            # rot = Rotation.from_rotvec(robot.GetActualTCPPose()[:3]).as_matrix()
            # tcpF = np.matmul(rot, np.array(robot.GetActualTCPForce()[:3]))
            # tcpInit = np.matmul(rot, np.array(robot.GetActualTCPForce()[:3]))
            # print(tcpF)
            # while (not rospy.is_shutdown()) and tcpF[2] >= tcpInit[2]-30:
            #         rot = Rotation.from_rotvec(robot.GetActualTCPPose()[:3]).as_matrix()
            #         tcpF = np.matmul(rot, np.array(robot.GetActualTCPForce()[:3]))
            #         print("tcp", tcpF)
            #         rospy.sleep(0.01)
            # robot._rtde_c.speedStop()
            # robot.ActivateTeachMode()
            # robot.CloseGripper()
            # rospy.sleep(1)
            # meters_to_ride = safe_distFdoor - math.fabs(doorPositionRelBase[1])
            # print("meters to ride ", meters_to_ride)
            # robot.MoveBaseX(meters_to_ride, -0.05)
            # rospy.sleep(1)
            # while (not rospy.is_shutdown()) and (robot.GetDistToTCPFromBaseXY() < 0.7):
            #     robot.ForceMode(robot.GetActualTCPPose(), [1, 0, 1, 0, 1, 0], [-250, 0 ,0, 0, 30, 0], 2, force_limits)
            # robot._rtde_c.forceModeStop()
            # robot.OpenGripper()

            # robot.MoveL(robot._rtde_c.poseTrans(robot.GetActualTCPPose(), [0, 0, -0.1, 0, 0, 0]), 0.1, 0.1)
            # robot.MoveJ( initPose, 0.1, 0.1 )


def YoloIntegration():
    from doorHandle import DoorHandle
    safe_distFdoor = 1.4
    force_limits = [0.5, 0.5, 0.5, math.pi/2, math.pi/2, math.pi/2]
    offsetFOpenedDoor = 0.15
    deltaUnblokDRfromSteady = np.array([0, 0, 0.03])

    rospy.init_node("tests")
    pub_start_move = rospy.Publisher("some_topic", Bool, queue_size=10)
    robot = Robot("192.168.131.40")
    robot._rtde_c.forceModeStop()
    door_handle = DoorHandle("/handle/points_yolo", "ur_arm_base")

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
            initPose = robot.GetActualQ()
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
            print(np.fabs(actual_opening_frame - start_opening_frame)
                  >= deltaUnblokDRfromSteady)

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
    # ToTestRandomShit()
    # GripHandle2()
    # GripHandle()
    # OffsetGet()
    # TestLocalization()
    # TestPointPublisher()
    YoloIntegration()


if __name__ == "__main__":
    main()
