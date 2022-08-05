#!/usr/bin/python3
# calculation section
from scipy.spatial.transform import Rotation
import numpy as np
import math

# default libs
import rospy
import tf

from collections import deque
from scipy.spatial.transform import Rotation
import copy



# auto generated
from aruco_localization.msg._aruco_msg import aruco_msg

# custom classes
from old_pipeline.aruco import Aruco, ArucoIdTypes
from old_pipeline.custom_types import ArucoRelManipulatorNode
from param_provider import ParamProvider
from husky_gripper import HuskyGripper
from husky_ur import HuskyUr
from husky_base import HuskyBase

class PositionHystrory:
    def __init__(self, eefPoseGetter, maxlen=1000) -> None:
        self._eefPoseGetter = eefPoseGetter
        self.hystory = deque(maxlen=maxlen)
        self._time = "time"
        self._pose = "pose"
        self.timer = rospy.Timer(rospy.Duration(nsecs=1), self.callback)
        

    def callback(self, arg):
        self.hystory.append({self._time: rospy.get_rostime(),
                             self._pose: self._eefPoseGetter()})
        manipulatorPose = self._eefPoseGetter()
        br = tf.TransformBroadcaster()
        cur_time = rospy.get_rostime()
        try:
            br.sendTransform(
                (manipulatorPose[0], manipulatorPose[1], manipulatorPose[2]),
                list(Rotation.from_rotvec(manipulatorPose[3:]).as_quat() ),
                cur_time,
                "ur_gripper",
                "ur_arm_base"
            )

            br.sendTransform(
                ParamProvider.rs_frame,
                (0, 0, 0, 1),
                cur_time,
                ParamProvider.rs_frame_name,
                "ur_gripper",
            )
        except rospy.exceptions.ROSException as e:
            rospy.logwarn("ERROR PUBLISH TO TF-TREE: {}".format(e))
        rospy.loginfo_once("UR5 PUBLIHED TO TF-TREE")

    def FindPoseByMinTimeDiff(self, time):
        actual = self.hystory.copy()
        minTimeDiff = abs(time.data.to_nsec() -
                          actual[0][self._time].to_nsec())
        minIndex = 0

        for i in range(len(actual)):
            curDiff = abs(
                actual[i][self._time].to_nsec() - time.data.to_nsec())
            if minTimeDiff >= curDiff:
                minTimeDiff = curDiff
                minIndex = i
        return actual[minIndex][self._pose]

    def GetHystory(self):
        return copy.copy(self.hystory)


class Robot(HuskyGripper, HuskyUr, HuskyBase):
    def __init__(self, UR_IP) -> None:
        
        self.poseLocked = False

        HuskyBase.__init__(self)
        HuskyGripper.__init__(self)
        HuskyUr.__init__(self, UR_IP)

        self._eefHystory = PositionHystrory(self.GetActualTCPPose)
        

    

    def OdomCallback(self, odom):
        HuskyBase.OdomCallback(self, odom)
        if self.poseLocked:
            self.CorrectPositionByTwist()



    def CorrectPositionByTwist(self):
        twist = copy.copy(self._baseActualTwist)
        pose = self.GetActualTCPPose()

        cmd = [twist.linear.y - np.linalg.norm(pose[:2])*twist.angular.z*math.cos(math.atan2(pose[1] - 0.389, pose[0]) + math.pi/2),
               twist.linear.x - np.linalg.norm(pose[:2])*twist.angular.z*math.sin(
            math.atan2(pose[1] - 0.389, pose[0]) + math.pi/2),
            -twist.linear.z,
            twist.angular.y,
            twist.angular.x,
            -twist.angular.z]
        self.SpeedL(cmd, 2)



    def GripHandle(self, ar: ArucoRelManipulatorNode, force_limits):
        handle_pos = ar.aruco.GetHandlePose()
        cmd = list(handle_pos) + list(ar.rotvecAlignedNormal)
        self.MoveL(cmd, 0.2, 0.2)
        self._rtde_c.forceMode(self.GetActualTCPPose(), [0, 0, 0, 1, 0, 1], [
                               0, 0, 0, 0, 0, 0], 2, force_limits)
        self.CloseGripper()
        rospy.sleep(1)
        return True
    


    def ManualPose(self):
        self.ActivateTeachMode()
        print(self.GetActualTCPPose())
        input("set init pose and press enter")
        self.DeactivateTeachMode()
