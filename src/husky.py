#!/usr/bin/python3
# calculation section
from scipy.spatial.transform import Rotation
import numpy as np
import math

# default libs
import rospy
import tf
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from collections import deque
import rtde_control
import rtde_receive
import time
from scipy.spatial.transform import Rotation
import copy

try:
    import actionlib
    from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
    from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq
except ImportError:
    rospy.logwarn("robotiq import error")

# auto generated
from aruco_localization.msg._aruco_msg import aruco_msg

# custom classes
from old_pipeline.aruco import Aruco, ArucoIdTypes
from old_pipeline.custom_types import ArucoRelManipulatorNode
from param_provider import ParamProvider


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


class Robot:
    def __init__(self, UR_IP) -> None:
        self._velocityTopicName = ParamProvider.vel_topic
        self._velocityTopic = rospy.Publisher(
            self._velocityTopicName, Twist, queue_size=1)

        self._odomTopicName = ParamProvider.odom_topic
        self._odomSub = rospy.Subscriber(
            self._odomTopicName, Odometry, callback=self.OdomCallback, queue_size=1)
        self._baseActualPosition = None
        self._baseActualTwist = None
        self._targetTwist = Twist()

        self._rtde_c = rtde_control.RTDEControlInterface(
            UR_IP, rtde_control.RTDEControlInterface.FLAG_VERBOSE | rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
        self._rtde_r = rtde_receive.RTDEReceiveInterface(UR_IP)
        self._eefHystory = PositionHystrory(self.GetActualTCPPose)

        self.poseLocked = False
        try:
            self._robotiq_client = actionlib.SimpleActionClient(
                'command_robotiq_action', CommandRobotiqGripperAction)
        except:
            self._robotiq_client = None

    # base control section

    def GetBaseTwist(self):
        return copy.copy(self._baseActualTwist)

    def GetBasePosition(self):
        return copy.copy(self._baseActualPosition)

    def OdomCallback(self, odom):
        self._baseActualPosition = odom.pose.pose
        self._baseActualTwist = odom.twist.twist
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

    def SetBaseSpeed(self, cmd):
        t = TwistStamped()
        t.twist.linear.x = cmd[0]
        t.twist.linear.y = cmd[1]
        t.twist.linear.z = cmd[2]
        t.twist.angular.x = cmd[3]
        t.twist.angular.y = cmd[4]
        t.twist.angular.z = cmd[5]
        t.header.stamp = rospy.get_rostime()
        self._velocityTopic.publish(t)

    def BaseSpeedCallBack(self, args):
        t = copy.copy(self._targetTwist)
        self._velocityTopic.publish(t)

    def MoveBaseX(self, meters, speed, checkRate_ms=50):
        self._targetTwist.linear.x = speed
        timer = rospy.Timer(rospy.Duration(
            checkRate_ms / 1000), self.BaseSpeedCallBack)

        startPosition = np.array([self.GetBasePosition().position.x,
                                  self.GetBasePosition().position.y,
                                  self.GetBasePosition().position.z])

        actualPosition = np.array([self.GetBasePosition().position.x,
                                   self.GetBasePosition().position.y,
                                   self.GetBasePosition().position.z])

        while (not rospy.is_shutdown()) and np.linalg.norm(startPosition - actualPosition) <= meters:
            rospy.sleep(checkRate_ms / 1000)
            actualPosition = np.array([self.GetBasePosition().position.x,
                                       self.GetBasePosition().position.y,
                                       self.GetBasePosition().position.z])

        timer.shutdown()

    def RotateBaseZ(self, angle, speed, checkRate_ms=500):
        pass

    # gripper

    def CloseGripper(self):
        if self._robotiq_client is not None:
            self._robotiq_client.wait_for_server()
            Robotiq.goto(self._robotiq_client, pos=0.0,
                         speed=0.1, force=1, block=False)

    def OpenGripper(self):
        if self._robotiq_client is not None:
            self._robotiq_client.wait_for_server()
            Robotiq.goto(self._robotiq_client, pos=1,
                         speed=0.1, force=1, block=False)

    # rtde library part

    def _IsNaNorInf(self, array):
        for q in array:
            if math.isnan(q) or math.isinf(q):
                return True
        return False

    def GetActualQ(self):
        return self._rtde_r.getActualQ()

    def GetActualTCPPose(self):
        return self._rtde_r.getActualTCPPose()

    def GetActualTCPSpeed(self):
        return self._rtde_r.getActualTCPSpeed()

    def GetActualQd(self):
        return self._rtde_r.getActualQd()

    def GetActualTCPForce(self):
        return self._rtde_r.getActualTCPForce()

    def MoveJ(self, pose, vel=1.05, acc=1.4, asyncro=False):
        if self._IsNaNorInf(pose):
            rospy.logwarn("robot controller got inf or nan")
            return False
        return self._rtde_c.moveJ(pose, vel, acc, asyncro)

    def MoveL(self, pose, vel=0.25, acc=1.2, asyncro=False):
        if self._IsNaNorInf(pose):
            rospy.logwarn("robot controller got inf or nan")
            return False
        return self._rtde_c.moveL(pose, vel, acc, asyncro)

    def SpeedJ(self, speed, acc=0.5, time=0.0):
        return self._rtde_c.speedJ(speed, acc, time)

    def SpeedL(self, speed, acc=0.25, time=0.0):
        return self._rtde_c.speedL(speed, acc, time)

    def SpeedStop(self, acc=10.0):
        self._rtde_c.speedStop(acc)

    def ActivateTeachMode(self):
        return self._rtde_c.teachMode()

    def DeactivateTeachMode(self):
        return self._rtde_c.endTeachMode()

    def ForceMode(self, targetFrame, selector, wrench, mode, limits):
        self._rtde_c.forceMode(targetFrame, selector, wrench, mode, limits)

    def PushUntilForce(self, selector, targetForce, limits, forceStep=[10]*6, timeOut=-1):
        startTime = time.time()
        prevPose = np.array(self.GetActualTCPPose())
        checkTime = time.time()
        while not rospy.is_shutdown():
            self.ForceMode(self.GetActualTCPPose(),
                           selector, targetForce, 2, limits)
            if timeOut > 0 and time.time() - startTime > timeOut:
                break
            rospy.sleep(0.1)
            if time.time() - checkTime > 2:
                checkTime = time.time()
                if all(np.array(self.GetActualTCPPose()) - prevPose < 0.03):
                    break
                prevPose = self.GetActualTCPPose()
        self._rtde_c.forceModeStop()

    def AlignYZtoXYPlane(self, vel=0.25, acc=1.2, asyncro=False):
        actual = self.GetActualTCPPose()
        rot = Rotation.from_rotvec(actualPose[3:]).as_matrix()
        x = np.matmul(rot, [1, 0, 0])
        y = np.matmul(rot, [0, 1, 0])
        z = np.matmul(rot, [0, 0, 1])
        newRot = Rotation.from_matrix([
            [0,    y[0]/math.sqrt(y[0]**2 + y[1]**2),
             z[0]/math.sqrt(z[0]**2 + z[1]**2)],
            [0,    y[1]/math.sqrt(y[0]**2 + y[1]**2),
             z[1]/math.sqrt(z[0]**2 + z[1]**2)],
            [x[2]/math.fabs(x[2]), 0,    0]
        ]).as_rotvec()
        cmd = np.concatenate((actualPose[0:3], newRot))
        self.MoveL(cmd, vel, acc, asyncro)

    def AlignXZtoXYPlane(self, vel=0.25, acc=1.2, asyncro=False):
        actual = self.GetActualTCPPose()
        rot = Rotation.from_rotvec(actual[3:]).as_matrix()
        x = np.matmul(rot, [1, 0, 0])
        y = np.matmul(rot, [0, 1, 0])
        z = np.matmul(rot, [0, 0, 1])
        if y[2] < 0:
            newRot = Rotation.from_matrix([
                [x[0]/math.sqrt(x[0]**2 + x[1]**2), 0, z[0] /
                 math.sqrt(z[0]**2 + z[1]**2)],
                [x[1]/math.sqrt(x[0]**2 + x[1]**2), 0,
                 z[1]/math.sqrt(z[0]**2 + z[1]**2)],
                [0,                                y[2] /
                 math.fabs(y[2]),    0]
            ]).as_rotvec()
        else:
            newRot = Rotation.from_matrix([
                [-x[0]/math.sqrt(x[0]**2 + x[1]**2), 0, z[0] /
                 math.sqrt(z[0]**2 + z[1]**2)],
                [-x[1]/math.sqrt(x[0]**2 + x[1]**2), 0,
                 z[1]/math.sqrt(z[0]**2 + z[1]**2)],
                [0,                                -
                 y[2]/math.fabs(y[2]),    0]
            ]).as_rotvec()

        cmd = np.concatenate((actual[0:3], newRot))
        self.MoveL(cmd, vel, acc, asyncro)

    def IfAlignedYZtoXYPlane(self, eps=0.1):
        actual = self.GetActualTCPPose()
        rot = Rotation.from_rotvec(actual[3:]).as_matrix()
        x = np.matmul(rot, [1, 0, 0])
        y = np.matmul(rot, [0, 1, 0])
        z = np.matmul(rot, [0, 0, 1])
        return (math.fabs(y[2]) < eps and math.fabs(z[2]) < eps)

    def RotvecFromBasis(self, basis):
        return Rotation.from_matrix([
            [basis[0][0], basis[1][0], basis[2][0]],
            [basis[0][1], basis[1][1], basis[2][1]],
            [basis[0][2], basis[1][2], basis[2][2]],
        ]).as_rotvec()

    def GripHandle(self, ar: ArucoRelManipulatorNode, force_limits):
        handle_pos = ar.aruco.GetHandlePose()
        cmd = list(handle_pos) + list(ar.rotvecAlignedNormal)
        self.MoveL(cmd, 0.2, 0.2)
        self._rtde_c.forceMode(self.GetActualTCPPose(), [0, 0, 0, 1, 0, 1], [
                               0, 0, 0, 0, 0, 0], 2, force_limits)
        self.CloseGripper()
        rospy.sleep(1)
        return True

    def ForceModeWithSingularityAwoid(self, force, selector, limits, R1=0.48, R2=0.9, fmax=350):
        def Force(x, R1, R2, fmax):
            n = 8*fmax/((R1-R2)**3)
            b = -(R1+R2)/2.0
            return n*((x+b)**3)

        def ToTCP(pose, vec):
            v = np.array(vec)
            rot = Rotation.from_rotvec(pose[3:]).as_matrix()
            return np.matmul(np.linalg.inv(rot), v)

        npF = np.array(force)
        actual = self.GetActualTCPPose()
        normal = np.linalg.norm(actual[:3])
        f = Force(math.sqrt(actual[0]**2 + actual[1]**2), R1, R2, fmax)
        force = [actual[0]*f/normal, actual[1]*f/normal, 0]

        forceInTCP = ToTCP(actual, force)
        cmd = np.array(list(forceInTCP) + [0]*3) + npF
        self._rtde_c.forceMode(actual, selector, cmd, 2, limits)

    def SoftDetachFromObj(self):
        self._rtde_c.forceMode(
            self.GetActualTCPPose(),
            [1, 1, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            2,
            [0.5, 0.5, 0.5, math.pi/2, math.pi/2, math.pi/2])

        self.OpenGripper()
        self._rtde_c.forceModeStop()

    def GetDistToTCPFromBaseXY(self):
        curTCP = np.array(self.GetActualTCPPose()[0:2])
        return np.linalg.norm(curTCP)

    def GetManipulatorHystory(self):
        return self._eefHystory

    def LockPose(self):
        self.poseLocked = True

    def PoseUnlock(self):
        self.poseLocked = False

    def OrientationByForceMode(self, estimationBasis, epsilon=0.1):
        estimationBasis = np.array(estimationBasis)
        newRot = Rotation.from_matrix(
            [np.transpose(estimationBasis)]
        ).as_rotvec()

        while (not rospy.is_shutdown()) and not (np.abs(np.array(self.GetActualTCPPose()[3:]) - newRot) < epsilon).all():
            diff = -np.array(self.GetActualTCPPose())[3:] + newRot
            f = [0, 0, 0] + list(10*diff[0])
            print(diff[0])
            self.ForceMode(self.GetActualTCPPose(),
                           [0, 0, 0, 1, 1, 1],
                           f,
                           2,
                           [2, 2, 2, math.pi, math.pi, math.pi])
            time.sleep(0.1)

    def ChangeOrientation(self, rotvec, vel = 0.25, acc = 1.2, asyncro = False):
        actual = self.GetActualTCPPose()
        self.MoveL([actual[0], actual[1], actual[2], rotvec[0], rotvec[1], rotvec[2]], vel, acc, asyncro)
