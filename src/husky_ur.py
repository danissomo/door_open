import rospy
import math
import numpy as np
import rtde_control, rtde_receive
import time
from scipy.spatial.transform import Rotation
from old_pipeline.custom_types import ArucoRelManipulatorNode
class HuskyUr:

    def __init__(self, UR_IP) -> None:
        self._rtde_c = rtde_control.RTDEControlInterface(
            UR_IP, rtde_control.RTDEControlInterface.FLAG_VERBOSE | rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
        self._rtde_r = rtde_receive.RTDEReceiveInterface(UR_IP)
    

    # rtde library part
    def ForceModeStop(self):
        self._rtde_c.forceModeStop()


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
        rot = Rotation.from_rotvec(actual[3:]).as_matrix()
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
        cmd = np.concatenate((actual[0:3], newRot))
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

    
    def MoveL_point_rot(self, pose, orient, vel=0.25, acc=1.2, asyncro=False):
        cmd = list(pose) + list(orient)
        self.MoveL(cmd, vel, acc, asyncro)
