#!/usr/bin/env python3
import rospy
from husky_base import HuskyBase
import rtde_control, rtde_receive
import numpy as np
from nav_msgs.msg import Odometry
import math
from scipy.spatial.transform import Rotation
class customBase(HuskyBase):
    def __init__(self) -> None:
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.131.40", 
        rtde_control.RTDEControlInterface.FLAG_VERBOSE | rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.131.40")
        self.rtde_c.teachMode()
        input("set 1 press enter")
        self.p1 = np.array(self.rtde_r.getActualTCPPose())
        input("set 2 press enter")
        self.p2 = np.array(self.rtde_r.getActualTCPPose())
        self.rtde_c.endTeachMode()
        self.rtde_c.moveL(self.p1)
        steps = 100
        self.trajectory = [ t*(self.p2 - self.p1) + self.p1 for t in np.linspace(0, 1, steps)]
        self.inc = 1
        self.cur = 0
        self.offset = np.zeros(6)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.ArmFollowTrajectory)

        super().__init__()

    def ArmFollowTrajectory(self, data):
        print(self.offset)
        self.rtde_c.servoL(self.trajectory[self.cur] + self.offset, 1, 1, 0.1, 0.1, 600)

        
        self.cur += self.inc
        if self.cur == len(self.trajectory) or self.cur == -1:
            self.inc *=-1
            self.cur += self.inc

    def OdomCallback(self, odom : Odometry):
        twistLin = [odom.twist.twist.linear.x,
                    odom.twist.twist.linear.y,
                    odom.twist.twist.linear.z ]
        twistAng = [odom.twist.twist.angular.x,
                    odom.twist.twist.angular.y,
                    odom.twist.twist.angular.z ]
        dt =0.10
        curPose = self.rtde_r.getActualTCPPose()
        plank = np.linalg.norm(np.array(curPose[0:2]) + np.array([0, 0.307]))
        b = 0.307
        k = -(curPose[1]-b)/curPose[0]
        vy = plank*twistAng[2]/math.sqrt(k**2+1)
        vx = vy*k
        rot = Rotation.from_euler("xyz", [twistAng[1]*dt,
            -twistAng[0]*dt,
            twistAng[2]*dt]).as_rotvec()
        self.offset -= np.array([
            (twistLin[1] + vx)*dt,
            (-twistLin[0] + vy)*dt,
            (twistLin[2])*dt,
            rot[0],
            rot[1],
            rot[2]
        ])
        return super().OdomCallback(odom)


if __name__ == "__main__":
    rospy.init_node("test_odom_catch")
    b = customBase()

    rospy.spin()