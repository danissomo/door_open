#!/usr/bin/env python3

import time
import rospy
import tf
from geometry_msgs.msg import PoseArray, PointStamped
import copy 
import numpy as np

class DoorHandle:
    def __init__(self, topicName, resultFrameName, callback = lambda doorHandle : 0, is_debug = False) -> None:
        self._externalCallback = callback
        self._tfListener = tf.TransformListener()
        self.topicToListenName = topicName
        self._topicSub = rospy.Subscriber( self.topicToListenName, 
                                           PoseArray, 
                                           callback=self.handleSkeletonCallback, 
                                           queue_size=10 )
        
        self.frameToTransformName = resultFrameName
        self.coordinateSystem = None
        self.handleSkeleton = None

        self._is_debug = is_debug

        self._update_flag = True 
        pass



    def handleSkeletonCallback(self, poseArray : PoseArray):
        if not self._update_flag: return
        #some capturing
        points_in_base = []
        try:
            self._tfListener.waitForTransform(self.frameToTransformName, poseArray.header.frame_id, rospy.Time.now(), rospy.Duration(4.0))
            for point in poseArray.poses:
                bufpoint = PointStamped()
                bufpoint.header = poseArray.header
                bufpoint.point.x = point.position.x
                bufpoint.point.y = point.position.y
                bufpoint.point.z = point.position.z
                points_in_base.append( self._tfListener.transformPoint(self.frameToTransformName, bufpoint) )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("EXCEPTION IN TF {}".format(e))
            return
        self.handleSkeleton = copy.copy(points_in_base)
        self._BuildCoordinateSystemOfHandle(self.handleSkeleton)

        if self._is_debug:
            self._Debug()
        
        self._externalCallback(self)
        rospy.loginfo_once("HANDLE DATA CAPTURED")



    def _Debug(self):
        from prettytable import PrettyTable
        table = PrettyTable()
        
        print("HANDLE FRAME")
        for i, pointS in  zip(range(len(self.handleSkeleton)), self.handleSkeleton):
            table.add_column(str(i), [ pointS.point.x, pointS.point.y, pointS.point.z])

        print(table)
        print()

        print("HANDLE COORDINATE SYSTEM")
        table = PrettyTable()
        table.add_row(["i", "j", "k"])
        for i in range(3):
            table.add_row([self.coordinateSystem[0][i], self.coordinateSystem[1][i], self.coordinateSystem[2][i]])
        print(table)
        
        print("-" * len("HANDLE COORDINATE SYSTEM"))




    def PointStampedToNumpy(self, p : PointStamped):
        return np.array([p.point.x, p.point.y, p.point.z])



    def _BuildCoordinateSystemOfHandle(self, points : list):
        zAxisTemp = self.PointStampedToNumpy(points[0]) - self.PointStampedToNumpy(points[1])
        yAxis = self.PointStampedToNumpy(points[1]) - self.PointStampedToNumpy(points[2])
        xAxis = np.cross( zAxisTemp, yAxis)
        zAxis = np.cross(xAxis, yAxis)

        xAxis /= np.linalg.norm(xAxis)
        yAxis /= np.linalg.norm(yAxis)
        zAxis /= np.linalg.norm(zAxis)

        self.coordinateSystem = [xAxis, yAxis, zAxis]



    def DeleteCallback(self):
        self._externalCallback = lambda doorHandle : 0



    def GetActualCoordSystem(self):
        cp_cords = copy.copy(self.coordinateSystem)
        self.coordinateSystem = None
        return cp_cords


    def GetActualMiddlePoint(self):
        cp_handle_points = copy.copy(self.handleSkeleton)
        if cp_handle_points is None: return
        self.handleSkeleton = None
        a = np.array([cp_handle_points[1].point.x, cp_handle_points[1].point.y, cp_handle_points[1].point.z])
        b = np.array([cp_handle_points[2].point.x, cp_handle_points[2].point.y, cp_handle_points[2].point.z])
        return (a + b) / 2.0


    def StopUpdateHandle(self):
        rospy.loginfo("HANDLE UPDATING PAUSED")
        self._update_flag = False
    
    def StartUpdateHandle(self):
        rospy.loginfo("HANDLE UPDATE STARTED")
        self._update_flag = True

    def WaitData(self, timeout = 1):
        start_time = time.time()
        while (self.handleSkeleton is None or self.coordinateSystem is None) and time.time() - start_time <= timeout:
            rospy.sleep(0.001)
        return time.time() - start_time <= timeout
    
    def ClearData(self):
        self.StopUpdateHandle()
        self.handleSkeleton = None
        self.coordinateSystem = None
        self.StartUpdateHandle()

            
if __name__ == "__main__":
    rospy.init_node("DebugHandleCapture")

    from husky import Robot
    robot = Robot("192.168.131.40")
    rospy.sleep(3)
    class B:
        def __init__(self, robot : Robot) -> None:
            self.robot = robot
        def callback(self, doorHandle: DoorHandle):
            rotvec = robot.RotvecFromBasis(copy.copy(doorHandle.coordinateSystem))
            middlePoint = doorHandle.PointStampedToNumpy(doorHandle.handleSkeleton[2]) + doorHandle.PointStampedToNumpy(doorHandle.handleSkeleton[3])
            middlePoint/=2.0
            robot.MoveL(list(middlePoint) + list(rotvec), 0.1, 0.01)
            doorHandle.DeleteCallback()
    
    customCallback = B(robot)
    #input("input something")
    #DoorHandle("/handle/points_yolo", "ur_arm_base", customCallback.callback)
    DoorHandle("/handle/points_yolo", "ur_arm_base", is_debug=True)
    rospy.loginfo("Debug started")
    rospy.spin()