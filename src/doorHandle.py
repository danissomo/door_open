#!/usr/bin/env python3

import time

import rospy
import tf
from geometry_msgs.msg import PoseArray, PointStamped
import copy 
import numpy as np

from utils import PointStampedToNumpy




class DoorHandle:
    def __init__(self, frame = 'local_map_lidar',) -> None:
        self.global_frame = frame
        self.coordinate_system_global = None        #array of length 3 with vectors(numpy) in map coordinates
        self.handle_keypoints_global = None         #array of PointStamped class in map coordinates
        
        #lover fields used for moving manipulator 
        self.coordinate_system_rel = None           #array of lenght 3 with vectors (numpy) in relative coordinates
        self.handle_keypoints_rel = None            #array of PointStamped class in any relative coordinates
    
    

    def _UpdateGlobalsParams(self, points : list):
        '''
        Global Frame. Building coordinate system from handle keypoints
        Temp Z-axis calulated from 0 an 1 keypoints.
        Y-Axis calculated from horizontal line from 1 and 2 keypoints.
        X-Axis - cross product of temp Z and Y.
        Real Z - cross product of X and Y.
        '''
        self.handle_keypoints_global = copy.copy(points)
        zAxisTemp = PointStampedToNumpy(points[0]) - PointStampedToNumpy(points[1])
        yAxis = PointStampedToNumpy(points[1]) - PointStampedToNumpy(points[2])
        xAxis = np.cross( zAxisTemp, yAxis)
        zAxis = np.cross(xAxis, yAxis)
        self.coordinate_system_global = [xAxis, yAxis, zAxis]
        for i, v in enumerate(self.coordinate_system_global):
            self.coordinate_system_global[i] /= np.linalg.norm(v)

        

    def _UpdateRelativeParams(self, points : list):
        '''
        @points - list of PointStamed
        Global Frame. Building coordinate system from handle keypoints.
        Relative frame is ur_arm_base.
        Using cross product of temporal Z-axis.
        Temp Z-axis calulated from 0 an 1 keypoints.
        '''
        self.handle_keypoints_rel = copy.copy(points)
        zAxisTemp = PointStampedToNumpy(points[0]) - PointStampedToNumpy(points[1])
        yAxis = PointStampedToNumpy(points[1]) - PointStampedToNumpy(points[2])
        xAxis = np.cross( zAxisTemp, yAxis)
        zAxis = np.cross(xAxis, yAxis)
        self.coordinate_system_rel = [xAxis, yAxis, zAxis]
        for i, v in enumerate(self.coordinate_system_rel):
            self.coordinate_system_rel[i] /= np.linalg.norm(v)

        

    def GetMiddlePointGlob(self):
        '''
        calculate point for determine door in map
        '''
        if self.handle_keypoints_global is None:
            import sys
            raise AttributeError("Handle keypoints is None").with_traceback(sys.exc_info()[2])
        rt = (    PointStampedToNumpy(self.handle_keypoints_global[1])
                + PointStampedToNumpy(self.handle_keypoints_global[2]) ) / 2.0
        return rt
    
    def GetMiddlePointRel(self):
        '''
        calculate point that used for grabbing handle, middle between 2 keypoints
        '''
        if self.handle_keypoints_rel is None:
            import sys
            raise AttributeError("Handle keypoints is None").with_traceback(sys.exc_info()[2])
        rt = (    PointStampedToNumpy(self.handle_keypoints_rel[1])
                + PointStampedToNumpy(self.handle_keypoints_rel[2]) ) / 2.0
        return rt





class DoorHandleHandler:
    def __init__(self, topicName, resultFrameName, callback = lambda doorHandle : 0, is_debug = False) -> None:
        '''
        PARAMS:
        @topicName - name of topic with handle data
        @callback - will be called after self callback, default None
        @is_debug - verbose printing 
        '''
        self._externalCallback = callback
        self._tfListener = tf.TransformListener()
        self.topicToListenName = topicName
        self._topicSub = rospy.Subscriber( self.topicToListenName, 
                                           PoseArray, 
                                           callback=self.handleSkeletonCallback, 
                                           queue_size=10 )
        
        self.frameToTransformName = resultFrameName
        self.actua_door_handle  = DoorHandle() #new interface

        self._is_debug = is_debug

        self._update_flag = True 


            

    def handleSkeletonCallback(self, poseArray : PoseArray):
        '''
        Callback that  called every time when data published in topic self.topicToListen.
        Uses tf for transform points from camera to manipulator frame and global framae.
        '''
        if not self._update_flag: return
        wait_for_transform = lambda from_frame, to_frame : \
                                self._tfListener.waitForTransform( to_frame, from_frame,  rospy.Time.now(), rospy.Duration(4.0))

        def _transform_pose_array(frame, poseArray : PoseArray):
            try:
                wait_for_transform(poseArray.header.frame_id, frame)
                tf_function = lambda point : self._tfListener.transformPoint(frame, PointStamped(point = copy.copy(point), header = poseArray.header))
                points_in_base =list (map(tf_function, poseArray.poses)) 
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("EXCEPTION IN TF {}".format(e))
                return None
            return points_in_base

        points_in_base = _transform_pose_array(self.frameToTransformName, poseArray)
        if points_in_base is None: return
        self.actua_door_handle._UpdateRelativeParams(points_in_base)
        if self._is_debug:
            self._Debug()
        self._externalCallback(self)
        rospy.loginfo_once("HANDLE DATA CAPTURED")

        '''
        This section working only if lolodom is online
        Transforming coordinates to map and save handle.
        '''
        points_in_world = _transform_pose_array("local_map_lidar", poseArray)
        if points_in_world is None: return
        self.actua_door_handle._UpdateGlobalsParams(points_in_world)



    def _Debug(self):
        from prettytable import PrettyTable
        table = PrettyTable()
        
        print("HANDLE FRAME")
        for i, pointS in  enumerate(self.actua_door_handle.coordinate_system_rel):
            table.add_column(str(i), [ pointS.point.x, pointS.point.y, pointS.point.z])

        print(table)
        print()

        print("HANDLE COORDINATE SYSTEM")
        table = PrettyTable()
        table.add_row(["i", "j", "k"])
        for i in range(3):
            table.add_row([self.actua_door_handle.coordinate_system_rel[j][i] for j in range(3)])
        print(table)
        
        print("-" * len("HANDLE COORDINATE SYSTEM"))



    def GetActualDoorHandle(self):
        cp_h = copy.copy(self.actua_door_handle)
        if cp_h.coordinate_system_rel is None:
            return None
        return cp_h



    def DeleteCallback(self):
        self._externalCallback = lambda doorHandle : 0




    def StopUpdateHandle(self):
        rospy.loginfo("HANDLE UPDATING PAUSED")
        self._update_flag = False
    


    def StartUpdateHandle(self):
        rospy.loginfo("HANDLE UPDATE STARTED")
        self._update_flag = True



    def WaitData(self, timeout = 1):
        start_time = time.time()
        while (self.actua_door_handle.coordinate_system_rel is None) and time.time() - start_time <= timeout:
            rospy.sleep(0.001)
        return time.time() - start_time <= timeout
    


    def ClearData(self):
        self.StopUpdateHandle()
        self.actua_door_handle = DoorHandle()
        self.StartUpdateHandle()




if __name__ == "__main__":
    '''
    For debug this DoorHandleHandler class 
    '''
    rospy.init_node("DebugHandleCapture")
    from husky import Robot
    robot = Robot("127.0.0.1")
    rospy.sleep(3)
    DoorHandleHandler("/handle/points_yolo", "ur_arm_base", is_debug=True)
    rospy.loginfo("Debug started")
    rospy.spin()