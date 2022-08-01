import numpy as np
from aruco_localization.msg._aruco_msg import aruco_msg
import rospy
import copy

class ArucoIdTypes:
    door_l = 0
    door_r = 1
    door_length = range(2, 256)

class Aruco:
    def __init__(self) -> None:
        self.center         = np.zeros(3)
        self.main_corner    = np.zeros(3)
        self.normal         = np.zeros(3)
        self.corners        = np.zeros((4, 3))
        self.id             = 0
        self.toArSpaceT_Matrix = np.zeros((4,4))
        self.toArSpaceR_Matrix = np.zeros((4,4))
        self.fromArSpaceT_Matrix = np.zeros((4,4))
        self.fromArSpaceR_Matrix = np.zeros((4,4))
        self.doorOffsetInArSpace =[-0.06406069,  0.00545401,  0.06892106,  1.        ]

        self.Xaxis = np.zeros(3)
        self.Yaxis = np.zeros(3)
        self.Zaxis = np.zeros(3)

    def ToManipulator(self, tcpPose):
        pass

    def Init(self, id, mainCorner, pointsArray):
        self.id = id;
        self.corners[0]     = np.array([pointsArray[0].x, pointsArray[0].y, pointsArray[0].z ])
        self.corners[1]     = np.array([pointsArray[1].x, pointsArray[1].y, pointsArray[1].z ])
        self.corners[2]     = np.array([pointsArray[2].x, pointsArray[2].y, pointsArray[2].z ])
        self.corners[3]     = np.array([pointsArray[3].x, pointsArray[3].y, pointsArray[3].z ])
        self.main_corner    = np.array([mainCorner.x, mainCorner.y, mainCorner.z])
        for corner in self.corners:
            self.center     += corner
        self.center         /= len(self.corners)
        self.normal         = np.cross(self.main_corner - self.corners[1], self.main_corner - self.corners[2])
        if self.normal[2] > 0:
            self.normal     = -self.normal
        self.normal /= np.linalg.norm(self.normal) 

        self.InitTransformationMatrices()
    
    def GetHandlePose(self):
        hinge = np.matmul(
            self.fromArSpaceT_Matrix,
            np.matmul(self.fromArSpaceR_Matrix, self.doorOffsetInArSpace)
        )
        
        
        return hinge[:3]

    def FromArucoBasis(self, point):
        p  = np.array([point[0], point[1], point[2], 1] )
        newP = np.matmul(
            self.fromArSpaceT_Matrix,
            np.matmul(self.fromArSpaceR_Matrix, p)
        )
        return newP[:3]

    def InitTransformationMatrices(self):
        xv = (self.corners[3] + self.corners[2])/2 - self.center
        xv /= np.linalg.norm(xv)
        yv = (self.corners[1] + self.corners[2])/2 - self.center
        yv /= np.linalg.norm(yv)
        zv = np.cross(xv, yv)
        zv /= np.linalg.norm(zv)

        self.Xaxis = copy.copy(xv)
        self.Yaxis = copy.copy(yv)
        self.Zaxis = copy.copy(zv)
        
        self.toArSpaceR_Matrix = np.array([
            [xv[0], xv[1], xv[2], 0],
            [yv[0], yv[1], yv[2], 0],
            [zv[0], zv[1], zv[2], 0],
            [    0,     0,     0, 1]
        ])

        self.fromArSpaceR_Matrix = np.linalg.inv(self.toArSpaceR_Matrix)

        trInBase = np.array([
            [1, 0, 0, -self.center[0]],
            [0, 1, 0, -self.center[1]],
            [0, 0, 1, -self.center[2]],
            [0, 0, 0,              1] ])
        self.toArSpaceT_Matrix = trInBase
        self.fromArSpaceT_Matrix = np.linalg.inv(self.toArSpaceT_Matrix)

class ArucoTimeStampedContainer:
    def __init__(self, topicName) -> None:
        self._topicName     = topicName
        self._buffer        = []
        self._sub           = rospy.Subscriber(self._topicName, aruco_msg, self.callback, queue_size=1)
        self._timeDictName  = "time"
        self._arucDictName  = "aruco"
        pass

    def callback(self, data : aruco_msg):
        if( len(self._buffer) > 0 and self._buffer[0][self._timeDictName] != data.timeTag):
            self._buffer.clear()
        ar = Aruco()
        ar.Init(data.arucoId, data.points[0], data.points)
        self._buffer.append({self._timeDictName : data.timeTag, self._arucDictName : ar})

    def GetArucoByIndex(self, i):
        return self._buffer[i][self._arucDictName], self._buffer[i][self._timeDictName]

    def Count(self):
        return len(self._buffer)

    def Clean(self):
        self._buffer.clear()

    def GetLast(self):
        pass

    def GetBuffer(self):
        return copy.copy(self._buffer)