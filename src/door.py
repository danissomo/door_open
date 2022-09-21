import copy

from geometry_msgs.msg import PointStamped
from doorHandle import DoorHandle
from scipy.spatial import KDTree
import numpy as np
from utils import PointStampedToNumpy
import rospy
from visualization_msgs.msg import Marker, MarkerArray
class Door:
    '''
    Extendable data type for doors
    @id - uniq num
    @door_handle - Handle that determine position 
    '''
    def __init__(self) -> None:
        self.id = 0
        self.door_handle : DoorHandle = DoorHandle()
           




class DoorContext:
    def __init__(self) -> None:
        self.is_left = True
        self.is_push = True
        self.door = Door()
        self.door_normal = np.zeros(3)
    
    def IsLeft(self):
        return self.is_left
    def IsRight(self):
        return not self.is_left
    def IsPush(self):
        return self.is_push
    def IsPull(self):
        return not self.is_push
    
    def SetLeft(self):
        self.is_left = True
    def SetRight(self):
        self.is_left = False
    def SetPush(self):
        self.is_push = True
    def SetPull(self):
        self.is_push = False  
    def Negate(self):
        '''
        Function for get negativee door if the position of robot is 
        negative relative to plane of door.
        '''
        rt = copy.copy(self)
        rt.is_left = self.IsRight()
        rt.is_push = self.IsPull()
        return rt
    def IsPositionPositive(self, point):
        '''
        Cheks side of given point relative to door plane.
        '''
        point = np.array([point.x, point.y, point.z])
        if isinstance(point, PointStamped):
            point = PointStampedToNumpy(point)
        rt = (point - self.door.door_handle.GetMiddlePointGlob()).dot(self.door_normal)
        rospy.loginfo(rt)
        return rt > 0

    def ToStr(self):
        rs =    f"id: {self.door.id}\n \
                left: {self.is_left}, push: {self.is_push}\n\
                normal {self.door_normal}\n\
                global {self.door.door_handle.GetMiddlePointGlob()}\n\
                relative {self.door.door_handle.GetMiddlePointRel}"
        return rs
    
    

class DoorContainer:
    def __init__(self) -> None:
        import os, pickle
        self._door_list_filename = "door_list.pkl"
        if os.path.exists(self._door_list_filename):
            self._door_list = pickle.load(self._door_list_filename)
            self._door_tree = KDTree([dctx.door.door_handle.GetMiddlePointGlob() for dctx in self._door_list])
            self._id_set = { d.door.id for d in self._door_list }
        else:           
            self._door_list = []
            self._door_tree : KDTree = None
            self._id_set = set()
        
        self.marker_pub = rospy.Publisher("handle_markers", MarkerArray, queue_size=10)
        self.timer_marker_maker = rospy.Timer(rospy.Duration(1), self.DrawMarkers)
        self._handle_colors = dict()


    def IsKnown(self, doorCtx: DoorContext, eps = 0.5):
        if len(self._door_list)  == 0:
            return False
        d, i = self._door_tree.query(doorCtx.door.door_handle.GetMiddlePointGlob())
        return np.linalg.norm(d) <= eps

    def AddIfNotKnown(self, doorCtx: DoorContext, eps = 0.5):
        if not self.IsKnown(doorCtx, eps):
            import random
            rn = random.randint(0, 2**100)
            while rn in self._id_set:
                rn = random.randint(0, 2**100)
            self._id_set.add(rn)
            doorCtx.door.id = rn
            self._door_list.append(copy.deepcopy(doorCtx))
            self._door_tree = KDTree([dctx.door.door_handle.GetMiddlePointGlob() for dctx in self._door_list])
        else:
            d, i = self._door_tree.query(doorCtx.door.door_handle.GetMiddlePointGlob())
            self._door_list[i].door.door_handle.coordinate_system_global = copy.copy(doorCtx.door.door_handle.coordinate_system_global)
            self._door_list[i].door.door_handle.coordinate_system_rel  = copy.copy(doorCtx.door.door_handle.coordinate_system_rel)


    def GetNearestCtx(self, position, eps = 0.5):
        if self._door_tree is None:
            return None
        d, i = self._door_tree.query(position)
        if np.linalg.norm(d) < eps:
            return self._door_list[i]
        else:
            return None


    def save(self):
        import pickle
        pickle.dump(self._door_list, self._door_list_filename, pickle.HIGHEST_PROTOCOL)



    def DrawMarkers(self, someData): 
        '''
        Draw circles of random color in rviz.
        Calling by rospy.Timer that starts in __init__
        TODO: think about ns and topic name, change to stl models of door and handle
        '''
        if len(self._door_list) == 0:
            return
        markerA = MarkerArray()

        marker = Marker()
        marker.type = Marker.SPHERE
        marker.header.frame_id = "local_map_lidar"
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1
        marker.ns = "my_namespace";
        marker.action = Marker.ADD
        marker.pose.orientation.x = 0 
        marker.pose.orientation.y = 0 
        marker.pose.orientation.z = 0 
        marker.pose.orientation.w = 1 
        for door_ctx in  self._door_list:
            door_ctx : DoorContext = door_ctx
            for pointS in door_ctx.door.door_handle.handle_keypoints_global:
                pointS : PointStamped= pointS
                marker.pose.position.x = pointS.point.x
                marker.pose.position.y = pointS.point.y
                marker.pose.position.z = pointS.point.z
            if door_ctx.door.id not in self._handle_colors:
                import random
                marker.color.r = random.random()
                marker.color.g = random.random()
                marker.color.b = random.random()
                self._handle_colors[door_ctx.door.id] = copy.copy(marker.color)
            else:
                marker.color = self._handle_colorsp[door_ctx.door.id]
        self.marker_pub.publish(markerA)