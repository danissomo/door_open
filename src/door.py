import copy
from random import randint
from geometry_msgs.msg import PoseStamped
from doorHandle import DoorHandle
from scipy.spatial import KDTree
import numpy as np
class Door:
    def __init__(self) -> None:
        self.id = 0
        self.door_handle : DoorHandle = DoorHandle()
           




class DoorContext:
    def __init__(self) -> None:
        self.is_left = True
        self.is_push = True
        self.door = Door()
    
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
        rt = copy.copy(self)
        rt.is_left = self.IsRight()
        rt.is_push = self.IsPull()
        return rt

    
class DoorContainer:
    def __init__(self) -> None:
        self._door_list = []
        self._door_tree : KDTree = None
        self._id_set = set()


    def IsKnown(self, doorCtx: DoorContext, eps = 0.5):
        d, i = self._door_tree.query(doorCtx.door.door_handle.GetMiddlePoint())
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
            self._door_tree = KDTree([dctx.door.door_handle.GetMiddlePoint() for dctx in self._door_list])
        else:
            d, i = self._door_tree.query(doorCtx.door.door_handle.GetMiddlePoint())
            self._door_list[i].door.door_handle.coordinate_system_global = copy.copy(doorCtx.door.door_handle.coordinate_system_global)
            self._door_list[i].door.door_handle.coordinate_system_rel  = copy.copy(doorCtx.door.door_handle.coordinate_system_rel)


    def find(self, ps : PoseStamped, eps) -> DoorContext:
        pass