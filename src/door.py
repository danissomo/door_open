import copy
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


    def addIfExUpdateIfNotEx(self, door_ctx : DoorContext,  eps = 1):
        d, i = self._door_tree.query(door_ctx.door.GlobPosAsNumpy())
        if np.linalg.norm(d) < eps:
            door_ctx_old : DoorContext = self._door_tree[i]
            door_ctx_old.door.door_handle = door_ctx.door

    def find(self, ps : PoseStamped, eps) -> DoorContext:
        pass