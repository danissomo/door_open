from selector_generator import SelectorVec
from enum import Enum, auto
import math

class FrameTypeEnum(Enum):
    eACTUAL = auto()
    eEXTERNAL = auto()
    eONSTART = auto()
    eNONE = auto()



class ForceModeParameters:
    def __init__(self, frameType = FrameTypeEnum.eNONE, selector = [0]*6, mode = 2, limits = [0]*6) -> None:
        self.selector = selector
        self.mode = mode
        self.limits = limits
        self.frameType = FrameTypeEnum

    

class Door:
    def __init__(self) -> None:
        self.doorHinge = None
        self.doorHandle = None
        self._is_left = True
        self._is_push = False
        self._angle = 0
        self._is_init = False
        self.forceToRotateHandle = ForceModeParameters()
        self.forceToUnblock = ForceModeParameters()

    def Init(self, is_left, is_push, angle = 0):
        self._is_init = True
        self._is_left =  is_left
        self._is_push = is_push
        self.angle = angle
       




    def IsRight(self):
        return not self._is_left
    def IsLeft(self):
        return self._is_left
    def IsPull(self):
        return not self._is_push
    def IsPush(self):
        return self._is_push