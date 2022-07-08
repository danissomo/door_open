from husky import PositionHystrory
from aruco import ArucoTimeStampedContainer, Aruco
from calculation import Calculator
import numpy as np
from custom_types import ArucoRelManipulatorNode


class StampedArucoRelManipulator:
    def Transform(arucoTimeStamped : ArucoTimeStampedContainer, poseHyst : PositionHystrory):
        cont = arucoTimeStamped.GetBuffer()
        buf = []
        for data in cont:
            pose = poseHyst.FindPoseByMinTimeDiff(data["time"])
            arucoRelCamera: Aruco = data["aruco"]
            if(abs(arucoRelCamera.normal[0]) > 0.8 or abs(arucoRelCamera.normal[1]) > 0.8):
                continue
            arucoRelBase = Calculator.RT_ArucoRelCameraToBase(arucoRelCamera, pose)
            rtVecAligned = Calculator.C_CameraNotmalToRotvec(pose, arucoRelCamera.normal)
            ndAr = ArucoRelManipulatorNode()
            ndAr.aruco = arucoRelBase
            ndAr.urPose =pose.copy()
            ndAr.time = data["time"]
            ndAr.rotvecAlignedNormal = rtVecAligned.copy()
            buf.append(ndAr)
            # print("aligned ", rtVecAligned)
            # print("normal ", arucoRelCamera.normal)

        return buf


    def TransformAndGetByIndex(arucoTimeStamped : ArucoTimeStampedContainer, poseHyst : PositionHystrory, index):
        arContainerRelBase = StampedArucoRelManipulator.Transform(arucoTimeStamped, poseHyst)
        if(len(arContainerRelBase) == 0):
            return None
        ar : ArucoRelManipulatorNode =  arContainerRelBase[index]
        arucoTimeStamped.Clean()
        return ar
