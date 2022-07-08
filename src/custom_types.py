from aruco import Aruco
import numpy as np
class ArucoRelManipulatorNode:
    def __init__(self):
        self.aruco : Aruco = None
        self.urPose : np.ndarray = np.zeros(6)
        self.time = 0
        self.cameraToTCPTransformM : np.ndarray = np.zeros((4,4))
        self.cameraToTCPRotationM : np.ndarray = np.zeros((3,3))
        self.rotvecAlignedNormal : np.ndarray = np.zeros(3)