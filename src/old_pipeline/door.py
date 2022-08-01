from scipy.spatial.transform import Rotation
import numpy as np
from aruco import Aruco

class Door:
    def __init__(self, arucoBasis : Aruco) -> None:
        self.aruco = arucoBasis
        self.handlePosition = [-0.06406069,  0.00545401,  0.06892106,  1.        ]
