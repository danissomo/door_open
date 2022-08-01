import numpy as np
from scipy.spatial.transform import Rotation
import math

from aruco import Aruco
class Calculator:
    #def T_CameraToManipulator(eefPose, point, l=0, h=0, k= 0):
    def T_CameraToManipulator(eefPose, point, l=0.08414625, h=0.12941748 , k= 0.03284863):
        #print(point)
        camToTCP         = np.array(    [[1, 0, 0,  -k], 
                                         [ 0, 1, 0, -h],
                                         [ 0, 0, 1, -l],
                                         [ 0, 0, 0,  1]  ])
                                         
        transfer_to_base = np.array(    [[1, 0, 0,  eefPose[0]], 
                                         [ 0, 1, 0,  eefPose[1]],
                                         [ 0, 0, 1,  eefPose[2]],
                                         [ 0, 0, 0,           1]  ])

        rotation_to_base = Rotation.from_rotvec(eefPose[3:]).as_matrix()

        homogen_point = np.concatenate([point, [1]])
        point_rel_tcp = np.matmul(camToTCP, homogen_point)[:3]

        point_rel_base = np.matmul(transfer_to_base, np.concatenate([np.matmul(rotation_to_base, point_rel_tcp), [1]])) [:3]
        return point_rel_base[:3]
    

    


    def C_CameraNotmalToRotvec(eefPose, normal):
        #normal[0] w/o *-1 bcse cam has x axis reversed rel to manipulator
        rotation_to_base = Rotation.from_rotvec(eefPose[3:]).as_matrix()
        strange_rotation = Rotation.from_euler("xyz", [math.atan2( -normal[1], -normal[2]), 
                                                       -math.atan2( normal[0], -normal[2]), math.pi/2]).as_matrix()
        strange_rotation = np.matmul(rotation_to_base, np.linalg.inv(strange_rotation))
        m = strange_rotation.copy()
        strange_rotation = Rotation.from_matrix(strange_rotation).as_rotvec()
        return strange_rotation

    
    def R_CmaeraNormalToBase(eefPose, normal):
        rotation_to_base = Rotation.from_rotvec(eefPose[3:]).as_matrix()
        
        
        n = np.matmul( np.linalg.inv(rotation_to_base), normal)
        n/=np.linalg.norm(n)
        normal[0]*=-1
        return n 

    def RT_ArucoRelCameraToBase(arRelCamera : Aruco, eefPose):
        arRelBase               = Aruco()
        arRelBase.center        = Calculator.T_CameraToManipulator(eefPose, arRelCamera.center)
        arRelBase.main_corner   = Calculator.T_CameraToManipulator(eefPose, arRelCamera.main_corner)
        arRelBase.corners       = [Calculator.T_CameraToManipulator(eefPose, corner) for corner in arRelCamera.corners]
        arRelBase.normal        = Calculator.R_CmaeraNormalToBase(eefPose, arRelCamera.normal)
        
        arRelBase.InitTransformationMatrices()
        return arRelBase
            
