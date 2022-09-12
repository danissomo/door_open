#!/usr/bin/python3
#std
import numpy as np
import math
from enum import Enum, auto
import copy

#ros
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray

#custom
from husky import Robot
from param_provider import ParamProvider
from doorHandle import DoorHandle

import time
## TODO
##  1. Продумать данные которые получает нода на вход
##  2. Как по этим данным определить местоположение
##  3. сделать открытие двери наружу
##  4. определение открывается ли дверь наружу или нет
##
##
##
##


from selector_generator import SelectorVec
    
class DoorOpen:
    
    class ResultEnum(Enum):
        SUCCESS = auto()
        NO_OBJECTS_FOUND = auto()
        UNREACHABLE = auto()
        SINGULARITY = auto()
        GRIPER_DETACHES = auto()
        GRIPPER_STUCK = auto()
        EXIT = auto()

    class SolutionProposer:
        def __init__(self, resultEnum, solveFunc = lambda : 0, argsToSolveFunc = None):
            self.resultEnum = resultEnum
            self.solve = solveFunc
            self.args = argsToSolveFunc


    def __init__(self) -> None:
        if ParamProvider.is_roslaunch:
            rospy.init_node("~")
        else: 
            rospy.init_node("door_open")
        rospy.on_shutdown(self.Cancel)
        self.force_limits = [1, 1, 1, math.pi, math.pi, math.pi]
        self.deltaUnblokDRfromSteady = np.array([0, 0, 0.1])

        self._speedScale = 1

        self.robot = Robot(ParamProvider.ur_ip)
        self.pub_start_move = rospy.Publisher(ParamProvider.base_controller_topic, Bool, queue_size=10)
        self.door_handle_handler =  DoorHandle(ParamProvider.yolo_topic, "ur_arm_base")
        
        self.robot.ForceModeStop()
        self.robot.ActivateTeachMode()
        self.robot.OpenGripper()

        self.sub = rospy.Subscriber(ParamProvider.door_hinge_topic, PoseArray, callback=self.YoloIntegration, queue_size=1)

        self.curDoorHinge = None
        self._is_debug = False
        #test
        rospy.loginfo("!!FOR START PRESS ENTER!!")
        input()
        self.OpenDoorPush(None)
        #self.YoloIntegration(None)

    def test(self):
        self.robot.ManualPose()
        self.robot.LookAt([0, - 1, 0])

    def YoloIntegration(self, doorHinge : PoseArray):
        self.curDoorHinge = doorHinge
        self.Init()
        if self._is_debug:
            self.robot.ManualPose()
            self.robot.AlignXZtoXYPlane()
            rospy.sleep()
        else:
            pass # выход в промежуточную точку
        
        if self.FindDoorHandle().resultEnum == DoorOpen.ResultEnum.NO_OBJECTS_FOUND:
            exit()
            

        rs = self.GripHandle()
        if rs.resultEnum == self.ResultEnum.UNREACHABLE:
            rospy.logwarn("UNREACHABLE POINT")
            exit()
        if rs.resultEnum == self.ResultEnum.NO_OBJECTS_FOUND:
            rospy.logwarn("NO OBJECTS")
            exit()
            
        # rotate handle
        gripped_frame = self.robot.GetActualTCPPose()
        self.RotateHandle()
        self.PullDoor()
        self.ReturnOrientation()
        self.PullWithCompensation()

        # область ответственности прерывается, выполняется отъезд
        self.robot.ForceModeStop()
        self.robot.ActivateTeachMode()
        self.pub_start_move.publish(Bool(True))
        if self.robot._rtde_r.getSafetyMode() == 0:  
            input("press")
            self.robot.MoveBaseX(1, -0.2)
        self.WaitController()    
        # требуется завершение в виде отсоединения от двери
        self.DetachFromDoor()
        self.Cancel()

    def OpenDoorPush(self,  doorHinge : PoseArray):
        self.curDoorHinge = doorHinge
        self.Init()
        if self._is_debug:
            self.robot.ManualPose()
            self.robot.AlignXZtoXYPlane()
            rospy.sleep()
        else:
            pass # выход в промежуточную точку
        
        if self.FindDoorHandle().resultEnum == DoorOpen.ResultEnum.NO_OBJECTS_FOUND:
            exit()
            

        rs = self.GripHandle()
        if rs.resultEnum == self.ResultEnum.UNREACHABLE:
            rospy.logwarn("UNREACHABLE POINT")
            exit()
        if rs.resultEnum == self.ResultEnum.NO_OBJECTS_FOUND:
            rospy.logwarn("NO OBJECTS")
            exit()
            
        # rotate handle
        gripped_frame = self.robot.GetActualTCPPose()
        self.RotateHandle()
        self.PushDoor()
        #self.ReturnOrientation()
        self.robot.ForceModeStop()
        self.robot.ActivateTeachMode()
        self.robot.MoveBaseX(1.4, 0.2)
        input()



    def Init(self):
        self.robot.ForceModeStop()
        self.robot.ActivateTeachMode()
        self.robot.OpenGripper()
        self.robot.DeactivateTeachMode()
        self.robot.Fold(1*self._speedScale, 0.5*self._speedScale)
        pass



    def FindDoorHandle(self):
        #position for find
        # change to watch on hinge and rotating in other pose
        self.robot.MoveJ(   self.robot.INITIAL_JOINTS,  
                            self._speedScale * 1, 
                            self._speedScale * 0.5 )

        searching_pose =  copy.copy( self.robot.INITIAL_JOINTS )
        #searching_pose[0] += math.pi / 2
        self.robot.MoveJ( searching_pose, 1, 0.5)
        first_j_pose = self.robot.GetActualQ()[ 0 ]
        self.door_handle_handler.ClearData()
        self.door_handle_handler.WaitData(5)
        self.door_handle_handler.ClearData()
        
        while (not rospy.is_shutdown()) and math.fabs(self.robot.GetActualQ()[0] - searching_pose[0])  <= math.pi/2:
            self.door_handle_handler.ClearData()
            if self.door_handle_handler.WaitData(1):
                break
            searching_pose[0]-=0.17
            self.robot.MoveJ(searching_pose)
        return self.SolutionProposer(self.ResultEnum.SUCCESS)



    def PushDoor(self):
        start_opening_frame = np.matmul(
            self.robot.GetActualRotMatrix(),
            self.robot.GetActualTCPPose()[0:3]
        )
        actual_opening_frame = np.matmul(
            self.robot.GetActualRotMatrix(),
            self.robot.GetActualTCPPose()[0:3]
        )
        start_time = time.time()
        wait_time = 2
        rospy.loginfo(actual_opening_frame - start_opening_frame)

        while (not rospy.is_shutdown()) and \
              (not all(np.fabs(actual_opening_frame - start_opening_frame) >= self.deltaUnblokDRfromSteady)):
            self.robot.ForceMode(   self.robot.GetActualTCPPose(), 
                                    SelectorVec().y().z().get(), 
                                    [0, -150, 250, 0, 0, 0 ], 
                                    2, 
                                    self.force_limits ) 
            actual_opening_frame = np.matmul(
                    self.robot.GetActualRotMatrix(),
                    self.robot.GetActualTCPPose()[0:3]
            )
            if np.linalg.norm(self.robot.GetActualTCPSpeed()[:3]) > 0.02:
                start_time = time.time()
            rospy.sleep(0.1)
            # if time.time() - start_time > wait_time:
            #     return self.SolutionProposer(self.ResultEnum.GRIPPER_STUCK)
        self.robot.ForceModeStop()
        return self.SolutionProposer(self.ResultEnum.SUCCESS)


    def GripHandle(self):
        rospy.sleep(3)
        self.door_handle_handler.ClearData()
        self.door_handle_handler.WaitData(3)
        self.door_handle_handler.StopUpdateHandle()
        handle_basis = self.door_handle_handler.GetActualCoordSystem()
        point_for_grip = self.door_handle_handler.GetActualMiddlePoint()
        self.door_handle_handler.StartUpdateHandle()
        if (handle_basis is not None) and (point_for_grip is not None):
            if np.linalg.norm(point_for_grip) >= self.robot.REACH_RADIUS:
                rospy.loginfo(point_for_grip)
                return self.SolutionProposer(self.ResultEnum.UNREACHABLE, )

            self.robot.MoveL_point_rot( point_for_grip, 
                                        self.robot.RotvecFromBasis(handle_basis), 
                                        1, 
                                        0.4,
                                        True)
            self.robot.ActivateTeachMode()
            self.robot.CloseGripper()
            rospy.sleep(0.1)
            self.robot.DeactivateTeachMode()
            return self.SolutionProposer(self.ResultEnum.SUCCESS)
        else:
            return self.SolutionProposer(self.ResultEnum.NO_OBJECTS_FOUND, exit)
        


    def RotateHandle(self):
        self.robot.PushUntilForce( SelectorVec().x().rz().get(),
                                    [ -350, 0, 0, 0, 0, -50 ], 
                                    self.force_limits )  # need mod for left or right



    def PullDoor(self):
        start_opening_frame = np.matmul(
            self.robot.GetActualRotMatrix(),
            self.robot.GetActualTCPPose()[0:3]
        )
        actual_opening_frame = np.matmul(
            self.robot.GetActualRotMatrix(),
            self.robot.GetActualTCPPose()[0:3]
        )

        while (not rospy.is_shutdown()) and (not all(np.fabs(actual_opening_frame - start_opening_frame) >= self.deltaUnblokDRfromSteady)):
            self.robot.ForceMode(   self.robot.GetActualTCPPose(), 
                                    SelectorVec().x().z().get(), 
                                    [ -150, 0, -150, 0, 0, 0 ], 
                                    2, 
                                    self.force_limits ) 
            actual_opening_frame = np.matmul(
                    self.robot.GetActualRotMatrix(),
                    self.robot.GetActualTCPPose()[0:3]
            )

        self.robot.ForceModeStop()
        return self.SolutionProposer(self.ResultEnum.SUCCESS)

    def PullWithCompensation(self):
        while (not rospy.is_shutdown()) and np.linalg.norm( np.array(self.robot.GetActualTCPPose()[:2]) ) > self.robot.FM_SAFE_RADIUS_REAR:
            rot = self.robot.GetActualRotMatrix()
            f_result = np.matmul(   rot, np.array( self.robot.GetActualTCPForce()[:3]) ) - np.array( [ 0, 0, 100 ] )
            rf = np.matmul( rot, np.array(self.robot.GetActualTCPForce()[3:]) )

            self.robot._rtde_c.forceMode(   self.robot.GetActualTCPPose(),
                                            SelectorVec().y().z().rx().get(), 
                                            [   0, 
                                                -f_result[1], 
                                                -150, 
                                                -8*self.robot.GetActualTCPForce()[3], 
                                                0, 
                                                0 ], 
                                            2, 
                                            self.force_limits )
        return self.SolutionProposer(self.ResultEnum.SUCCESS)

        

    
    def ReturnOrientation(self):
        # align with xy plane, need mod for left or right
        while (not rospy.is_shutdown()) and not self.robot.IfAlignedYZtoXYPlane(0.15):
            self.robot.ForceMode(   self.robot.GetActualTCPPose(), 
                                    SelectorVec().x().z().rz().get(), 
                                    [ 50, 0, 0, 0, 0, 30 ], 
                                    2, 
                                    self.force_limits )

        self.robot.ForceModeStop()
        self.SolutionProposer(self.ResultEnum.SUCCESS)



    def Cancel(self):
        try:
            self.robot.ForceModeStop()
            self.robot.SpeedStop()
            self.robot.ActivateTeachMode()
            self.robot.OpenGripper()
            rospy.sleep(1)
            self.robot.DeactivateTeachMode()
            self.robot.MoveJ(self.robot.INITIAL_JOINTS)
            self.robot.Fold()
            self.robot._rtde_c = None
        except AttributeError as e:
            rospy.logfatal(e)
        


    def DetachFromDoor(self):
        pass



    def WaitController(self):
        input()



   
def main():
    d = DoorOpen()
    rospy.spin()


if __name__ == "__main__":
    main()
