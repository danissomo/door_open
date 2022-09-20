#!/usr/bin/python3
#std
import numpy as np
import math
from enum import Enum, auto
import copy

from sympy import true

#ros
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray

#custom
from husky import Robot
from utils import ParamProvider
from doorHandle import DoorHandleHandler
from door import DoorContainer, DoorContext, Door

import time
'''
TODO:
1. Add possibility for using door context
2. Saving and loading door from container
3. Find new door and ask. 
'''


from utils import SelectorVec
    
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
        #new feature
        self.door_ctx = DoorContext()
        self.doors_stash = DoorContainer()

        self.force_limits = [1, 1, 1, math.pi, math.pi, math.pi]
        self.deltaUnblokDRfromSteady = np.array([0, 0, 0.1])

        self._speedScale = 1

        self.robot = Robot(ParamProvider.ur_ip)
        self.pub_start_move = rospy.Publisher(ParamProvider.base_controller_topic, Bool, queue_size=10)
        self.door_handle_handler =  DoorHandleHandler(ParamProvider.yolo_topic, "ur_arm_base")
        
        self.robot.ForceModeStop()
        self.robot.ActivateTeachMode()
        self.robot.OpenGripper()

        self.sub = rospy.Subscriber(ParamProvider.door_hinge_topic, PoseArray, callback=self.YoloIntegration, queue_size=1)

        self.curDoorHinge = None
        self._is_debug = False
        #test
        while not rospy.is_shutdown():
            rospy.loginfo("!!FOR START PRESS ENTER!!")
            input()
            self.NewPipeline()
        #self.OpenDoorPush(None)
        #self.YoloIntegration(None)

    def Dialog(self):
        inp = [ input("Is left? y/n"), 
                input("Is push? y/n") ]
        self.door_ctx.is_left = inp[0] == 'y' or inp[0] == 'Y'
        self.door_ctx.is_push = inp[1] == 'y' or inp[1] == 'Y'
        rs = self.FindDoorHandle()
        if rs.resultEnum == self.ResultEnum.SUCCESS:
            self.door_handle_handler.ClearData()
            self.door_handle_handler.WaitData(3)
            self.door_ctx.door.door_handle =  self.door_handle_handler.GetActualDoorHandle()
            if self.door_ctx.door.door_handle.coordinate_system_global is not None:
                self.door_ctx.door_normal = -self.door_ctx.door.door_handle.coordinate_system_global[2]
                #debug
                rospy.logdebug(self.door_ctx.IsPositionPositive(self.robot.GetBasePosition()))
                assert self.door_ctx.IsPositionPositive(self.robot.GetBasePosition()) == True
                self.doors_stash.AddIfNotKnown(self.door_ctx)
                return self.SolutionProposer(self.ResultEnum.SUCCESS)
        else:
            return rs
        


    def NewPipeline(self):
        '''
        New Edition of pipeline with context and others
        '''
        self.Init()
        if self._is_debug:
            self.robot.ManualPose()
            self.robot.AlignXZtoXYPlane()
            rospy.sleep()
        else:
            pass # выход в промежуточную точку
        robot_pos = self.robot.GetBasePosition()
        nearest = self.doors_stash.GetNearestCtx(robot_pos, 1.0)
        if nearest is None:
            rs = self.Dialog()
            if rs.resultEnum == self.ResultEnum.NO_OBJECTS_FOUND:
                rospy.logwarn("NO OBJECTS")
                return rs
        else:
            self.FindDoorHandle()
            rospy.sleep(2)
            handle = self.door_handle_handler.GetActualDoorHandle()
            self.door_ctx.door.door_handle = handle
            self.doors_stash.AddIfNotKnown(self.door_ctx)
        if not self.door_ctx.IsPositionPositive(self.robot.GetBasePosition()):
            self.door_ctx = self.door_ctx.Negate()
        
        self.GripHandle()
        self.RotateHandle()
        self.PreOpenDoor()
        if self.door_ctx.IsPush():
            self.robot.ForceModeStop()
            self.robot.ActivateTeachMode()
            self.robot.MoveBaseX(1.4, 0.2)
            self.DetachFromDoor()
            self.Cancel()
        else:
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
        return self.SolutionProposer(self.ResultEnum.SUCCESS)
        

    def test(self):
        self.robot.ManualPose()
        self.robot.LookAt([0, - 1, 0])

    


    def PreOpenDoor(self):
        if self.door_ctx.IsPull():
            force_vec = [0, -150, -150, 0, 0, 0 ]
        else: 
            force_vec = [0, -150, 250, 0, 0, 0 ]

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
                                    SelectorVec().y().z().get(), 
                                    force_vec, 
                                    2, 
                                    self.force_limits ) 
            actual_opening_frame = np.matmul(
                    self.robot.GetActualRotMatrix(),
                    self.robot.GetActualTCPPose()[0:3]
            )

        self.robot.ForceModeStop()
        return self.SolutionProposer(self.ResultEnum.SUCCESS)
        


    # def YoloIntegration(self, doorHinge : PoseArray):
    #     '''
    #     door pull left 
    #     '''
    #     self.curDoorHinge = doorHinge
    #     self.Init()
    #     if self._is_debug:
    #         self.robot.ManualPose()
    #         self.robot.AlignXZtoXYPlane()
    #         rospy.sleep()
    #     else:
    #         pass # выход в промежуточную точку
        
    #     if self.FindDoorHandle().resultEnum == DoorOpen.ResultEnum.NO_OBJECTS_FOUND:
    #         exit()
            

    #     rs = self.GripHandle()
    #     if rs.resultEnum == self.ResultEnum.UNREACHABLE:
    #         rospy.logwarn("UNREACHABLE POINT")
    #         exit()
    #     if rs.resultEnum == self.ResultEnum.NO_OBJECTS_FOUND:
    #         rospy.logwarn("NO OBJECTS")
    #         exit()
            
    #     # rotate handle
    #     gripped_frame = self.robot.GetActualTCPPose()
    #     self.RotateHandle()
    #     self.PullDoor()
    #     self.ReturnOrientation()
    #     self.PullWithCompensation()

    #     # область ответственности прерывается, выполняется отъезд
    #     self.robot.ForceModeStop()
    #     self.robot.ActivateTeachMode()
    #     self.pub_start_move.publish(Bool(True))
    #     if self.robot._rtde_r.getSafetyMode() == 0:  
    #         input("press")
    #         self.robot.MoveBaseX(1, -0.2)
    #     self.WaitController()    
    #     # требуется завершение в виде отсоединения от двери
    #     self.DetachFromDoor()
    #     self.Cancel()



    # def OpenDoorPush(self,  doorHinge : PoseArray):
    #     self.curDoorHinge = doorHinge
    #     self.Init()
    #     if self._is_debug:
    #         self.robot.ManualPose()
    #         self.robot.AlignXZtoXYPlane()
    #         rospy.sleep()
    #     else:
    #         pass # выход в промежуточную точку
        
    #     if self.FindDoorHandle().resultEnum == DoorOpen.ResultEnum.NO_OBJECTS_FOUND:
    #         exit()
            

    #     rs = self.GripHandle()
    #     if rs.resultEnum == self.ResultEnum.UNREACHABLE:
    #         rospy.logwarn("UNREACHABLE POINT")
    #         exit()
    #     if rs.resultEnum == self.ResultEnum.NO_OBJECTS_FOUND:
    #         rospy.logwarn("NO OBJECTS")
    #         exit()
            
    #     # rotate handle
    #     gripped_frame = self.robot.GetActualTCPPose()
    #     self.RotateHandle()
    #     self.PushDoor()
    #     #self.ReturnOrientation()
    #     self.robot.ForceModeStop()
    #     self.robot.ActivateTeachMode()
    #     self.robot.MoveBaseX(1.4, 0.2)
    #     input()



    def Init(self):
        self.robot.ForceModeStop()
        self.robot.ActivateTeachMode()
        self.robot.OpenGripper()
        self.robot.DeactivateTeachMode()
        self.robot.Fold(1*self._speedScale, 0.5*self._speedScale)
        pass



    def FindDoorHandle(self):
        '''
        Rotating base joint until hansdle will be detected.
        Start pose look to front.
        End - 90 degree from start position
        '''
        self.robot.MoveJ(   self.robot.INITIAL_JOINTS,  
                            self._speedScale * 1, 
                            self._speedScale * 0.5 )

        searching_pose =  copy.copy( self.robot.INITIAL_JOINTS )
        self.robot.MoveJ( searching_pose, 1, 0.5)
        self.door_handle_handler.ClearData()
        self.door_handle_handler.WaitData(2)
        self.door_handle_handler.ClearData()
        delta_q = -0.17 if self.door_ctx.IsLeft() else 0.17
        # manipulator left +
        #manipulator right -
        while (not rospy.is_shutdown()) and math.fabs(self.robot.GetActualQ()[0] - searching_pose[0])  <= math.pi/2:
            self.door_handle_handler.ClearData()
            if self.door_handle_handler.WaitData(1):
                 return self.SolutionProposer(self.ResultEnum.SUCCESS)
            searching_pose[0] += delta_q
            self.robot.MoveJ(searching_pose)
        return self.SolutionProposer(self.ResultEnum.NO_OBJECTS_FOUND)



    # def PushDoor(self):
    #     start_opening_frame = np.matmul(
    #         self.robot.GetActualRotMatrix(),
    #         self.robot.GetActualTCPPose()[0:3]
    #     )
    #     actual_opening_frame = np.matmul(
    #         self.robot.GetActualRotMatrix(),
    #         self.robot.GetActualTCPPose()[0:3]
    #     )
    #     start_time = time.time()
    #     wait_time = 2
    #     rospy.loginfo(actual_opening_frame - start_opening_frame)

    #     while (not rospy.is_shutdown()) and \
    #           (not all(np.fabs(actual_opening_frame - start_opening_frame) >= self.deltaUnblokDRfromSteady)):
    #         self.robot.ForceMode(   self.robot.GetActualTCPPose(), 
    #                                 SelectorVec().y().z().get(), 
    #                                 [0, -150, 250, 0, 0, 0 ], 
    #                                 2, 
    #                                 self.force_limits ) 
    #         actual_opening_frame = np.matmul(
    #                 self.robot.GetActualRotMatrix(),
    #                 self.robot.GetActualTCPPose()[0:3]
    #         )
    #         if np.linalg.norm(self.robot.GetActualTCPSpeed()[:3]) > 0.02:
    #             start_time = time.time()
    #         rospy.sleep(0.1)
    #         # if time.time() - start_time > wait_time:
    #         #     return self.SolutionProposer(self.ResultEnum.GRIPPER_STUCK)
    #     self.robot.ForceModeStop()
    #     return self.SolutionProposer(self.ResultEnum.SUCCESS)

    # def PullDoor(self):
    #         start_opening_frame = np.matmul(
    #             self.robot.GetActualRotMatrix(),
    #             self.robot.GetActualTCPPose()[0:3]
    #         )
    #         actual_opening_frame = np.matmul(
    #             self.robot.GetActualRotMatrix(),
    #             self.robot.GetActualTCPPose()[0:3]
    #         )

    #         while (not rospy.is_shutdown()) and (not all(np.fabs(actual_opening_frame - start_opening_frame) >= self.deltaUnblokDRfromSteady)):
    #             self.robot.ForceMode(   self.robot.GetActualTCPPose(), 
    #                                     SelectorVec().x().z().get(), 
    #                                     [ -150, 0, -150, 0, 0, 0 ], 
    #                                     2, 
    #                                     self.force_limits ) 
    #             actual_opening_frame = np.matmul(
    #                     self.robot.GetActualRotMatrix(),
    #                     self.robot.GetActualTCPPose()[0:3]
    #             )

    #         self.robot.ForceModeStop()
    #         return self.SolutionProposer(self.ResultEnum.SUCCESS)

    def GripHandle(self):
        rospy.sleep(3)
        self.door_handle_handler.ClearData()
        self.door_handle_handler.WaitData(3)
        self.door_handle_handler.StopUpdateHandle()
        door_h = self.door_handle_handler.GetActualDoorHandle()
        self.door_handle_handler.StartUpdateHandle()
        if door_h is not None:
            point_for_grip = door_h.GetMiddlePointRel()
            if np.linalg.norm(point_for_grip) >= self.robot.REACH_RADIUS:
                rospy.loginfo(point_for_grip)
                return self.SolutionProposer(self.ResultEnum.UNREACHABLE, )

            self.robot.MoveL_point_rot( point_for_grip, 
                                        self.robot.RotvecFromBasis(door_h.coordinate_system_rel), 
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
        if self.door_ctx.IsLeft():
            f = [ -350, 0, 0, 0, 0, -50 ]
        else:
            f = [ -350, 0, 0, 0, 0, 50 ]
        self.robot.PushUntilForce( SelectorVec().x().rz().get(), f, self.force_limits )



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
