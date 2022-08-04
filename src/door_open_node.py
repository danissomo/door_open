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


class SelectorVec:
    def __init__(self) -> None:
        self.vec = [0] * 6
    def x(self):
        self.vec[0] = 1
        return self
    def y(self):
        self.vec[1] = 1
        return self
    def z(self):
        self.vec[2] = 1
        return self
    def rx(self):
        self.vec[3] = 1
        return self
    def ry(self):
        self.vec[4] = 1
        return self
    def rz(self):
        self.vec[5] = 1
        return self
    def get(self):
        return self.vec
    
class DoorOpen:
    
    class ResultEnum(Enum):
        SUCCESS = auto()
        NO_OBJECTS_FOUND = auto()
        UNREACHABLE = auto()
        SINGULARITY = auto()
        GRIPER_DETACHES = auto()
        GRIPPER_STUCK = auto()

    def __init__(self) -> None:
        rospy.init_node("~open")
        rospy.on_shutdown(self.Cancel)
        self.force_limits = [0.5, 0.5, 0.5, math.pi/2, math.pi/2, math.pi/2]
        self.deltaUnblokDRfromSteady = np.array([0, 0, 0.03])

        self._speedScale = 1

        self.robot = Robot(ParamProvider.ur_ip)
        self.pub_start_move = rospy.Publisher(ParamProvider.base_controller_topic, Bool, queue_size=10)
        self.door_handle_handler =  DoorHandle(ParamProvider.yolo_topic, "ur_arm_base")
        
        self.robot.ForceModeStop()
        self.robot.ActivateTeachMode()
        self.robot.OpenGripper()

        self.sub = rospy.Subscriber(ParamProvider.door_hinge_topic, PoseArray, callback=self.YoloIntegration, queue_size=1)

        self.curDoorHinge = None
        self._is_debug = True
        #test
        input("Press enter")
        self.YoloIntegration(None)


    def YoloIntegration(self, doorHinge : PoseArray):
        self.curDoorHinge = doorHinge
        self.Init()
        if self._is_debug:
            self.robot.ManualPose()
            self.robot.AlignXZtoXYPlane()
            rospy.sleep()
        else:
            pass # выход в промежуточную точку
        
        if self.FindDoorHandle() == DoorOpen.ResultEnum.NO_OBJECTS_FOUND:
            self.Cancel()
            return

        rs = self.GripHandle()
        if rs == self.ResultEnum.UNREACHABLE:
            rospy.logwarn("UNREACHABLE POINT")
            self.Cancel()
            return
        if rs == self.ResultEnum.NO_OBJECTS_FOUND:
            rospy.logwarn("NO OBJECTS")
            self.Cancel()
            return
        # rotate handle
        gripped_frame = self.robot.GetActualTCPPose()
        self.RotateHandle()

        # try open, need mod for left or right
        self.PullDoor()

        # align with xy plane, need mod for left or right
        while (not rospy.is_shutdown()) and not self.robot.IfAlignedYZtoXYPlane(0.15):
            self.robot.ForceMode(   self.robot.GetActualTCPPose(), 
                                    SelectorVec().x().z().rz().get(), 
                                    [ 50, 0, 0, 0, 0, 30 ], 
                                    2, 
                                    self.force_limits )

        self.robot.ForceModeStop()

        #open in place, need mod for left or right
        while (not rospy.is_shutdown()) and np.linalg.norm( np.array(self.robot.GetActualTCPPose()[:2]) ) > 0.5:
            rot = self.robot.GetActualRotMatrix()
            f_result = np.matmul(   rot, 
                                    np.array( self.robot.GetActualTCPForce()[:3]) ) - np.array( [ 0, 0, 100 ] )
            rf = np.matmul( rot, 
                            np.array(self.robot.GetActualTCPForce()[3:]) )

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
        

        # область ответственности прерывается, выполняется отъезд
        self.robot.ForceModeStop()
        self.robot.ActivateTeachMode()
        self.pub_start_move.publish(Bool(True))  
        self.WaitController()    
        # требуется завершение в виде отсоединения от двери
        self.DetachFromDoor()
        self.Cancel()


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
        searching_pose[0] += math.pi / 2
        self.robot.MoveJ( searching_pose, 1, 0.5)
        first_j_pose = self.robot.GetActualQ()[0]
        self.door_handle_handler.ClearData()
        self.robot.SpeedJ([ -0.5, 0, 0, 0, 0, 0 ], 0.01)
        handle_was = False
        count_trys = 0
        while (not rospy.is_shutdown()) and math.fabs(self.robot.GetActualQ()[0] - searching_pose[0])  <= math.pi:
            self.door_handle_handler.ClearData()
            rs = self.door_handle_handler.WaitData(1.5)
            if handle_was and not rs:
                count_trys += 1
            if count_trys > 3:
                break
            if (not handle_was) and rs:
                handle_was = True

        self.robot.SpeedStop()
        if not handle_was:
            return self.ResultEnum.NO_OBJECTS_FOUND
        handle_midle_angle_pose = [(first_j_pose + self.robot.GetActualQ()[0])/2] + searching_pose[1:]
        self.robot.MoveJ(   handle_midle_angle_pose, 
                            self._speedScale * 0.1, 
                            self._speedScale * 0.1 )       
        return self.ResultEnum.SUCCESS



    def GripHandle(self):
        self.door_handle_handler.WaitData()
        self.door_handle_handler.StopUpdateHandle()
        handle_basis = self.door_handle_handler.GetActualCoordSystem()
        point_for_grip = self.door_handle_handler.GetActualMiddlePoint()
        self.door_handle.StartUpdateHandle()
        if (handle_basis is not None) and (point_for_grip is not None):
            if np.linalg.norm(point_for_grip) >= self.robot.REACH_RADIUS:
                return self.ResultEnum.UNREACHABLE

            self.robot.MoveL_point_rot( point_for_grip, 
                                        self.robot.RotvecFromBasis(handle_basis), 
                                        0.5, 
                                        0.1)
            self.robot.ActivateTeachMode()
            self.robot.CloseGripper()
            rospy.sleep(0.1)
            self.robot.DeactivateTeachMode()
            return self.ResultEnum.SUCCESS
        else:
            return self.ResultEnum.NO_OBJECTS_FOUND
        



    def RotateHandle(self):
        self.robot.PushUntilForce( SelectorVec().x().rz().get(),
                                    [ -150, 0, 0, 0, 0, -50 ], 
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
                                    SelectorVec().x().z().rz().get(), 
                                    [ -150, 0, -150, 0, 0, -50 ], 
                                    2, 
                                    self.force_limits ) 
            actual_opening_frame = np.matmul(
                    self.robot.GetActualRotMatrix(),
                    self.robot.GetActualTCPPose()[0:3]
            )

        self.robot.ForceModeStop()
        pass



    def Cancel(self):
        self.robot.ForceModeStop()
        self.robot.SpeedStop()
        self.robot.ActivateTeachMode()
        self.robot.OpenGripper()
        



    def DetachFromDoor(self):
        pass



    def WaitController(self):
        pass



def main():
    d = DoorOpen()
    rospy.spin()


if __name__ == "__main__":
    main()
