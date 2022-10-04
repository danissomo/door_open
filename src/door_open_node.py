#!/usr/bin/python3
#std
import numpy as np
import math
from enum import Enum, auto
import copy


#ros
import rospy
from std_msgs.msg import Bool


#custom
from husky import Robot
from utils import ParamProvider
from door_handle import DoorHandleHandler
from door import DoorContainer, DoorContext

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
            rospy.init_node("door_open", log_level=rospy.DEBUG)
        rospy.on_shutdown(self.Cancel)
        #new feature
        self.door_ctx = DoorContext()
        self.doors_stash = DoorContainer()

        self.force_limits = [*[1]*3, *[math.pi]*3]
        self.dist_of_unlock_door = np.array([0, 0, 0.05])

        self._speed_scale = 1

        self.robot = Robot(ParamProvider.ur_ip)
        self.pub_start_opening = rospy.Publisher(ParamProvider.start_opening, Bool, queue_size=10)
        self.door_handle_handler =  DoorHandleHandler(ParamProvider.yolo_topic, "ur_arm_base")
        
        self.robot.ForceModeStop()
        self.robot.ActivateTeachMode()
        self.robot.OpenGripper()

        self.sub_start_manipulator = rospy.Subscriber(ParamProvider.start_manipulator_topic, Bool, callback=self.NewPipeline, queue_size=10)
        self.pub_start_passing = rospy.Publisher(ParamProvider.sart_passing_topic, Bool, queue_size=10)
        self._is_debug = False
        # while not rospy.is_shutdown():
        #     rospy.loginfo("!!FOR START PRESS ENTER!!")
        #     input()
        #     self.NewPipeline()


    def Init(self):
        self.robot.ForceModeStop()
        self.robot.ActivateTeachMode()
        self.robot.OpenGripper()
        self.robot.DeactivateTeachMode()
        self.robot.Fold(1*self._speed_scale, 0.5*self._speed_scale)
        pass

    

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



    def Dialog(self):
        # inp = [ input("Is left? y/n: "), 
        #         input("Is push? y/n: ") ]
        inp = ['n', 'n']
        self.door_ctx.is_left = inp[0] == 'y' or inp[0] == 'Y'
        self.door_ctx.is_push = inp[1] == 'y' or inp[1] == 'Y'
        rs = self.FindDoorHandle()
        if rs.resultEnum == self.ResultEnum.SUCCESS:
            self.door_handle_handler.ClearData()
            self.door_handle_handler.WaitData(10)
            self.door_ctx.door.door_handle =  self.door_handle_handler.GetActualDoorHandle()
            if self.door_ctx.door.door_handle.coordinate_system_global is not None:
                self.door_ctx.door_normal = - self.door_ctx.door.door_handle.coordinate_system_global[2]
                #debug
                rospy.logdebug(self.door_ctx.IsPositionPositive(self.robot.GetBasePosition().position))
                rospy.logdebug(self.door_ctx.door_normal)
                rospy.logdebug(self.door_ctx.ToStr())
                assert self.door_ctx.IsPositionPositive(self.robot.GetBasePosition().position) == True
                self.doors_stash.AddIfNotKnown(self.door_ctx)
                return self.SolutionProposer(self.ResultEnum.SUCCESS)
            else:
                return self.SolutionProposer(self.ResultEnum.NO_OBJECTS_FOUND)
        else:
            return rs
        


    def NewPipeline(self, args = None):
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
        nearest = self.doors_stash.GetNearestCtx(robot_pos.position, 1.0)
        if nearest is None:
            rs = self.Dialog()
            if rs.resultEnum == self.ResultEnum.NO_OBJECTS_FOUND:
                rospy.logwarn("NO OBJECTS")
                return rs
        else:
            self.FindDoorHandle()
            rospy.sleep(2)
            handle = self.door_handle_handler.GetActualDoorHandle()
            self.door_ctx = self.doors_stash.GetNearestCtx(handle.GetMiddlePointGlob())
            self.door_ctx.door.door_handle = handle
            self.doors_stash.AddIfNotKnown(self.door_ctx)
        if not self.door_ctx.IsPositionPositive(self.robot.GetBasePosition().position):
            rospy.logwarn("POSITION NEGATIVE")
            self.door_ctx = self.door_ctx.Negate()
        rospy.logdebug(self.door_ctx.ToStr())
        self.GripHandle()
        self.RotateHandle()
        self.PreOpenDoor()
        self.robot.ForceModeStop()
        self.robot.ActivateTeachMode()
        if self.door_ctx.IsPush():
            self.pub_start_opening.publish(Bool(True))
            self.WaitController()
            self.DetachFromDoor()
            self.Cancel()
        else:
            #self.ReturnOrientation()
            #self.PullWithCompensation()
            self.pub_start_opening.publish(Bool(True))
            self.WaitController()    
            self.DetachFromDoor()
            self.pub_start_passing.publish(Bool(True))
        return self.SolutionProposer(self.ResultEnum.SUCCESS)
        


    def PreOpenDoor(self):
        if self.door_ctx.IsPull():
            force_vec = [0, -150, -150, *[0]*3 ]
        else: 
            force_vec = [0, -150,  250, *[0]*3 ]

        start_opening_frame = self.robot.GetActualRotMatrix() @ self.robot.GetActualTCPPose()[0:3]
        
        actual_opening_frame = self.robot.GetActualRotMatrix() @ self.robot.GetActualTCPPose()[0:3]
        

        while (not rospy.is_shutdown()) and (not all(np.fabs(actual_opening_frame - start_opening_frame) >= self.dist_of_unlock_door)):
            self.robot.ForceMode(self.robot.GetActualTCPPose(), SelectorVec().y().z().get(), force_vec, 2, self.force_limits ) 
            actual_opening_frame = self.robot.GetActualRotMatrix() @ self.robot.GetActualTCPPose()[0:3]
            
        self.robot.ForceModeStop()
        return self.SolutionProposer(self.ResultEnum.SUCCESS)



    def FindDoorHandle(self):
        '''
        Rotating base joint until hansdle will be detected.
        Start pose look to front.
        End - 90 degree from start position
        '''
        self.robot.MoveJ(   self.robot.INITIAL_JOINTS,  
                            self._speed_scale * 1, 
                            self._speed_scale * 0.5 )

        searching_pose =  copy.copy( self.robot.INITIAL_JOINTS )
        self.robot.MoveJ( searching_pose, 1, 0.5)
        self.door_handle_handler.ClearData()
        self.door_handle_handler.WaitData(2)
        self.door_handle_handler.ClearData()
        rospy.sleep(3)
        delta_q = 0.25 if self.door_ctx.IsLeft() else -0.25
        # manipulator left +
        #manipulator right -
        while (not rospy.is_shutdown()) and math.fabs(self.robot.GetActualQ()[0] - self.robot.INITIAL_JOINTS[0])  <= math.pi/2:
            self.door_handle_handler.ClearData()
            if self.door_handle_handler.WaitData(3):
                 return self.SolutionProposer(self.ResultEnum.SUCCESS)
            searching_pose[0] += delta_q
            self.robot.MoveJ(searching_pose)
        searching_pose[0] += delta_q
        self.robot.MoveJ(searching_pose)
        rospy.sleep(5) 
        return self.SolutionProposer(self.ResultEnum.NO_OBJECTS_FOUND)



    def GripHandle(self):
        door_h = self.door_ctx.door.door_handle
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
            f = [ 350, *[0]*4, -50 ]
        else:
            f = [ 350, *[0]*4, 50 ]
        self.robot.PushUntilForce( SelectorVec().x().rz().get(), f, self.force_limits )



    def PullWithCompensation(self):
        while (not rospy.is_shutdown()) and np.linalg.norm( np.array(self.robot.GetActualTCPPose()[:2]) ) > self.robot.FM_SAFE_RADIUS_REAR:
            rot = self.robot.GetActualRotMatrix()
            f_result = (rot @ np.array( self.robot.GetActualTCPForce()[:3]) ) - [ 0, 0, 100 ] 
            rf = rot @ np.array(self.robot.GetActualTCPForce()[3:]) 
            force = [0, -f_result[1], -150, -8*self.robot.GetActualTCPForce()[3], 0, 0 ]
            self.robot._rtde_c.forceMode(self.robot.GetActualTCPPose(), SelectorVec().y().z().rx().get(), force, 2, self.force_limits )
        return self.SolutionProposer(self.ResultEnum.SUCCESS)

        

    def ReturnOrientation(self):
        # align with xy plane, need mod for left or right
        while (not rospy.is_shutdown()) and not self.robot.IfAlignedYZtoXYPlane(0.15):
            self.robot.ForceMode( self.robot.GetActualTCPPose(), SelectorVec().x().z().rz().get(), [50, *[0]*4, 30], 2, self.force_limits )

        self.robot.ForceModeStop()
        return self.SolutionProposer(self.ResultEnum.SUCCESS)

        

    def DetachFromDoor(self):
        self.robot.ActivateTeachMode()
        self.robot.OpenGripper()
        self.robot.Fold()



    def WaitController(self):
        rospy.wait_for_message(ParamProvider.finish_manipulator, Bool, rospy.Duration(1000))




def main():
    d = DoorOpen()
    rospy.spin()


if __name__ == "__main__":
    main()
