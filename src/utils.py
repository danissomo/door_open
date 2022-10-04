from geometry_msgs.msg import PointStamped
import numpy as np

def PointStampedToNumpy(p : PointStamped):
        return np.array([p.point.x, p.point.y, p.point.z])


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



import rospy
class ParamProvider:
    is_roslaunch            : bool = rospy.get_param("~is_roslaunch", False)
    odom_topic              : str = rospy.get_param("~odom_topic", "/husky_velocity_controller/odom")
    vel_topic               : str = rospy.get_param("~vel_topic", "/husky_velocity_controller/cmd_vel")
    yolo_topic              : str = rospy.get_param("~yolo_topic", "/handle/points_yolo")
    start_opening           : str = rospy.get_param("~base_controller_topic", "/start_opening")
    start_manipulator_topic : str = rospy.get_param("~start_manipulator_topic", "/start_manipulator")
    finish_manipulator      : str = rospy.get_param("~finish_manipulator_topic", "/finish_manipulator")
    sart_passing_topic      : str = rospy.get_param("~start_passing_topic", "/start_passing")
    ur_ip                   : str = rospy.get_param("~ur_ip", "192.168.131.40")
    rs_frame_name           : str = rospy.get_param("~rs_frame_name", "rs_camera")
    rs_frame                : tuple = (
                                rospy.get_param("~rs_frame_x", -0.03284863),
                                rospy.get_param("~rs_frame_y", -0.08),
                                rospy.get_param("~rs_frame_z", -0.08414625)
                            )
    door_hinge_topic        : str = rospy.get_param("~door_hinge_topic", "door_hinge")
