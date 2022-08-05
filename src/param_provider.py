import rospy
class ParamProvider:
    is_roslaunch            : bool = rospy.get_param("~is_roslaunch", False)
    odom_topic              : str = rospy.get_param("~odom_topic", "/husky_velocity_controller/odom")
    vel_topic               : str = rospy.get_param("~vel_topic", "/husky_velocity_controller/cmd_vel")
    yolo_topic              : str = rospy.get_param("~yolo_topic", "/handle/points_yolo")
    base_controller_topic   : str = rospy.get_param("~base_controller_topic", "/some_topic")
    ur_ip                   : str = rospy.get_param("~ur_ip", "192.168.131.40")
    rs_frame_name           : str = rospy.get_param("~rs_frame_name", "rs_camera")
    rs_frame                : tuple = (
                                rospy.get_param("~rs_frame_x", -0.03284863),
                                rospy.get_param("~rs_frame_y", -0.08),
                                rospy.get_param("~rs_frame_z", -0.08414625)
                            )
    door_hinge_topic        : str = rospy.get_param("~door_hinge_topic", "door_hinge")
