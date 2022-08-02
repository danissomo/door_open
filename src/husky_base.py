import rospy
import copy
from param_provider import ParamProvider
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
class HuskyBase:
    def __init__(self) -> None:
        self._velocityTopicName = ParamProvider.vel_topic
        self._velocityTopic = rospy.Publisher(
            self._velocityTopicName, 
            Twist, 
            queue_size=1)

        self._odomTopicName = ParamProvider.odom_topic
        self._odomSub = rospy.Subscriber(
            self._odomTopicName, Odometry, 
            callback=self.OdomCallback, 
            queue_size=1)
            
        self._baseActualPosition = None
        self._baseActualTwist = None
        self._targetTwist = Twist()



    def GetBaseTwist(self):
        return copy.copy(self._baseActualTwist)

    def GetBasePosition(self):
        return copy.copy(self._baseActualPosition)


    def SetBaseSpeed(self, cmd):
        t = TwistStamped()
        t.twist.linear.x = cmd[0]
        t.twist.linear.y = cmd[1]
        t.twist.linear.z = cmd[2]
        t.twist.angular.x = cmd[3]
        t.twist.angular.y = cmd[4]
        t.twist.angular.z = cmd[5]
        t.header.stamp = rospy.get_rostime()
        self._velocityTopic.publish(t)

    def BaseSpeedCallBack(self, args):
        t = copy.copy(self._targetTwist)
        self._velocityTopic.publish(t)

    def MoveBaseX(self, meters, speed, checkRate_ms=50):
        self._targetTwist.linear.x = speed
        timer = rospy.Timer(rospy.Duration(
            checkRate_ms / 1000), self.BaseSpeedCallBack)

        startPosition = np.array([self.GetBasePosition().position.x,
                                  self.GetBasePosition().position.y,
                                  self.GetBasePosition().position.z])

        actualPosition = np.array([self.GetBasePosition().position.x,
                                   self.GetBasePosition().position.y,
                                   self.GetBasePosition().position.z])

        while (not rospy.is_shutdown()) and np.linalg.norm(startPosition - actualPosition) <= meters:
            rospy.sleep(checkRate_ms / 1000)
            actualPosition = np.array([self.GetBasePosition().position.x,
                                       self.GetBasePosition().position.y,
                                       self.GetBasePosition().position.z])

        timer.shutdown()

    def RotateBaseZ(self, angle, speed, checkRate_ms=500):
        pass