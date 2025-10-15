"""
    Jason Hughes
    January 2025

    Subscribe to pixhawk information
"""

import rospy
import numpy as np

from rospy import Node
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

class SubscriberHandler(Node):

    def __init__(self) -> None:
        self.odom_sub_ = self.create_subscription(Odometry, "/odom", self.odom_callback, 1)

        # protected variables
        self.easting_ = 0.0
        self.northing_ = 0.0
        self.altitude_ = 0.0

        self.rpy_ = np.zeros(3)

    def odom_callback(self, msg : Odometry) -> None:
        self.easting_ = msg.pose.pose.position.x
        self.northing_ = msg.pose.pose.position.y
        self.altitude_ = msg.pose.pose.position.z

        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.rpy_ = Rotation.from_quat(q).as_euler('xyz', degrees=False)
