"""
    Jason Hughes
    March 2025

    Ros Organizer
"""

import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
import message_filters

from flyer.core.flyer import FlyerCore, ImageOdometryPair, ImageGraphPair
from flyer.core.rigid_transform import RigidTransform
from flyer.ros.conversions import ROSConversionUtils

class FlyerNode(Node):

    def __init__(self, node_name=None, **kwargs) -> None:
        super(FlyerNode, self).__init__(node_name or 'flyer_node', **kwargs)

        # parameters
        self.declare_parameter("gdino_config_path", "./")
        gdino_config_path = self.get_parameter("gdino_config_path").value
        self.declare_parameter("calib_config_path", "./")
        calib_config_path = self.get_parameter("calib_config_path").value
        self.declare_parameter("inference_rate", 1)
        rate = self.get_parameter("inference_rate").value
        self.declare_parameter("origin_easting", 0.0)
        origin_easting = self.get_parameter("origin_easting").value
        self.declare_parameter("origin_northing", 0.0)
        origin_northing = self.get_parameter("origin_northing").value
        self.declare_parameter("downres_image", False)
        self.downres_ = self.get_parameter("downres_image").value
        self.declare_parameter("use_compressed", False)
        use_compressed = self.get_parameter("use_compressed").value

        self.pair_ = ImageOdometryPair()
        self.flyer_ = FlyerCore(gdino_config_path, (origin_easting, origin_northing), calib_config_path)
        self.flyer_.start()

        # callback groups
        data_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        inf_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        # subscribers
        if not use_compressed:
            self.img_sub_ = message_filters.Subscriber(self, Image, "/image", callback_group=data_group)
        else:
            self.img_sub_= message_filters.Subscriber(self, CompressedImage, "/image", callback_group=data_group)
        self.odm_sub_ = message_filters.Subscriber(self, Odometry, "/odom", callback_group=data_group)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.img_sub_, self.odm_sub_], 1, slop=0.001)
        if not use_compressed:
            self.ts.registerCallback(self.image_odom_callback)
        else:
            self.ts.registerCallback(self.compressed_image_odom_callback)

        self.txt_sub_ = self.create_subscription(String, "/text", self.text_callback, 1, callback_group=data_group)

        # publishers
        self.graph_pub_ = self.create_publisher(String, "/graph", 1)
        self.annot_pub_ = self.create_publisher(Image, "/image/annotated", 1)

        # timers
        self.timer_ = self.create_timer(1.0/rate, self.inference_callback, callback_group=inf_group)
        self.get_logger().info("[FLYER] Flyer Node Initialized")

    def __del__(self) -> None:
        self.flyer_.stop()

    def text_callback(self, msg : String) -> None:
        self.flyer_.set_text(msg.data)
 
    def image_odom_callback(self, img_msg : Image, odom_msg : Odometry) -> None:
        self.get_logger().info("[FLYER] Recieved Image Odom Pair", once=True)
        img = ROSConversionUtils.blackfly_image_msg_to_array(img_msg)
        odom = ROSConversionUtils.odomtry_msg_to_transform(odom_msg)

        self.pair_ = ImageOdometryPair(img, odom)

    def compressed_image_odom_callback(self, img_msg : CompressedImage, odom_msg : Odometry) -> None:
        self.get_logger().info("[FLYER] Recieved Image Odom Pair", once=True)
        img = ROSConversionUtils.blackfly_compressed_image_msg_to_array(img_msg)
        odom = ROSConversionUtils.odometry_msg_to_transform(odom_msg)

        self.pair_ = ImageOdometryPair(img, odom)

    def inference_callback(self) -> None:
        self.get_logger().info("[FLYER] Starting inference", once=True)
        if (self.pair_.image is not None and self.pair_.odometry is not None):
            self.flyer_.push(self.pair_)
            results = self.flyer_.get_results()
            #TODO publish annotated image and graph
