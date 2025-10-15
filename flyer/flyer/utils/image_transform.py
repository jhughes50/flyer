import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from sensor_msgs.msg import CompressedImage, Image
import cv_bridge

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1  # Only keep the latest message
)

class ImageVizTransformer(Node):

    def __init__(self) -> None:
        super(ImageVizTransformer, self).__init__("img_viz_util")

        self.bridge_ = cv_bridge.CvBridge()
        #self.data_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.img_sub_ = self.create_subscription(CompressedImage, "/flir/image_raw/compressed", self.img_callback, qos_profile=qos_profile)#, callback_group=self.data_group)
        self.pub_ = self.create_publisher(Image, "/image/viz", qos_profile=qos_profile)

    def img_callback(self, msg : CompressedImage) -> None:
        img = self.bridge_.compressed_imgmsg_to_cv2(msg)
        img = cv2.cvtColor(img, cv2.COLOR_BAYER_RG2RGB)

        nmsg = self.bridge_.cv2_to_imgmsg(img, encoding='bgr8')
        self.pub_.publish(nmsg)

def main(args=None) -> None:
    rclpy.init(args=args)
    
    config = {}
    node = ImageVizTransformer()
    executor = SingleThreadedExecutor()

    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
