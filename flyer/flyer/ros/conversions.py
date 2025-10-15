import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from typing import Tuple, List

from scipy.spatial.transform import Rotation

from flyer.core.rigid_transform import RigidTransform

class ROSConversionUtils:
    """ written as c++ static functions, idk why I did this """

    @staticmethod
    def blackfly_image_msg_to_array(msg : Image) -> np.ndarray:
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image

    @staticmethod
    def blackfly_compressed_image_msg_to_array(msg : CompressedImage) -> np.ndarray:
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        return image

    @staticmethod
    def array_to_image_msg(arr : np.ndarray, encoding : str = "bgr8") -> Image:
        ros_image = bridge.cv2_to_imgmsg(arr, encoding)
        return ros_image

    @staticmethod
    def odometry_msg_to_transform(msg : Odometry) -> RigidTransform: 
        orient = msg.pose.pose.orientation
        position = msg.pose.pose.position
        R = Rotation.from_quat([orient.x, orient.y, orient.z, orient.w])
        T = np.array([position.x, position.y, position.z])
        
        return RigidTransform(T, R.as_matrix())
