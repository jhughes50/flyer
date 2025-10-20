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
        bayer_img = np_arr.reshape((msg.height, msg.width)) 
        image = cv2.cvtColor(bayer_img, cv2.COLOR_BayerRGGB2BGR)
        return image

    @staticmethod
    def blackfly_compressed_image_msg_to_array(msg : CompressedImage) -> np.ndarray:
        np_arr = np.frombuffer(msg.data, np.uint8)
        bayer_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        image = cv2.cvtColor(bayer_img, cv2.COLOR_BayerRGGB2BGR)
        return image

    @staticmethod
    def array_to_image_msg(img : np.ndarray, downres : bool = False) -> Image:
        msg = Image()

        if downres:
            h, w = img.shape[:2]
            msg.height = h // 4
            msg.width = w // 4
            msg.encoding = 'bgr8'

            msg.step = w * img.shape[2]
            img = cv2.resize(img, (w//4, h//4))
            msg.data = img.tobytes()
        else:
            h, w = img.shape[:2]
            msg.height = h
            msg.width = w
            msg.encoding = 'bgr8'

            msg.step = w * img.shape[2]
            msg.data = img.tobytes()
        return msg

    @staticmethod
    def odometry_msg_to_transform(msg : Odometry) -> RigidTransform: 
        orient = msg.pose.pose.orientation
        position = msg.pose.pose.position
        R = Rotation.from_quat([orient.x, orient.y, orient.z, orient.w])
        #T = np.array([position.x, position.y, position.z])
        T = np.array([position.x, position.y, 40.0])
        
        return RigidTransform(T, R.as_matrix())
