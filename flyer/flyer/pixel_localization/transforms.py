
import numpy as np
from flyer.core.rigid_transform import RigidTransform


class Transforms:

    def __init__(self) -> None:
        self.enu_to_ned = np.array([[0., 1., 0., 0.],
                                    [1., 0., 0., 0.],
                                    [0., 0., -1., 0.],
                                    [0., 0., 0., 1.]])
                                   
        # IMU to Camera
        self.T_cam_imu = RigidTransform.from_matrix(np.array([[0.0, -1.0, 0.0,  0.0],
                                                              [1.0, 0.0, 0.0, 0.0],
                                                              [0.0, 0.0, 1.0, 0.0],
                                                              [0.0, 0.0, 0.0,  1.0]]))
        # Camera to IMU 
        self.T_imu_cam = self.T_cam_imu.inv()
