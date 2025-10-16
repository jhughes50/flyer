import cv2
import yaml
import numpy as np

SCALE_FACTOR = 1

class Camera:

    def __init__(self, path):
        
        with open(path, "r") as file:
            params = yaml.safe_load(file)

        cam = params["cam0"]

        self.K = np.eye(3)
        self.K[0,0] = cam["intrinsics"][0] / SCALE_FACTOR
        self.K[1,1] = cam["intrinsics"][1] / SCALE_FACTOR
        self.K[0,2] = cam["intrinsics"][2] / SCALE_FACTOR
        self.K[1,2] = cam["intrinsics"][3] / SCALE_FACTOR

        self.D = np.array(cam["distortion_coeffs"])

        self.width = int(cam["resolution"][0] / SCALE_FACTOR)
        self.height = int(cam["resolution"][1] / SCALE_FACTOR)

        self.resolution = (self.width, self.height)

    @staticmethod
    def Load(path):
        return Camera(path)

    def project(self, camera_coords : np.ndarray) -> np.ndarray:
        camera_coords = np.array(camera_coords)
        if camera_coords.ndim == 1:
            camera_coords = camera_coords[None, :]
        assert camera_coords.ndim == 2 and camera_coords.shape[1] == 3, f"Input must be of shape (N, 3), but got {camera_coords.shape}"
        assert (camera_coords[:, -1] > 0).all(), f"Z coordinate must be positive for projection, got {camera_coords}"

        # Normalize the coordinates
        normalized = camera_coords / camera_coords[:, -1].reshape(-1, 1)  # divide by z to get normalized coordinates

        # Apply intrinsic parameters
        pixel_coords = normalized @ self.K.T  # (3, N)
        pixel_coords = pixel_coords[:, :2]

        return pixel_coords

    def reproject(self, pixel_coords : np.ndarray, fix_distortion : bool = True) -> np.ndarray:

        pixel_coords = np.resize(pixel_coords, (1,2))
        assert pixel_coords.ndim == 2 and pixel_coords.shape[1] == 2, f"Input must be of shape (N, 2), but got {pixel_coords.shape}"

        if self.D is not None and fix_distortion:
            # undistort when given the distortion coefficients
            pixel_coords = pixel_coords.reshape(-1, 1, 2).astype(np.float32)
            undistorted = cv2.undistortPoints(pixel_coords, self.K, self.D)
            rays = np.concatenate([undistorted[:, 0], np.ones((len(undistorted), 1))], axis=1)
        else:
            # pinhole model
            x = (pixel_coords[:, 0] - self.cx) / self.fx
            y = (pixel_coords[:, 1] - self.cy) / self.fy
            rays = np.stack([x, y, np.ones_like(x)], axis=1)

        return rays

    def get_camera_alt(self, alt, R, t_cam_body):
        alt_offset = R_imu_world @ t_cam_body 
        assert alt_offset <= 0, "ERROR"
        return alt - alt_offset
