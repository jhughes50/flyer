import utm
import numpy as np

from flyer.pixel_localizer.transforms import Transforms
from flyer.pixel_localizer.camera import Camera

class PixelLocalizer:

    def __init__(self, path : str) -> None:
        self.transforms = Transforms()
        self.camera = Camera.Load(path)
        self.enu_to_ned = np.array([[0.0, 1.0, 0.0],
                                   [1.0, 0.0, 0.0],
                                   [0.0, 0.0, -1.0]])


    def gps_to_utm(self, lat : float, lon : float) -> np.ndarray:
        lat, lon, zone_c, zone_i = utm.from_latlon(lat, lon)
        return np.ndarray((lat, lon))

    def pixel_to_world(self, pixel : np.ndarray, position : np.ndarray, orientation : np.ndarray) -> np.ndarray:
        pixel_coord = pixel.reshape(-1, 2)
        ray_cam = self.camera.reproject(pixel_coord, fix_distortion=True)
        
        ray_imu = (self.transforms.T_imu_cam.rotation @ ray_cam.reshape(-1, 1)).flatten()
        ray_world = (orientation @ ray_imu.reshape(-1,1)).flatten()

        #cam_alt = self.camera.get_camera_alt(position[-1], orientation)
        scale = -position[-1] / ray_world[-1]

        ray_imu *= scale

        pixel_imu = self.transforms.T_imu_cam.translation + ray_imu

        pixel_world = position + (orientation @ pixel_imu.reshape(-1, 1)).flatten()

