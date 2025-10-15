"""
    Jason Hughes
    January 2025

    Code to localize the pixels
"""
import cv2
import yaml
import numpy as np

from typing import Tuple, Dict

class PixelLocalizer:

    def __init__(self, path : str = "./blackfly-5mm.py") -> None:

        params = self.get_params(path)

        self.width_, self.height_ = params["resolution"]

        self.hfov_ = params["hfov"]
        self.vfov_ = params["vfov"]
        coeffs = params["intrinsics"]
        self.intrinsics_ = np.array([[coeffs[0], 0, coeffs[2]],
                                     [0, coeffs[1], coeffs[3]],
                                     [0, 0, 1]], dtype=np.float32)

        self.dist_ = params["distortion_coeffs"]
        self.dist_.append(0.0)
        self.dist_ = np.array(self.dist_, dtype=np.float32)

        self.easting_ = 0.0
        self.northing_ = 0.0
        self.alt_ = 0.0
        self.rpy_ = np.zeros(3)

        self.declination_ = np.radians(11.83) # 11.83 degrees W in philly


    def get_params(self, path : str) -> Dict:
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        return data["cam0"]
    
    def create_rotation_matrix(self) -> np.ndarray:
        roll, pitch, yaw = self.rpy_
        roll, pitch = 0.0, 0.0
        R_yaw = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        R_pitch = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        R_roll = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        R = R_roll @ R_pitch @ R_yaw
        
        return R


    def pixel_to_relative(self, pixel : Tuple[int]) -> Tuple[float]:
        """ 
        pixel frame to relative x and y in meters
        """
        v = np.array([pixel[0], pixel[1], -1.0])
        
        #R = self.create_rotation_matrix()
        vector = self.rpy_.T @ v

        #print(vector)
        
        scale = self.alt_ / vector[2]
        #scale = 40.0 / vector[2]
        x = vector[0] * scale
        y = vector[1] * scale

        return x, y

    def get_quadrant(self, pixel : Tuple[int]) -> int:
        x, y = pixel
        
        # convert to center frame 
        x = x - self.width_ // 2
        y = self.height_ // 2 - y
        
        if x >= 0 and y >= 0:
            return 0
        elif x < 0 and y >= 0:
            return 1
        elif x < 0 and y < 0:
            return 2
        elif x >= 0 and y < 0:
            return 3
        else:
            raise Exception("Invalid pixel coordinates")


    def relative_to_world(self, local : Tuple[float], quad : int) -> Tuple[float]:
        x, y = abs(local[0]), abs(local[1])
        if quad == 0:
            e = self.easting_ + x
            n = self.northing_ + y
        elif quad == 1:
            e = self.easting_ - x
            n = self.northing_ + y
        elif quad == 2:
            e = self.easting_ - x
            n = self.northing_ - y
        elif quad == 3:
            e = self.easting_ + x
            n = self.northing_ - y
 
        return e, n

    def __call__(self, pixel : Tuple[int], utm : Tuple[float], rpy : np.ndarray) -> Tuple[float]:
        """ rpy : 3x3 rotation matrix """
        self.easting_, self.northing_, self.alt_ = utm
        self.rpy_ = rpy
        
        quadrant = self.get_quadrant(pixel)
        pixel = cv2.undistortPoints(np.array(pixel, dtype=np.float32).reshape(1,1,2), self.intrinsics_, self.dist_)#, P=self.intrinsics_)
        pixel = pixel[0][0]
        
        rel = self.pixel_to_relative(pixel)
        #print("REL: ", rel)
        #print(self.easting_, self.northing_, rel)
        utm = self.relative_to_world(rel, quadrant)

        return utm

    def localize(self, pixel : Tuple[int], utm : Tuple[float], rpy : np.ndarray) -> Tuple[float]:
        return self(pixel, utm, rpy)


if __name__ == "__main__":
    # unit test here 
    pass
