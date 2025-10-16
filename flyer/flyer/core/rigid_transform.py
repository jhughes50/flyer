
import numpy as np

from scipy.spatial.transform import Rotation

class RigidTransform:

    def __init__(self, translation : np.ndarray, rotation : np.ndarray) -> None:
        self.translation = translation
        self.rotation = rotation

        self.pose = np.eye(4)
        self.pose[:3,:3] = rotation
        self.pose[:3,3] = translation

    @classmethod
    def from_matrix(cls, arr : np.ndarray) -> 'RigidTransform':
        return cls(arr[:-1,-1], arr[:3,:3])

    def force_alt(self, alt : float) -> None:
        self.translation[-1] = alt

    def inv(self) -> 'RigidTransform':
        r_inv = self.rotation.T
        t_inv = -r_inv @ self.translation
        return RigidTransform(t_inv, r_inv)

    def euler(self) -> np.ndarray:
        rot = Rotation.from_matrix(self.rotation)
        return rot.as_euler('xyz', degrees=True)
