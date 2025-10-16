"""
    Jason Hughes
    October 2025

    Flyer Core
"""
import cv2
import utm
import copy
import json
import time
import yaml
import threading
import numpy as np

from flyer.segrapher.grounded_sam2 import GroundedSam2Runner
#from flyer.segrapher.segrapher import Segrapher
from flyer.pixel_localization.localizer import PixelLocalizer
from flyer.core.rigid_transform import RigidTransform
from flyer.utils.postprocess import postprocess_labels

from typing import Dict, Optional, List, Tuple
from dataclasses import dataclass
from collections import deque

from flyer.utils.kml import create_drone_kml


@dataclass
class ImageOdometryPair:
    image : Optional[np.ndarray] = None
    odometry: Optional[RigidTransform] = None

@dataclass
class ImageGraphPair:
    image : Optional[np.ndarray] = None
    graph : Optional[str] = None

class FlyerCore:

    def __init__(self, model_config_path : str, origin : Tuple, calib_path : str) -> None:
        with open(model_config_path) as f:
            model_config = yaml.safe_load(f)

        self.model_ = GroundedSam2Runner(model_config)
        self.labels_ = "road ."

        #self.segrapher_ = Segrapher({"origin": origin}, calib_path)
        self.localizer_ = PixelLocalizer(calib_path)

        self.process_buffer_ = deque(maxlen=100)
        self.process_mutex_ = threading.Lock()
        
        self.result_buffer_ = list()
        self.result_mutex_ = threading.Lock()

        self.running_ = False 
        self.process_thread_ = None
        self.count_ = 0

    def set_text(self, txt : str) -> None:
        self.labels_ = txt

    def start(self) -> None:
        print("[FLYER] Starting Flyer Processing")
        self.running_ = True
        self.thread_ = threading.Thread(target=self.processor)

    def stop(self) -> None:
        print("[FLYER] Stopping Flyer Processing")
        self.running_ = False
        if self.process_thread_:
            self.process_thread_.join()

    def push(self, pair : ImageOdometryPair) -> None:
        with self.process_mutex_:
            self.process_buffer_.append(pair)

    def processor(self) -> None:
        while self.running_:
            if len(self.process_buffer_) < 1:
                time.sleep(0.5)
                continue 
            with self.process_mutex_:
                # FIFO buffer
                pair = self.process_buffer_.popleft()
            try:
                ret = self.model_.run(pair.image, self.labels_)
            except AssertionError as e:
                print("No Detections")
                continue

            #labels = postprocess_labels(ret["labels"])

            ## TODO update this
            #self.segrapher_.add_objects(labels, ret["centers"], ret["scores"], self.utm_, self.rotation_)
            #self.segrapher_.add_large_region(labels, ret["centers"], ret["scores"], self.utm_, self.rotation_, ret["masks"])
    
            #with self.result_mutex_:
            #    self.result_buffer_.append(ImageGraphPair(ret["annotated"], json.dumps(self.segrapher_.data)))
            
    def get_results(self) -> List[ImageGraphPair]:
        with self.result_mutex_:
            result_copy = copy.deepcopy(self.result_buffer_)
            self.result_buffer_.clear()
        return result_copy

    def _test_localization(self, pair : ImageOdometryPair) -> None:
        xcoords = [0, pair.image.shape[0]]
        ycoords = [0, pair.image.shape[1]]

        utm_coords = np.zeros((2, 2, 3))
        print("Translation: ", pair.odometry.translation)
        print("Rotation: ", pair.odometry.rotation)
        for i in range(2):
            for j in range(2):
                coord = np.array((xcoords[i], ycoords[j]))
                print("coord: ", coord)
                utm_coords[i, j] = self.localizer_.pixel_to_world(coord, pair.odometry.translation, pair.odometry.rotation)
                print(utm_coords[i,j]) 
        position = utm.to_latlon(pair.odometry.translation[0], pair.odometry.translation[1], 18, 'S')
        top_left = utm.to_latlon(utm_coords[0, 0, 0], utm_coords[0,0,1], 18 ,'S')
        bottom_right = utm.to_latlon(utm_coords[-1, -1, 0,], utm_coords[-1,-1,1], 18, 'S')
        top_right = utm.to_latlon(utm_coords[0, -1, 0,], utm_coords[0,-1,1], 18, 'S')
        bottom_left = utm.to_latlon(utm_coords[-1,0, 0,], utm_coords[-1,0,1], 18, 'S')

        poses = [position, top_left, bottom_right, top_right, bottom_left]

        create_drone_kml(poses, self.count_)
        cv2.imwrite("/home/jason/data/img%i.png" %self.count_, pair.image)
        self.count_ += 1
