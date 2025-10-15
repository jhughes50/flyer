"""
    Jason Hughes
    October 2025

    Flyer Core
"""
import copy
import json
import time
import yaml
import threading
import numpy as np

from flyer.segrapher.grounded_sam2 import GroundedSam2Runner
from flyer.segrapher.segrapher import Segrapher
from flyer.pixel_localization.pixel_localizer import PixelLocalizer
from flyer.core.rigid_transform import RigidTransform
from flyer.utils.postprocess import postprocess_labels

from typing import Dict, Optional, List, Tuple
from dataclasses import dataclass
from collections import deque


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

        self.process_buffer_ = deque(maxlen=100)
        self.process_mutex_ = threading.Lock()
        
        self.result_buffer_ = list()
        self.result_mutex_ = threading.Lock()

        self.running_ = False 
        self.process_thread_ = None

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

        for i in range(2):
            for j in range(2):
                coord = (xcoords[i], ycoords[i])
                utm_coord = self.localizer_(coord, pair.odometry.translation, pair.odometry.orientation)
                
