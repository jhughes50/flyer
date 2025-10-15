"""
    Jason Hughes
    March 2025
    Grounded SAM 2 Runner script
"""

import os
import sys
import cv2
import yaml
import torch
import numpy as np
import supervision as sv
import pycocotools.mask as mask_util
from pathlib import Path
from torchvision.ops import box_convert
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
from PIL import Image
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
from torchvision import transforms 
from typing import List, Dict, Tuple


class Annotators:
    def __init__(self) -> None:
        self.box = sv.BoxAnnotator()
        self.label = sv.LabelAnnotator()
        self.mask = sv.MaskAnnotator()


class GroundedSam2Runner():

    def __init__(self, config : Dict = { }) -> None:
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print("[GROUNDED-SAM2] Using Device: ", self.device)
        if self.device == "cuda":
            torch.autocast("cuda", dtype=torch.bfloat16).__enter__()

        model_id = config["grounding_dino_model_id"]
        
        # sam2 model
        sam2_checkpoint = config["sam2_checkpoint"]
        model_cfg = config["sam2_model_config"]

        sam2_model = build_sam2(model_cfg, sam2_checkpoint, device=self.device)
        self.sam2_predictor = SAM2ImagePredictor(sam2_model)

        # build grounding dino model
        self.processor = AutoProcessor.from_pretrained(model_id)
        self.grounding_model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id, local_files_only=False).to(self.device)
        
        self.transform = transforms.Compose([transforms.ToTensor(),
                                             transforms.Resize((800,800)),
                                             transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])

        for param in self.grounding_model.parameters():
            param.data = param.data.contiguous()

        self.text_ = config["default_text"]
        self.box_threshold = config["box_threshold"]
        self.text_threshold = config["text_threshold"]

        self.annotator = Annotators()

        print("[GROUNDED-SAM2] Model Initialized")


    @property
    def text(self) -> str:
        return self.text_

    @text.setter
    def text(self, t : str) -> None:
        self.text_ = t

    def get_furthest_point_from_edge_cv2(self, mask : np.ndarray) -> Tuple[float, float]:
        mask = mask.astype(np.uint8)
        dist_img = cv2.distanceTransform(mask, distanceType=cv2.DIST_L2, maskSize=5).astype(np.float32)
        cy, cx = np.where(dist_img==dist_img.max())
        cx, cy = cx.mean(), cy.mean()  # there are sometimes cases where there are multiple values returned for the visual center
        return cx, cy

    @torch.no_grad()
    def predict(self, image_source : np.ndarray, text : str = " ") -> Tuple[np.ndarray, List[float], sv.Detections, List[str], np.ndarray]: #TODO type this consistently
        image_source = image_source.copy()
        image_source = cv2.cvtColor(image_source, cv2.COLOR_BGR2RGB)
        self.sam2_predictor.set_image(image_source)

        inputs = self.processor(images=image_source, text=text, return_tensors="pt").to(self.device)
        outputs = self.grounding_model(**inputs)

        results = self.processor.post_process_grounded_object_detection(outputs, 
                                                                        inputs.input_ids, 
                                                                        threshold=self.box_threshold, 
                                                                        text_threshold=self.text_threshold, 
                                                                        target_sizes=[image_source.shape[:-1]])

        input_boxes = results[0]["boxes"].cpu().numpy()

        if torch.cuda.get_device_properties(0).major >= 8:
            # TODO Are we ampere architecture
            # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True

        masks, scores, logits = self.sam2_predictor.predict(point_coords=None,
                                                            point_labels=None,
                                                            box=input_boxes,
                                                            multimask_output=False,)

        if masks.ndim == 4:
            masks = masks.squeeze(1)


        confidences = results[0]["scores"].cpu().numpy().tolist()
        class_names = results[0]["text_labels"]
        class_ids = np.array(list(range(len(class_names))))
        
        #labels = [f"{class_name} {confidence:.2f}" for class_name, confidence in zip(class_names, confidences)]
        labels = [f"{class_name}" for class_name, confidence in zip(class_names, confidences)]
        detections = sv.Detections(xyxy=input_boxes,  # (n, 4)
                                   mask=masks.astype(bool),  # (n, h, w)
                                   class_id=class_ids)

        dets = np.zeros((masks.shape[0], 2))
        
        for i, mask in enumerate(masks):
            cx, cy = self.get_furthest_point_from_edge_cv2(mask)
            dets[i, 0] = cx
            dets[i, 1] = cy

        return masks, confidences, detections, labels, dets

    def run(self, image : np.ndarray, text : str = None) -> Dict:
        img_copy = image.copy()
        if text is None:
            masks, confidences, detections, labels, dets = self.predict(image, self.text_)
        else:
            text = self.text_ + text
            masks, confidences, detections, labels, dets = self.predict(image, text)
        
        annotated_frame = self.annotator.box.annotate(scene=img_copy, detections=detections)
        annotated_frame = self.annotator.label.annotate(scene=annotated_frame, detections=detections, labels=labels)
        annotated_frame = self.annotator.mask.annotate(scene=annotated_frame, detections=detections)
        #print(masks.shape)
        centers = np.zeros((len(detections), 2))
        for i in range(masks.shape[0]):
            centers[i,:] = self.get_furthest_point_from_edge_cv2(masks[i,:,:])

        ret = {"timestamp": None, "centers": centers, "labels": labels, "masks": masks, "annotated": annotated_frame, "scores": confidences}
        return ret 


if __name__ == "__main__":
    with open("/home/jason/ws/src/symbiote-ag/config/grounding_dino.yaml") as f:
        config = yaml.safe_load(f)
    gsam = GroundedSam2Runner(config)
    #img = Image.open("./test.jpg")#.convert("BGR")
    #img = np.asarray(img)
    img = cv2.imread("./test.jpg")
    cv2.imwrite("original.png", img)
    print(img.size)
    ret = gsam.run(img)
    print(ret["centers"])
    cv2.imwrite("output.png", ret["annotated"])
