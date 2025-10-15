
import utm
import json
import glob
import yaml
from PIL import Image
import cv2
import time
import numpy as np
from skimage.morphology import skeletonize
from itertools import combinations

from flyer.utils.graphing import image2graph, road_graph
from flyer.utils.data_classes import BoundingBox, DetectionResult
from flyer.pixel_localization.pixel_localizer import PixelLocalizer

class Segrapher():
    def __init__(self, config: dict, calib_path : str) -> None: 
        self.data = {}
        self.data["origin"] = config["origin"]
        self.data["objects"] = []
        self.data["regions"] = []
        self.data["object_connections"] = []
        self.data["region_connections"] = []
        self.data["robot_location"] = [""]
        self.data["heirarchy"] = []
        self.obj_count = {"road": 0}
        
        self.pixel_localizer = PixelLocalizer(calib_path)

        self.dists = np.zeros(2)
        self.ontology = {}
        self.ontology["vehicle"] = {"parent": "parking"}
        self.ontology["pothole"] = {"parent": "road"}
        self.bad_objects = ['road', 'building', 'cars']
        self.total_pixels = 2048 * 1536
        self.last_road_mask = None

    def object_heirarchy(self, labels: list, centers: np.array, lat, lon, alt) -> None:
        for i, label in enumerate(labels):
            if label in self.ontology:
                if self.ontology[label]["parent"] in labels:
                    self.data["heirarchy"].append(["{}_{}".format(label, self.obj_count[label]),
                                              "{}_{}".format(self.ontology[label]["parent"], self.obj_count[self.ontology[label]["parent"]])])

    def distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        
        return distance

    def is_close(self, e : float, n : float, label : str) -> bool:
        for obj in self.data["objects"]:
            if self.distance((e,n), obj["coords"]) < 10.0 and label in obj["name"]:
                return True
        return False

    def add_objects(self, labels: list, centers: np.array, scores, utm : tuple, rpy : np.ndarray) -> None:
        if self.obj_count["road"] == 0:
            return
        for i, label in enumerate(labels):
            if label == "orange barrel":
                if label not in self.obj_count:
                    self.obj_count[label] = 0
                else:
                    self.obj_count[label] += 1

                name = "{}_{}".format(label, self.obj_count[label])
                e, n = self.pixel_localizer(tuple(centers[i]), utm, rpy)
                #print("Adding object ", label, "  ", label in self.bad_objects)
                e = e - self.data["origin"][0]
                n = n - self.data["origin"][1]
                if not self.is_close(e, n, label) and scores[i] > 0.3:
                    self.data["objects"].append({"name": name, "coords": [e, n]})
                    self.data["object_connections"].append(["{}_{}".format(label, self.obj_count[label]), "road_{}".format(self.obj_count["road"])])
    
    def get_largest_road_mask_index(self, labels : list, masks : np.ndarray) -> int:
        index = 0
        largest_mask_sum = 0
        contains_road = False
        for i, mask in enumerate(masks):
            if labels[i] == "road":
                contains_road = True
                mask_sum = np.sum(mask)
                if mask_sum > largest_mask_sum and mask_sum > (0.2 * self.total_pixels):
                    index = i
                    largest_mask_sum = mask_sum
            else:
                continue

        return index, contains_road
        
    def add_large_region(self, labels: list, centers: np.array, scores, utm : tuple, rpy : np.ndarray, masks : np.ndarray) -> None:
        accepted = []
        road_idx, contains_road = self.get_largest_road_mask_index(labels, masks)
        print("contains road " , contains_road, " road mask ", road_idx)
        for i, label in enumerate(labels):
            if contains_road:
                if label == "road" and road_idx == i:

                    name = "{}_{}".format(label, self.obj_count[label])
                    e, n = self.pixel_localizer(tuple(centers[i]), utm, rpy)

                    e = e - self.data["origin"][0]
                    n = n - self.data["origin"][1]

                    if (self.obj_count[label] >= 1):
                        closest = self.obj_count[label] - 1 
                        closest_dist = np.linalg.norm(np.array([e,n]) - self.data["regions"][closest]["coords"])
                        for k, region in enumerate(self.data["regions"]):
                            dist = np.linalg.norm(np.array([e,n]) - region["coords"])
                            if dist < closest_dist:
                                closest = k
                                closest_dist = np.linalg.norm(np.array([e,n]) - self.data["regions"][closest]["coords"])
                        if closest_dist > 5 and closest_dist < 30.0:
                            self.obj_count[label] += 1
                            name = "{}_{}".format(label, self.obj_count[label])
                            accepted.append(i)
                            self.data["regions"].append({"name": name, "coords": [e, n]})
                            self.data["region_connections"].append(["road_{}".format(closest+1), "road_{}".format(self.obj_count[label])])
                            for kk, region in enumerate(self.data["regions"]):
                                dist = np.linalg.norm(np.array([e,n]) - region["coords"])
                                if dist < 15. and kk != closest - 1:
                                    self.data["region_connections"].append(["road_{}".format(kk+1), "road_{}".format(self.obj_count[label])])
                                    break
                    else:
                        self.obj_count[label] += 1
                        name = "{}_{}".format(label, self.obj_count[label])
                        self.data["regions"].append({"name": name, "coords": [e, n]})


    def add_regions_as_objects(self, labels: list, centers: np.array, scores, utm : tuple, rpy : np.ndarray) -> None:
        accepted = []
        for i, label in enumerate(labels):
            if label == "road":
                #if label not in self.obj_count:
                #    self.obj_count[label] = 0
                #else:
                #    self.obj_count[label] += 1

                name = "{}_{}".format(label, self.obj_count[label])
                e, n = self.pixel_localizer(tuple(centers[i]), utm, rpy)

                e = e - self.data["origin"][0]
                n = n - self.data["origin"][1]

                if (self.obj_count[label] >= 1):
                    closest = self.obj_count[label] - 1 
                    closest_dist = np.linalg.norm(np.array([e,n]) - self.data["regions"][closest]["coords"])
                    for k, region in enumerate(self.data["regions"]):
                        dist = np.linalg.norm(np.array([e,n]) - region["coords"])
                        if dist < closest_dist:
                            closest = k
                            closest_dist = np.linalg.norm(np.array([e,n]) - self.data["regions"][closest]["coords"])
                    if closest_dist > 5 and closest_dist < 30.0:
                        self.obj_count[label] += 1
                        name = "{}_{}".format(label, self.obj_count[label])
                        accepted.append(i)
                        self.data["regions"].append({"name": name, "coords": [e, n]})
                        name = "{}_{}".format(label, self.obj_count[label])
                        accepted.append(i)
                        self.data["regions"].append({"name": name, "coords": [e, n]})
                        self.data["region_connections"].append(["road_{}".format(closest+1), "road_{}".format(self.obj_count[label])])
                        for kk, region in enumerate(self.data["regions"]):
                            dist = np.linalg.norm(np.array([e,n]) - region["coords"])
                            if dist < 15. and kk != closest - 1:
                                self.data["region_connections"].append(["road_{}".format(kk+1), "road_{}".format(self.obj_count[label])])
                                break
                else:
                    self.obj_count[label] += 1
                    name = "{}_{}".format(label, self.obj_count[label])
                    self.data["regions"].append({"name": name, "coords": [e, n]})

    def check_intersect(self, mask, pt1, pt2):
        pass

    def add_regions(self, labels: list, centers: np.array, utm : tuple, rpy : np.ndarray, masks : np.ndarray) -> None:
        for i, label in enumerate(labels):
            if label == "road":
                #points = self.find_points_on_road(masks[i])
                #print("points1 :", points, centers)
                mask_rest = None
                for j, mask in enumerate(masks):
                    if i!=j and labels[j] != "road":
                        if mask_rest is None:
                            mask_rest = mask
                        else:
                            mask_rest = cv2.bitwise_or(mask, mask_rest)
                if mask_rest is not None:
                    mask_rest = cv2.bitwise_not(mask_rest)
                    masks[i] = cv2.bitwise_and(masks[i], mask_rest)
               
                (V, E) = road_graph(masks[i])
                accepted = []
                for x, p in enumerate(V):
                    if mask_rest is not None:
                        if not mask_rest[p[1], p[0]]:
                            continue
                    #intersects = False
                    #print(p)
                    #for j, mask in enumerate(masks):
                    #    if i != j:
                    #        if mask[p[1], p[0]]:
                    #            intersects = True
                    #            print("Intersects with: ", labels[j])
                    #            break
                    #if intersects:
                    #    continue
                    #p = centers[i,:]
                    e, n = self.pixel_localizer(tuple(p), utm, rpy)
                    e = e - self.data["origin"][0]
                    n = n - self.data["origin"][1]
                            

                    if (self.obj_count[label] >= 1):
                        closest = self.obj_count[label] - 1 
                        closest_dist = np.linalg.norm(np.array([e,n]) - self.data["regions"][closest]["coords"])
                        for k, region in enumerate(self.data["regions"]):
                            dist = np.linalg.norm(np.array([e,n]) - region["coords"])
                            if dist < closest_dist:
                                closest = k
                                closest_dist = np.linalg.norm(np.array([e,n]) - self.data["regions"][closest]["coords"])
                        if closest_dist > 5 and closest_dist < 30.0:
                            self.obj_count[label] += 1
                            name = "{}_{}".format(label, self.obj_count[label])
                            accepted.append(x)
                            dist = np.linalg.norm(np.array([e,n]) - region["coords"])
                            if dist < 15. and kk != closest - 1:
                                self.data["region_connections"].append(["road_{}".format(kk+1), "road_{}".format(self.obj_count[label])])
                                break
                    else:
                        self.obj_count[label] += 1
                        name = "{}_{}".format(label, self.obj_count[label])
                        self.data["regions"].append({"name": name, "coords": [e, n]})

                    #print(x)


    def find_points_on_road(self, mask):
        samples_x = np.random.randint(0, mask.shape[0], 20)
        samples_y = np.random.randint(0, mask.shape[1], 20)
        accepted = []
        for i in range(20):
            if mask[samples_x[i], samples_y[i]] != 0:
                accepted.append([samples_x[i], samples_y[i]])

        accepted = np.asarray(accepted)
        return accepted

if __name__ == "__main__": 
    with open("./config/grounding_dino.yaml") as f:
        config = yaml.safe_load(f)
    gsam = GroundedSam2Runner(config)
    img = Image.open("./test.png").convert("RGB")
    img = np.asarray(img)
    print(img.size)
    ret = gsam.run(img)
    print(ret["centers"])
    cv2.imwrite("output.png", ret["annotated"])

    lat, lon, alt = 39.942008, -75.199641, 0.0
    segrapher = Segrapher({"origin":(0,0)})
    print(ret["labels"])
    segrapher.add_objects(ret["labels"], ret["centers"], lat, lon, alt)
    segrapher.add_regions(ret["labels"], ret["centers"], lat, lon, lat, ret["masks"])
