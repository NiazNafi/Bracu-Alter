#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from pathlib import Path
from models.common import DetectMultiBackend
from utils.general import (check_img_size, non_max_suppression, scale_boxes, increment_path)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from position_msgs.msg import ObjectPosition, ObjectPositions
import cv_bridge
bridge = CvBridge()

import os
import sys


FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

class CameraSubscriber:
    def __init__(self):
        rospy.init_node('camera_subscriber', anonymous=True)

        # Use absolute path to ensure the file is found
        self.weights = ROOT / 'charli.pt'  # Change this to the actual path of charli.pt
        self.imgsz = (640, 480)
        self.conf_thres = 0.25
        self.iou_thres = 0.45
        self.max_det = 1000
        self.classes = None
        self.agnostic_nms = False
        self.augment = False
        self.visualize = False
        self.line_thickness = 3
        self.hide_labels = False
        self.hide_conf = False
        self.half = False
        self.stride = 32
        self.dnn = False

        # Debugging print statements
        print(f"Using weights file: {self.weights}")
        assert Path(self.weights).is_file(), f"Error: weights file '{self.weights}' does not exist."

        self.device = select_device('')
        self.model = DetectMultiBackend(self.weights, device=self.device, dnn=self.dnn, fp16=self.half)
        stride, self.names, pt = self.model.stride, self.model.names, self.model.pt
        imgsz = check_img_size(self.imgsz, s=stride)
        self.model.warmup(imgsz=(1 if pt or self.model.triton else 1, 3, *imgsz))

        self.subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.camera_callback)
        self.depth_subscriber = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        self.position_publisher = rospy.Publisher('/objects_position/message', ObjectPositions, queue_size=10)

        self.depth_image = np.zeros((480, 640), dtype=np.uint16)

        # Camera intrinsic parameters
        self.fx = 616.0
        self.fy = 616.0
        self.cx = 315.85
        self.cy = 238.32

    def depth_callback(self, data):
        try:
            cv_ptr = bridge.imgmsg_to_cv2(data, desired_encoding="16UC1")
            self.depth_image = cv_ptr
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)

    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        img0 = img.copy()
        img = img[np.newaxis, :, :, :]
        img = np.stack(img, 0)
        img = img[..., ::-1].transpose((0, 3, 1, 2))
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.model.device)
        img = img.half() if self.model.fp16 else img.float()
        img /= 255
        if len(img.shape) == 3:
            img = img[None]

        pred = self.model(img, augment=self.augment, visualize=self.visualize)
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)

        object_positions_msg = ObjectPositions()
        object_positions_msg.header.stamp = rospy.Time.now()
        object_positions_msg.header.frame_id = "position"

        for i, det in enumerate(pred):
            if len(det):
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], img0.shape).round()

                for *xyxy, conf, cls in reversed(det):
                    x1, y1, x2, y2 = map(int, xyxy)
                    x_c = (x1 + x2) // 2
                    y_c = (y1 + y2) // 2
                    dist = self.depth_image[y_c, x_c]

                    if dist > 0:  # Avoid invalid depth values
                        obj_pos = ObjectPosition()
                        obj_pos.Class = self.names[int(cls)]
                        obj_pos.x = int(dist * (x_c - self.cx) / self.fx - 35)  # 35 is the offset
                        obj_pos.y = int(dist * (y_c - self.cy) / self.fy)
                        obj_pos.z = int(dist)

                        object_positions_msg.object_positions.append(obj_pos)

                    annotator = Annotator(img0, line_width=self.line_thickness, example=str(self.names))
                    label = f'{self.names[int(cls)]} {conf:.2f}' if not self.hide_labels else None
                    annotator.box_label(xyxy, label, color=colors(int(cls), True))

        self.position_publisher.publish(object_positions_msg)

        cv2.imshow("IMAGE", img0)
        cv2.waitKey(4)

if __name__ == '__main__':
    camera_subscriber = CameraSubscriber()
    rospy.spin()

