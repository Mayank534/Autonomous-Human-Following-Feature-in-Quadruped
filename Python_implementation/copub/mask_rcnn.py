import cv2
import numpy as np
import os

class MaskRCNN:
    def __init__(self):
        package_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(package_dir, 'dnn', 'frozen_inference_graph_coco.pb')
        config_path = os.path.join(package_dir, 'dnn', 'mask_rcnn_inception_v2_coco_2018_01_28.pbtxt')
        classes_path = os.path.join(package_dir, 'dnn', 'classes.txt')

        if not os.path.exists(model_path) or not os.path.exists(config_path) or not os.path.exists(classes_path):
            raise FileNotFoundError(f"Model file, config file, or classes file not found: {model_path}, {config_path}, {classes_path}")

        self.net = cv2.dnn.readNetFromTensorflow(model_path, config_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        # self.net = cv2.dnn.readNetFromTensorflow(model_path, config_path)
        # self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        # self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        np.random.seed(2)
        self.colors = np.random.randint(0, 255, (90, 3))

        self.detection_threshold = 0.7
        self.mask_threshold = 0.3

        self.classes = []
        with open(classes_path, "r") as file_object:
            for class_name in file_object.readlines():
                class_name = class_name.strip()
                self.classes.append(class_name)

        self.obj_boxes = []
        self.obj_classes = []
        self.obj_centers = []
        self.obj_contours = []

        self.distances = []

    def detect_objects_mask(self, bgr_frame):
        blob = cv2.dnn.blobFromImage(bgr_frame, swapRB=True)
        self.net.setInput(blob)

        boxes, masks = self.net.forward(["detection_out_final", "detection_masks"])

        frame_height, frame_width, _ = bgr_frame.shape
        detection_count = boxes.shape[2]

        self.obj_boxes = []
        self.obj_classes = []
        self.obj_centers = []
        self.obj_contours = []

        for i in range(detection_count):
            box = boxes[0, 0, i]
            class_id = int(box[1])
            score = box[2]
            if class_id != 0 or score < self.detection_threshold:  # Only detect 'person' class
                continue

            color = self.colors[class_id]

            x = int(box[3] * frame_width)
            y = int(box[4] * frame_height)
            x2 = int(box[5] * frame_width)
            y2 = int(box[6] * frame_height)
            self.obj_boxes.append([x, y, x2, y2])

            cx = (x + x2) // 2
            cy = (y + y2) // 2
            self.obj_centers.append((cx, cy))

            self.obj_classes.append(class_id)

            mask = masks[i, class_id]
            roi_height, roi_width = y2 - y, x2 - x
            mask = cv2.resize(mask, (roi_width, roi_height))
            _, mask = cv2.threshold(mask, self.mask_threshold, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(np.array(mask, np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self.obj_contours.append(contours)

        return self.obj_boxes, self.obj_classes, self.obj_contours, self.obj_centers

    def draw_object_mask(self, bgr_frame):
        for box, class_id, contours in zip(self.obj_boxes, self.obj_classes, self.obj_contours):
            x, y, x2 = box[0], box[1], box[2]
            y2 = box[3]
            roi = bgr_frame[y: y2, x: x2]
            color = self.colors[class_id]

            roi_copy = np.zeros_like(roi)

            for cnt in contours:
                cv2.drawContours(roi, [cnt], -1, (int(color[0]), int(color[1]), int(color[2])), 3)
                cv2.fillPoly(roi_copy, [cnt], (int(color[0]), int(color[1]), int(color[2])))
                roi = cv2.addWeighted(roi, 1, roi_copy, 0.5, 0.0)
                bgr_frame[y: y2, x: x2] = roi
        return bgr_frame

    def draw_object_info(self, bgr_frame, depth_frame):
        for box, class_id, obj_center in zip(self.obj_boxes, self.obj_classes, self.obj_centers):
            x, y, x2, y2 = box

            color = self.colors[class_id]
            color = (int(color[0]), int(color[1]), int(color[2]))

            cx, cy = obj_center

            depth_mm = depth_frame[cy, cx]

            cv2.line(bgr_frame, (cx, y), (cx, y2), color, 1)
            cv2.line(bgr_frame, (x, cy), (x2, cy), color, 1)

            class_name = self.classes[class_id]
            cv2.rectangle(bgr_frame, (x, y), (x + 250, y + 70), color, -1)
            cv2.putText(bgr_frame, class_name.capitalize(), (x + 5, y + 25), 0, 0.8, (255, 255, 255), 2)
            cv2.putText(bgr_frame, "{} cm".format(depth_mm / 10), (x + 5, y + 60), 0, 1.0, (255, 255, 255), 2)
            cv2.rectangle(bgr_frame, (x, y), (x2, y2), color, 1)

        return bgr_frame
