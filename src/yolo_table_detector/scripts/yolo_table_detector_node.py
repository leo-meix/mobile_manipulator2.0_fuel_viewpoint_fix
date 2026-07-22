#!/usr/bin/env python3
"""
YOLO Table Detector Node (Instance Segmentation) — subscribes to Gazebo camera,
runs YOLO instance segmentation, publishes annotated images with masks + bboxes.

Subscribes:  /robot1_camera/color/image_raw  (sensor_msgs/Image)
Publishes:   /yolo_table_detector/annotated   (sensor_msgs/Image)  — bbox + mask overlay
             /yolo_table_detector/mask         (sensor_msgs/Image)  — binary mask only

Run:
    rosrun yolo_table_detector yolo_table_detector_node.py

Or with launch:
    roslaunch yolo_table_detector table_detector.launch
"""
import os
import sys

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

# Color palette for instance masks (BGR)
MASK_COLORS = [
    (0, 255, 0),    # green
    (255, 0, 0),    # blue
    (0, 0, 255),    # red
    (0, 255, 255),  # yellow
    (255, 0, 255),  # magenta
    (255, 255, 0),  # cyan
]


def draw_masks(image, masks_data, boxes_data, alpha=0.4):
    """Overlay segmentation masks onto an image with per-instance colors.

    Args:
        image: BGR numpy array (H, W, 3).
        masks_data: torch tensor (N, H, W) of float masks in [0, 1].
        boxes_data: torch tensor (N, 4) xyxy or None (used for contour fallback).
        alpha: blend strength for mask overlay.

    Returns:
        image with masks blended.
    """
    overlay = image.copy()
    h, w = image.shape[:2]
    n_masks = masks_data.shape[0]

    for i in range(n_masks):
        # Binary mask at original resolution
        mask_np = (masks_data[i].cpu().numpy() * 255).astype(np.uint8)
        if mask_np.shape[:2] != (h, w):
            mask_np = cv2.resize(mask_np, (w, h), interpolation=cv2.INTER_NEAREST)

        color = MASK_COLORS[i % len(MASK_COLORS)]

        # Color fill
        color_mask = np.zeros_like(image, dtype=np.uint8)
        color_mask[mask_np > 127] = color
        overlay = cv2.addWeighted(overlay, 1.0 - alpha, color_mask, alpha, 0)

        # Mask contour
        contours, _ = cv2.findContours(mask_np, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay, contours, -1, color, 2)

    return overlay


class YOLOTableDetector:
    """ROS node that runs YOLO instance segmentation on camera frames."""

    def __init__(self):
        rospy.init_node("yolo_table_detector", anonymous=False)

        # --- Parameters ---
        image_topic = rospy.get_param(
            "~image_topic", "/robot1_camera/color/image_raw"
        )
        default_weights = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "weights", "best.pt")
        )
        weights_path = rospy.get_param("~weights_path", default_weights)
        self.conf_threshold = rospy.get_param("~confidence_threshold", 0.1)
        self.show_display = rospy.get_param("~show_display", True)
        self.publish_annotated = rospy.get_param("~publish_annotated", True)
        self.publish_mask = rospy.get_param("~publish_mask", True)

        # --- Load YOLO model ---
        if not os.path.exists(weights_path):
            rospy.logerr("Model file not found: %s", weights_path)
            sys.exit(1)

        rospy.loginfo("Loading YOLO model from: %s", weights_path)
        self.model = YOLO(weights_path)
        rospy.loginfo("Model type: %s", self.model.task or "detect/segment")
        rospy.loginfo("Model classes: %s", self.model.names)
        rospy.loginfo("Confidence threshold: %.2f", self.conf_threshold)

        # --- CV Bridge ---
        self.bridge = CvBridge()

        # --- Publishers ---
        if self.publish_annotated:
            self.annotated_pub = rospy.Publisher(
                "/yolo_table_detector/annotated", Image, queue_size=1
            )
        if self.publish_mask:
            self.mask_pub = rospy.Publisher(
                "/yolo_table_detector/mask", Image, queue_size=1
            )

        # --- Subscriber ---
        self.sub = rospy.Subscriber(
            image_topic, Image, self.image_callback, queue_size=1
        )
        rospy.loginfo("Subscribed to: %s", image_topic)

        if self.show_display:
            rospy.loginfo("OpenCV display window enabled (press 'q' to close)")

        rospy.loginfo("YOLO Table Detector ready.")

    def image_callback(self, msg):
        """Process incoming camera frame."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("CV Bridge error: %s", e)
            return

        # --- YOLO inference ---
        results = self.model(cv_image, conf=self.conf_threshold, verbose=False)

        annotated = cv_image.copy()
        mask_overlay = cv_image.copy()
        binary_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)

        for result in results:
            boxes = result.boxes
            masks = result.masks

            if boxes is None or len(boxes) == 0:
                continue

            num_detections = len(boxes)

            # --- Draw segmentation masks ---
            if masks is not None and masks.data is not None and len(masks.data) > 0:
                mask_overlay = draw_masks(mask_overlay, masks.data, boxes.xyxy)
                # Build combined binary mask for publishing
                for i in range(len(masks.data)):
                    mask_np = (masks.data[i].cpu().numpy() * 255).astype(np.uint8)
                    h, w = cv_image.shape[:2]
                    if mask_np.shape[:2] != (h, w):
                        mask_np = cv2.resize(mask_np, (w, h), interpolation=cv2.INTER_NEAREST)
                    binary_mask = cv2.bitwise_or(binary_mask, mask_np)

            # --- Draw bounding boxes ---
            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self.model.names.get(cls_id, f"cls_{cls_id}")
                label_str = f"{label} {conf:.2f}"

                # Box color
                color = MASK_COLORS[i % len(MASK_COLORS)]

                # Label background
                (tw, th), _ = cv2.getTextSize(label_str, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(mask_overlay, (x1, y1 - th - 8), (x1 + tw + 4, y1),
                              color, -1)
                # Label text
                cv2.putText(mask_overlay, label_str, (x1 + 2, y1 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                # Bounding box
                cv2.rectangle(mask_overlay, (x1, y1), (x2, y2), color, 2)

            rospy.loginfo_throttle(2, "Detected %d instance(s): %s",
                                   num_detections,
                                   [(self.model.names.get(int(b.cls[0]), "?") +
                                     f"({float(b.conf[0]):.2f})")
                                    for b in boxes])

        # --- Publish annotated image (bbox + mask) ---
        if self.publish_annotated:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(mask_overlay, encoding="bgr8")
                annotated_msg.header = msg.header
                self.annotated_pub.publish(annotated_msg)
            except CvBridgeError as e:
                rospy.logerr("CV Bridge publish error: %s", e)

        # --- Publish binary mask ---
        if self.publish_mask:
            try:
                mask_msg = self.bridge.cv2_to_imgmsg(binary_mask, encoding="mono8")
                mask_msg.header = msg.header
                self.mask_pub.publish(mask_msg)
            except CvBridgeError as e:
                rospy.logerr("CV Bridge mask publish error: %s", e)

        # --- Show OpenCV window ---
        if self.show_display:
            cv2.imshow("YOLO Table Detector (segmentation)", mask_overlay)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                rospy.loginfo("User closed display window. Shutting down.")
                rospy.signal_shutdown("User quit")
                cv2.destroyAllWindows()

    def shutdown(self):
        """Cleanup on node shutdown."""
        if self.show_display:
            cv2.destroyAllWindows()
        rospy.loginfo("YOLO Table Detector stopped.")


if __name__ == "__main__":
    try:
        detector = YOLOTableDetector()
        rospy.on_shutdown(detector.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
