"""
face_detect_node.py
-------------------
Subscribes to /camera/image_raw, runs face detection, and publishes
the normalised (x, y) error between the detected face centre and the
frame centre as a geometry_msgs/Point on /face_tracking/error.

  error.x  : horizontal offset  (-1.0 = far left,  +1.0 = far right)
  error.y  : vertical offset    (-1.0 = far top,   +1.0 = far bottom)
  error.z  : normalised face area (0.0–1.0); used by flight_ctrl to
              judge distance — larger area → drone is too close.

Topics subscribed:
  /camera/image_raw          (sensor_msgs/Image)

Topics published:
  /face_tracking/error       (geometry_msgs/Point)
  /face_tracking/debug_image (sensor_msgs/Image)   ← annotated frame

The node uses OpenCV's YuNet detector (fast, accurate, ships with
OpenCV 4.8+). Falls back to Haar cascades if the model file is absent.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import os


# Path to YuNet model — download once and place here, or leave blank to use Haar cascade fallback
YUNET_MODEL_PATH = os.path.expanduser('~/models/face_detection_yunet_2023mar.onnx')

# Haar cascade is always available as a fallback
HAAR_CASCADE_PATH = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'

# Minimum detection confidence for YuNet (0–1).
# Lowered from 0.7 → 0.5 because rendered Gazebo PBR textures produce weaker
# gradient responses than real photos, causing under-confidence.
YUNET_CONFIDENCE = 0.5


class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')

        # --- Subscribers ---
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # --- Publishers ---
        self.error_pub = self.create_publisher(Point, '/face_tracking/error', 10)
        self.debug_pub = self.create_publisher(Image, '/face_tracking/debug_image', 10)

        self.bridge = CvBridge()
        self.detector = self._init_detector()

        self.get_logger().info('Face detection node started.')

    # ------------------------------------------------------------------
    # Detector initialisation — YuNet preferred, Haar cascade fallback
    # ------------------------------------------------------------------
    def _init_detector(self):
        if os.path.exists(YUNET_MODEL_PATH):
            detector = cv2.FaceDetectorYN.create(
                YUNET_MODEL_PATH,
                '',
                (320, 240),   # match actual camera resolution (was 640x480)
                score_threshold=YUNET_CONFIDENCE,
                nms_threshold=0.3,
                top_k=1
            )
            self.detector_type = 'yunet'
            self.get_logger().info(f'Using YuNet detector: {YUNET_MODEL_PATH}')
        else:
            detector = cv2.CascadeClassifier(HAAR_CASCADE_PATH)
            self.detector_type = 'haar'
            self.get_logger().warn(
                'YuNet model not found — using Haar cascade (less accurate). '
                f'Download the model to: {YUNET_MODEL_PATH}'
            )
        return detector

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        face_box = self._detect_face(frame, w, h)

        debug_frame = frame.copy()
        error_msg   = Point()

        if face_box is not None:
            fx, fy, fw, fh = face_box

            # Centre of face in pixel coords
            cx = fx + fw / 2.0
            cy = fy + fh / 2.0
            self.get_logger().info(f'Face detected at xy coordinates: ({cx},{cy})', throttle_duration_sec=10)

            # Normalise to [-1, 1] relative to frame centre
            error_msg.x = (cx - w / 2.0) / (w / 2.0)
            error_msg.y = (cy - h / 2.0) / (h / 2.0)

            # Normalised face area — proxy for distance
            error_msg.z = (fw * fh) / (w * h)

            # Annotate debug image
            cv2.rectangle(debug_frame, (fx, fy), (fx + fw, fy + fh), (0, 255, 0), 2)
            cv2.circle(debug_frame, (int(cx), int(cy)), 5, (0, 255, 0), -1)
            cv2.putText(
                debug_frame,
                f'err x:{error_msg.x:+.2f} y:{error_msg.y:+.2f}',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
            )
        else:
            # No face — publish zero error so the drone hovers in place
            error_msg.x = 0.0
            error_msg.y = 0.0
            error_msg.z = 0.0
            cv2.putText(
                debug_frame, 'No face detected',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
            )

        self.error_pub.publish(error_msg)
        self.debug_pub.publish(
            self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
        )

    # ------------------------------------------------------------------
    # Detector dispatch
    # ------------------------------------------------------------------
    def _detect_face(self, frame, w, h):
        """Returns (x, y, w, h) of the largest/most confident face, or None."""
        if self.detector_type == 'yunet':
            return self._detect_yunet(frame, w, h)
        else:
            return self._detect_haar(frame)

    def _detect_yunet(self, frame, w, h):
        self.detector.setInputSize((w, h))
        _, faces = self.detector.detect(frame)
        if faces is None or len(faces) == 0:
            return None
        # faces[i] = [x, y, w, h, ...landmarks..., confidence]
        best = faces[0]
        return (int(best[0]), int(best[1]), int(best[2]), int(best[3]))

    def _detect_haar(self, frame):
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # minSize reduced from (60,60) → (20,20): at 320×240 with the drone
        # hovering 3 m from the face_target, the rendered head is only ~48×40 px
        # in frame — the old threshold was silently discarding every detection.
        # minNeighbors reduced from 5 → 3: rendered textures produce fewer
        # overlapping detections than real photos, so we need a looser merge.
        faces = self.detector.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=3, minSize=(20, 20)
        )
        if len(faces) == 0:
            return None
        # Pick the largest face
        areas = [w * h for (_, _, w, h) in faces]
        x, y, w, h = faces[np.argmax(areas)]
        return (int(x), int(y), int(w), int(h))


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
