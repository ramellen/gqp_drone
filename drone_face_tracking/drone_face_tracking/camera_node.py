"""
camera_node.py
--------------
Captures frames from a camera and publishes them as sensor_msgs/Image
on /camera/image_raw. Used on real hardware (Pi 4 companion). In Gazebo
simulation, ros_gz_bridge handles this topic instead and this node is
not launched.

Backends supported:
  - picamera2 : Native Pi Camera Module via libcamera. Default on the Pi.
  - opencv    : cv2.VideoCapture for USB webcams, /dev/videoN devices,
                or HTTP/RTSP URLs. Used as a fallback on dev machines.

Backend selection:
  ROS parameter `camera_source` ∈ {'auto', 'picamera2', 'opencv'}
    'auto' (default) → picamera2 if importable, else opencv.

Other ROS parameters:
  frame_width    (int, default 640)
  frame_height   (int, default 480)
  fps            (int, default 30)
  opencv_source  (str, default '0')   only used when backend = opencv.
                                       int-coerced when possible — '0'
                                       becomes /dev/video0; URLs and
                                       device paths pass through unchanged.

Topic published:
  /camera/image_raw  (sensor_msgs/Image, bgr8)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

# picamera2 is only available on the Pi (libcamera + python3-picamera2).
# Import is guarded so the node still works on dev machines via OpenCV.
try:
    from picamera2 import Picamera2
    _PICAMERA2_AVAILABLE = True
except ImportError:
    _PICAMERA2_AVAILABLE = False


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        # --- Parameters ---
        self.declare_parameter('camera_source', 'auto')
        self.declare_parameter('frame_width',   640)
        self.declare_parameter('frame_height',  480)
        self.declare_parameter('fps',            30)
        self.declare_parameter('opencv_source', '0')

        src = self.get_parameter('camera_source').value
        self.W   = int(self.get_parameter('frame_width').value)
        self.H   = int(self.get_parameter('frame_height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.opencv_source = self.get_parameter('opencv_source').value

        # Resolve 'auto'
        if src == 'auto':
            src = 'picamera2' if _PICAMERA2_AVAILABLE else 'opencv'

        self.backend = src
        self.bridge = CvBridge()

        # --- Initialise selected backend ---
        if self.backend == 'picamera2':
            if not _PICAMERA2_AVAILABLE:
                self.get_logger().error(
                    "camera_source=picamera2 but picamera2 is not installed. "
                    "On the Pi: `sudo apt install python3-picamera2`."
                )
                raise RuntimeError("picamera2 not available")
            self._init_picamera2()
        elif self.backend == 'opencv':
            self._init_opencv()
        else:
            raise ValueError(
                f"Unknown camera_source: {src!r} "
                "(expected 'auto', 'picamera2', or 'opencv')"
            )

        # --- Publisher + capture timer ---
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

        self.get_logger().info(
            f"camera_node started: backend={self.backend}, "
            f"{self.W}x{self.H} @ {self.fps} Hz on /camera/image_raw"
        )

    # ──────────────────────────────────────────────────────────────────
    # Backend init
    # ──────────────────────────────────────────────────────────────────

    def _init_picamera2(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (self.W, self.H), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()

    def _init_opencv(self):
        # Coerce string source to int when it looks numeric
        # ('0' → 0 → /dev/video0). Leaves URLs and device paths alone.
        src = self.opencv_source
        try:
            src = int(src)
        except (TypeError, ValueError):
            pass

        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)

        if not self.cap.isOpened():
            self.get_logger().error(
                f"Could not open OpenCV source {src!r}. "
                "Check that the device exists or the URL is reachable."
            )

    # ──────────────────────────────────────────────────────────────────
    # Frame grab
    # ──────────────────────────────────────────────────────────────────

    def _read_frame(self):
        if self.backend == 'picamera2':
            # picamera2's "RGB888" format returns a HxWx3 numpy array
            # in R, G, B channel order. Convert to BGR for OpenCV / ROS
            # convention.
            #
            # If face detection looks blue-tinted on hardware (cheeks
            # appear blue, lips appear blue), the byte order is reversed
            # — swap to COLOR_BGR2RGB or drop the conversion entirely.
            frame_rgb = self.picam2.capture_array()
            return cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # opencv backend
        ret, frame = self.cap.read()
        return frame if ret else None

    def timer_callback(self):
        frame = self._read_frame()
        if frame is None:
            self.get_logger().warn('Failed to read frame from camera.')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        self.publisher_.publish(msg)

    # ──────────────────────────────────────────────────────────────────
    # Cleanup
    # ──────────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self.backend == 'picamera2' and hasattr(self, 'picam2'):
            try:
                self.picam2.stop()
            except Exception:
                pass
        elif self.backend == 'opencv' and hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
