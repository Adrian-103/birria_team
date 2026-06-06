#!/usr/bin/env python3
"""
line_follower_cv.py  --  ROS2 computer-vision line follower (differential robot)

Pipeline (per frame):
    /camera/image_raw (sensor_msgs/Image)
        -> CvBridge -> BGR
        -> grayscale -> Gaussian blur            (denoise)
        -> threshold (Otsu / adaptive / fixed)    (isolate the dark line, robust to lighting)
        -> trapezoidal ROI mask                   (look only at the road ahead)
        -> morphology open+close                  (kill specks, close gaps)
        -> Canny edges                            (clean thin edges for Hough)
        -> cv2.HoughLinesP                        (detect the two edges of the line)
        -> filter near-vertical segments, extrapolate each to an evaluation row,
           length-weighted average -> line x
        -> (optional) centroid fallback if Hough finds nothing
        -> error = line_x - roi_center_x

Publishes:
    /line_error   (std_msgs/Float32)  pixel offset of the line from ROI center.
                                       POSITIVE  = line is to the RIGHT of center.
                                       NEGATIVE  = line is to the LEFT of center.
                                       (set normalize_error=true for a ~[-1, 1] range)
    /line_status  (std_msgs/Bool)     True if a line is currently detected.
    <debug_image_topic>               annotated image for remote viewing (optional).

Every parameter below is dynamically reconfigurable at runtime with `ros2 param set`,
so you can tune the ROI and thresholds live while watching the debug stream.
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge


# --------------------------------------------------------------------------- #
#  Pure computer-vision core (no ROS dependency -> easy to unit-test)
# --------------------------------------------------------------------------- #
@dataclass
class DetectorParams:
    # ROI trapezoid, all as fractions of image size so it scales with resolution.
    roi_top_y: float = 0.55          # vertical start of the trapezoid (0=top, 1=bottom)
    roi_bottom_y: float = 1.0        # vertical end of the trapezoid
    roi_top_width: float = 0.45      # width of the (far) top edge, fraction of image width
    roi_bottom_width: float = 0.95   # width of the (near) bottom edge
    # Preprocessing
    gaussian_kernel: int = 5         # odd; 0/1 disables blur
    threshold_method: str = "otsu"   # "otsu" | "adaptive" | "fixed"
    binary_threshold: int = 70       # used when method == "fixed"
    adaptive_block_size: int = 31    # odd; used when method == "adaptive"
    adaptive_c: int = 10             # used when method == "adaptive"
    morph_kernel: int = 5            # 0/1/2 disables morphology
    morph_iterations: int = 1
    # Canny
    canny_low: int = 50
    canny_high: int = 150
    # Hough (probabilistic Hough line transform)
    hough_rho: float = 1.0
    hough_theta_deg: float = 1.0
    hough_threshold: int = 25
    hough_min_line_length: int = 30
    hough_max_line_gap: int = 30
    # Line filtering / decision
    max_line_angle_deg: float = 45.0  # reject segments deviating more than this from vertical
    min_valid_segments: int = 1       # how many good segments are needed to call it a line
    eval_line_frac: float = 0.9       # where inside the ROI we measure x (0=top, 1=bottom/near)
    # Robustness fallback
    use_centroid_fallback: bool = True
    min_blob_area: int = 400          # min white-mask area for the centroid fallback
    # Output
    normalize_error: bool = False     # divide error by half image width -> ~[-1, 1]


class LineDetector:
    """Detect a dark line on a light background and return its offset from center."""

    def __init__(self, params: DetectorParams):
        self.p = params

    def _roi_polygon(self, w, h):
        cx = w / 2.0
        top_y = self.p.roi_top_y * h
        bot_y = self.p.roi_bottom_y * h
        top_hw = self.p.roi_top_width * w / 2.0
        bot_hw = self.p.roi_bottom_width * w / 2.0
        pts = np.array([
            [cx - top_hw, top_y],
            [cx + top_hw, top_y],
            [cx + bot_hw, bot_y],
            [cx - bot_hw, bot_y],
        ], dtype=np.int32)
        return pts, top_y, bot_y, cx

    def _binarize(self, gray):
        m = self.p.threshold_method
        if m == "otsu":
            _, b = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        elif m == "adaptive":
            bs = self.p.adaptive_block_size
            if bs % 2 == 0:
                bs += 1
            bs = max(bs, 3)
            b = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                      cv2.THRESH_BINARY_INV, bs, self.p.adaptive_c)
        else:  # fixed
            _, b = cv2.threshold(gray, self.p.binary_threshold, 255, cv2.THRESH_BINARY_INV)
        return b

    def process(self, bgr):
        h, w = bgr.shape[:2]
        pts, top_y, bot_y, cx = self._roi_polygon(w, h)
        roi_center_x = cx

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        k = self.p.gaussian_kernel
        if k >= 3:
            if k % 2 == 0:
                k += 1
            gray = cv2.GaussianBlur(gray, (k, k), 0)

        binary = self._binarize(gray)

        # Restrict everything to the trapezoidal ROI.
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 255)
        binary = cv2.bitwise_and(binary, mask)

        # Remove specks (open) then close small gaps in the line (close).
        mk = self.p.morph_kernel
        if mk >= 3:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (mk, mk))
            it = max(1, self.p.morph_iterations)
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=it)
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=it)

        edges = cv2.Canny(binary, self.p.canny_low, self.p.canny_high)
        lines = cv2.HoughLinesP(
            edges, self.p.hough_rho, math.radians(self.p.hough_theta_deg),
            self.p.hough_threshold,
            minLineLength=self.p.hough_min_line_length,
            maxLineGap=self.p.hough_max_line_gap,
        )

        # Evaluate the line position at one representative row inside the ROI.
        eval_y = top_y + self.p.eval_line_frac * (bot_y - top_y)

        valid_segments, x_estimates, weights = [], [], []
        if lines is not None:
            for seg in lines:
                x1, y1, x2, y2 = seg[0]
                dx, dy = float(x2 - x1), float(y2 - y1)
                if dy == 0:                      # horizontal -> not part of a forward line
                    continue
                angle_from_vertical = math.degrees(math.atan2(abs(dx), abs(dy)))
                if angle_from_vertical > self.p.max_line_angle_deg:
                    continue
                x_at_eval = x1 + (dx / dy) * (eval_y - y1)   # extrapolate to eval row
                if x_at_eval < -w or x_at_eval > 2 * w:      # absurd extrapolation -> drop
                    continue
                valid_segments.append((int(x1), int(y1), int(x2), int(y2)))
                x_estimates.append(x_at_eval)
                weights.append(math.hypot(dx, dy))           # longer = more trustworthy

        detected, line_x, method = False, None, "none"
        if len(valid_segments) >= self.p.min_valid_segments and x_estimates:
            line_x = float(np.average(x_estimates, weights=weights))
            detected, method = True, "hough"
        elif self.p.use_centroid_fallback:
            M = cv2.moments(binary, binaryImage=True)
            if M["m00"] > self.p.min_blob_area:
                line_x = float(M["m10"] / M["m00"])
                detected, method = True, "centroid"

        error = 0.0
        if detected and line_x is not None:
            error = float(line_x - roi_center_x)
            if self.p.normalize_error:
                error /= (w / 2.0)

        return {
            "detected": detected, "error": error, "line_x": line_x,
            "roi_center_x": roi_center_x, "eval_y": eval_y, "method": method,
            "num_segments": len(valid_segments), "roi_pts": pts,
            "all_lines": lines, "valid_segments": valid_segments,
            "binary": binary,
        }


def draw_debug(bgr, r, show_mask=True):
    """Render ROI, Hough segments, detected center and error onto a copy of the frame."""
    dbg = bgr.copy()
    h, w = dbg.shape[:2]
    cxi = int(round(r["roi_center_x"]))

    cv2.polylines(dbg, [r["roi_pts"]], True, (0, 255, 255), 2)        # ROI = yellow
    cv2.line(dbg, (cxi, 0), (cxi, h), (255, 0, 0), 1)                 # ROI center = blue

    if r["all_lines"] is not None:                                   # all Hough = grey
        for seg in r["all_lines"]:
            x1, y1, x2, y2 = seg[0]
            cv2.line(dbg, (x1, y1), (x2, y2), (130, 130, 130), 1)
    for (x1, y1, x2, y2) in r["valid_segments"]:                      # accepted = green
        cv2.line(dbg, (x1, y1), (x2, y2), (0, 255, 0), 2)

    if r["detected"] and r["line_x"] is not None:                    # detection = red
        lx, ey = int(round(r["line_x"])), int(round(r["eval_y"]))
        cv2.line(dbg, (lx, 0), (lx, h), (0, 0, 255), 1)
        cv2.line(dbg, (cxi, ey), (lx, ey), (0, 0, 255), 2)           # the error itself
        cv2.circle(dbg, (lx, ey), 6, (0, 0, 255), -1)

    status = "LINE" if r["detected"] else "NO LINE"
    color = (0, 255, 0) if r["detected"] else (0, 0, 255)
    cv2.putText(dbg, f"{status} [{r['method']}]", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    cv2.putText(dbg, f"error={r['error']:.1f}  segs={r['num_segments']}", (10, 56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    if show_mask:                                                    # binary mask thumbnail
        th, tw = h // 4, w // 4
        thumb = cv2.resize(cv2.cvtColor(r["binary"], cv2.COLOR_GRAY2BGR), (tw, th))
        dbg[0:th, w - tw:w] = thumb
        cv2.rectangle(dbg, (w - tw, 0), (w - 1, th), (0, 255, 255), 1)
    return dbg


# --------------------------------------------------------------------------- #
#  ROS2 node
# --------------------------------------------------------------------------- #
class LineFollowerNode(Node):

    # (name, default) for every detector parameter, declared verbatim as ROS params.
    _DETECTOR_PARAMS = [
        ("roi_top_y", 0.55), ("roi_bottom_y", 1.0),
        ("roi_top_width", 0.45), ("roi_bottom_width", 0.95),
        ("gaussian_kernel", 5), ("threshold_method", "otsu"),
        ("binary_threshold", 70), ("adaptive_block_size", 31), ("adaptive_c", 10),
        ("morph_kernel", 5), ("morph_iterations", 1),
        ("canny_low", 50), ("canny_high", 150),
        ("hough_rho", 1.0), ("hough_theta_deg", 1.0), ("hough_threshold", 25),
        ("hough_min_line_length", 30), ("hough_max_line_gap", 30),
        ("max_line_angle_deg", 45.0), ("min_valid_segments", 1),
        ("eval_line_frac", 0.9),
        ("use_centroid_fallback", True), ("min_blob_area", 400),
        ("normalize_error", False),
    ]

    def __init__(self):
        super().__init__("line_follower_cv")

        # ---- topics & infrastructure params ----
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("error_topic", "/line_error")
        self.declare_parameter("status_topic", "/line_status")
        self.declare_parameter("debug_image_topic", "/line_follower/debug_image")
        self.declare_parameter("debug", False)
        self.declare_parameter("debug_compressed", True)   # JPEG over the wire = WiFi friendly
        self.declare_parameter("debug_resize_width", 480)  # 0 = keep full size
        self.declare_parameter("debug_publish_every_n", 2) # throttle debug stream
        self.declare_parameter("debug_show_mask", True)
        self.declare_parameter("hold_error_on_loss", True) # keep last error when line is lost

        # ---- detector params ----
        for name, default in self._DETECTOR_PARAMS:
            self.declare_parameter(name, default)

        self._params_dirty = True
        self._detector = LineDetector(DetectorParams())
        self._sync_detector_params()
        self.add_on_set_parameters_callback(self._on_set_params)

        # ---- pub / sub ----
        self.bridge = CvBridge()
        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)

        image_topic = self.get_parameter("image_topic").value
        self.sub = self.create_subscription(
            Image, image_topic, self._image_cb, qos_profile_sensor_data)

        self.pub_error = self.create_publisher(
            Float32, self.get_parameter("error_topic").value, reliable)
        self.pub_status = self.create_publisher(
            Bool, self.get_parameter("status_topic").value, reliable)

        self._debug_compressed = self.get_parameter("debug_compressed").value
        debug_topic = self.get_parameter("debug_image_topic").value
        if self._debug_compressed:
            self.pub_debug = self.create_publisher(
                CompressedImage, debug_topic + "/compressed", qos_profile_sensor_data)
        else:
            self.pub_debug = self.create_publisher(
                Image, debug_topic, qos_profile_sensor_data)

        self._last_error = 0.0
        self._frame_count = 0
        self.get_logger().info(f"line_follower_cv up. Subscribing to '{image_topic}'.")
        self.get_logger().info(
            "error sign: + means line is RIGHT of center, - means LEFT of center.")

    # ---- dynamic reconfigure ----
    def _on_set_params(self, params):
        # Values are applied by rclpy only after we return success; rebuild on next frame.
        self._params_dirty = True
        return SetParametersResult(successful=True)

    def _sync_detector_params(self):
        kwargs = {name: self.get_parameter(name).value for name, _ in self._DETECTOR_PARAMS}
        self._detector.p = DetectorParams(**kwargs)
        self._params_dirty = False

    # ---- main loop ----
    def _image_cb(self, msg: Image):
        if self._params_dirty:
            self._sync_detector_params()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        result = self._detector.process(frame)

        # /line_status -- always published
        self.pub_status.publish(Bool(data=bool(result["detected"])))

        # /line_error
        if result["detected"]:
            self._last_error = result["error"]
            self.pub_error.publish(Float32(data=float(result["error"])))
        elif self.get_parameter("hold_error_on_loss").value:
            # Keep steering toward where the line last was (helps re-acquire it).
            self.pub_error.publish(Float32(data=float(self._last_error)))
        else:
            self.pub_error.publish(Float32(data=0.0))

        # debug image
        if self.get_parameter("debug").value:
            self._publish_debug(frame, result, msg.header)

    def _publish_debug(self, frame, result, header):
        every = max(1, int(self.get_parameter("debug_publish_every_n").value))
        self._frame_count += 1
        if self._frame_count % every != 0:
            return

        dbg = draw_debug(frame, result, self.get_parameter("debug_show_mask").value)

        width = int(self.get_parameter("debug_resize_width").value)
        if width > 0 and dbg.shape[1] > width:
            scale = width / dbg.shape[1]
            dbg = cv2.resize(dbg, (width, int(dbg.shape[0] * scale)))

        if self._debug_compressed:
            ok, buf = cv2.imencode(".jpg", dbg, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if not ok:
                return
            out = CompressedImage()
            out.header = header
            out.format = "jpeg"
            out.data = buf.tobytes()
            self.pub_debug.publish(out)
        else:
            out = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
            out.header = header
            self.pub_debug.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
