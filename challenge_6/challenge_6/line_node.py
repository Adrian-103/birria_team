import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np


class line_node(Node):
    """
    ROS2 node for center-line following on a three-line track.

    Pipeline
    --------
    1. Grayscale + Gaussian blur
    2. Adaptive or fixed binary threshold  →  white blobs on black background
    3. Morphological open/close            →  remove speckle noise
    4. ROI crop (bottom fraction)
    5. Connected-component analysis        →  one label per line blob
    6. Area filtering                      →  reject noise & shadows
    7. Sort blobs by X centroid            →  left / center / right
    8. EMA-filtered centroid of center blob →  stable position estimate
    9. error = setpoint_x - filtered_cx    →  published on /line_error

    Why connected components instead of Hough?
    -------------------------------------------
    HoughLinesP pools all three lines into a single mean X, making it
    impossible to isolate the center line. Connected components give you
    one centroid *per line*, so you can rank them by X and always pick
    the middle one regardless of how far the robot has drifted.

    Parameters (all tunable at runtime via ros2 param set)
    -------------------------------------------------------
    debug         (bool,  False) – show annotated OpenCV window
    alpha         (float, 0.35) – EMA smoothing: lower = smoother but laggier
    setpoint_x    (float, 0.0)  – horizontal offset from image centre [px].
                                   Positive moves setpoint right, negative left.
                                   Use this to compensate a physically off-centre
                                   camera.
    roi_start     (float, 0.55) – top of ROI as fraction of image height
    thresh_value  (int,   100)  – fixed threshold (0-255). Set to -1 to use
                                   adaptive (THRESH_OTSU), which adapts to
                                   changing light conditions automatically.
    min_area      (int,   800)  – minimum blob area to be considered a line [px²]
    max_area      (int,   80000)– maximum blob area (reject merged blobs / glare)
    """

    def __init__(self):
        super().__init__('line_node')

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self.error_pub = self.create_publisher(Float32, '/line_error', 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.bridge = CvBridge()

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('debug', False)
        self.declare_parameter('alpha', 0.35)
        self.declare_parameter('setpoint_x', 0.0)   # px offset from image centre
        self.declare_parameter('roi_start', 0.55)   # fraction of height
        self.declare_parameter('thresh_value', 100) # -1 → Otsu adaptive
        self.declare_parameter('min_area', 800)
        self.declare_parameter('max_area', 80000)

        self._load_params()

        # ── State ─────────────────────────────────────────────────────────────
        self.first_sample = True
        self.filtered_cx = 0.0

        self.get_logger().info('LineFollowerNode started. Waiting for images…')

    # ─────────────────────────────────────────────────────────────────────────
    def _load_params(self):
        """Cache all ROS parameters. Called once at init (and can be re-called
        if you extend this node with a parameter-event callback)."""
        self.debug       = self.get_parameter('debug').value
        self.alpha       = self.get_parameter('alpha').value
        self.setpoint_x  = self.get_parameter('setpoint_x').value
        self.roi_start   = self.get_parameter('roi_start').value
        self.thresh_value = self.get_parameter('thresh_value').value
        self.min_area    = self.get_parameter('min_area').value
        self.max_area    = self.get_parameter('max_area').value

    # ─────────────────────────────────────────────────────────────────────────
    def image_callback(self, msg: Image):
        # Re-read parameters every frame so you can tune live with ros2 param set
        self._load_params()

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]

        # ── 1. Pre-processing ─────────────────────────────────────────────────
        gray   = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gray, (5, 5), 1.4)

        # ── 2. Threshold → white lines on black background ───────────────────
        #   thresh_value == -1  →  Otsu (automatically picks the best threshold
        #                          based on the image histogram; works well when
        #                          lighting changes).
        #   thresh_value >= 0   →  fixed threshold (faster, predictable).
        #
        # THRESH_BINARY_INV is used when the track lines are DARKER than the
        # floor. If your lines are LIGHTER than the floor, swap to THRESH_BINARY.
        if self.thresh_value < 0:
            _, binary = cv.threshold(
                blurred, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)
        else:
            _, binary = cv.threshold(
                blurred, self.thresh_value, 255, cv.THRESH_BINARY_INV)

        # ── 3. Morphological cleanup ──────────────────────────────────────────
        # Opening (erode then dilate) removes thin speckle noise while
        # preserving the wider line blobs.
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
        cleaned = cv.morphologyEx(binary, cv.MORPH_OPEN, kernel, iterations=2)
        cleaned = cv.morphologyEx(cleaned, cv.MORPH_CLOSE, kernel, iterations=2)

        # ── 4. Region of Interest ─────────────────────────────────────────────
        # Keep only the bottom portion of the image: the robot only needs to
        # "see" the track immediately ahead of it, not the whole scene.
        roi_top = int(height * self.roi_start)
        roi = cleaned[roi_top:height, 0:width]

        # ── 5. Connected-Component Analysis ───────────────────────────────────
        # Each continuous white blob gets a unique integer label. This is the
        # key advantage over Hough: each line becomes its own object.
        num_labels, labels, stats, centroids = cv.connectedComponentsWithStats(
            roi, connectivity=8)

        # ── 6. Filter blobs by area ───────────────────────────────────────────
        # Reject label 0 (background). Keep only blobs in [min_area, max_area].
        valid_blobs = []
        for i in range(1, num_labels):
            area = stats[i, cv.CC_STAT_AREA]
            if self.min_area <= area <= self.max_area:
                cx, cy = centroids[i]
                valid_blobs.append({'cx': cx, 'cy': cy, 'area': area,
                                    'stats': stats[i]})

        if len(valid_blobs) == 0:
            self.get_logger().warn(
                'No valid blobs found. Check threshold and ROI parameters.')
            return

        # ── 7. Sort by X → identify left / center / right ────────────────────
        # Sorting by centroid X gives us lanes in left-to-right order.
        valid_blobs.sort(key=lambda b: b['cx'])

        if len(valid_blobs) >= 3:
            # Perfect case: all three lines visible — pick the middle one.
            center_blob = valid_blobs[len(valid_blobs) // 2]
            line_label = 'center (3+ blobs)'
        elif len(valid_blobs) == 2:
            # Two lines visible: the center line is between them. Since we
            # sorted by X, the true center line is blob index 1 — unless we've
            # drifted so far that one outer line left the frame, in which case
            # we fall back to the rightmost remaining blob (closest to center).
            # In practice, using index 1 (rightmost of two) works well when
            # the robot is roughly on track. Adjust if your camera placement
            # makes one side disappear first.
            center_blob = valid_blobs[1]
            line_label = 'fallback (2 blobs, using right)'
        else:
            # Only one blob visible: best-effort, use whatever we can see.
            center_blob = valid_blobs[0]
            line_label = 'fallback (1 blob)'

        raw_cx = center_blob['cx']

        # ── 8. Exponential Moving Average filter ──────────────────────────────
        # Smooths jitter between frames. alpha=1.0 → no smoothing (raw value).
        if self.first_sample:
            self.filtered_cx = raw_cx
            self.first_sample = False
        else:
            self.filtered_cx = (self.alpha * raw_cx
                                + (1.0 - self.alpha) * self.filtered_cx)

        # ── 9. Error calculation ──────────────────────────────────────────────
        # The setpoint is the image centre + the user-defined offset.
        # Positive error  → line is to the LEFT  of setpoint  → turn left
        # Negative error  → line is to the RIGHT of setpoint  → turn right
        image_center = width / 2.0
        setpoint     = image_center + self.setpoint_x
        error        = setpoint - self.filtered_cx

        err_msg = Float32()
        err_msg.data = float(error)
        self.error_pub.publish(err_msg)

        # ── Debug visualisation ───────────────────────────────────────────────
        if self.debug:
            debug_frame = frame.copy()
            y_offset = roi_top  # shift ROI coordinates back to full-frame space

            # Draw all valid blobs
            colors = {
                'left':   (0, 165, 255),  # orange
                'center': (0, 255, 0),    # green
                'right':  (0, 165, 255),  # orange
            }
            for idx, blob in enumerate(valid_blobs):
                x, y, w, h = (blob['stats'][cv.CC_STAT_LEFT],
                               blob['stats'][cv.CC_STAT_TOP],
                               blob['stats'][cv.CC_STAT_WIDTH],
                               blob['stats'][cv.CC_STAT_HEIGHT])
                color = (0, 255, 0) if blob is center_blob else (0, 165, 255)
                cv.rectangle(debug_frame,
                              (x, y + y_offset),
                              (x + w, y + y_offset + h),
                              color, 2)
                cv.circle(debug_frame,
                           (int(blob['cx']), int(blob['cy']) + y_offset),
                           5, color, -1)

            # Draw raw centroid (white) and filtered centroid (red)
            row = y_offset + roi.shape[0] // 2
            cv.circle(debug_frame, (int(raw_cx), row), 6, (255, 255, 255), -1)
            cv.circle(debug_frame, (int(self.filtered_cx), row), 8, (0, 0, 255), 2)

            # Draw setpoint line
            setpoint_px = int(image_center + self.setpoint_x)
            cv.line(debug_frame,
                     (setpoint_px, y_offset),
                     (setpoint_px, height),
                     (255, 0, 255), 1)

            # Draw ROI top boundary
            cv.line(debug_frame, (0, roi_top), (width, roi_top), (128, 128, 128), 1)

            # Overlay text
            cv.putText(debug_frame,
                        f'blobs={len(valid_blobs)}  mode={line_label}',
                        (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv.putText(debug_frame,
                        f'cx={self.filtered_cx:.1f}  sp={setpoint_px}  err={error:.1f}',
                        (10, 40), cv.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            cv.imshow('LineFollower debug', debug_frame)
            cv.waitKey(1)


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = line_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()