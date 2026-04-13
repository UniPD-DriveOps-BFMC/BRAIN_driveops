"""
traffic_sign_node.py — BFMC Traffic Sign Detection
No cv_bridge dependency — uses raw NumPy conversion for ROS2 Image messages.
Three modes: SIM (ROS2 topic) | MOCK (webcam/video) | OAK-D Pro (depthai)
"""

import os
import json
import time
import numpy as np
import cv2
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

_PACKAGE_DIR   = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_MODEL = os.path.join(_PACKAGE_DIR, 'best.onnx')

CLASS_NAMES = [
    'stop', 'parking', 'priority', 'roundabout',
    'one_way', 'no_entry', 'exit_highway',
    'entrance_highway', 'crosswalk',
]

CLASS_COLORS_BGR = [
    (0,   0,   220),  # stop
    (0,   165, 255),  # parking
    (0,   200,   0),  # priority
    (200,   0, 200),  # roundabout
    (0,   200, 200),  # one_way
    (0,     0, 160),  # no_entry
    (200,  80,   0),  # exit_highway
    (200, 130,   0),  # entrance_highway
    (50,  200,  50),  # crosswalk
]

_DEFAULT_TRIGGER_DIST = {
    'stop':             0.60,
    'parking':          0.80,
    'priority':         0.60,
    'roundabout':       0.70,
    'one_way':          0.70,
    'no_entry':         0.80,
    'exit_highway':     0.60,
    'entrance_highway': 0.80,
    'crosswalk':        0.50,
}

_DEFAULT_COOLDOWN = {
    'stop':             5.0,
    'parking':          8.0,
    'priority':         4.0,
    'roundabout':       5.0,
    'one_way':          5.0,
    'no_entry':         5.0,
    'exit_highway':     8.0,
    'entrance_highway': 8.0,
    'crosswalk':        3.0,
}

# Supported ROS2 Image encodings → NumPy dtype + OpenCV conversion
_ENCODING_MAP = {
    'rgb8':    (np.uint8,  cv2.COLOR_RGB2BGR),
    'bgr8':    (np.uint8,  None),
    'rgba8':   (np.uint8,  cv2.COLOR_RGBA2BGR),
    'bgra8':   (np.uint8,  cv2.COLOR_BGRA2BGR),
    'mono8':   (np.uint8,  cv2.COLOR_GRAY2BGR),
    'mono16':  (np.uint16, cv2.COLOR_GRAY2BGR),
    '8UC3':    (np.uint8,  None),
    '8UC4':    (np.uint8,  cv2.COLOR_BGRA2BGR),
}


def imgmsg_to_cv2(msg: Image) -> np.ndarray:
    """
    Convert a ROS2 sensor_msgs/Image to a BGR OpenCV frame.
    Pure NumPy — no cv_bridge required.
    """
    enc   = msg.encoding.lower()
    dtype, cvt = _ENCODING_MAP.get(enc, (np.uint8, None))

    # Determine number of channels from step and width
    channels = msg.step // (msg.width * np.dtype(dtype).itemsize)
    channels = max(channels, 1)

    arr = np.frombuffer(msg.data, dtype=dtype).reshape(
        (msg.height, msg.width, channels) if channels > 1
        else (msg.height, msg.width)
    )

    if cvt is not None:
        arr = cv2.cvtColor(arr, cvt)
    elif arr.ndim == 2:
        arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)

    return arr.copy()   # writable copy


def cv2_to_imgmsg(frame: np.ndarray) -> Image:
    """
    Convert a BGR OpenCV frame to a ROS2 sensor_msgs/Image.
    Pure NumPy — no cv_bridge required.
    """
    msg          = Image()
    msg.height   = frame.shape[0]
    msg.width    = frame.shape[1]
    msg.encoding = 'bgr8'
    msg.step     = frame.shape[1] * frame.shape[2]
    msg.data     = frame.tobytes()
    return msg


class TrafficSignNode(Node):

    def __init__(self):
        super().__init__('traffic_sign_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('model_path',       '')
        self.declare_parameter('detection_topic',  '/traffic_sign/detection')
        self.declare_parameter('image_topic',      '/traffic_sign/image')
        self.declare_parameter('conf_threshold',   0.50)
        self.declare_parameter('iou_threshold',    0.40)
        self.declare_parameter('imgsz',            640)
        self.declare_parameter('depth_min_mm',     100)
        self.declare_parameter('depth_max_mm',     2000)
        self.declare_parameter('debug_view',       True)
        self.declare_parameter('use_sim',          False)
        self.declare_parameter('sim_image_topic',  '/oak/rgb/image_raw')
        self.declare_parameter('use_mock_camera',  False)
        self.declare_parameter('mock_source',      '0')

        for cls in CLASS_NAMES:
            self.declare_parameter(
                f'trigger_dist_{cls}', _DEFAULT_TRIGGER_DIST[cls])
            self.declare_parameter(
                f'cooldown_{cls}', _DEFAULT_COOLDOWN[cls])

        # ── Read ──────────────────────────────────────────────────────────────
        model_path          = self.get_parameter('model_path').value \
                              or _DEFAULT_MODEL
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold  = self.get_parameter('iou_threshold').value
        self.imgsz          = self.get_parameter('imgsz').value
        self.depth_min_mm   = self.get_parameter('depth_min_mm').value
        self.depth_max_mm   = self.get_parameter('depth_max_mm').value
        self.debug_view     = self.get_parameter('debug_view').value
        self.use_sim        = self.get_parameter('use_sim').value
        self.sim_topic      = self.get_parameter('sim_image_topic').value
        self.use_mock       = self.get_parameter('use_mock_camera').value
        self.mock_source    = self.get_parameter('mock_source').value
        detection_topic     = self.get_parameter('detection_topic').value
        image_topic         = self.get_parameter('image_topic').value

        self.trigger_dist = {
            cls: self.get_parameter(f'trigger_dist_{cls}').value
            for cls in CLASS_NAMES
        }
        self.cooldown = {
            cls: self.get_parameter(f'cooldown_{cls}').value
            for cls in CLASS_NAMES
        }

        # ── ONNX Runtime ──────────────────────────────────────────────────────
        if not os.path.isfile(model_path):
            self.get_logger().error(f'ONNX model not found: {model_path}')
            raise FileNotFoundError(model_path)

        self.session    = ort.InferenceSession(
            model_path, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info(
            f'ONNX loaded: {os.path.basename(model_path)}'
        )

        # ── Publishers (no cv_bridge — use raw imgmsg) ────────────────────────
        self.det_pub = self.create_publisher(String, detection_topic, 10)
        self.img_pub = self.create_publisher(Image,  image_topic,     10)

        self.last_trigger: dict[str, float] = {cls: 0.0 for cls in CLASS_NAMES}
        self._frame  = None
        self._cap    = None
        self._device = None

        # ── Mode selection ────────────────────────────────────────────────────
        if self.use_sim:
            self.create_subscription(
                Image, self.sim_topic, self._sim_image_cb, 10)
            self.get_logger().info(
                f'SIM mode — subscribing to {self.sim_topic}')

        elif self.use_mock:
            src = int(self.mock_source) if self.mock_source.isdigit() \
                  else self.mock_source
            self._cap = cv2.VideoCapture(src)
            if not self._cap.isOpened():
                raise RuntimeError(
                    f'Cannot open mock source: {self.mock_source}')
            self.get_logger().info(f'MOCK mode — source: {self.mock_source}')

        else:
            self._device, self._q_rgb, self._q_disp, \
                self._max_disp = self._build_oak_pipeline()
            self.get_logger().info('OAK-D Pro mode')

        self.create_timer(1.0 / 20.0, self._cb)

        mode = ('SIM'  if self.use_sim  else
                'MOCK' if self.use_mock else 'OAK')
        self.get_logger().info(
            f'TrafficSignNode ready [{mode}]\n'
            f'  model     → {model_path}\n'
            f'  detection → {detection_topic}\n'
            f'  image out → {image_topic}\n'
            f'  debug     → {self.debug_view}'
        )
        for cls in CLASS_NAMES:
            self.get_logger().info(
                f'  {cls:<20} trigger={self.trigger_dist[cls]:.2f}m '
                f'cooldown={self.cooldown[cls]:.1f}s'
            )

    # ── Simulator image callback — pure NumPy, no cv_bridge ───────────────────
    def _sim_image_cb(self, msg: Image) -> None:
        try:
            self._frame = imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f'imgmsg_to_cv2 error: {e}')

    # ── OAK-D Pro pipeline ────────────────────────────────────────────────────
    def _build_oak_pipeline(self):
        import depthai as dai

        pipeline = dai.Pipeline()

        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(self.imgsz, self.imgsz)
        cam.setResolution(
            dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(20)

        mono_l = pipeline.create(dai.node.MonoCamera)
        mono_l.setResolution(
            dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_l.setBoardSocket(dai.CameraBoardSocket.CAM_B)

        mono_r = pipeline.create(dai.node.MonoCamera)
        mono_r.setResolution(
            dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_r.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(
            dai.node.StereoDepth.PresetType.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(self.imgsz, self.imgsz)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        mono_l.out.link(stereo.left)
        mono_r.out.link(stereo.right)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName('rgb')
        cam.preview.link(xout_rgb.input)

        xout_disp = pipeline.create(dai.node.XLinkOut)
        xout_disp.setStreamName('disp')
        stereo.disparity.link(xout_disp.input)

        device   = dai.Device(pipeline)
        q_rgb    = device.getOutputQueue('rgb',  maxSize=2, blocking=False)
        q_disp   = device.getOutputQueue('disp', maxSize=2, blocking=False)
        max_disp = stereo.initialConfig.getMaxDisparity()

        return device, q_rgb, q_disp, max_disp

    # ── Preprocess ────────────────────────────────────────────────────────────
    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        img  = cv2.resize(frame, (self.imgsz, self.imgsz))
        img  = img.astype(np.float32) / 255.0
        blob = np.transpose(img, (2, 0, 1))[np.newaxis, ...]
        return np.ascontiguousarray(blob)

    # ── YOLOv8 postprocess ────────────────────────────────────────────────────
    def _postprocess(self, output: np.ndarray, orig_h: int, orig_w: int):
        pred       = output[0].T
        boxes      = pred[:, :4]
        class_conf = pred[:, 4:]
        scores     = np.max(class_conf, axis=1)
        cls_ids    = np.argmax(class_conf, axis=1)

        mask    = scores >= self.conf_threshold
        boxes   = boxes[mask]
        scores  = scores[mask]
        cls_ids = cls_ids[mask]

        if len(boxes) == 0:
            return []

        cx, cy, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        x1 = (cx - w / 2) * orig_w
        y1 = (cy - h / 2) * orig_h
        x2 = (cx + w / 2) * orig_w
        y2 = (cy + h / 2) * orig_h

        detections = []
        for cls_id in np.unique(cls_ids):
            idx  = np.where(cls_ids == cls_id)[0]
            b    = np.stack([x1[idx], y1[idx], x2[idx], y2[idx]], axis=1)
            s    = scores[idx]
            keep = cv2.dnn.NMSBoxes(
                b.tolist(), s.tolist(),
                self.conf_threshold, self.iou_threshold
            )
            for k in keep:
                ki = k[0] if isinstance(k, (list, tuple, np.ndarray)) else k
                detections.append({
                    'cls':  int(cls_ids[idx[ki]]),
                    'conf': float(scores[idx[ki]]),
                    'x1':   int(np.clip(x1[idx[ki]], 0, orig_w)),
                    'y1':   int(np.clip(y1[idx[ki]], 0, orig_h)),
                    'x2':   int(np.clip(x2[idx[ki]], 0, orig_w)),
                    'y2':   int(np.clip(y2[idx[ki]], 0, orig_h)),
                })
        return detections

    # ── Distance (OAK mode only) ───────────────────────────────────────────────
    def _get_distance_m(self, disp, x1, y1, x2, y2) -> float:
        if disp is None:
            return -1.0
        BASELINE_MM = 75.0
        FOCAL_PX    = 880.0
        cx  = (x1 + x2) // 2
        cy  = (y1 + y2) // 2
        rw  = max(1, (x2 - x1) // 4)
        rh  = max(1, (y2 - y1) // 4)
        roi = disp[max(0, cy-rh):cy+rh, max(0, cx-rw):cx+rw]
        if roi.size == 0:
            return -1.0
        med = np.median(roi[roi > 0])
        if med <= 0:
            return -1.0
        depth_mm = (BASELINE_MM * FOCAL_PX) / med
        if not (self.depth_min_mm < depth_mm < self.depth_max_mm):
            return -1.0
        return depth_mm / 1000.0

    # ── Publish detection ─────────────────────────────────────────────────────
    def _publish_detection(self, class_name, dist_m, conf):
        payload  = json.dumps({
            'sign':       class_name,
            #'distance_m': round(dist_m, 3) if dist_m > 0 else None,
            'confidence': round(conf, 3),
        })
        msg      = String()
        msg.data = payload
        self.det_pub.publish(msg)
        self.get_logger().info(f'TRIGGER → {payload}')

    # ── 20 Hz callback ────────────────────────────────────────────────────────
    def _cb(self) -> None:
        disp = None

        if self.use_sim:
            frame = self._frame
        elif self.use_mock:
            ret, frame = self._cap.read()
            if not ret:
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self._cap.read()
            frame = frame if ret else None
        else:
            in_rgb  = self._q_rgb.tryGet()
            in_disp = self._q_disp.tryGet()
            frame   = in_rgb.getCvFrame()  if in_rgb  else None
            disp    = in_disp.getFrame()   if in_disp else None

        if frame is None:
            return

        H, W = frame.shape[:2]

        output = self.session.run(
            None, {self.input_name: self._preprocess(frame)})
        dets   = self._postprocess(output[0], H, W)

        for det in dets:
            det['dist_m'] = self._get_distance_m(
                disp, det['x1'], det['y1'], det['x2'], det['y2'])

        if not self.use_sim and not self.use_mock:
            dets = [d for d in dets if d['dist_m'] > 0]
            dets.sort(key=lambda d: d['dist_m'])

        # Draw
        annotated = frame.copy()
        mode_str  = 'SIM' if self.use_sim else ('MOCK' if self.use_mock else 'OAK')
        cv2.putText(annotated, f'[{mode_str}] {len(dets)} sign(s)',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200,200,200), 2)

        for i, det in enumerate(dets):
            cls_id     = det['cls']
            class_name = CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) \
                         else f'cls_{cls_id}'
            color      = CLASS_COLORS_BGR[cls_id % len(CLASS_COLORS_BGR)]
            x1, y1, x2, y2 = det['x1'], det['y1'], det['x2'], det['y2']

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color,
                          3 if i == 0 else 1)
            if i == 0:
                cv2.rectangle(annotated,
                              (x1-3, y1-3), (x2+3, y2+3), (255,255,255), 1)

            dist_str = f"{det['dist_m']:.2f}m" if det['dist_m'] > 0 else 'n/a'
            label    = f"{class_name} {dist_str} ({det['conf']:.0%})"
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            cv2.rectangle(annotated,
                          (x1, y1-th-10), (x1+tw+4, y1), color, -1)
            cv2.putText(annotated, label, (x1+2, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 1)

        # Trigger
        now = time.time()
        if dets:
            det        = dets[0]
            cls_id     = det['cls']
            class_name = CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) \
                         else f'cls_{cls_id}'
            dist_m     = det['dist_m']
            conf       = det['conf']
            trig_dist  = self.trigger_dist.get(class_name, 0.60)
            trig_cool  = self.cooldown.get(class_name, 5.0)

            distance_ok = (dist_m > 0 and dist_m <= trig_dist) \
                          if (not self.use_sim and not self.use_mock) else True
            cooldown_ok = (now - self.last_trigger.get(class_name, 0.0)) \
                          >= trig_cool

            if distance_ok and cooldown_ok:
                self._publish_detection(class_name, dist_m, conf)
                self.last_trigger[class_name] = now
                cv2.putText(annotated, f'*** TRIGGER: {class_name} ***',
                            (10, 85), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0,0,255), 2)

            label_hdr = (f"CLOSEST: {class_name}  {dist_m:.2f}m"
                         if dist_m > 0 else f"DETECTED: {class_name}")
            cv2.putText(annotated, label_hdr, (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)

        # Publish annotated image — pure NumPy, no cv_bridge
        try:
            self.img_pub.publish(cv2_to_imgmsg(annotated))
        except Exception as e:
            self.get_logger().warn(f'img publish: {e}')

        # Debug window
        if self.debug_view:
            cv2.imshow('Traffic Sign Detection', annotated)
            cv2.waitKey(1)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def destroy_node(self):
        if self._cap:
            self._cap.release()
        if self._device:
            self._device.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
