"""Subscribe only: no robot publishers/services; all distances are metres."""
import json
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import message_filters
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def info_dict(msg):
    return dict(frame=msg.header.frame_id, width=msg.width, height=msg.height,
                distortion_model=msg.distortion_model, k=list(msg.k), d=list(msg.d),
                r=list(msg.r), p=list(msg.p))


def depth_metres(depth, encoding):
    if encoding == '16UC1':
        result = depth.astype(np.float32) * 0.001  # ROS depth convention: mm
    elif encoding == '32FC1':
        result = depth.astype(np.float32)  # ROS depth convention: metres
    else:
        raise ValueError(f'Unsupported depth encoding: {encoding}')
    result[(~np.isfinite(result)) | (result <= 0)] = np.nan
    return result


def backproject(u, v, z, info):
    k = np.array(info.k, dtype=np.float64).reshape(3, 3)
    if k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('CameraInfo is uncalibrated (fx/fy <= 0)')
    p = np.array([[[u, v]]], dtype=np.float64)
    d = np.array(info.d, dtype=np.float64)
    if info.distortion_model in ('plumb_bob', 'rational_polynomial'):
        ray = cv2.undistortPoints(p, k, d if d.size else None).reshape(2)
    elif info.distortion_model == 'equidistant':
        ray = cv2.fisheye.undistortPoints(p, k, d).reshape(2)
    elif not d.size or np.allclose(d, 0):
        ray = np.array([(u-k[0, 2])/k[0, 0], (v-k[1, 2])/k[1, 1]])
    else:
        raise ValueError(f'Unsupported distortion model: {info.distortion_model}')
    return [float(ray[0]*z), float(ray[1]*z), float(z)]


def same_distortion(a, b):
    # ROS plumb_bob has 5 terms; rational_polynomial has 8. Zero extension
    # represents the same mapping, and avoids a NumPy broadcasting failure.
    models = ('plumb_bob', 'rational_polynomial')
    if a.distortion_model in models and b.distortion_model in models:
        if len(a.d) > 8 or len(b.d) > 8:
            return False
        da = np.pad(np.array(a.d), (0, 8-len(a.d)))
        db = np.pad(np.array(b.d), (0, 8-len(b.d)))
        return bool(np.allclose(da, db, atol=1e-8, rtol=0))
    return (a.distortion_model == b.distortion_model and len(a.d) == len(b.d)
            and bool(np.allclose(a.d, b.d, atol=1e-8, rtol=0)))


class CenterPoint(Node):
    def __init__(self):
        super().__init__('camera_center_test')
        defaults = dict(color_topic='/camera/color/image_raw',
                        depth_topic='/camera/depth/image_raw',
                        color_info_topic='/camera/color/camera_info',
                        depth_info_topic='/camera/depth/camera_info',
                        show_gui=True, duration_sec=0.0, sync_slop_sec=0.03,
                        reliable_qos=True,
                        roi_radius=3, expected_fps=15.0,
                        output_dir=str(Path.home()/'orbbec_ws/results/center_test'))
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        if self.cfg['sync_slop_sec'] <= 0 or self.cfg['roi_radius'] < 1:
            raise ValueError('sync_slop_sec must be positive and roi_radius >= 1')
        self.out = Path(self.cfg['output_dir']); self.out.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        cv2.setNumThreads(1)
        qos = QoSProfile(depth=10, reliability=(ReliabilityPolicy.RELIABLE
                         if self.cfg['reliable_qos'] else ReliabilityPolicy.BEST_EFFORT))
        self.started = time.monotonic(); self.last_sync = self.started
        self.last_print = self.started; self.last_save = 0.0
        self.counts = Counter(); self.issues = Counter(); self.last_stamp = {}
        self.first_stamp = {}; self.max_gap = Counter(); self.last_arrival = {}
        self.max_arrival_gap = Counter(); self.sync_max = 0.0
        self.samples = []; self.latest = {}; self.intrinsics = {}
        self.last_valid_save = 0.0
        self.subs = []
        for key, typ in [('color', Image), ('depth', Image),
                         ('color_info', CameraInfo), ('depth_info', CameraInfo)]:
            sub = message_filters.Subscriber(self, typ, self.cfg[key+'_topic'],
                                            qos_profile=qos)
            sub.registerCallback(lambda msg, key=key: self.track(key, msg))
            self.subs.append(sub)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            self.subs, queue_size=15, slop=self.cfg['sync_slop_sec'])
        self.sync.registerCallback(self.process)
        self.create_timer(1.0, self.health)
        self.done = False
        self.get_logger().info('Subscribe-only RGB-D test. Optical axes: X right, Y down, Z forward. '
                               '16UC1 -> mm/1000; 32FC1 -> m. No hand-eye transform applied.')

    def track(self, key, msg):
        now = time.monotonic(); ts = stamp(msg)
        self.counts[key] += 1
        if key in self.last_stamp:
            gap = ts-self.last_stamp[key]
            self.max_gap[key] = max(self.max_gap[key], gap)
            self.max_arrival_gap[key] = max(self.max_arrival_gap[key], now-self.last_arrival[key])
            if gap <= 0:
                self.issues[key+'_nonmonotonic_stamp'] += 1
            if gap > 1.5/self.cfg['expected_fps']:
                self.issues[key+'_large_stamp_gap'] += 1
        self.first_stamp.setdefault(key, ts)
        self.last_stamp[key] = ts; self.last_arrival[key] = now

    def warn(self, reason):
        self.issues[reason] += 1
        self.get_logger().warning(reason, throttle_duration_sec=3.0)

    def process(self, color, depth, ci, di):
        now = time.monotonic(); self.last_sync = now; self.counts['synchronized'] += 1
        stamps = [stamp(m) for m in (color, depth, ci, di)]
        delta = max(stamps)-min(stamps); self.sync_max = max(self.sync_max, delta)
        if delta > self.cfg['sync_slop_sec']:
            self.warn('DATA_UNSYNCHRONIZED: timestamp span exceeds tolerance'); return
        frames = {m.header.frame_id for m in (color, depth, ci, di)}
        if len(frames) != 1 or not color.header.frame_id:
            self.warn('NOT_ALIGNED: RGB/depth/CameraInfo frame_id differ'); return
        if len({(m.width, m.height) for m in (color, depth, ci, di)}) != 1:
            self.warn('NOT_ALIGNED: image/CameraInfo dimensions differ'); return
        if (not np.allclose(ci.k, di.k, atol=1e-4, rtol=0)
                or not np.allclose(ci.p, di.p, atol=1e-4, rtol=0)
                or not np.allclose(ci.r, di.r, atol=1e-6, rtol=0)
                or not same_distortion(ci, di)):
            self.warn('NOT_ALIGNED: RGB/depth intrinsics differ'); return
        try:
            rgb = self.bridge.imgmsg_to_cv2(color, 'bgr8')
            raw = self.bridge.imgmsg_to_cv2(depth, 'passthrough')
            dm = depth_metres(raw, depth.encoding)
            u, v = color.width//2, color.height//2
            r = self.cfg['roi_radius']
            roi = dm[max(0,v-r):v+r+1, max(0,u-r):u+r+1]
            valid = roi[np.isfinite(roi)]
            self.intrinsics = dict(color=info_dict(ci), aligned_depth=info_dict(di))
            self.latest = dict(frame=color.header.frame_id, encoding=depth.encoding,
                               pixel=[u,v], stamp=stamp(color), sync_span_sec=delta,
                               valid_roi_pixels=int(valid.size), roi_pixels=int(roi.size),
                               invalid_fraction=float(1-np.isfinite(dm).mean()),
                               raw_zero_count=int(np.count_nonzero(raw == 0)),
                               raw_nonfinite_count=int(np.count_nonzero(~np.isfinite(raw))))
            if valid.size < max(3, int(np.ceil(roi.size*0.25))):
                self.warn('INVALID_DEPTH: fewer than 25% (minimum 3) valid ROI pixels')
                label = 'INVALID DEPTH'; self.latest['xyz_m'] = None
            else:
                z = float(np.median(valid)); xyz = backproject(u,v,z,ci)
                self.latest.update(xyz_m=xyz, roi_depth_min_m=float(valid.min()),
                                   roi_depth_max_m=float(valid.max()))
                self.counts['valid_xyz'] += 1
                if now-self.last_print >= 1.0:
                    self.samples.append(self.latest.copy()); self.last_print = now
                    self.get_logger().info(f'{color.header.frame_id}: X={xyz[0]:.4f} '
                                           f'Y={xyz[1]:.4f} Z={xyz[2]:.4f} m; '
                                           f'ROI {valid.size}/{roi.size}; dt={delta*1000:.2f} ms')
                label = f'X={xyz[0]:.3f} Y={xyz[1]:.3f} Z={xyz[2]:.3f} m'
            canvas = rgb.copy()
            cv2.drawMarker(canvas, (u,v), (0,0,255), cv2.MARKER_CROSS, 22, 2)
            cv2.rectangle(canvas, (u-r,v-r), (u+r,v+r), (0,255,0), 1)
            cv2.putText(canvas, label, (12,28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            dv = cv2.applyColorMap(np.uint8(np.nan_to_num(np.clip(dm/3,0,1))*255), cv2.COLORMAP_TURBO)
            dv[~np.isfinite(dm)] = 0
            cv2.drawMarker(dv, (u,v), (255,255,255), cv2.MARKER_CROSS, 22, 1)
            cv2.putText(dv, 'Depth scale: 0..3 m; black=invalid', (12,28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            panel = np.hstack((canvas,dv))
            if now-self.last_save >= 10:
                cv2.imwrite(str(self.out/'preview.png'), panel)
                cv2.imwrite(str(self.out/'color.png'), rgb)
                np.save(self.out/'depth_m.npy', dm)
                self.last_save = now
            if self.latest.get('xyz_m') is not None and now-self.last_valid_save >= 5:
                cv2.imwrite(str(self.out/'last_valid_preview.png'), panel)
                (self.out/'last_valid_sample.json').write_text(
                    json.dumps(self.latest, indent=2), encoding='utf-8')
                self.last_valid_save = now
            if self.cfg['show_gui']:
                cv2.imshow('Gemini335 RGB + aligned depth (q to quit)', panel)
                if cv2.waitKey(1) & 0xff == ord('q'):
                    self.done = True
        except (ValueError, cv2.error, RuntimeError) as exc:
            self.warn('PROCESS_ERROR: '+str(exc))

    def health(self):
        now = time.monotonic()
        if now-self.last_sync > 3:
            self.warn('NO_SYNCHRONIZED_DATA: check image/CameraInfo topics, QoS and timestamps')
        for key in ('color','depth'):
            if now-self.last_arrival.get(key,self.started) > 3:
                self.warn('STREAM_MISSING: '+key+' has no frame for >3 s')
        if self.cfg['duration_sec'] > 0 and now-self.started >= self.cfg['duration_sec']:
            self.done = True
        self.save()

    def save(self):
        rates = {k:(self.counts[k]-1)/(ts-self.first_stamp[k])
                 for k,ts in self.last_stamp.items() if ts > self.first_stamp[k]}
        report = dict(elapsed_sec=time.monotonic()-self.started, config=self.cfg,
                      counts=dict(self.counts), issues=dict(self.issues),
                      stamp_fps=rates, max_stamp_gap_sec=dict(self.max_gap),
                      max_arrival_gap_sec=dict(self.max_arrival_gap),
                      maximum_sync_span_sec=self.sync_max, latest=self.latest,
                      intrinsics=self.intrinsics, samples=self.samples)
        (self.out/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')


def main(args=None):
    rclpy.init(args=args); node = CenterPoint()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.save(); node.destroy_node(); cv2.destroyAllWindows()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
