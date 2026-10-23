import unittest
from collections import Counter
from types import SimpleNamespace
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from camera_center_test.center_point import depth_metres, backproject, same_distortion, CenterPoint


class GeometryTest(unittest.TestCase):
    def info(self):
        msg = CameraInfo()
        msg.k = [600., 0., 320., 0., 610., 240., 0., 0., 1.]
        msg.d = [0., 0., 0., 0., 0.]
        msg.distortion_model = 'plumb_bob'
        return msg

    def test_depth_units_and_invalid(self):
        result = depth_metres(np.array([[0, 1250]], dtype=np.uint16), '16UC1')
        self.assertTrue(np.isnan(result[0, 0]))
        self.assertAlmostEqual(float(result[0, 1]), 1.25)
        result = depth_metres(np.array([[np.inf, np.nan, -1., 0., 2.]], dtype=np.float32), '32FC1')
        self.assertTrue(np.isnan(result[0, :4]).all())
        self.assertEqual(float(result[0, 4]), 2.)
        with self.assertRaises(ValueError): depth_metres(np.zeros((1, 1)), 'mono16')

    def test_optical_axes(self):
        self.assertTrue(np.allclose(backproject(380, 301, 2., self.info()), [.2, .2, 2.]))
        self.assertTrue(np.allclose(backproject(320, 240, 2., self.info()), [0., 0., 2.]))

    def test_distortion_round_trip(self):
        info = self.info(); info.d = [0.1, -0.02, 0.001, -0.001, 0.005]
        point = np.array([[[.3, -.2, 1.2]]])
        pixel, _ = cv2.projectPoints(point, np.zeros(3), np.zeros(3),
                                    np.array(info.k).reshape(3, 3), np.array(info.d))
        u, v = pixel.reshape(2)
        self.assertTrue(np.allclose(backproject(u, v, 1.2, info), point.reshape(3), atol=1e-7))

    def test_distortion_compatibility(self):
        a, b = self.info(), self.info()
        a.distortion_model = 'rational_polynomial'; a.d = [0.]*8
        self.assertTrue(same_distortion(a, b))
        a.d[0] = .01
        self.assertFalse(same_distortion(a, b))

    def validation_context(self):
        reasons = []
        ctx = SimpleNamespace(last_sync=0., counts=Counter(), sync_max=0.,
                              cfg={'sync_slop_sec': .03}, warn=reasons.append)
        color, depth, ci, di = Image(), Image(), self.info(), self.info()
        for msg in (color, depth, ci, di):
            msg.header.frame_id = 'camera_color_optical_frame'
            msg.header.stamp.sec = 1
            msg.width = 640; msg.height = 480
        return ctx, reasons, (color, depth, ci, di)

    def test_unsynchronized_data_rejected(self):
        ctx, reasons, msgs = self.validation_context()
        msgs[1].header.stamp.nanosec = 50_000_000
        CenterPoint.process(ctx, *msgs)
        self.assertTrue(reasons[0].startswith('DATA_UNSYNCHRONIZED'))

    def test_unaligned_frames_rejected(self):
        ctx, reasons, msgs = self.validation_context()
        msgs[1].header.frame_id = 'camera_depth_optical_frame'
        CenterPoint.process(ctx, *msgs)
        self.assertTrue(reasons[0].startswith('NOT_ALIGNED'))

    def test_unaligned_intrinsics_rejected(self):
        ctx, reasons, msgs = self.validation_context()
        msgs[3].k[0] = 500.
        CenterPoint.process(ctx, *msgs)
        self.assertTrue(reasons[0].startswith('NOT_ALIGNED'))


if __name__ == '__main__': unittest.main()
