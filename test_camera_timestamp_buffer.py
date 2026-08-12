import sys
import threading
import types
import unittest
from collections import deque

import numpy as np


# Buffer accessors do not need a RealSense device; make the module importable in CI.
try:
    import pyrealsense2  # noqa: F401
except ModuleNotFoundError:
    sys.modules["pyrealsense2"] = types.ModuleType("pyrealsense2")

from camera import RealSenseCamera


class CameraTimestampBufferTests(unittest.TestCase):
    def _camera_with_buffer(self):
        cam = RealSenseCamera.__new__(RealSenseCamera)
        cam._lock = threading.Lock()
        image = np.zeros((2, 2, 3), dtype=np.uint8)
        depth = np.zeros((2, 2), dtype=np.uint16)
        cam._buf = deque([
            (100.0, 10100.0, "hardware_clock", image, depth),
            (200.0, 90200.0, "hardware_clock", image + 1, depth + 1),
        ])
        return cam

    def test_cross_camera_api_returns_host_not_device_timestamp(self):
        cam = self._camera_with_buffer()
        _color, _depth, ts_ms = cam.get_latest()
        self.assertEqual(ts_ms, 200.0)

        _color, _depth, host_ts, device_ts, domain = cam.get_at_with_timestamps(180.0)
        self.assertEqual(host_ts, 200.0)
        self.assertEqual(device_ts, 90200.0)
        self.assertEqual(domain, "hardware_clock")


if __name__ == "__main__":
    unittest.main()
