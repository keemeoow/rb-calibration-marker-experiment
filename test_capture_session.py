import json
import os
import tempfile
import unittest

from capture_session import allocate_next_capture_session


class CaptureSessionTests(unittest.TestCase):
    def test_allocates_session01_with_calibration_subdir_and_manifest(self):
        with tempfile.TemporaryDirectory() as temp_root:
            data_root = os.path.join(temp_root, "data")
            session = allocate_next_capture_session(data_root)

            self.assertEqual(session.session_id, "session01")
            self.assertEqual(session.index, 1)
            self.assertTrue(os.path.isdir(session.capture_root))
            self.assertTrue(os.path.isdir(os.path.join(session.session_root, "blind_test")))
            self.assertTrue(os.path.isdir(os.path.join(session.session_root, "calib_out")))
            self.assertTrue(os.path.isdir(os.path.join(session.session_root, "predictions")))
            with open(session.manifest_path, encoding="utf-8") as handle:
                manifest = json.load(handle)
            self.assertEqual(manifest["session_id"], "session01")
            self.assertEqual(manifest["calibration_capture_root"], session.capture_root)

    def test_advances_from_max_and_never_reuses_empty_session(self):
        with tempfile.TemporaryDirectory() as temp_root:
            data_root = os.path.join(temp_root, "data")
            os.makedirs(os.path.join(data_root, "session01"))
            os.makedirs(os.path.join(data_root, "session03"))
            os.makedirs(os.path.join(data_root, "session_notes"))

            session = allocate_next_capture_session(data_root)

            self.assertEqual(session.session_id, "session04")
            self.assertTrue(os.path.isdir(os.path.join(data_root, "session01")))
            self.assertTrue(os.path.isdir(os.path.join(data_root, "session03")))

    def test_repeated_allocations_are_monotonic(self):
        with tempfile.TemporaryDirectory() as temp_root:
            first = allocate_next_capture_session(temp_root)
            second = allocate_next_capture_session(temp_root)

            self.assertEqual(first.session_id, "session01")
            self.assertEqual(second.session_id, "session02")


if __name__ == "__main__":
    unittest.main()
