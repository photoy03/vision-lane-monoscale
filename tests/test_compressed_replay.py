#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/run"))
from compressed_to_raw_replay import decode_jpeg


class CompressedReplayTests(unittest.TestCase):
    def test_jpeg_decodes_to_three_channel_bgr_frame(self):
        source = np.zeros((16, 24, 3), dtype=np.uint8)
        source[:, :] = (30, 90, 180)
        encoded_ok, encoded = cv2.imencode(".jpg", source)
        self.assertTrue(encoded_ok)
        decoded = decode_jpeg(encoded.tobytes())
        self.assertEqual(decoded.shape, (16, 24, 3))
        self.assertEqual(decoded.dtype, np.uint8)
        self.assertTrue(np.allclose(decoded[8, 12], source[8, 12], atol=8))

    def test_rejects_empty_or_invalid_payload(self):
        for payload in (b"", b"not a jpeg"):
            with self.subTest(payload=payload):
                with self.assertRaises(ValueError):
                    decode_jpeg(payload)


if __name__ == "__main__":
    unittest.main()
