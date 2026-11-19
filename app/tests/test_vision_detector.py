"""Offline tests for the YOLOv8 vision module (no GPU, no weights, no app)."""
import io
import unittest

from PIL import Image

from carapp.core.vision import VisionDetector
from carapp.core.vision.detector import VisionDetection, VisionObject, parse_tiles


def frame(width=640, height=480, color=(90, 120, 90)):
    buffer = io.BytesIO()
    Image.new("RGB", (width, height), color).save(buffer, "JPEG")
    return buffer.getvalue()


class FakeBackend:
    """Stands in for UltralyticsBackend: fixed rows, fixed class list."""

    def __init__(self, rows, names=("person", "car", "traffic light")):
        self.rows, self.names, self.model_path = rows, dict(enumerate(names)), "fake.pt"
        self.calls = []

    def infer(self, image, confidence=.25):
        self.calls.append((image.size, confidence))
        return list(self.rows)

    def class_names(self):
        return dict(self.names)

    def describe(self):
        return "fake"


class DetectionResultTests(unittest.TestCase):
    def test_objects_are_queryable_by_label(self):
        result = VisionDetection(
            objects=(VisionObject("person", .9, (10, 10, 20, 20)),
                     VisionObject("car", .8, (50, 50, 20, 20))),
            image_size=(640, 480))
        self.assertEqual(result.labels(), {"person", "car"})
        self.assertEqual(len(result.find("car")), 1)
        self.assertEqual(result.find("bus"), ())

    def test_empty_result(self):
        self.assertEqual(VisionDetection().objects, ())
        self.assertEqual(VisionDetection().labels(), set())


class ParserTests(unittest.TestCase):
    def test_tiles_accept_normalized_regions(self):
        self.assertEqual(parse_tiles([(0., 0., .5, .5)]), ((0., 0., .5, .5),))

    def test_tiles_reject_out_of_frame_and_too_many(self):
        with self.assertRaises(ValueError):
            parse_tiles([(.8, 0., .5, .5)])
        with self.assertRaises(ValueError):
            parse_tiles([(0., 0., .5, .5)] * 9)

    def test_tiles_disable(self):
        self.assertEqual(parse_tiles(None), ())
        self.assertEqual(parse_tiles([]), ())


class DetectorTests(unittest.TestCase):
    def test_full_frame_boxes_are_reported_in_pixels(self):
        backend = FakeBackend([(100., 120., 140., 160., .9, "person")])
        detector = VisionDetector(backend=backend, confidence=.25)
        result = detector.detect(frame(), "jpeg")
        self.assertEqual(result.image_size, (640, 480))
        self.assertEqual(result.objects[0].box, (100, 120, 40, 40))
        self.assertEqual(result.objects[0].label, "person")
        self.assertEqual(backend.calls[0][0], (640, 480))

    def test_tile_boxes_are_offset_back_to_the_frame(self):
        backend = FakeBackend([(10., 10., 50., 50., .8, "person")])
        detector = VisionDetector(backend=backend, confidence=.25, tiles=[(.5, .5, .5, .5)])
        result = detector.detect(frame(), "jpeg")
        # Full-frame pass first, then the 320x240 crop at (320, 240). The two
        # boxes do not overlap, so both survive the merge.
        self.assertEqual(backend.calls[1][0], (320, 240))
        boxes = [item.box for item in result.objects]
        self.assertIn((10, 10, 40, 40), boxes)
        self.assertIn((330, 250, 40, 40), boxes)

    def test_same_label_overlap_is_merged_keeping_best_score(self):
        backend = FakeBackend([(100., 100., 140., 140., .6, "person")])
        detector = VisionDetector(backend=backend, confidence=.25, tiles=[(0., 0., 1., 1.)])
        result = detector.detect(frame(), "jpeg")
        self.assertEqual(len(result.objects), 1)

    def test_different_labels_are_kept_separately(self):
        backend = FakeBackend([(100., 100., 140., 140., .6, "person"),
                               (100., 100., 140., 140., .5, "car")])
        detector = VisionDetector(backend=backend, confidence=.25)
        self.assertEqual(len(detector.detect(frame(), "jpeg").objects), 2)

    def test_no_filtering_is_applied_to_any_label(self):
        # The module must not impose traffic-light rules on generic detections.
        backend = FakeBackend([(20., 200., 60., 240., .9, "person")])
        detector = VisionDetector(backend=backend, confidence=.25)
        self.assertEqual(len(detector.detect(frame(), "jpeg").objects), 1)

    def test_bad_payload_and_format_are_rejected(self):
        detector = VisionDetector(backend=FakeBackend([]), confidence=.25)
        with self.assertRaises(ValueError):
            detector.detect(b"not an image", "jpeg")
        with self.assertRaises(ValueError):
            detector.detect(frame(), "bmp")

    def test_parameter_validation(self):
        with self.assertRaises(ValueError):
            VisionDetector(backend=FakeBackend([]), confidence=0)
        with self.assertRaises(ValueError):
            VisionDetector(backend=FakeBackend([]), confidence=.25, tile_confidence=1.5)
        with self.assertRaises(ValueError):
            VisionDetector(backend=FakeBackend([]), merge_iou=2.)

    def test_from_config_reads_the_vision_block(self):
        detector = VisionDetector.from_config(
            {"confidence": .4, "imgsz": 320, "tiles": [(0., 0., .5, .5)]},
            infer=lambda image, confidence: [])
        self.assertEqual(detector.confidence, .4)
        self.assertEqual(detector.tiles, ((0., 0., .5, .5),))
        self.assertEqual(detector.describe(), "注入推理")


if __name__ == "__main__":
    unittest.main()
