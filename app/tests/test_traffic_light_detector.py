"""Model-output boundaries and an actual local OpenVINO smoke test."""
import io
import copy
from importlib.util import find_spec
import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
from PIL import Image, ImageDraw

from carapp.core.traffic_light_detector import (MODEL_DIR, TrafficLightDetector,
                                               parse_red_aspect, parse_zoom_regions)
from carapp.core.traffic_light import TrafficLightController
from carapp.features.base import AppContext
from carapp.config import DEFAULTS
from carapp.model.state import CarState


def encode(image, mode="png"):
    stream = io.BytesIO()
    image.save(stream, "JPEG" if mode == "jpeg" else "PNG")
    return stream.getvalue()


def output(*rows):
    return np.array(rows, dtype=np.float32).reshape(1, -1, 6)


class TrafficLightDetectorTests(unittest.TestCase):
    def make(self, *rows):
        self.infer = Mock(return_value=output(*rows))
        return TrafficLightDetector(infer=self.infer)

    def test_model_red_class_triggers_without_colour_or_housing_rules(self):
        detector = self.make((130, 60, 190, 140, .8, 3))
        result = detector.detect(encode(Image.new("RGB", (320, 320), "white")), "png")
        self.assertTrue(result.red)
        self.assertEqual(result.boxes, ((130, 60, 60, 80),))
        self.assertAlmostEqual(result.scores[0], .8)

    def test_red_pixels_and_round_red_objects_do_not_replace_model_predictions(self):
        detector = self.make()
        for image in (Image.new("RGB", (320, 320), "red"), Image.new("RGB", (320, 320), "white")):
            ImageDraw.Draw(image).ellipse((130, 50, 190, 110), fill="red")
            self.assertFalse(detector.detect(encode(image), "png").red)
        self.assertEqual(self.infer.call_count, 2)

    def test_only_red_class_at_original_node_confidence_triggers(self):
        detector = self.make((130, 60, 190, 140, .99, 4),
                             (130, 60, 190, 140, .49, 3),
                             (130, 60, 190, 140, .99, 0))
        frame = encode(Image.new("RGB", (320, 320), "red"))
        self.assertFalse(detector.detect(frame, "png").red)
        self.infer.return_value = output((130, 60, 190, 140, .5, 3))
        self.assertTrue(detector.detect(frame, "png").red)

    def test_letterbox_rgb_float32_and_source_box_coordinates(self):
        detector = self.make((100, 60, 150, 90, .9, 3))
        result = detector.detect(encode(Image.new("RGB", (640, 480), (255, 128, 0))), "png")
        tensor = self.infer.call_args.args[0]
        self.assertEqual(tensor.shape, (1, 3, 320, 320))
        self.assertEqual(tensor.dtype, np.float32)
        self.assertTrue(tensor.flags.c_contiguous)
        np.testing.assert_allclose(tensor[0, :, 40, 0], (1., 128/255., 0.))
        np.testing.assert_allclose(tensor[0, :, 0, 0], (114/255.,) * 3)
        self.assertEqual(result.boxes, ((200, 40, 100, 60),))

    def test_portrait_padding_is_removed_before_frontal_filtering(self):
        detector = self.make((100, 60, 140, 110, .9, 3))
        result = detector.detect(encode(Image.new("RGB", (240, 640), "white")), "png")
        self.assertEqual(result.boxes, ((0, 120, 80, 100),))

    def test_side_bottom_and_padding_boxes_do_not_stop(self):
        detector = self.make((0, 60, 30, 100, .9, 3),
                             (140, 300, 180, 318, .9, 3))
        self.assertFalse(detector.detect(encode(Image.new("RGB", (320, 320), "white")), "png").red)
        detector = self.make((140, 0, 180, 30, .9, 3))
        self.assertFalse(detector.detect(encode(Image.new("RGB", (640, 480), "white")), "png").red)

    def test_jpeg_png_and_jpg_inputs_use_same_model(self):
        detector = self.make((130, 60, 190, 140, .8, 3))
        for mode in ("jpeg", "jpg", "png"):
            payload = encode(Image.new("RGB", (320, 320), "white"), "jpeg" if mode == "jpg" else mode)
            self.assertTrue(detector.detect(payload, mode).red)

    def test_road_objects_remain_available_near_bottom_for_scene_rules(self):
        detector = self.make((80, 280, 240, 315, .8, 5),
                             (130, 60, 190, 140, .8, 1),
                             (130, 60, 190, 140, .8, 2))
        result = detector.detect(encode(Image.new("RGB", (320, 320), "white")), "png")
        self.assertFalse(result.red)
        self.assertEqual(result.image_size, (320, 320))
        self.assertEqual([o.label for o in result.objects], ["crosswalk", "right", "park"])
        self.assertEqual(result.objects[0].box, (80, 280, 160, 35))

    def test_invalid_image_and_unknown_format_raise_before_inference(self):
        detector = self.make()
        for payload, mode in ((b"invalid", "jpeg"), (b"", "png"), (b"invalid", "gif")):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                detector.detect(payload, mode)
        self.infer.assert_not_called()

    def test_bad_inference_output_is_an_error_never_a_clear_frame(self):
        detector = self.make()
        frame = encode(Image.new("RGB", (320, 320), "white"))
        for rows in (np.zeros((1, 6, 300)), output((130, 60, 190, 140, np.nan, 3)),
                     output((130, 60, 190, 140, 1.1, 3)),
                     output((130, 60, 190, 140, .9, 99)),
                     output((130, 60, 190, 140, .9, 3.2))):
            with self.subTest(shape=rows.shape), self.assertRaises(ValueError):
                self.infer.return_value = rows
                detector.detect(frame, "png")

    def test_inference_failure_propagates_without_colour_fallback(self):
        detector = self.make()
        self.infer.side_effect = RuntimeError("model unavailable")
        with self.assertRaisesRegex(RuntimeError, "model unavailable"):
            detector.detect(encode(Image.new("RGB", (320, 320), "red")), "png")

    def test_missing_dependency_or_weight_has_actionable_enable_error(self):
        detector = TrafficLightDetector()
        with patch("carapp.core.traffic_light_detector.find_spec", return_value=None):
            with self.assertRaisesRegex(ImportError, "requirements.txt"):
                detector.ensure_available()
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(FileNotFoundError):
                TrafficLightDetector(model_dir=folder).ensure_available()

    def test_metadata_is_read_from_packaged_model_and_requires_red_green(self):
        detector = self.make()
        names, size = detector._load_metadata()
        self.assertEqual((names[3], names[4], size), ("red", "green", 320))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"metadata.yaml"
            path.write_text("task: detect\nend2end: true\nimgsz: [320, 320]\nnames: {0: person}\n")
            with self.assertRaisesRegex(ValueError, "red / green"):
                TrafficLightDetector(model_dir=folder, infer=self.infer)._load_metadata()

    def test_zoom_regions_recover_small_lights_and_map_boxes_to_the_frame(self):
        infer = Mock(side_effect=[output(), output((100, 60, 130, 100, .86, 3))])
        detector = TrafficLightDetector(infer=infer, zoom=((0., 0., .5, .6),))
        result = detector.detect(encode(Image.new("RGB", (640, 480), "white")), "png")
        self.assertEqual(infer.call_count, 2)
        self.assertEqual(infer.call_args_list[0].args[0].shape, (1, 3, 320, 320))
        self.assertTrue(result.red)
        self.assertEqual(result.boxes, ((100, 44, 30, 40),))
        self.assertEqual([item.label for item in result.objects], ["red"])

    def test_zoom_pass_duplicates_merge_into_one_object(self):
        infer = Mock(side_effect=[output((130, 60, 250, 140, .7, 3)),
                                  output((130, 60, 250, 140, .9, 3))])
        detector = TrafficLightDetector(infer=infer, zoom=((0., 0., 1., 1.),))
        result = detector.detect(encode(Image.new("RGB", (320, 320), "white")), "png")
        self.assertEqual(len(result.objects), 1)
        self.assertAlmostEqual(result.scores[0], .9)

    def test_zoom_regions_use_their_own_lower_confidence(self):
        infer = Mock(side_effect=[output((130, 60, 250, 140, .45, 3)),
                                  output((130, 60, 250, 140, .45, 3))])
        detector = TrafficLightDetector(infer=infer, confidence=.5, zoom_confidence=.35,
                                        zoom=((0., 0., 1., 1.),))
        result = detector.detect(encode(Image.new("RGB", (320, 320), "white")), "png")
        self.assertTrue(result.red)
        self.assertEqual(result.boxes, ((130, 60, 120, 80),))
        for bad in (0., 1.1, float("nan")):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                TrafficLightDetector(zoom_confidence=bad)

    def test_zoom_regions_are_validated_before_any_inference(self):
        for bad in ("left", (0., 0., .5, .6), ((0., 0., .5),), ((0., 0., .5, .6),) * 9,
                    ((-.01, 0., .5, .6),), ((.6, 0., .5, .6),), ((0., .9, .5, .2),),
                    ((0., 0., .01, .6),), ((0., 0., .5, float("nan")),)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                TrafficLightDetector(zoom=bad)
        self.assertEqual(parse_zoom_regions(None), ())

    def test_invalid_confidence_is_rejected(self):
        for bad in (0., -1., 1.1, float("nan"), float("inf")):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                TrafficLightDetector(confidence=bad)

    def test_red_aspect_window_rejects_tall_or_wide_red_objects(self):
        frame = encode(Image.new("RGB", (320, 320), "white"))
        infer = Mock(return_value=output((140, 100, 180, 200, .95, 3)))  # 40x100, h/w 2.5
        strict = TrafficLightDetector(infer=infer, red_aspect=(.6, 1.6))
        self.assertFalse(strict.detect(frame, "png").red)
        infer.return_value = output((140, 100, 180, 140, .95, 3))  # 40x40 lamp
        self.assertTrue(strict.detect(frame, "png").red)
        # An empty window keeps the vendor behaviour: every red box counts.
        infer.return_value = output((140, 100, 180, 200, .95, 3))
        self.assertTrue(TrafficLightDetector(infer=infer).detect(frame, "png").red)
        self.assertTrue(TrafficLightDetector(infer=infer, red_aspect=[]).detect(frame, "png").red)

    def test_red_aspect_window_is_validated(self):
        for bad in ([.6], [.6, 1.6, 1.8], "wide", [1.6, .6], [0., 1.], ["a", 1.5],
                    [.6, float("nan")]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                TrafficLightDetector(red_aspect=bad)
        self.assertIsNone(parse_red_aspect(None))
        self.assertEqual(parse_red_aspect([.6, 1.6]), (.6, 1.6))

    def test_weak_red_candidates_are_rechecked_in_a_square_context_crop(self):
        frame = encode(Image.new("RGB", (640, 480), "white"))
        band = output((200, 128, 220, 148, .10, 3))    # only the candidate threshold
        crop = output((133, 106, 187, 160, .81, 3))    # same lamp, read in context
        infer = Mock(side_effect=[output(), band, crop])
        detector = TrafficLightDetector(infer=infer, zoom=((0., 0., .5, .3),),
                                        zoom_confidence=.35, candidate_confidence=.06)
        result = detector.detect(frame, "png")
        self.assertEqual(infer.call_count, 3)
        self.assertTrue(result.red)
        self.assertAlmostEqual(result.scores[0], .81)
        self.assertEqual(result.boxes, ((200, 40, 20, 20),))

    def test_low_scoring_band_boxes_that_are_not_red_are_dropped(self):
        frame = encode(Image.new("RGB", (640, 480), "white"))
        infer = Mock(side_effect=[output(), output((200, 128, 220, 148, .10, 5))])
        detector = TrafficLightDetector(infer=infer, zoom=((0., 0., .5, .3),),
                                        zoom_confidence=.35, candidate_confidence=.06)
        result = detector.detect(frame, "png")
        self.assertEqual(infer.call_count, 2)
        self.assertFalse(result.red)
        self.assertEqual(result.objects, ())

    def test_candidate_confidence_must_not_exceed_the_zoom_confidence(self):
        for bad in (0., -.1, .36, 1., float("inf"), "x"):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                TrafficLightDetector(zoom_confidence=.35, candidate_confidence=bad)
        self.assertAlmostEqual(TrafficLightDetector(zoom_confidence=.35).candidate_confidence, .35)

    @unittest.skipUnless(find_spec("openvino"), "optional OpenVINO runtime is not installed")
    def test_packaged_model_runs_on_cpu_without_vendor_source_directory(self):
        self.assertTrue((MODEL_DIR / "best_traffic_26.bin").is_file())
        detector = TrafficLightDetector()
        detector.ensure_available()
        self.assertFalse(detector.detect(encode(Image.new("RGB", (640, 480), "white")), "png").red)
        request = detector._request
        self.assertIsNotNone(request)
        self.assertFalse(detector.detect(encode(Image.new("RGB", (320, 240), "white")), "png").red)
        self.assertIs(detector._request, request)


class TrafficModelInterlockTests(unittest.TestCase):
    def test_model_labels_drive_existing_pause_resume_and_error_hold(self):
        now, label = [10.], [None]
        def infer(tensor):
            if label[0] == "error":
                raise RuntimeError("inference failed")
            if label[0] == "crosswalk":
                return output((80, 195, 240, 235, .9, 5))
            return output() if label[0] is None else output((130, 60, 190, 140, .9, label[0]))
        control = TrafficLightController(lambda: None, detector=TrafficLightDetector(infer=infer),
                                         clock=lambda: now[0])
        self.addCleanup(control.close)
        sent = []
        app = types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS), state=CarState(), traffic_light=control,
            bridge=types.SimpleNamespace(publish=lambda topic, message: sent.append(message) or True), log=lambda message: None)
        ctx = AppContext(app)
        control.set_enabled(True)
        frame = encode(Image.new("RGB", (320, 320), "red"))
        def send(at, class_id):
            now[0], label[0] = at, class_id
            control.submit(at, frame, "png")
            deadline = time.monotonic() + 1.
            while time.monotonic() < deadline:
                with control.lock:
                    if control.policy.last_at == at:
                        break
                time.sleep(.005)
            else:
                self.fail("model worker did not finish")
        for at in (10., 10.4, 10.81):
            send(at, None)
        self.assertTrue(ctx.publish_twist(.06, .2, vy=.03))
        self.assertEqual(sent[-1]["linear"]["x"], .06)
        for at in (11.1, 11.4, 11.7):
            send(at, "crosswalk")
        self.assertTrue(control.snapshot()["crosswalk_active"])
        send(12., 3)
        self.assertTrue(ctx.publish_twist(.06, .2, vy=.03))
        self.assertEqual(sent[-1]["linear"], dict(x=0., y=0., z=0.))
        self.assertEqual(sent[-1]["angular"]["z"], 0.)
        for at in (12.3, 12.7):
            send(at, 4)
            self.assertTrue(control.snapshot()["blocked"])
        send(13.11, 4)
        self.assertFalse(control.snapshot()["blocked"])
        self.assertTrue(ctx.publish_twist(.06))
        self.assertEqual(sent[-1]["linear"]["x"], .06)
        send(13.4, "error")
        self.assertTrue(control.snapshot()["blocked"])
        self.assertIn("inference failed", control.snapshot()["error"])


if __name__ == "__main__":
    unittest.main()
