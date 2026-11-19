"""Vision detection package: YOLOv8 on GPU, no OpenVINO dependency.

Public surface used by the app and by tests:

* :class:`VisionDetector` — tiled detection with ROI/label filtering.
* :class:`VisionObject` / :class:`VisionDetection` — frame results.
* :class:`UltralyticsBackend` — weights, device selection and inference.
"""
from carapp.core.vision.backend import BackendUnavailable, UltralyticsBackend
from carapp.core.vision.detector import (
    DEFAULT_MODEL,
    MODEL_DIR,
    VisionDetection,
    VisionDetector,
    VisionObject,
)

__all__ = [
    "BackendUnavailable",
    "DEFAULT_MODEL",
    "MODEL_DIR",
    "UltralyticsBackend",
    "VisionDetection",
    "VisionDetector",
    "VisionObject",
]
