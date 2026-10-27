"""Import a saved ROS trinary map without depending on Tk or a ROS installation.

Cell classification follows Nav2 Humble's nav2_map_server/map_io.cpp. The car
receives a normalized trinary PGM rather than local paths from the YAML file.
"""
from dataclasses import dataclass
import io
import math
from pathlib import Path

from carapp.core.map_data import MapModel, world_to_grid


MAX_YAML_BYTES = 65_536
MAX_IMAGE_BYTES = 32 * 1024 * 1024


@dataclass(frozen=True)
class NavigationMap:
    source_path: Path
    grid_message: dict
    yaml_text: str
    image_bytes: bytes
    width: int
    height: int
    resolution: float
    origin: tuple

    def as_upload(self):
        # Humble's StaticLayer uses origin x/y only; uploading a rotated raster
        # would make its obstacle positions disagree with the map on screen.
        angle = math.atan2(math.sin(self.origin[2]), math.cos(self.origin[2]))
        if abs(angle) > 1e-8:
            raise ValueError("该地图可预览，但 Nav2 Humble 导航要求地图原点朝向为 0；"
                             "请重新保存为 origin yaw=0 的轴对齐地图")
        return {"yaml_text": self.yaml_text, "image_bytes": self.image_bytes}

    @property
    def grid(self):
        return {"width": self.width, "height": self.height,
                "resolution": self.resolution, "origin": self.origin,
                "data": self.grid_message["data"]}


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("地图 %s 需要有限数值" % name)
    return float(value)


def _metadata(path):
    try:
        import yaml
    except ImportError as exc:
        raise ValueError("地图导入需要 PyYAML，请安装 app/requirements.txt 中的依赖") from exc

    class UniqueLoader(yaml.SafeLoader):
        pass

    def unique_mapping(loader, node, deep=False):
        values = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if not isinstance(key, str) or key in values:
                raise ValueError("地图 YAML 含重复或无效字段")
            values[key] = loader.construct_object(value_node, deep=deep)
        return values

    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)
    with path.open("rb") as stream:
        content = stream.read(MAX_YAML_BYTES + 1)
    if len(content) > MAX_YAML_BYTES:
        raise ValueError("地图 YAML 文件过大")
    try:
        doc = yaml.load(content.decode("utf-8-sig"), Loader=UniqueLoader)
    except (yaml.YAMLError, UnicodeError) as exc:
        raise ValueError("地图 YAML 格式无效") from exc
    if not isinstance(doc, dict):
        raise ValueError("地图 YAML 需要字段映射")
    required = {"image", "resolution", "origin", "negate", "occupied_thresh", "free_thresh"}
    if not required.issubset(doc):
        raise ValueError("地图 YAML 缺少字段：" + ", ".join(sorted(required - doc.keys())))
    if doc.get("mode", "trinary") != "trinary":
        raise ValueError("目前仅支持 trinary 二维栅格地图，请以 trinary 格式重新保存")
    return doc


def load_navigation_map(yaml_path):
    """Load YAML + PGM/PNG in a worker; maximum four million map cells.

    Source files remain unchanged. ``grid_message`` has ROS bottom-row-first
    occupancy values; ``image_bytes`` is a canonical top-row-first PGM.
    """
    path = Path(yaml_path).expanduser().resolve(strict=True)
    doc = _metadata(path)
    resolution = _finite(doc["resolution"], "resolution")
    if not .001 <= resolution <= 10.:
        raise ValueError("地图分辨率需在 0.001–10 m/格")
    origin_values = doc["origin"]
    if not isinstance(origin_values, (list, tuple)) or len(origin_values) != 3:
        raise ValueError("地图 origin 需要 [x, y, yaw] 三个数值")
    origin = tuple(_finite(v, "origin") for v in origin_values)
    if abs(origin[0]) > 1e6 or abs(origin[1]) > 1e6:
        raise ValueError("地图原点超出范围")
    occupied = _finite(doc["occupied_thresh"], "occupied_thresh")
    free = _finite(doc["free_thresh"], "free_thresh")
    if not 0. <= free < occupied <= 1.:
        raise ValueError("地图阈值需满足 0 ≤ free_thresh < occupied_thresh ≤ 1")
    negate = doc["negate"]
    if negate not in (0, 1) or not isinstance(negate, (int, bool)):
        raise ValueError("地图 negate 仅允许 0 或 1")
    image_name = doc["image"]
    if (not isinstance(image_name, str) or not image_name or "://" in image_name
            or any(ord(c) < 32 for c in image_name)):
        raise ValueError("地图 image 需要本地图片路径")
    image_path = (path.parent / image_name).resolve(strict=True)
    with image_path.open("rb") as stream:
        raw = stream.read(MAX_IMAGE_BYTES + 1)
    if len(raw) > MAX_IMAGE_BYTES:
        raise ValueError("地图图片文件过大（最大 32 MB）")
    try:
        from PIL import Image
    except ImportError as exc:
        raise ValueError("地图导入需要 Pillow，请安装 app/requirements.txt 中的依赖") from exc
    try:
        with Image.open(io.BytesIO(raw)) as image:
            width, height = image.size
            if (not 1 <= width <= MapModel.MAX_SIDE or not 1 <= height <= MapModel.MAX_SIDE
                    or width * height > MapModel.MAX_CELLS):
                raise ValueError("地图尺寸超出范围（最多 400 万单元）")
            if image.format not in ("PPM", "PNG"):
                raise ValueError("地图图片仅支持 PGM/PNG")
            has_alpha = "A" in image.getbands() or "transparency" in image.info
            pixels = image.convert("RGBA" if has_alpha else "RGB").tobytes()
    except (OSError, Image.DecompressionBombError) as exc:
        raise ValueError("地图图片无法读取或尺寸超出范围") from exc
    channels = 4 if has_alpha else 3
    top_cells = bytearray(width * height)
    pgm = bytearray(width * height)
    for cell in range(width * height):
        start = cell * channels
        shade = sum(pixels[start:start + channels]) / (channels * 255.)
        probability = shade if negate else 1. - shade
        value = 100 if probability > occupied else 0 if probability < free else 255
        top_cells[cell] = value
        pgm[cell] = 0 if value == 100 else 254 if value == 0 else 205
    data = tuple(-1 if v == 255 else int(v)
                 for row in range(height - 1, -1, -1)
                 for v in top_cells[row * width:(row + 1) * width])
    yaw = origin[2]
    msg = {"header": {"frame_id": "map"},
           "info": {"width": width, "height": height, "resolution": resolution,
                    "origin": {"position": {"x": origin[0], "y": origin[1], "z": 0.},
                               "orientation": {"x": 0., "y": 0.,
                                               "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}}},
           "data": data}
    canonical = ("image: map.pgm\nmode: trinary\nresolution: %.17g\n"
                 "origin: [%.17g, %.17g, %.17g]\nnegate: 0\n"
                 "occupied_thresh: 0.65\nfree_thresh: 0.196\n") % (resolution, *origin)
    image_bytes = ("P5\n%d %d\n255\n" % (width, height)).encode("ascii") + bytes(pgm)
    return NavigationMap(path, msg, canonical, image_bytes, width, height, resolution, origin)


def valid_target(bundle_or_grid, x, y):
    """A point must be inside a known free cell; Nav2 checks body clearance."""
    try:
        if not math.isfinite(x) or not math.isfinite(y):
            return False
        grid = bundle_or_grid.grid if isinstance(bundle_or_grid, NavigationMap) else bundle_or_grid
        gx, gy = world_to_grid(grid, x, y)
        col, row = math.floor(gx), math.floor(gy)
        if not (0 <= col < grid["width"] and 0 <= row < grid["height"]):
            return False
        return grid["data"][row * grid["width"] + col] == 0
    except (KeyError, TypeError, ValueError, OverflowError):
        return False
