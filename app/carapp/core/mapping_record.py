"""PC-only evidence for mapping; records never write camera video to the car."""
import json
import time
from pathlib import Path


class MappingRecord:
    def __init__(self, folder, camera, vehicle):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.camera = camera
        self.owned = bool(camera and not camera.recording)
        self.video = camera.start_record(str(self.folder)) if self.owned else getattr(camera, "record_path", None)
        self.report = dict(vehicle=vehicle, video=self.video, events=[], remote_writes=[],
                           car_source_changes=[], note="录像和关键帧直接存电脑；车端仅临时运行文件和地图数据。")
        try:
            self.event("建图操作记录开始")
        except OSError:
            if self.owned:
                camera.stop_record()
            raise

    def event(self, text, **details):
        self.report["events"].append(dict(time=time.time(), text=text, **details))
        paths = details.get("remote_paths") or []
        self.report["remote_writes"].extend(p for p in paths if p not in self.report["remote_writes"])
        self.flush()

    def flush(self):
        temporary = self.folder / "操作记录.json.tmp"
        temporary.write_text(json.dumps(self.report, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(self.folder / "操作记录.json")

    def frame(self, label, frame):
        if frame:
            _, payload, mode = frame
            (self.folder / (label + (".jpg" if mode == "jpeg" else ".png"))).write_bytes(payload)

    def finish(self):
        # A later manual recording belongs to its initiator, not this session.
        if self.owned and self.camera and self.camera.record_path == self.video:
            self.camera.stop_record()
        self.event("建图操作记录结束")
