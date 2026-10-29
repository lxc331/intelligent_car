"""Version-bound localization, road mask and graph; building runs off the Tk thread."""
from dataclasses import asdict,dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import uuid

from carapp.core.navigation_map import load_navigation_map
from carapp.core.road_network import load_road_network
from carapp.core.sandbox_geometry import fit_registration,Registration
from carapp.core.sandbox_layout import standard_layout,build_keepout


@dataclass
class SandboxProject:
    path: Path
    localization: object
    roads: object
    keepout: object
    network: object
    registration: Registration
    metadata: dict

    def upload(self):
        upload=self.localization.as_upload()
        mask=self.keepout.as_upload()
        upload.update(keepout_yaml=mask['yaml_text'].replace('image: map.pgm','image: keepout.pgm'),keepout_bytes=mask['image_bytes'],project_id=self.metadata['id'])
        return upload


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def asset_path(root,value):
    if not isinstance(value,str) or Path(value).is_absolute() or '..' in Path(value).parts:
        raise ValueError('工程文件必须位于工程文件夹内')
    path=(root/value).resolve(strict=True)
    if not path.is_relative_to(root.resolve()):raise ValueError('工程文件不能越出工程目录')
    return path


def load_project(path):
    path=Path(path).resolve(strict=True);root=path.parent
    if path.stat().st_size>131072:raise ValueError('工程描述文件过大')
    meta=json.loads(path.read_text(encoding='utf8'))
    if meta.get('version')!=1 or meta.get('frame')!='map' or not meta.get('calibrated'):
        raise ValueError('工程未完成配准或版本不支持')
    required={'map.yaml','map.pgm','roads.yaml','roads.pgm','keepout.yaml','keepout.pgm','road_network.yaml'}
    if set(meta.get('sha256',{}))!=required:raise ValueError('工程文件校验清单不完整')
    if not math.isfinite(meta['max_rms']) or not 0<meta['max_rms']<=.075:raise ValueError('配准误差阈值无效')
    for name,expected in meta['sha256'].items():
        if digest(asset_path(root,name))!=expected:raise ValueError('工程文件已变化，需重新配准/生成：'+name)
    local=load_navigation_map(asset_path(root,'map.yaml'))
    roads=load_navigation_map(asset_path(root,'roads.yaml'))
    keepout=load_navigation_map(asset_path(root,'keepout.yaml'))
    local.as_upload()
    if (keepout.width,keepout.height,keepout.resolution,keepout.origin)!=(local.width,local.height,local.resolution,local.origin):
        raise ValueError('道路范围与定位地图坐标不一致')
    calibration=meta['calibration']
    reg=fit_registration(calibration['source'],calibration['target'],max_rms=meta['max_rms'])
    stored=Registration(**calibration['transform'])
    if any(abs(getattr(reg,key)-getattr(stored,key))>1e-8 for key in ('x','y','yaw')):
        raise ValueError('配准关系与对应点不一致')
    network=load_road_network(asset_path(root,'road_network.yaml'))
    return SandboxProject(path,local,roads,keepout,network,reg,meta)


def build_project(localization_path,road_path,source,target,destination,*,max_rms=.075):
    localization=load_navigation_map(localization_path);localization.as_upload()
    roads=load_navigation_map(road_path)
    registration=fit_registration(source,target,max_rms=max_rms)
    network,widths=standard_layout(roads,registration)
    destination=Path(destination).resolve()
    if destination.exists():raise ValueError('工程目录已存在，请使用新目录')
    destination.parent.mkdir(parents=True,exist_ok=True)
    stage=Path(tempfile.mkdtemp(prefix='.sandbox_',dir=destination.parent))
    try:
        for name,bundle in (('map',localization),('roads',roads)):
            image=bundle.image_bytes
            (stage/(name+'.pgm')).write_bytes(image)
            (stage/(name+'.yaml')).write_text(bundle.yaml_text.replace('image: map.pgm','image: '+name+'.pgm'),encoding='utf8')
        (stage/'keepout.pgm').write_bytes(build_keepout(localization,roads,registration))
        (stage/'keepout.yaml').write_text(localization.yaml_text.replace('image: map.pgm','image: keepout.pgm'),encoding='utf8')
        network.save(stage/'road_network.yaml')
        meta=dict(version=1,id=uuid.uuid4().hex,frame='map',calibrated=True,max_rms=max_rms,
            calibration=dict(source=source,target=target,transform=asdict(registration)),
            edge_widths=widths,narrow_enabled=False,narrow_nodes=[2,5,8],
            sources=dict(localization=str(localization.source_path),roads=str(roads.source_path)),
            sha256={p.name:digest(p) for p in stage.iterdir()})
        (stage/'sandbox.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf8')
        load_project(stage/'sandbox.json')
        os.rename(stage,destination)
        return load_project(destination/'sandbox.json')
    except Exception:
        # This directory was created exclusively for this failed build.
        if stage.resolve().parent==destination.parent and stage.name.startswith('.sandbox_'):
            shutil.rmtree(stage)
        raise
