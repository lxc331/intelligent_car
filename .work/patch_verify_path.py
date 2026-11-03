# -*- coding: utf-8 -*-
import json
import pathlib

runtime = pathlib.Path('app/carapp/core/sandbox_runtime.py')
text = runtime.read_text(encoding='utf-8')
if "request.get('verify_path')" not in text:
    old = """                threshold=max(253,min(254,int(request.get('pass_cost',253))))
                if not swept_clear(grid,points,self.path_footprint,threshold):
                    raise RuntimeError(_describe_block(grid,points,self.path_footprint,threshold))"""
    new = """                threshold=max(253,min(254,int(request.get('pass_cost',253))))
                # 整条路径预检默认关闭：代价图是实时局部观测，容易把可走的
                # 窄道判成阻挡；真正的避障交给 Nav2 的代价图与跟随控制器。
                # 需要临时恢复时把 navigation.sandbox_verify_path 设为真。
                if request.get('verify_path') and not swept_clear(grid,points,self.path_footprint,threshold):
                    raise RuntimeError(_describe_block(grid,points,self.path_footprint,threshold))"""
    assert old in text
    text = text.replace(old, new)
    runtime.write_text(text, encoding='utf-8')
    print('runtime patched')

remote = pathlib.Path('app/carapp/core/navigation_remote.py')
text = remote.read_text(encoding='utf-8')
if 'verify_path=False' not in text:
    old = "def follow_path(self, points, *, project_id, intent_epoch=None, pass_cost=253):"
    new = "def follow_path(self, points, *, project_id, intent_epoch=None, pass_cost=253, verify_path=False):"
    assert old in text
    text = text.replace(old, new)
if "'verify_path'" not in text:
    old = "raw=json.dumps(dict(points=points,project_id=project_id,pass_cost=pass_cost),"
    new = "raw=json.dumps(dict(points=points,project_id=project_id,pass_cost=pass_cost,verify_path=bool(verify_path)),"
    assert old in text
    text = text.replace(old, new)
remote.write_text(text, encoding='utf-8')
print('remote ok')

page = pathlib.Path('app/carapp/features/sandbox_navigation.py')
text = page.read_text(encoding='utf-8')
if 'sandbox_verify_path' not in text:
    old = "f.session.submit('follow_path',route.points,project_id=self.project.metadata['id'])"
    new = ("f.session.submit('follow_path',route.points,project_id=self.project.metadata['id'],\n"
           "                verify_path=bool(f.ctx.cfg.get('navigation',{}).get('sandbox_verify_path',False)))")
    assert old in text
    text = text.replace(old, new)
    page.write_text(text, encoding='utf-8')
print('page ok')

cfg = pathlib.Path('app/carapp/config.py')
text = cfg.read_text(encoding='utf-8')
if 'sandbox_verify_path' not in text:
    old = '        "sandbox_footprint": {"front": .19, "rear": .09, "half": .08},'
    new = ('        "sandbox_footprint": {"front": .19, "rear": .09, "half": .08},\n'
           '        # 车端是否在发车前用实时代价图复核整条路径；默认关闭，避免窄道被误判。\n'
           '        "sandbox_verify_path": False,')
    assert old in text
    cfg.write_text(text.replace(old, new), encoding='utf-8')
print('cfg ok')

settings = pathlib.Path('app/config/settings.json')
data = json.loads(settings.read_text(encoding='utf-8'))
data.setdefault('navigation', {})['sandbox_verify_path'] = False
settings.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8', newline='\n')
print('settings ok')
