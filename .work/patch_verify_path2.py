# -*- coding: utf-8 -*-
import json
import pathlib

page = pathlib.Path('app/carapp/features/sandbox_navigation.py')
text = page.read_text(encoding='utf-8')
if 'sandbox_verify_path' not in text:
    old = "if f.session.submit('follow_path',route.points,project_id=self.project.metadata['id'],pass_cost=cost):self._running=True"
    new = ("if f.session.submit('follow_path',route.points,project_id=self.project.metadata['id'],pass_cost=cost,\n"
           "                    verify_path=bool(f.ctx.cfg.get('navigation',{}).get('sandbox_verify_path',False))):self._running=True")
    assert old in text
    page.write_text(text.replace(old, new), encoding='utf-8')
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
