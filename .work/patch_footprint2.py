# -*- coding: utf-8 -*-
import pathlib

editor = pathlib.Path('app/carapp/road_editor.py')
text = editor.read_text(encoding='utf-8')
text = text.replace('from carapp.config import APP_DIR',
                    'from carapp.config import APP_DIR, load_config, sandbox_footprint')
text = text.replace('editor = NetworkEditor(project)',
                    'editor = NetworkEditor(project, footprint=sandbox_footprint(load_config()))')
text = text.replace('self.editor = NetworkEditor(project)',
                    'self.editor = NetworkEditor(project, footprint=sandbox_footprint(load_config()))')
editor.write_text(text, encoding='utf-8')

navconfig = pathlib.Path('app/carapp/core/sandbox_navconfig.py')
text = navconfig.read_text(encoding='utf-8')
old = "footprint = '[[0.24,0.105],[0.24,-0.105],[-0.11,-0.105],[-0.11,0.105]]'"
new = "footprint = '[[0.19,0.08],[0.19,-0.08],[-0.09,-0.08],[-0.09,0.08]]'"
if old in text:
    text = text.replace(old, new)
navconfig.write_text(text, encoding='utf-8')
print('ok', 'footprint' in text)
