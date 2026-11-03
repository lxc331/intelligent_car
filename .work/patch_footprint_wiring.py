# -*- coding: utf-8 -*-
import pathlib

features = pathlib.Path('app/carapp/features/sandbox_navigation.py')
text = features.read_text(encoding='utf-8')
if 'from carapp.config import sandbox_footprint' not in text:
    text = text.replace('from carapp.core.sandbox_route import plan_route',
                        'from carapp.config import sandbox_footprint\nfrom carapp.core.sandbox_route import plan_route')
old = "            try:self.events.put((epoch,'route',(plan_route(project,start,goal,blocked=blocked),start),None))"
new = ("            footprint=sandbox_footprint(f.ctx.cfg)\n"
       "            try:self.events.put((epoch,'route',(plan_route(project,start,goal,blocked=blocked,footprint=footprint),start),None))")
assert old in text
text = text.replace(old, new)
features.write_text(text, encoding='utf-8')

editor = pathlib.Path('app/carapp/core/network_editor.py')
text = editor.read_text(encoding='utf-8')
text = text.replace('self.project = save_network(self.project.path, self.network(), self.edge_widths())',
                    'self.project = save_network(self.project.path, self.network(), self.edge_widths(), self.footprint)')
editor.write_text(text, encoding='utf-8')
print('ok')
