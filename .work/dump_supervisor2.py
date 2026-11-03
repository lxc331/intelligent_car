import sys
sys.path.insert(0,r'.work')
import car_py
code = r"""
import glob,os,time
for d in sorted(glob.glob('/tmp/carapp_nav_*'), key=os.path.getmtime)[-2:]:
    print('====',d)
    for name in ('supervisor.log','nav2.log'):
        p=os.path.join(d,name)
        if not os.path.exists(p): print('missing',name); continue
        lines=open(p,errors='replace').read().splitlines()
        print('---',name,len(lines))
        print('\n'.join(lines[-18:]))
"""
out,err=car_py.run_python(code,timeout=90)
print(out[-6000:])
