import io,sys
sys.path.insert(0,r'.work')
import car_py
code = r'''
import glob,json,os,time
dirs=sorted(glob.glob('/tmp/carapp_nav_*'),key=os.path.getmtime)
print('\n'.join(dirs[-4:]))
d=dirs[-1]
for name in ('state.json','supervisor.log','nav2.log'):
    p=os.path.join(d,name)
    print('====',p, os.path.getsize(p) if os.path.exists(p) else 'missing')
    if name=='state.json' and os.path.exists(p):
        print(json.dumps(json.load(open(p)),ensure_ascii=False)[:1200])
    elif os.path.exists(p):
        lines=open(p,errors='replace').read().splitlines()
        print('\n'.join(lines[-25:]))
'''
print(car_py.run(code, timeout=90))
