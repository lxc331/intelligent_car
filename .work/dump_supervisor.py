import sys
sys.path.insert(0,r'.work')
import car_py
code = r"""
import glob,os,time,re
for d in sorted(glob.glob('/tmp/carapp_nav_*'), key=os.path.getmtime)[-3:]:
    st=os.path.getmtime(d)
    print('====',d,time.strftime('%H:%M:%S',time.localtime(st)))
    for name in ('supervisor.log',):
        p=os.path.join(d,name)
        if not os.path.exists(p): print('missing',p); continue
        txt=open(p,errors='replace').read().splitlines()
        keep=[l for l in txt if re.search(r'cancel|stop|abort|fail|error|blocked|request|goal', l, re.I)]
        print('\n'.join(keep[-20:]))
"""
out,err=car_py.run_python(code,timeout=90)
print(out[-4000:]); print('ERR',err[-500:])
