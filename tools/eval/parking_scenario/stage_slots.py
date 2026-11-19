import json, math, sys, numpy as np
d=json.load(open(sys.argv[1])); ref=[r for r in json.load(open(sys.argv[2])) if r['keep']]; L=d['lines']
find=lambda kind,p: min((l for l in L if l['kind']==kind), key=lambda l: math.dist(l['a'],p)); out=[]
for j,name in enumerate(d['stages']):
    n=0
    for r in ref:
        e,yaw,w=np.array(r['entrance']),r['yaw'],r['width']; f=np.array([math.cos(yaw),math.sin(yaw)]); l=np.array([-f[1],f[0]])
        n+= all(x['count'][j]>=3 for x in (find('front',(e-l*w/2).tolist()),find('side',(e-l*w/2).tolist()),find('side',(e+l*w/2).tolist())))
    out.append('%s %d'%(name,n))
print('slots with all three lines reaching each stage:', ', '.join(out))
