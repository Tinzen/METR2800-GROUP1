import json, numpy as np, collections
meta=json.load(open('robot_meta.json')); raw=open('robot_meshes.bin','rb').read(); M=meta['meshes']
def pts(i):
    m=M[i]; return np.frombuffer(raw, np.float32, m['nv']*3, m['pOff']).reshape(-1,3).astype(float)
root=meta['root']['children'][0]; subs={c['name']:c for c in root['children']}
owner={}
for c in root['children']:
    for i in c.get('meshes',[]): owner[i]=c['name']
out_ids=set(subs['Offset CF rod outtake v7']['meshes']); bucket_ids=set(subs['Outtake v6']['meshes'])
sar=[i for i in out_ids if M[i]['name'].startswith('SAR2-40')]
P=np.vstack([pts(i) for i in sar]); C0=P.mean(0)
_,_,vt=np.linalg.svd(P-C0, full_matrices=False); AX=vt[0]*(1 if vt[0][2]>0 else -1); N2=vt[1]; N3=np.cross(AX,N2)
SCOOP_NAMES={'Offset-Hex-Hub v1','1908-0025-0032 v1','Component45'}
SCOOP_IDS={118,37,80,119}
groups={}
for i in range(len(M)):
    p=pts(i); lo,hi=p.min(0),p.max(0); c=p.mean(0)
    if i in bucket_ids: g='bucket'
    elif i in out_ids:
        q=c-C0; a,b,d=q@AX,q@N2,q@N3
        nml=(M[i]['name'] or '').lower()
        is_motor=any(k in nml for k in ('5000-','5103','3422','motor','encoder','gearbox'))
        # rails sit on the stack centreline (n3=0), wheels/shims/nuts at n3=-16.5;
        # the motor + gearbox sit on the other side (n3 +15..+40) and must stay fixed
        if is_motor or d > 12: g='base'
        elif d>=-30 and abs(a)<=192 and -52<=b<=50:   # within the 400 mm rail span; end brackets stay fixed
            g='stage%d'%int(np.clip(round((40-b)/10),0,8))
        elif b< -52 and abs(a)<=230 and -60<=d<=12: g='stage8'
        else: g='base'
    elif i in SCOOP_IDS or (owner.get(i,'').startswith('new intake') and M[i]['name'] in SCOOP_NAMES and lo[2]>330): g='scoop'
    elif lo[2]>=104 and lo[1]>=-45 and hi[1]<=45 and lo[0]>=-352 and hi[0]<=15: g='arm'
    else: g='base'
    groups[i]=g
if __name__=='__main__':
    cnt=collections.Counter(groups.values()); tris=collections.Counter()
    for i,g in groups.items(): tris[g]+=M[i]['ni']//3
    for g in sorted(cnt): print(f"{g:8s} meshes {cnt[g]:4d} tris {tris[g]:7d}")
    print("AX",AX.round(4),"N2",N2.round(3),"C0",C0.round(1))
    for g in ['arm','scoop','bucket','stage8','stage0']:
        ids=[i for i in groups if groups[i]==g]; Q=np.vstack([pts(i) for i in ids]); print(g, Q.min(0).round(0), Q.max(0).round(0))
    print("arm members:", sorted(set(M[i]['name'] or '#%d'%i for i in groups if groups[i]=='arm'))[:40])
    json.dump({'groups':groups,'AX':AX.tolist(),'N2':N2.tolist(),'C0':C0.tolist()}, open('groups.json','w'))
