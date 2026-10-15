import json, base64, numpy as np, pyfqmr, re, collections
exec(open('classify.py').read().split("if __name__")[0])
P_A = np.array([0.0, 0.0, 120.0])                       # arm pivot (hub centre)
P_S = np.array([0.0, 0.0, 354.0])                       # scoop pivot (horn centre)
P_P = C0 + AX * (-200.0)                                # default slide tilt pivot: lower end of base stage
COL = {'carbon':'#2c2f31','gold':'#c9a227','brass':'#b8892b','alu':'#b7bbbf','steel':'#8f969c','black':'#232527','print':'#1c1d1f','pcb':'#2f6b3f','red':'#a8322d'}
def colour(name, owner_name, grp):
    n = name.lower()
    if grp in ('scoop','bucket'): return 'print'
    if re.search(r'(rod|tube|\d+mm v\d|^\d+mm$|cf )', n): return 'carbon'
    if 'motor shield' in n or '5000-0002-0001 body' in n or 'encoder cap' in n: return 'gold'
    if n.startswith('2302-') or n.startswith('2305-') or 'gear' in n: return 'brass'
    if n.startswith('sar2-40'): return 'alu'
    if 'zhoucheng' in n or 'bearing' in n or 'hub' in n or '5103' in n or '3422' in n: return 'steel'
    if 'pcb' in n: return 'pcb'
    if 'switch' in n: return 'red'
    return 'black'
SKIP = re.compile(r'(screw|hex nut|luosi|_9019|_9139|_9012|snap ring)', re.I)
# drop parts that float free of the rest of the model (e.g. a stray standoff left in the CAD)
_B = [(pts(i).min(0), pts(i).max(0)) for i in range(len(M))]
ISOLATED = set()
for _i, (_lo, _hi) in enumerate(_B):
    _gap = min(np.linalg.norm(np.maximum(0, np.maximum(_lo - _B[j][1], _B[j][0] - _hi))) for j in range(len(M)) if j != _i)
    if _gap > 30: ISOLATED.add(_i)
print('removed isolated meshes:', [(i, M[i]['name']) for i in sorted(ISOLATED)])
bodies = collections.defaultdict(lambda: collections.defaultdict(lambda: [[], []]))  # body -> colour -> [pos list, idx list]
stats = collections.Counter(); kept = 0; dropped = 0
for i in range(len(M)):
    m = M[i]; g = groups[i]; p = pts(i)
    idx = np.frombuffer(raw, np.uint32, m['ni'], m['iOff']).reshape(-1, 3).astype(np.int64)
    diag = np.linalg.norm(p.max(0) - p.min(0))
    if SKIP.search(m['name'] or '') or diag < 6 or i in ISOLATED: dropped += 1; continue
    nt = len(idx)
    # weld duplicate vertices so the decimator sees connected surfaces
    q = np.round(p * 100).astype(np.int64)
    uq, inv = np.unique(q, axis=0, return_inverse=True)
    pw = uq / 100.0; fw = inv.reshape(-1)[idx]
    target = nt if nt <= 300 else max(200, int(nt * (0.07 if nt > 3000 else (0.14 if nt > 1000 else 0.35))))
    if target < nt:
        s = pyfqmr.Simplify(); s.setMesh(pw, fw); s.simplify_mesh(target_count=target, aggressiveness=6, preserve_border=True, verbose=0)
        pw, fw, _ = s.getMesh()
    off = {'arm': P_A, 'scoop': P_S}.get(g, P_P if (g.startswith('stage') or g == 'bucket') else np.zeros(3))
    c = colour(m['name'] or '', owner.get(i, ''), g)
    B = bodies[g][c]; base = sum(len(x) for x in B[0]); B[0].append(np.asarray(pw) - off); B[1].append(np.asarray(fw) + base)
    stats[g] += len(fw); kept += 1
allp = np.vstack([np.vstack(cc[0]) for b in bodies.values() for cc in b.values()])
lo, hi = allp.min(0), allp.max(0); scale = (hi - lo) / 65535.0
out = {'units': 'mm', 'frame': 'robot CAD frame, Z up', 'qmin': lo.tolist(), 'qscale': scale.tolist(), 'colours': COL,
       'pivots': {'arm': P_A.tolist(), 'scoop': P_S.tolist(), 'slide': P_P.tolist()},
       'slideAxis': AX.tolist(), 'slideNormal': N2.tolist(), 'slideTiltAxis': np.cross(AX, N2).tolist(), 'stages': 9,
       'bodies': {}}
tot = 0
for g, cols in bodies.items():
    parts = []
    for c, (pl, il) in cols.items():
        P = np.vstack(pl); I = np.vstack(il).astype(np.uint32)
        Q = np.clip(np.round((P - lo) / scale), 0, 65535).astype(np.uint16)
        parts.append({'colour': c, 'nv': len(P), 'nt': len(I), 'pos': base64.b64encode(Q.tobytes()).decode(),
                      'idx': base64.b64encode(I.tobytes()).decode()})
        tot += len(I)
    out['bodies'][g] = parts
# bucket tip (furthest point along slide axis), robot frame, for auto mm/tick
bk = np.vstack([pts(i) for i in groups if groups[i] == 'bucket']); out['bucketTip'] = bk[np.argmax(bk @ AX)].tolist()
js = 'window.DOVER_MODEL = ' + json.dumps(out, separators=(',', ':')) + ';\n'
open('twin_model.js', 'w').write('// Dover robot geometry for tools/run_viewer.html, generated from "METR2800 Robot.step"\n' + js)
print('kept', kept, 'dropped', dropped, 'tris total', tot, 'bytes', len(js)); print(dict(stats))
print('pivot slide', P_P.round(1), 'bucket tip', np.round(out['bucketTip'], 1))
