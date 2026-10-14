import sys, collections
cols = "t,st,ph,aT,aC,sT,sC,sO,oT,oS,oP,oV,oPWM,oFF,fl,lm".split(",")
rows = []
for line in open(sys.argv[1]):
    if line.startswith("D,"):
        v = list(map(int, line.strip().split(",")[1:]))
        rows.append(dict(zip(cols, v)))
names = "IDLE DOWN SCOOP PICKUP TRANSFER BEFORE_EXTEND OUTTAKE_EXTEND OUTTAKE_RETRACT DONE".split()
dts = [b["t"]-a["t"] for a,b in zip(rows, rows[1:])]
print(f"rows={len(rows)} dt min/max={min(dts)}/{max(dts)} ms  loop max us overall={max(r['lm'] for r in rows)}  median lm={sorted(r['lm'] for r in rows)[len(rows)//2]}")
by = collections.defaultdict(list)
for r in rows: by[r["st"]].append(r)
for st, rs in sorted(by.items()):
    err = [abs(r["oS"]-r["oP"]) for r in rs]
    print(f"{names[st]:16s} n={len(rs):4d} |oS-oP| max={max(err):5d}  pwm[min,max]=[{min(r['oPWM'] for r in rs)},{max(r['oPWM'] for r in rs)}]  sO range=[{min(r['sO'] for r in rs)},{max(r['sO'] for r in rs)}] aC range=[{min(r['aC'] for r in rs)},{max(r['aC'] for r in rs)}]")
