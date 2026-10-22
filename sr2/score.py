import json,sys,os
sys.path.insert(0,'/workspace/bookmarks_oct19'); from signs19 import S
PRE=int(os.environ.get('PRE',60)); POST=int(os.environ.get('POST',40))
def run(m):
  out=[];tot=dict(sh_ok=0,sh_wrong=0,miss=0);nr=0
  G={}
  for bm,seg,f0,f1,lab in S: G.setdefault(bm,[]).append((seg,f0,f1,lab))
  for bm,parts in G.items():
    W=[]
    for seg,f0,f1,lab in parts:
      L=json.load(open(f'out/{m}/{seg}.json'))['log']; lab=int(lab)
      W+=[r for r in L if f0-PRE<=r['f']<=f1+POST]
    R=[r for r in W if 'tid' in r]; nr+=len(R)
    ok=sum(1 for r in R if r['signs'] and r['signs'][0][0]==lab); wr=[r['signs'][0] for r in R if r['signs'] and r['signs'][0][0]!=lab]; em=sum(1 for r in R if not r['signs'])
    hud=sorted({h for r in W if 'hud' in r for h in r['hud']}|{r['live'] for r in W if r.get('live')})
    if not hud: s='no show'; tot['miss']+=1
    elif hud==[lab]: s='SHOW ok'; tot['sh_ok']+=1
    elif lab in hud: s=f'SHOW {hud}'; tot['sh_ok']+=1
    else: s=f'SHOW {hud}'; tot['sh_wrong']+=1
    out.append(f'#{bm} {lab} | {len(R)} {ok}/{len(wr)}/{em} {s}'+(f' w:{wr[0][0]}@{wr[0][1]}' if wr else ''))
  return out,tot,nr
ms=sys.argv[1:]
res=[run(m) for m in ms]
for i in range(len(res[0][0])): print(' || '.join(r[0][i] for r in res))
for m,r in zip(ms,res): print(m,r[1],'reads',r[2])
