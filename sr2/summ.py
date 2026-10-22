import os,sys,importlib,json,subprocess
ms=sys.argv[1:]
for S in ('signs19','signs_other'):
  os.environ['SIGNS']=S
  import score3; importlib.reload(score3)
  base=score3.run('v23')[0]
  print(f'== {S}')
  for m in ms:
    rows,tp,tl,nr,fp=score3.run(m)
    nw=[f"{r[0]}:{r[1]}->{r[2]}" for r,b in zip(rows,base) if r[3]=='wrong' and not (b[3]=='wrong' and b[2]==r[2])]
    wr=[f"{r[0]}:{r[1]}->{r[2]}" for r in rows if r[3]=='wrong']
    print(f"{m:12s} prompt {tp['ok']}/{tp['wrong']}/{tp['miss']}  plate {tl['ok']}/{tl['wrong']}/{tl['miss']}  outside {len(fp)}  wrongs {wr}  new-vs-v23 {nw}")
