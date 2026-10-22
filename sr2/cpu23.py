import json,sys,os
segs=[l.strip() for l in open(os.environ.get('SEGS','segs.txt')) if l.strip()]
for m in sys.argv[1:]:
  P=R=D=0; pk_p=pk_r=0
  for g in segs:
    try: L=json.load(open(f'out/{m}/{g}.json'))['log']
    except FileNotFoundError: continue
    D+=60
    ev=[(r['f']/20,r.get('np',0)*0.124,0.8 if 'tid' in r else 0) for r in L]
    P+=sum(e[1] for e in ev); R+=sum(e[2] for e in ev)
    for i in range(0,1200,20):
      w=[e for e in ev if i/20<=e[0]<i/20+10]
      pk_p=max(pk_p,sum(e[1] for e in w)/10); pk_r=max(pk_r,sum(e[2] for e in w)/10)
  print(f'{m}: proposer avg {100*P/D:.1f}% peak10s {100*pk_p:.0f}% | reader avg {100*R/D:.1f}% peak10s {100*pk_r:.0f}% | total avg {100*(P+R)/D:.1f}%')
