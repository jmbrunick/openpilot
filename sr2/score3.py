import os
"""v2.3 scoring. prompt = last proposal raised in the sign window; plate = Yes model
(plate changes only when a proposal equals the posted value = Yes; anything else = No).
Old builds (no 'prop' field): prompt = plate = value shown at the sign (score2 rule)."""
import json,sys,os
sys.path.insert(0,'/workspace/bookmarks_oct19'); sys.path.insert(0,'/workspace/sr2')
S=__import__(os.environ.get('SIGNS','signs19')).S
PRE=60; PPRE=int(os.environ.get('PPRE',400)); POST=int(os.environ.get('POST',10))
def run(m):
  G={}
  for bm,seg,f0,f1,lab in S: G.setdefault(bm,[]).append((seg,f0,f1,int(lab)))
  segs=sorted({s[1] for s in S},key=lambda g:(g.split('--')[0],int(g.split('--')[2])))
  logs={g:json.load(open(f'out/{m}/{g}.json'))['log'] for g in segs}
  newv='prop' in json.dumps(logs[segs[1]][:2000]) or any('prop' in r for L in logs.values() for r in L)
  win={g:[] for g in segs}
  for bm,parts in G.items():
    for seg,f0,f1,lab in parts: win[seg].append((f0-PRE,f1+POST,lab,bm,f0-PPRE))
  plate_at={}; falsep=[]; early=[]; prompts={}
  route=None; plate=0; prev_live=0
  for g in segs:
    if g.split('--')[0]!=route: route=g.split('--')[0]; plate=0
    for r in logs[g]:
      if newv and r.get('prop'):
        p=r['prop']
        if os.environ.get('PLATE')=='follow': plate=p
        inw=[w for w in win[g] if w[0]<=r['f']<=w[1]]
        if not inw:
          ah=sorted([w for w in win[g] if w[4]<=r['f']<w[0]],key=lambda w:w[0])[:1]
          if ah: inw=ah; early.append((ah[0][3],w_:=ah[0][0]+PRE-r['f']))
        if inw:
          for w in inw: prompts[w[3]]=p
          if any(w[2]==p for w in inw): plate=p
        else: falsep.append((g[-6:],r['f'],p))
      if not newv and r.get('live') and r.get('live')!=prev_live and not [w for w in win[g] if w[4]<=r['f']<=w[1]]: falsep.append((g[-6:],r['f'],r['live']))
      prev_live=r.get('live',0)
      r['_plate']=plate
  rows=[];tp=dict(ok=0,wrong=0,miss=0);tl=dict(ok=0,wrong=0,miss=0);nr=0
  for bm,parts in G.items():
    lab=parts[0][3]; seg,f0,f1,_=parts[-1]
    n=sum(1 for sg,a,b,_ in parts for r in logs[sg] if a-PRE<=r['f']<=b+POST and 'tid' in r); nr+=n
    px=[r.get('h') for sg,a,b,_ in parts for r in logs[sg] if a-PRE<=r['f']<=b+POST and 'tid' in r]
    at=[r for r in logs[seg] if r['f']<=f1+POST]
    if newv:
      pv=prompts.get(bm,0); lv=at[-1]['_plate'] if at else 0
    else:
      pv=lv=(at[-1].get('live',0) if at else 0)
    kp='ok' if pv==lab else ('miss' if not pv else 'wrong'); kl='ok' if lv==lab else ('miss' if not lv else 'wrong')
    tp[kp]+=1; tl[kl]+=1; rows.append((bm,lab,pv,kp,lv,kl,n,px))
  run.early=early
  return rows,tp,tl,nr,falsep
if __name__=='__main__':
  ms=sys.argv[1:]; res={m:run(m) for m in ms}
  print('sign | '+' | '.join(ms))
  for i,row in enumerate(res[ms[0]][0]):
    print(f'#{row[0]} {row[1]} | '+' | '.join(f"P:{res[m][0][i][3]}{res[m][0][i][2] or ''} L:{res[m][0][i][5]}{res[m][0][i][4] or ''} n={res[m][0][i][6]}" for m in ms))
  for m in ms: print(m,'prompt',res[m][1],'plate',res[m][2],'reads',res[m][3],'false-prompts-outside',len(res[m][4]))
