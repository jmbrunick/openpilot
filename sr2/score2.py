"""Score at the sign's position: the plate value at the last tick <= f_end+POST.
correct = posted value; wrong = any other value (incl. an old value still held); miss = blank."""
import json,sys,os
sys.path.insert(0,'/workspace/bookmarks_oct19'); from signs19 import S
POST=int(os.environ.get('POST',10)); PRE=60
def run(m):
  G={}
  for bm,seg,f0,f1,lab in S: G.setdefault(bm,[]).append((seg,f0,f1,int(lab)))
  rows=[];tot=dict(ok=0,wrong=0,miss=0);nr=0
  for bm,parts in G.items():
    R=[];val=0;lab=parts[0][3]
    for seg,f0,f1,_ in parts:
      L=json.load(open(f'out/{m}/{seg}.json'))['log']
      R+=[r for r in L if f0-PRE<=r['f']<=f1+POST and 'tid' in r]
    seg,f0,f1,_=parts[-1]; L=json.load(open(f'out/{m}/{seg}.json'))['log']
    at=[r for r in L if r['f']<=f1+POST]
    val=at[-1].get('live',0) if at else 0
    k='ok' if val==lab else ('miss' if not val else 'wrong'); tot[k]+=1; nr+=len(R)
    px=[r.get('h') for r in R]
    rows.append((bm,lab,val,k,len(R),px))
  return rows,tot,nr
ms=sys.argv[1:]; res={m:run(m) for m in ms}
print('sign lab | '+' | '.join(ms))
for i,row in enumerate(res[ms[0]][0]):
  print(f'#{row[0]} {row[1]} | '+' | '.join(f"{res[m][0][i][3]}:{res[m][0][i][2]} n={res[m][0][i][4]} px={res[m][0][i][5]}" for m in ms))
for m in ms: print(m,res[m][1],'reads',res[m][2])
