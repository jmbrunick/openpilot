import json,urllib.request,subprocess,os
T=open('/workspace/comma_jwt.txt').read().strip(); D='1c95345a3286a5db'
segs=[l.strip() for l in open('segs_other.txt') if l.strip()]
F={}
for s in segs:
  r='--'.join(s.split('--')[:2]); n=s.split('--')[2]
  if r not in F:
    q=urllib.request.Request(f"https://api.commadotai.com/v1/route/{D}%7C{r}/files",headers={"Authorization":"JWT "+T}); F[r]=json.load(urllib.request.urlopen(q))
  for kind,dst in (('cameras',f'/workspace/r19/{s}.hevc'),('qlogs',f'/workspace/sr2/ql/{s}.zst')):
    if os.path.exists(dst): continue
    u=[x for x in F[r][kind] if f'/{n}/' in x]
    if not u: print('nourl',kind,s,flush=True); continue
    os.makedirs(os.path.dirname(dst),exist_ok=True)
    subprocess.run(["curl","-s","--retry","3","-o",dst+'.part',u[0]]); os.replace(dst+'.part',dst)
  if os.path.exists(f'/workspace/sr2/ql/{s}.zst') and not os.path.exists(f'/workspace/sr2/ctx/{s}.pkl'):
    subprocess.run(["python3","ext.py",s,f'/workspace/sr2/ql/{s}.zst'])
  print('ok',s,flush=True)
print('DLDONE',flush=True)
