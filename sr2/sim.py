"""Replay a45bd4af (v2) or #354 two-stage pipeline on Oct 19 fcamera with v4.1.
Device-time model: proposer pass PROP_MS per propose() call (car median 123 ms), reader READ_MS (car median ~800 ms),
single worker (proposer then reader, sequential), real schedule_after_read duty budget (25%/10 s, detect_hz 1).
cpu-load host pressure / modeld lag NOT simulated (unknown offline). v2: cpu-budget dropped while tracker.budget_exempt.
Usage: sim.py PKG(v2|354) seg"""
import os,sys,json,time,pickle,bisect
MODE,seg=sys.argv[1],sys.argv[2]
PK={'v2':'/workspace/sr2/pkg','354':'/workspace/h17/rp/pkg354'}[MODE]
sys.path.insert(0,PK); sys.path.insert(1,'/workspace/speedsign-eval'); os.environ['NAP_SPEED_SIGN_THREADS']='1'
from openpilot.selfdrive.speedsignd import two_stage as TS
from openpilot.selfdrive.speedsignd.detect import OnnxSpeedSignDetector,_onnx_session
from openpilot.selfdrive.speedsignd.debounce import SignDebounce
from openpilot.selfdrive.speedsignd.hud import LiveSignHold
import openpilot.selfdrive.speedsignd as _P; _P.__path__.append('/workspace/sr2/pkg/openpilot/selfdrive/speedsignd')
import openpilot as _O; _O.__path__.append('/workspace/sr2/pkg/openpilot')
import importlib.util
def load(name,path):
  s=importlib.util.spec_from_file_location(name,path); m=importlib.util.module_from_spec(s); sys.modules[name]=m; s.loader.exec_module(m); return m
SD=load('sdsched','/workspace/sr2/sdsched.py')
G=load('sg2','/workspace/sr2/pkg/openpilot/selfdrive/speedsignd/strip_geom.py') if MODE=='v2' else None
import sslib
PROP_MS=float(os.environ.get('PROP_MS',123)); READ_MS=float(os.environ.get('READ_MS',800))
MP='/workspace/v41/v41.onnx'; sess,be=_onnx_session(MP); getattr(sess,'allow_compile',lambda a:None)(True)
DET=OnnxSpeedSignDetector(MP,sess,backend=be); PROP=TS.Proposer('/workspace/proposer/sign_proposer.onnx'); assert PROP.ok
C=pickle.load(open(f'/workspace/sr2/ctx/{seg}.pkl','rb'))
enc={fid:t for t,fid,_ in C['enc']}
def at(arr,t,k=1):
  if not arr: return None
  ts=[a[0] for a in arr]; i=min(max(bisect.bisect(ts,t)-1,0),len(arr)-1); return arr[i] if k is None else arr[i][k]
def speed(t):
  g=[a for a in C['gps']]; 
  if g:
    x=at(g,t,None)
    if x and abs(x[0]-t)<2 and x[2]: return float(x[1])
  return at(C['v'],t)
def rpy(t):
  x=at(C['cal'],t,None); return x[2] if x and x[1]=='calibrated' else None
def edge(t):
  x=at(C['md'],t,None)
  return None if x is None or abs(x[0]-t)>0.3 else (x[2],x[3],x[4])
OUTD=os.environ.get('OUT',MODE)
tr=TS.SignTracker(); deb=SignDebounce(); hold=LiveSignHold()
log=[]; next_prop=-1e9; next_detect=-1e9; busy=-1e9; spans=[]; scan_i=0; win=0
box=dict(prop=[],read=[])
period_s=1.0; budget_s=0.1; cap_s=0.8
t00=None
for i,y,u,v in sslib.frames(f'/workspace/r19/{seg}.hevc',threads=1):
  t=enc.get(i)
  if t is None: continue
  if t00 is None: t00=t
  if t<next_prop or t<busy: continue
  vt=speed(t) if MODE=='v2' else None
  if MODE=='v2':
    main,up=G.strips(1928,1208,rpy=rpy(t),edge=edge(t)); scan_i+=1
    if up is not None and scan_i%2: up=None
    srect=TS.union_rect(main,up)
  else: main,up,srect=TS.STRIP_RECT,None,TS.STRIP_RECT
  a=time.perf_counter(); props=PROP.propose(sslib.nv12_crop(y,u,v,main)); npass=1
  if up is not None: props=list(props)+list(PROP.propose(sslib.nv12_crop(y,u,v,up))); npass=2
  box['prop'].append((time.perf_counter()-a)*1000/npass)
  dev_t=npass*PROP_MS/1000
  allow=t>=next_detect
  if MODE=='v2':
    nt=t+tr.proposer_period_s(t,vt); cand=TS.select_read_target(tr,props,t,allow_read=allow,v_ego=vt,next_t=nt,strip=srect)
  else:
    cand=TS.select_read_target(tr,props,t,allow_read=allow)
  rec=dict(f=i,t=round(t-t00,2),v=None if vt is None else round(vt,1),np=len(props),up=up is not None,main=main)
  if cand is not None:
    rect=TS.capture_rect(cand.crop_box()); a=time.perf_counter()
    signs=DET.detect(None,nv12=sslib.nv12_crop(y,u,v,rect)); box['read'].append((time.perf_counter()-a)*1000); d=DET.diag_dict()
    r=deb.update_split(signs,t); live,hm,hc=hold.update(r.hud,t)
    rec.update(tid=cand.tid,rect=rect,h=round(cand.box[3]),signs=[(int(s.mph),round(float(s.conf),3)) for s in signs],hud=[int(s.mph) for s in r.hud],live=hm if live else 0)
    TS.SignTracker.mark_read(tr,cand,got_mph=bool(r.hud),neg=TS.neg_read(d))
    end=t+dev_t+READ_MS/1000
    ex=MODE=='v2' and tr.budget_exempt(end)
    nm=SD.schedule_after_read(now=end,infer_s=READ_MS/1000,period_s=period_s,budget_s=budget_s,cap_s=cap_s,backoff_s=0.0,
      sign_like=False,window_used=win,window_count=1,burst_left=0,burst_window=0,burst_cluster_s=0.0,spans=spans)[0]
    spans.append((end,READ_MS/1000)); spans=[s for s in spans if end-s[0]<=SD.DUTY_WINDOW_S]
    next_detect=max(next_detect,nm); busy=end; rec['exempt']=ex
  else:
    busy=t+dev_t; _l,_m,_c=hold.update([],t); rec['live']=_m if _l else 0
  per=tr.proposer_period_s(t,vt) if MODE=='v2' else tr.proposer_period_s(t)
  next_prop=max(t+per,busy)
  log.append(rec)
os.makedirs(f'/workspace/sr2/out/{OUTD}',exist_ok=True)
json.dump(dict(log=log,box=box,t00=t00),open(f'/workspace/sr2/out/{OUTD}/{seg}.json','w'))
import statistics as st
print(MODE,seg,'ticks',len(log),'reads',sum('tid' in r for r in log),'prop_ms_box',round(st.median(box['prop']),2),'read_ms_box',round(st.median(box['read']),1) if box['read'] else None)
