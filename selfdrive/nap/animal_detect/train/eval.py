# ruff: noqa
# Off-car training tool (box paths hard-coded). Not imported by openpilot.
"""Offline eval with the on-car code path: NV12 frames -> roi tiles -> ONNX (onnxruntime here, tinygrad on car)
-> postprocess.decode -> fuse.Tracker. Usage: eval.py MODEL.onnx deer|night [conf]"""
import sys,os,json,subprocess,math,collections
sys.path.insert(0,'/tmp/oproot'); sys.path.insert(0,'/workspace/animal_det/op')
import numpy as np,cv2,onnxruntime as ort
from openpilot.selfdrive.nap.animal_detect import roi,postprocess,fuse,config
from openpilot.selfdrive.nap.animal_detect.animaldetd import cams_from_calib
W,H=1928,1208
CAL19E=(0.00086,-0.0738,-0.0217,1.228,0.0,0.018,-0.026)
CAL169=(0.00068,-0.0863,-0.0193,1.22,0.0,0.020,-0.027)
def frames(p,step=1,start=0,stop=10**9):
  cmd=['ffmpeg','-v','error','-threads','4','-i',p,'-f','rawvideo','-pix_fmt','nv12','-']
  pr=subprocess.Popen(cmd,stdout=subprocess.PIPE,bufsize=W*H*3//2)
  n=0
  while True:
    b=pr.stdout.read(W*H*3//2)
    if len(b)<W*H*3//2 or n>stop: break
    if n>=start and (n-start)%step==0:
      a=np.frombuffer(b,np.uint8); yield n,a[:W*H].reshape(H,W),a[W*H:].reshape(H//2,W)
    n+=1
  pr.kill()
class Model:
  def __init__(s,path):
    so=ort.SessionOptions(); so.intra_op_num_threads=int(os.environ.get('NT','4'))
    s.sess=ort.InferenceSession(path,so,providers=['CPUExecutionProvider'])
  def __call__(s,tiles):
    x=np.full((3,3,320,640),114,np.float32)
    for i,t in enumerate(tiles): x[i]=t.transpose(2,0,1)
    return s.sess.run(None,{'input':x})[0][:len(tiles)],0.0
def run_frame(model,y,uv,cam,conf):
  tiles=roi.road_tiles(cam); imgs=[roi.bgr_from_nv12(y,uv,t) for t in tiles]
  out,_=model(imgs); res=postprocess.decode(out,conf=conf)
  dets=[]
  for t,found in zip(tiles,res):
    for cls,c,box in found:
      sb=roi.tile_to_source(t,box); dets.append(fuse.Detection(cls,c,'road',sb,roi.bearing_deg(cam,0.5*(sb[0]+sb[2])),roi.ground_range(cam,sb[3])))
  return dets,imgs
def iou(a,b): return postprocess.iou(a,b)
if __name__=='__main__':
  mp,mode=sys.argv[1],sys.argv[2]; conf=float(sys.argv[3]) if len(sys.argv)>3 else config.DET_CONF
  model=Model(mp); tag=os.path.basename(mp).replace('.onnx','')
  if mode=='deer':
    sys.path.insert(0,'/workspace/animal_scope')
    src=open('/workspace/animal_scope/make_v2.py').read(); D=eval(src.split('D=',1)[1].split('\nSIGN')[0])
    keys=sorted(k for k,v in D.items() if v)
    def gt(f):
      if f in D and D[f]: return D[f]
      lo=max([k for k in keys if k<=f],default=None); hi=min([k for k in keys if k>=f],default=None)
      if lo is None or hi is None: return None
      a,b=D[lo],D[hi]; w=(f-lo)/max(1,hi-lo)
      st=a[4] if a[4]==b[4] else ('hidden' if 'hidden' in (a[4],b[4]) else 'unsure' if 'unsure' in (a[4],b[4]) else 'faint')
      return tuple(a[i]*(1-w)+b[i]*w for i in range(4))+(st,)
    road,_=cams_from_calib(CAL19E)
    rows=[]; tr=fuse.Tracker(); best=None
    for n,y,uv in frames('/workspace/animal_det/fcamera19e_93.hevc',1,700,860):
      dets,imgs=run_frame(model,y,uv,road,0.05)
      g=gt(n); hit=None
      for d in dets:
        if g and iou(d.box,g[:4])>=0.25 and (hit is None or d.conf>hit.conf): hit=d
      allan=[d for d in dets if d.is_animal]
      fp=[d for d in dets if d.conf>=conf and not (g and iou(d.box,g[:4])>=0.25)]
      if n%4==0:   # 5 Hz tracker like the car
        fd=fuse.fuse([d for d in dets if d.conf>=conf],[],tr)
        conf_animal=[d for d in fd if d.is_animal and d.confirmed and g and iou(d.box,g[:4])>=0.25]
      else: conf_animal=None
      rows.append(dict(f=n,gt=g[4] if g else None,hit_cls=hit.cls if hit else None,hit_conf=round(hit.conf,3) if hit else 0.0,
        hit_iou=round(iou(hit.box,g[:4]),2) if hit else 0.0,hit_range=round(hit.range_cam,1) if hit and math.isfinite(hit.range_cam) else None,
        fp=[(d.cls,round(d.conf,2),[int(v) for v in d.box]) for d in fp],confirmed=None if conf_animal is None else bool(conf_animal)))
      if hit and (best is None or hit.conf>best[0]): best=(hit.conf,n,imgs,hit)
    vis=[r for r in rows if r['gt'] in ('ok','faint')]
    det=[r for r in vis if r['hit_cls'] in config.ANIMAL_CLASSES and r['hit_conf']>=conf]
    anyc=[r for r in vis if r['hit_cls'] is not None and r['hit_conf']>=conf]
    conf5=[r for r in rows if r['confirmed']]
    summ=dict(model=tag,conf=conf,visible_frames=len(vis),animal_hits=len(det),any_class_hits=len(anyc),
              first_animal_frame=min([r['f'] for r in det],default=None),confirmed_5hz_cycles=len(conf5),
              first_confirmed=min([r['f'] for r in conf5],default=None),
              max_animal_conf=max([r['hit_conf'] for r in vis if r['hit_cls'] in config.ANIMAL_CLASSES],default=0),
              false_boxes=sum(len(r['fp']) for r in rows),frames=len(rows))
    print(json.dumps(summ)); json.dump(dict(summary=summ,rows=rows),open(f'/workspace/animal_det/eval_deer_{tag}.json','w'),indent=0)
  else:
    SEGS=[('/workspace/animal_det/night/r19e/fcamera_%d'%s,CAL19E,'19e',s) for s in (90,91,92,94,95,96,97,98,99,100)]
    SEGS+=[('/workspace/r169/fcamera_%d'%s,CAL169,'169',s) for s in (47,54,56,58,59,62,64,65,66,67,68)]
    if os.environ.get('SEGS'): SEGS=SEGS[:int(os.environ['SEGS'])]
    os.makedirs(f'/workspace/animal_det/fa_{tag}',exist_ok=True)
    events=[]; cycles=0; raw=collections.Counter()
    for p,cal,r,s in SEGS:
      if not os.path.exists(p): continue
      road,_=cams_from_calib(cal); tr=fuse.Tracker(); seen=set()
      for n,y,uv in frames(p,4):
        dets,imgs=run_frame(model,y,uv,road,conf); cycles+=1
        for d in dets: raw[d.cls]+=1
        for d in fuse.fuse(dets,[],tr):
          if d.confirmed and d.track_id not in seen:
            seen.add(d.track_id)
            x0,y0,x1,y1=[int(v) for v in d.box]
            bgr=cv2.cvtColor(np.vstack([y,uv]),cv2.COLOR_YUV2BGR_NV12)
            cx,cy=(x0+x1)//2,(y0+y1)//2; X0,Y0=max(0,cx-160),max(0,cy-90)
            crop=bgr[Y0:Y0+180,X0:X0+320].copy(); cv2.rectangle(crop,(x0-X0,y0-Y0),(x1-X0,y1-Y0),(0,0,255),1)
            fn=f'{r}_{s}_{n}_{d.cls}_{d.conf:.2f}.jpg'; cv2.imwrite(f'/workspace/animal_det/fa_{tag}/{fn}',crop)
            events.append(dict(route=r,seg=s,frame=n,cls=d.cls,conf=round(d.conf,3),box=[x0,y0,x1,y1],range=round(d.range_cam,1) if math.isfinite(d.range_cam) else None,img=fn))
      print(r,s,'cycles',cycles,'events',len(events),flush=True)
    hours=cycles/config.RATE_HZ/3600
    summ=dict(model=tag,conf=conf,hours=round(hours,3),cycles=cycles,confirmed_events=len(events),events_per_hour=round(len(events)/hours,1) if hours else None,
              by_class=dict(collections.Counter(e['cls'] for e in events)),raw_dets=dict(raw))
    print(json.dumps(summ)); json.dump(dict(summary=summ,events=events),open(f'/workspace/animal_det/eval_night_{tag}.json','w'),indent=0)
