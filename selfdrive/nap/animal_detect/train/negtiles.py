# ruff: noqa
# Off-car training tool (box paths hard-coded). Not imported by openpilot.
# Domain hard-negative tiles from Justin's drives (NOT the eval segments). Teacher filter: COCO yolox-nano (Apache).
import subprocess,numpy as np,cv2,os,sys,json,torch; torch.set_num_threads(2)
sys.path.insert(0,'/workspace/animal_det/tools'); import yx
W,H=1928,1208; VH=800; U=964
OUT='/workspace/animal_det/data/neg'; os.makedirs(OUT,exist_ok=True)
SEGS=[('r169',s) for s in (41,42)]+[('r19f',s) for s in (8,65)]+[('r17c',s) for s in (12,36)]+[('r189',s) for s in (14,)]
m=yx.load_coco(); BAD={0,14,15,16,17,18,19,20,21,22,23}
kept=dropped=0
def frames(p,step):
  cmd=['ffmpeg','-v','error','-threads','6','-i',p,'-vf',f"select='not(mod(n\\,{step}))'",'-vsync','0','-f','rawvideo','-pix_fmt','bgr24','-']
  pr=subprocess.Popen(cmd,stdout=subprocess.PIPE,bufsize=W*H*3)
  while True:
    b=pr.stdout.read(W*H*3)
    if len(b)<W*H*3: break
    yield np.frombuffer(b,np.uint8).reshape(H,W,3)
for d,s in SEGS:
  p=f'/workspace/animal_det/night/r19e/fcamera_28' if d=='r19e_28' else f'/workspace/{d}/fcamera_{s}'
  if not os.path.exists(p): continue
  night = d in ('r19e_28',) or (d=='r169' and s>=37)
  step=20 if night else 40
  for k,f in enumerate(frames(p,step)):
    y0=VH-110
    tiles=[np.ascontiguousarray(f[y0:y0+320,U-640:U]),np.ascontiguousarray(f[y0:y0+320,U:U+640])]
    res=yx.infer(m,tiles,80,conf=0.25)
    for j,(t,r) in enumerate(zip(tiles,res)):
      bad = r is not None and any(int(c) in BAD for c in r[:,6].tolist())
      if bad: dropped+=1; continue
      cv2.imwrite(f'{OUT}/{d}_{s}_{k}_{j}{"_n" if night else ""}.jpg',t,[cv2.IMWRITE_JPEG_QUALITY,90]); kept+=1
  print(d,s,'kept',kept,'dropped',dropped,flush=True)
