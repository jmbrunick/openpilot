# ruff: noqa
# Off-car training tool (box paths hard-coded). Not imported by openpilot.
import os,sys,json,random,math,time,glob
import numpy as np,cv2,torch
sys.path.insert(0,'/workspace/animal_det/tools'); import yx
D='/workspace/animal_det/data'; TH,TW=320,640; NC=4
torch.set_num_threads(int(os.environ.get('NT','6')))
LAB=json.load(open(f'{D}/labels_ext.json'))
keys=sorted(LAB); random.Random(1).shuffle(keys)
VAL=keys[:300]; TRN=keys[300:]
NEG=sorted(glob.glob(f'{D}/neg/*.jpg')); NEGN=[n for n in NEG if n.endswith('_n.jpg')]
print('train',len(TRN),'val',len(VAL),'neg',len(NEG),'night neg',len(NEGN),flush=True)
def load(k):
  im=cv2.imread(f'{D}/img/{k}'); h,w=im.shape[:2]
  b=np.array([[x0*w,y0*h,x1*w,y1*h,c] for c,x0,y0,x1,y1,g in LAB[k]],np.float32).reshape(-1,5)
  return im,b
def place(canvas,boxes,im,b,x,y,s):
  im2=cv2.resize(im,(max(1,int(im.shape[1]*s)),max(1,int(im.shape[0]*s))),interpolation=cv2.INTER_AREA if s<1 else cv2.INTER_LINEAR)
  H,W=canvas.shape[:2]; h,w=im2.shape[:2]
  x0,y0=max(0,x),max(0,y); x1,y1=min(W,x+w),min(H,y+h)
  if x1<=x0 or y1<=y0: return
  canvas[y0:y1,x0:x1]=im2[y0-y:y1-y,x0-x:x1-x]
  bb=b.copy(); bb[:,:4]*=s; bb[:,[0,2]]+=x; bb[:,[1,3]]+=y
  ow=bb[:,2]-bb[:,0]; oh=bb[:,3]-bb[:,1]
  bb[:,[0,2]]=bb[:,[0,2]].clip(x0,x1); bb[:,[1,3]]=bb[:,[1,3]].clip(y0,y1)
  keep=((bb[:,2]-bb[:,0])>0.45*ow)&((bb[:,3]-bb[:,1])>0.45*oh)
  boxes.extend(bb[keep].tolist())
def lowlight(im,rng):
  f=im.astype(np.float32)/255
  f=f**rng.uniform(1.4,2.8)*rng.uniform(0.25,0.8)
  f[...,0]*=rng.uniform(0.9,1.15); f[...,2]*=rng.uniform(0.85,1.15)
  f=f*255+rng.normal(0,rng.uniform(2,10),f.shape)
  return f.clip(0,255).astype(np.uint8)
def glare(im,rng):
  H,W=im.shape[:2]; out=im.astype(np.float32)
  for _ in range(rng.integers(1,4)):
    cx,cy=rng.uniform(0,W),rng.uniform(0,H*0.8); r=rng.uniform(4,40)
    yy,xx=np.ogrid[:H,:W]; g=np.exp(-((xx-cx)**2+(yy-cy)**2)/(2*r*r))
    out+=g[...,None]*rng.uniform(120,255)
    if rng.random()<0.5: out[int(max(0,cy-1)):int(cy+2),:]+=np.exp(-np.abs(np.arange(W)-cx)/rng.uniform(20,120))[None,:,None]*120
  return out.clip(0,255).astype(np.uint8)
def mblur(im,rng):
  k=int(rng.integers(3,9)); K=np.zeros((k,k),np.float32); K[k//2,:]=1/k
  return cv2.filter2D(im,-1,K)
def cut(im,b,rng):
  # crop one object with context margin
  i=rng.integers(len(b)); x0,y0,x1,y1,c=b[i]; w,h=x1-x0,y1-y0; m=rng.uniform(0.05,0.25)
  X0,Y0=int(max(0,x0-m*w)),int(max(0,y0-m*h)); X1,Y1=int(min(im.shape[1],x1+m*w)),int(min(im.shape[0],y1+m*h))
  return im[Y0:Y1,X0:X1],np.array([[x0-X0,y0-Y0,x1-X0,y1-Y0,c]],np.float32)
def sample(k,rng,train=True):
  r=rng.random()
  boxes=[]
  if not train:
    im,b=load(k); s=min(TW/im.shape[1],TH/im.shape[0]); can=np.full((TH,TW,3),114,np.uint8); place(can,boxes,im,b,0,0,s)
    return can,np.array(boxes,np.float32).reshape(-1,5)
  if r<0.35 and NEG:
    night=rng.random()<0.45 and NEGN
    can=cv2.imread(random.choice(NEGN if night else NEG)).copy()
    for _ in range(rng.integers(1,4)):
      im,b=load(random.choice(TRN)); p,pb=cut(im,b,rng)
      if p.shape[0]<8 or p.shape[1]<8: continue
      th=rng.uniform(14,150); s=th/p.shape[0]
      if p.shape[1]*s>TW*0.6: s=TW*0.6/p.shape[1]
      y=int(rng.uniform(60,TH-th*0.6)); x=int(rng.uniform(-p.shape[1]*s*0.2,TW-p.shape[1]*s*0.8))
      if night or can.mean()<60:
        lum=max(15,can[max(0,y):y+int(th),:].mean()); p=(p.astype(np.float32)*(lum*rng.uniform(1.0,2.5)/max(1,p.mean()))).clip(0,255).astype(np.uint8)
        if rng.random()<0.6: p=cv2.GaussianBlur(p,(3,3),0)
      place(can,boxes,p,pb,x,y,s)
  elif r<0.70:
    can=np.full((TH,TW,3),114,np.uint8); ch,cw=TH//2,TW//2
    for gy in range(2):
      for gx in range(2):
        im,b=load(random.choice(TRN)); s=min(cw/im.shape[1],ch/im.shape[0])*rng.uniform(0.6,1.0)
        place(can,boxes,im,b,gx*cw+int(rng.uniform(0,cw-im.shape[1]*s+1)),gy*ch+int(rng.uniform(0,ch-im.shape[0]*s+1)),s)
  elif r<0.87:
    im,b=load(random.choice(TRN)); s=min(TW/im.shape[1],TH/im.shape[0])*rng.uniform(0.5,1.6)
    can=np.full((TH,TW,3),114,np.uint8)
    place(can,boxes,im,b,int(rng.uniform(min(0,TW-im.shape[1]*s),max(0,TW-im.shape[1]*s)+1)),int(rng.uniform(min(0,TH-im.shape[0]*s),max(0,TH-im.shape[0]*s)+1)),s)
  else:
    can=cv2.imread(random.choice(NEGN if (NEGN and rng.random()<0.5) else NEG)).copy()
  if rng.random()<0.5:
    can=can[:,::-1].copy()
    if boxes: bb=np.array(boxes); bb[:,[0,2]]=TW-bb[:,[2,0]]; boxes=bb.tolist()
  if rng.random()<0.35 and can.mean()>60: can=lowlight(can,rng)
  if rng.random()<0.2: can=glare(can,rng)
  if rng.random()<0.15: can=mblur(can,rng)
  if rng.random()<0.5:
    hsv=cv2.cvtColor(can,cv2.COLOR_BGR2HSV).astype(np.float32); hsv[...,1]*=rng.uniform(0.6,1.3); hsv[...,2]*=rng.uniform(0.7,1.3)
    can=cv2.cvtColor(hsv.clip(0,255).astype(np.uint8),cv2.COLOR_HSV2BGR)
  b=np.array(boxes,np.float32).reshape(-1,5)
  b=b[((b[:,2]-b[:,0])>=4)&((b[:,3]-b[:,1])>=4)]
  return can,b
class DS(torch.utils.data.IterableDataset):
  def __iter__(self):
    wi=torch.utils.data.get_worker_info(); seed=(wi.id if wi else 0)*7919+int(time.time()*1000)%100000
    rng=np.random.default_rng(seed); random.seed(seed)
    while True:
      im,b=sample(None,rng)
      t=np.zeros((60,5),np.float32); b=b[:60]
      if len(b): t[:len(b),0]=b[:,4]; t[:len(b),1]=(b[:,0]+b[:,2])/2; t[:len(b),2]=(b[:,1]+b[:,3])/2; t[:len(b),3]=b[:,2]-b[:,0]; t[:len(b),4]=b[:,3]-b[:,1]
      yield torch.from_numpy(np.ascontiguousarray(im.transpose(2,0,1))).float(),torch.from_numpy(t)
def warm_model():
  m=yx.build(NC); sd=torch.load('/workspace/animal_det/yolox_nano.pth',map_location='cpu')['model']
  own=m.state_dict()
  for k,v in sd.items():
    if k in own and own[k].shape==v.shape: own[k]=v
  # class rows: person<-0, deer<-mean(horse17,sheep18,cow19), large<-mean(horse,cow,bear21), small<-mean(dog16,cat15)
  MAP=[[0],[17,18,19],[17,19,21],[15,16]]
  for i in range(3):
    W=sd[f'head.cls_preds.{i}.weight']; B=sd[f'head.cls_preds.{i}.bias']
    own[f'head.cls_preds.{i}.weight']=torch.stack([W[r].mean(0) for r in MAP]); own[f'head.cls_preds.{i}.bias']=torch.stack([B[r].mean() for r in MAP])
  m.load_state_dict(own); return m
if __name__=='__main__':
  if os.environ.get('PREVIEW'):
    rng=np.random.default_rng(3); random.seed(3); ims=[]
    for i in range(12):
      im,b=sample(None,rng); im=im.copy()
      for x0,y0,x1,y1,c in b: cv2.rectangle(im,(int(x0),int(y0)),(int(x1),int(y1)),[(0,0,255),(0,255,0),(255,0,0),(0,255,255)][int(c)],2)
      ims.append(im)
    cv2.imwrite('/workspace/animal_det/aug_preview.jpg',np.vstack([np.hstack(ims[i:i+2]) for i in range(0,12,2)])); sys.exit()
  ITERS=int(os.environ.get('ITERS','3000')); BS=int(os.environ.get('BS','24'))
  m=warm_model(); m.train()
  g_bn,g_w,g_b=[],[],[]
  for k,v in m.named_modules():
    if hasattr(v,'bias') and isinstance(v.bias,torch.nn.Parameter): g_b.append(v.bias)
    if isinstance(v,torch.nn.BatchNorm2d): g_bn.append(v.weight)
    elif hasattr(v,'weight') and isinstance(v.weight,torch.nn.Parameter): g_w.append(v.weight)
  LR=float(os.environ.get('LR','0.004'))
  opt=torch.optim.SGD(g_bn,lr=LR,momentum=0.9,nesterov=True); opt.add_param_group({'params':g_w,'weight_decay':5e-4}); opt.add_param_group({'params':g_b})
  dl=iter(torch.utils.data.DataLoader(DS(),batch_size=BS,num_workers=int(os.environ.get('NW','3')),persistent_workers=True,prefetch_factor=4))
  t0=time.time(); WARM=150
  for it in range(1,ITERS+1):
    lr=LR*min(1,it/WARM)*(0.05+0.95*0.5*(1+math.cos(math.pi*it/ITERS)))
    for g in opt.param_groups: g['lr']=lr
    x,t=next(dl)
    out=m(x,t); loss=out['total_loss']
    opt.zero_grad(); loss.backward(); opt.step()
    if it%20==0: print(f"it {it} loss {loss.item():.3f} iou {out['iou_loss'].item():.3f} obj {out['conf_loss'].item():.3f} cls {out['cls_loss'].item():.3f} lr {lr:.5f} {(time.time()-t0)/it:.2f}s/it",flush=True)
    if it%500==0 or it==ITERS: torch.save(m.state_dict(),f'/workspace/animal_det/ckpt_{it}.pt'); torch.save(m.state_dict(),'/workspace/animal_det/ckpt_last.pt')
