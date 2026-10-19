# ruff: noqa
# Off-car training tool (box paths hard-coded). Not imported by openpilot.
import csv,random,json,collections,os,io,urllib.request,concurrent.futures as cf
import cv2,numpy as np
random.seed(0)
D='/workspace/animal_det/data'
CLS={'/m/01g317':0,'/m/09kx5':1,'/m/03k3r':2,'/m/01xq0k1':2,'/m/0cnyhnx':2,'/m/07bgp':2,'/m/03fwl':2,'/m/068zj':2,'/m/01dws':2,'/m/0bt9lr':3,'/m/01yrx':3,'/m/0306r':3,'/m/0dq75':3}
boxes=collections.defaultdict(list); split={}
src={}
for r in csv.reader(open(f'{D}/oi_boxes.csv')):
  if r[2] not in CLS: continue
  if r[11]=='1': continue  # depiction
  boxes[r[0]].append((CLS[r[2]],float(r[4]),float(r[6]),float(r[5]),float(r[7]),int(r[10])))
  src[r[0]]=r[1]
bycls=collections.defaultdict(set)
for i,b in boxes.items():
  for c,*_ in b: bycls[c].add(i)
print({c:len(v) for c,v in bycls.items()})
pick=set(bycls[1])
lg=sorted(bycls[2]-pick); random.shuffle(lg); pick|=set(lg[:2200])
sm=sorted(bycls[3]-pick); random.shuffle(sm); pick|=set(sm[:900])
# persons: prefer images with 1-6 non-group person boxes that are not huge (street-like)
pp=[i for i in sorted(bycls[0]-pick) if all(c==0 for c,*_ in boxes[i]) and 1<=len(boxes[i])<=8 and min((b[3]-b[1])*(b[4]-b[2]) for b in boxes[i])<0.08]
random.shuffle(pp); pick|=set(pp[:2200])
pick=sorted(pick); print('pick',len(pick))
def fetch(i):
  out=f'{D}/img/oi_{i}.jpg'
  if os.path.exists(out): return i,True
  for sub in ('train','validation','test'):
    try:
      raw=urllib.request.urlopen(f'https://s3.amazonaws.com/open-images-dataset/{sub}/{i}.jpg',timeout=30).read()
      im=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
      if im is None: continue
      s=640/max(im.shape[:2])
      if s<1: im=cv2.resize(im,None,fx=s,fy=s,interpolation=cv2.INTER_AREA)
      cv2.imwrite(out,im,[cv2.IMWRITE_JPEG_QUALITY,88]); return i,True
    except Exception: continue
  return i,False
ok={}
with cf.ThreadPoolExecutor(48) as ex:
  for n,(i,g) in enumerate(ex.map(fetch,pick)):
    ok[i]=g
    if n%1000==0: print(n,flush=True)
labels={}
for i in pick:
  if ok[i]: labels[f'oi_{i}.jpg']=[[c,x0,y0,x1,y1,g] for c,x0,y0,x1,y1,g in boxes[i]]
# Missouri camera traps
j=json.load(open(f'{D}/missouri_camera_traps_set1.json'))
cats={c['id']:c['name'] for c in j['categories']}
M={'red_deer':1,'red_brocket_deer':1,'white_tailed_deer':1,'roe_deer':1,'wild_boar':2,'mouflon':2,'red_fox':3,'collared_peccary':2}
imgs={i['id']:i for i in j['images']}
anns=collections.defaultdict(list)
for a in j['annotations']:
  if 'bbox' in a and cats[a['category_id']] in M: anns[a['image_id']].append((M[cats[a['category_id']]],a['bbox']))
def fetch_m(iid):
  fn=imgs[iid]['file_name']; out=f"{D}/img/mct_{iid}.jpg"
  try:
    raw=urllib.request.urlopen('https://storage.googleapis.com/public-datasets-lila/missouricameratraps/images/'+urllib.request.quote(fn),timeout=60).read()
    im=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR); H,W=im.shape[:2]
    s=640/max(H,W); im=cv2.resize(im,None,fx=s,fy=s,interpolation=cv2.INTER_AREA); cv2.imwrite(out,im,[cv2.IMWRITE_JPEG_QUALITY,88])
    return iid,[[c,max(0,x/W),max(0,y/H),min(1,(x+w)/W),min(1,(y+h)/H),0] for c,(x,y,w,h) in anns[iid]]
  except Exception as e: return iid,None
with cf.ThreadPoolExecutor(32) as ex:
  for iid,lab in ex.map(fetch_m,list(anns)):
    if lab: labels[f'mct_{iid}.jpg']=lab
json.dump(labels,open(f'{D}/labels_ext.json','w'))
cc=collections.Counter(b[0] for v in labels.values() for b in v)
print('images',len(labels),'boxes per class',dict(cc))
