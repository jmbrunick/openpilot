# ruff: noqa
# Off-car training tool (box paths hard-coded). Not imported by openpilot.
import sys,torch,numpy as np,cv2
sys.path.insert(0,'/workspace/animal_det/YOLOX')
from yolox.models import YOLOX,YOLOPAFPN,YOLOXHead
from yolox.utils import postprocess
def build(nc=80,depthwise=True):
  def init(M):
    for m in M.modules():
      if isinstance(m,torch.nn.BatchNorm2d): m.eps=1e-3; m.momentum=0.03
  inch=[256,512,1024]
  backbone=YOLOPAFPN(0.33,0.25,in_channels=inch,act='silu',depthwise=depthwise)
  head=YOLOXHead(nc,0.25,in_channels=inch,act='silu',depthwise=depthwise)
  m=YOLOX(backbone,head); m.apply(init); m.head.initialize_biases(1e-2); return m
def load_coco():
  m=build(80); sd=torch.load('/workspace/animal_det/yolox_nano.pth',map_location='cpu')['model']; m.load_state_dict(sd); return m.eval()
def infer(m,tiles,nc,conf=0.25,nms=0.45):
  x=torch.from_numpy(np.stack([t.transpose(2,0,1) for t in tiles]).astype(np.float32))
  with torch.no_grad(): out=m(x)
  return postprocess(out,nc,conf,nms,class_agnostic=True)
