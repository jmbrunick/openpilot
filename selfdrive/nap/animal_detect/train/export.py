# ruff: noqa
# Off-car training tool (box paths hard-coded). Not imported by openpilot.
import sys,torch,os
sys.path.insert(0,'/workspace/animal_det/tools'); import yx
ck=sys.argv[1]; out=sys.argv[2]
m=yx.build(4)
if ck!='warm': m.load_state_dict(torch.load(ck,map_location='cpu'))
else:
  import train; m=train.warm_model()
m.eval(); m.head.decode_in_inference=True
x=torch.zeros(3,3,320,640)
torch.onnx.export(m,x,out,input_names=['input'],output_names=['output'],opset_version=13,dynamo=False)
import onnx; mo=onnx.load(out); onnx.checker.check_model(mo)
print('exported',out,os.path.getsize(out),[ (o.name,[d.dim_value for d in o.type.tensor_type.shape.dim]) for o in mo.graph.output], sorted({n.op_type for n in mo.graph.node}))
