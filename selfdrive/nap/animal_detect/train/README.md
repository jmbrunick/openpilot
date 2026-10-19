# Off-car training / export / eval scripts (not run on the car)

How models/animal_yolox_nano.onnx was built (box paths are hard-coded; edit before reuse):
1. `build_ds.py` - Open Images V7 boxes (deer, large/small animals, people) + LILA Missouri camera traps.
2. `negtiles.py` - road tiles from the owner's drives as hard negatives (eval segments held out).
3. `train.py` - fine-tune Megvii YOLOX-nano (Apache-2.0, needs a YOLOX checkout on PYTHONPATH) at 320x640.
4. `export.py` - ONNX export (opset 13, decoded output), then onnxslim.
5. `eval.py` - replays fcamera NV12 through this module's roi/postprocess/fuse with onnxruntime.
