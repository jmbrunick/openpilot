# animal_yolox_nano.onnx

Camera-first animal/person detector for the NAP log-only animal detector (animaldetd).

- Architecture: YOLOX-nano (Megvii, https://github.com/Megvii-BaseDetection/YOLOX), **Apache-2.0**.
  Depthwise PAFPN, width 0.25, depth 0.33, ~0.9 M parameters, 0.64 GMAC per 320x640 tile.
- Start point: Megvii's COCO `yolox_nano.pth` (Apache-2.0). Class rows warm-started from
  COCO person / horse+sheep+cow / horse+cow+bear / cat+dog.
- No Ultralytics / AGPL code, weights or labels were used to build or train it.
- Input: `input` float32 [3, 3, 320, 640], BGR 0..255, no normalisation (two native-
  resolution road tiles + one reduced wide tile). Output: `output` [3, 4200, 9] decoded
  (cx, cy, w, h, objectness, person, deer, largeAnimal, smallAnimal) in tile pixels.
- Classes: person, deer, largeAnimal (horse, cattle, sheep, goat, pig, bear, boar, mouflon),
  smallAnimal (dog, cat, fox, raccoon).

## Training data
- Open Images V7 boxes (annotations CC BY 4.0, images under their own licences as listed by
  Open Images): all non-depiction Deer images, plus samples of Horse, Cattle, Bull, Sheep,
  Goat, Pig, Bear, Dog, Cat, Fox, Raccoon and Person.
- LILA BC Missouri Camera Traps (CDLA-Permissive): boxed deer / boar / mouflon / fox frames,
  mostly night infrared.
- Hard negatives: road tiles from the owner's own drives (day, dusk and night), with
  frames holding any COCO person/animal (teacher: the Apache COCO YOLOX-nano) dropped.
  The evaluation segments were held out.
- Augmentation: copy-paste of objects onto real road tiles at 14-150 px height (night
  tiles matched in brightness), mosaic, low-light gamma + noise, headlight glare, motion
  blur, HSV jitter, flips.
- Not used: COCO images (host timed out), NightOwls and BDD100K (need registration).

## Limits
Trained on a CPU only (no GPU on the build box), a short fine-tune. Night recall is bound
by headlight reach (~50-70 m low beam). Log only until the on-car gates pass.
