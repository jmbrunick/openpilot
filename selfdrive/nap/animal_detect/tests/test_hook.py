import math
from types import SimpleNamespace as NS

from openpilot.selfdrive.nap.animal_detect.hook import animal_match


def body(*dets):
  return NS(detections=list(dets))


def d(cls="deer", conf=0.7, tid=0, matched=False, rng=40.0, lat=2.0):
  return NS(cls=cls, conf=conf, radarTrackId=tid, radarMatched=matched, range=rng, lateral=lat)


def test_match_by_radar_track():
  assert animal_match(body(d(tid=9, matched=True, rng=float("nan"))), 0.0, 0.1, 9, 80.0, -5.0) == ("deer", 0.7)


def test_match_by_geometry():
  assert animal_match(body(d(rng=42.0, lat=2.5)), 0.0, 0.1, 3, 40.0, 2.0) == ("deer", 0.7)


def test_no_match_far_away():
  cls, conf = animal_match(body(d(rng=80.0)), 0.0, 0.1, 3, 40.0, 2.0)
  assert cls == "" and math.isnan(conf)


def test_stale_or_missing():
  assert animal_match(body(d()), 0.0, 5.0, 3, 40.0, 2.0)[0] == ""
  assert animal_match(None, 0.0, 0.0, 3, 40.0, 2.0)[0] == ""


def test_best_conf_wins_and_garbage_is_safe():
  assert animal_match(body(d(conf=0.4), d(cls="person", conf=0.8)), 0.0, 0.1, 3, 40.0, 2.0) == ("person", 0.8)
  assert animal_match(NS(detections=None), 0.0, 0.1, 3, 40.0, 2.0)[0] == ""
