import json
import math
from types import SimpleNamespace

from openpilot.selfdrive.speedsignd.auto_confirm import AutoConfirm, match_source
from openpilot.selfdrive.speedsignd.confirm import PlateState

LAT, LON = 45.3088, -95.5797


def ahead(m, brg=292.0):
  """Point m metres from the read point along brg."""
  r = math.radians(brg)
  return (LAT + m * math.cos(r) / 111_320.0, LON + m * math.sin(r) / (111_320.0 * math.cos(math.radians(LAT))))


def plate_with(mph=50, conf=0.92, way=1557241351, heading=292.0, lat=LAT, lon=LON):
  p = PlateState()
  p.propose(mph, conf, way_id=way, heading=heading, lat=lat, lon=lon, read_id=("r", 1), now=100.0)
  return p


def md(base=0, valid=False, osm=0, mpp=0):
  return SimpleNamespace(teslaBaseMph=base, teslaValid=valid, osmRawMph=osm, teslaMppMph=mpp)


def gps(m, brg=292.0):
  lat, lon = ahead(m)
  return lat, lon, brg


def test_match_sources():
  p = plate_with().proposal
  assert match_source(p, md(50, True, 60), *gps(80)) == "tesla"
  assert match_source(p, md(60, True, 50), *gps(80)) == "osm"
  assert match_source(p, md(50, False, 0), *gps(80)) is None          # base not debounced
  assert match_source(p, md(0, False, 0, mpp=50), *gps(80)) is None   # MPP never confirms
  assert match_source(p, md(60, True, 60), *gps(80)) is None          # no map agrees


def test_gates():
  assert match_source(plate_with().proposal, md(50, True), *gps(30)) is None     # still before the new zone
  assert match_source(plate_with().proposal, md(50, True), *gps(500)) is None    # too far past the read
  assert match_source(plate_with().proposal, md(50, True), *gps(80, brg=112.0)) is None   # opposite heading
  assert match_source(plate_with(conf=0.84).proposal, md(50, True), *gps(80)) is None
  assert match_source(plate_with(way=0).proposal, md(50, True), *gps(80)) is None        # no road at the read
  assert match_source(plate_with(lat=0.0, lon=0.0).proposal, md(50, True), *gps(80)) is None
  assert match_source(plate_with().proposal, md(50, True), None, None, None) is None


def test_tick_confirms_via_request_override_and_closes_prompt(tmp_path):
  plate = plate_with()
  calls = []

  def req(mph, rec, seq, **kw):
    calls.append((mph, rec, kw))
    return True
  ac = AutoConfirm(log_path=str(tmp_path / "auto_confirm.jsonl"))
  ac.enabled = True
  assert ac.tick(plate, md(50, True), gps(30), 101.0, request=req) is None and plate.proposal is not None
  assert ac.tick(plate, md(50, True), gps(80), 104.0, request=req) == "tesla"
  assert plate.proposal is None and plate.plate_mph == 0 and plate.confirmed_mph == 50   # both screens clear
  mph, rec, kw = calls[0]
  assert mph == 50 and kw == {"way_id": 1557241351, "heading": 292.0, "auto": "tesla"}
  assert rec["lat"] == LAT and rec["lon"] == LON
  row = json.loads((tmp_path / "auto_confirm.jsonl").read_text())
  assert row["source"] == "tesla" and row["mph"] == 50 and row["secs_after_prompt"] == 4.0
  # the same read is not asked again
  assert plate.propose(50, 0.92, way_id=1, read_id=("r", 1), now=105.0) is None


def test_disabled_does_nothing(tmp_path):
  plate = plate_with()
  ac = AutoConfirm(log_path=str(tmp_path / "a.jsonl"))
  assert ac.tick(plate, md(50, True), gps(80), 104.0, request=lambda *a, **k: True) is None
  assert plate.proposal is not None


def test_refresh_reads_param():
  class P:
    def __init__(self, v):
      self.v = v

    def get_bool(self, k):
      assert k == "NAPSignAutoConfirm"
      return self.v
  ac = AutoConfirm()
  ac.refresh(P(True), 0.0)
  assert ac.enabled
  ac.refresh(P(False), 1.0)       # rate-limited
  assert ac.enabled
  ac.refresh(P(False), 10.0)
  assert not ac.enabled
