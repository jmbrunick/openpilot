"""NAPTeslaLimitPrimary: Tesla base limit as the current limit (Oct 23 2026)."""
from openpilot.selfdrive.mapd.tesla_limit import (
  HOLD_S, MPH_TO_MS, PRIMARY_HOLD_S, SOURCE_LEARNED, SOURCE_TESLA, TeslaLimitFusion, TeslaPrimary,
  next_for_published, select_current,
)

ROAD = ("US 12", "primary")


def feed(f, p, t0, t1, mph, conf=99, road=ROAD, dt=0.1):
  """10 Hz base frames from t0 to t1; returns [(t, primary mph)] at each frame."""
  out, t = [], t0
  while t < t1 - 1e-9:
    f.add_base(t, mph, conf)
    out.append((t, p.update(f, t, road)))
    t = round(t + dt, 3)
  return out


def test_priority_sign_over_tesla_over_learned_over_osm():
  osm = 55 * MPH_TO_MS
  assert select_current(45 * MPH_TO_MS, 50, osm, 40 * MPH_TO_MS) == (45 * MPH_TO_MS, "sign")
  assert select_current(None, 50, osm, 40 * MPH_TO_MS) == (50 * MPH_TO_MS, SOURCE_TESLA)
  assert select_current(None, 0, osm, 40 * MPH_TO_MS) == (40 * MPH_TO_MS, SOURCE_LEARNED)
  assert select_current(None, 0, osm, None) == (osm, None)
  assert select_current(None, 0, osm) == (osm, None)


def test_debounce_before_tesla_is_used():
  f, p = TeslaLimitFusion(), TeslaPrimary()
  out = feed(f, p, 0.0, HOLD_S + 0.5, 45)
  assert all(m == 0 for t, m in out if t < HOLD_S - 1e-6)   # not held 3 s yet -> OSM
  assert out[-1][1] == 45


def test_short_flash_never_published():
  f, p = TeslaLimitFusion(), TeslaPrimary()
  out = feed(f, p, 0.0, 1.5, 60)            # 0 -> 60 for 1.5 s -> 0
  out += feed(f, p, 1.5, 4.0, 0, conf=0)
  assert all(m == 0 for _, m in out)


def test_dropout_held_then_released():
  f, p = TeslaLimitFusion(), TeslaPrimary()
  feed(f, p, 0.0, 4.0, 50)
  # 2 s of 0 / low conf: held at 50, flagged held
  out = feed(f, p, 4.0, 6.0, 0, conf=0)
  assert all(m == 50 for _, m in out) and p.held
  # dropout longer than the hold: back to OSM
  out = feed(f, p, 6.0, 4.0 + PRIMARY_HOLD_S + 1.0, 0, conf=0)
  assert out[-1][1] == 0


def test_same_value_back_bridges_redebounce():
  f, p = TeslaLimitFusion(), TeslaPrimary()
  feed(f, p, 0.0, 4.0, 50)
  feed(f, p, 4.0, 5.0, 0, conf=40)          # low-conf blip resets the fusion hold
  out = feed(f, p, 5.0, 9.5, 50)            # 50 again: 3 s re-debounce, no OSM flip
  assert all(m == 50 for _, m in out)
  assert f.state(9.4).valid


def test_new_value_waits_for_debounce_old_held():
  f, p = TeslaLimitFusion(), TeslaPrimary()
  feed(f, p, 0.0, 4.0, 50)
  out = feed(f, p, 4.0, 4.0 + HOLD_S + 0.3, 35)
  assert out[0][1] == 50 and out[-1][1] == 35   # old stable until the new value holds 3 s


def test_road_change_drops_hold():
  f, p = TeslaLimitFusion(), TeslaPrimary()
  feed(f, p, 0.0, 4.0, 50)
  f.add_base(4.1, 0, 0)
  assert p.update(f, 4.1, ROAD) == 50
  f.add_base(4.2, 0, 0)
  assert p.update(f, 4.2, ("Main St", "residential")) == 0


def test_stale_can_held_then_osm():
  f, p = TeslaLimitFusion(), TeslaPrimary()
  feed(f, p, 0.0, 4.0, 50)
  assert p.update(f, 6.5, ROAD) == 50       # no frames (stale) inside the hold
  assert p.update(f, 4.0 + PRIMARY_HOLD_S + 0.5, ROAD) == 0


def test_lookahead_drops_next_equal_to_tesla():
  # Published Tesla 45 while OSM still says 55; OSM's 45 300 m ahead is not an
  # upcoming change (Tesla already shows it).
  assert next_for_published((45 * MPH_TO_MS, 300.0), 45 * MPH_TO_MS) is None
  # A real lower OSM limit ahead still drives the early slowdown from Tesla's value.
  nxt = (35 * MPH_TO_MS, 300.0)
  assert next_for_published(nxt, 45 * MPH_TO_MS) == nxt
  assert next_for_published(None, 45 * MPH_TO_MS) is None


def test_mpp_never_sets_limit():
  f, p = TeslaLimitFusion(), TeslaPrimary()
  t = 0.0
  while t < 6.0:
    f.add_mpp(t, 65)
    f.add_base(t, 0, 99)
    assert p.update(f, t, ROAD) == 0
    t += 0.1
  assert f.state(5.9).mpp_mph == 65
