"""NAPTeslaLimitPrimary: Tesla base limit as the current limit (Oct 23 2026)."""
from openpilot.selfdrive.mapd.tesla_limit import (
  HOLD_S, MPH_TO_MS, PRIMARY_HOLD_S, ROAD_CHANGE_HOLD_S, SOURCE_LEARNED, SOURCE_TESLA, TeslaLimitFusion, TeslaPrimary,
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


def test_priority_sign_over_learned_over_tesla_over_osm():
  osm = 55 * MPH_TO_MS
  assert select_current(45 * MPH_TO_MS, 50, osm, 40 * MPH_TO_MS) == (45 * MPH_TO_MS, "sign")
  assert select_current(None, 50, osm, 40 * MPH_TO_MS) == (40 * MPH_TO_MS, SOURCE_LEARNED)
  assert select_current(None, 0, osm, 40 * MPH_TO_MS) == (40 * MPH_TO_MS, SOURCE_LEARNED)
  assert select_current(None, 50, osm, None) == (50 * MPH_TO_MS, SOURCE_TESLA)
  # no cap: Tesla beats OSM higher or lower
  assert select_current(None, 65, osm, None) == (65 * MPH_TO_MS, SOURCE_TESLA)
  assert select_current(None, 30, osm, None) == (30 * MPH_TO_MS, SOURCE_TESLA)
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
  assert p.update(f, 4.2, ("Main St", "residential")) == 50     # 1-frame grace
  t = 3.9 + ROAD_CHANGE_HOLD_S + 0.2
  f.add_base(t, 0, 0)
  assert p.update(f, t, ("Main St", "residential")) == 0          # well short of PRIMARY_HOLD_S


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


def _real_car_dropouts(drop_s, period_s, t_end=300.0, mph=60, dt=0.4):
  """Oct 23 route 000001c2 pattern: mux 3 at 2.5 Hz, base drops to 0 for drop_s
  every period_s (alternating 15 / 20 s), then the same value comes back."""
  frames, t, nxt, k = [], 0.0, 10.0, 0
  while t < t_end:
    if nxt <= t < nxt + drop_s:
      frames.append((t, 0))
    else:
      if t >= nxt + drop_s:
        nxt += period_s[k % len(period_s)]
        k += 1
      frames.append((t, mph))
    t = round(t + dt, 3)
  return frames


def test_real_car_periodic_dropouts_never_flip_to_osm():
  osm = 55 * MPH_TO_MS
  for drop_s in (0.4, 1.0, 3.0, 4.0, 5.0):
    f, p = TeslaLimitFusion(), TeslaPrimary()
    srcs, holes = [], 0
    for t, mph in _real_car_dropouts(drop_s, (15.0, 20.0)):
      f.add_base(t, mph, 99)
      cur = p.update(f, t, ROAD)
      holes += t >= HOLD_S + 0.5 and not f.state(t).valid
      if t >= HOLD_S + 0.5:
        srcs.append(select_current(None, cur, osm, None))
    assert all(r == (60 * MPH_TO_MS, SOURCE_TESLA) for r in srcs), drop_s
    # the plain debounced value (auto-confirm / review notes) still shows the hole
    assert holes > 0   # the plain debounced value still shows the holes
  assert PRIMARY_HOLD_S >= 6.0


def test_real_car_missing_frames_held():
  # frames missing entirely (stale CAN) for 4 s every 15 s: held, no flip
  f, p = TeslaLimitFusion(), TeslaPrimary()
  out = []
  for t, mph in _real_car_dropouts(4.0, (15.0,)):
    if mph:
      f.add_base(t, mph, 99)
    out.append((t, p.update(f, t, ROAD)))
  assert all(m == 60 for t, m in out if t >= HOLD_S + 0.5)


def test_road_flicker_during_rebounce_keeps_tesla():
  # Oct 21 Atlantic Ave: dropout, then the match flickers to a cross street while
  # the same 30 re-debounces -> stay on Tesla 30, no OSM blip
  f, p = TeslaLimitFusion(), TeslaPrimary()
  feed(f, p, 0.0, 4.0, 30)
  feed(f, p, 4.0, 4.4, 0)
  out = feed(f, p, 4.4, 6.0, 30, road=("North 2nd Street", "residential"))
  out += feed(f, p, 6.0, 9.0, 30)
  assert all(m == 30 for _, m in out)


def test_road_change_without_tesla_value_goes_osm():
  f, p = TeslaLimitFusion(), TeslaPrimary()
  feed(f, p, 0.0, 4.0, 30)
  out = feed(f, p, 4.0, 4.0 + ROAD_CHANGE_HOLD_S + 1.0, 0, road=("US 12", "trunk"))
  assert all(m == 30 for t, m in out if t - 3.9 <= ROAD_CHANGE_HOLD_S)   # 1-frame grace
  assert out[-1][1] == 0


def test_one_frame_dropout_at_way_name_change_keeps_tesla():
  # Pacific Ave SE -> Litchfield Ave SE: one 0 frame right at the name change
  f, p = TeslaLimitFusion(), TeslaPrimary()
  feed(f, p, 0.0, 4.0, 30)
  new = ("Litchfield Avenue Southeast", "trunk")
  out = feed(f, p, 4.0, 4.4, 0, road=new) + feed(f, p, 4.4, 9.0, 30, road=new)
  assert all(m == 30 for _, m in out)


def test_new_value_at_road_change_no_osm_gap():
  # US 12 60 -> Atlantic Ave 30: 60 until 30 is published, never OSM in between
  f, p = TeslaLimitFusion(), TeslaPrimary()
  feed(f, p, 0.0, 4.0, 60)
  out = feed(f, p, 4.0, 4.0 + HOLD_S + 0.5, 30, road=("Atlantic Avenue", "trunk"))
  vals = [m for _, m in out]
  assert 0 not in vals and vals[0] == 60 and vals[-1] == 30


def test_new_value_after_long_dropout_no_osm_gap():
  # held 45 runs out its hold while 60 is still debouncing: 45 -> 60, never OSM
  f, p = TeslaLimitFusion(), TeslaPrimary()
  feed(f, p, 0.0, 4.0, 45)
  out = feed(f, p, 4.0, 4.0 + PRIMARY_HOLD_S - 1.0, 0)
  out += feed(f, p, 4.0 + PRIMARY_HOLD_S - 1.0, 4.0 + PRIMARY_HOLD_S + HOLD_S, 60)
  vals = [m for _, m in out]
  assert 0 not in vals and vals[-1] == 60
  # but a value that never settles is not held forever
  assert p.update(f, 4.0 + 2 * PRIMARY_HOLD_S + 2 * HOLD_S, ROAD) == 0
