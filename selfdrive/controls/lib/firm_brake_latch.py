"""Firm-brake latch for the Pre-AP follow actuator guard.

15a 11:26:49: a firm plan (-1.5) on a mid-gap close stepped -1.5 -> -0.22
-> -1.5 at the actuator when v_rel crossed 1.5 m/s, because the guard's
mild-floor clip (lead_approach.guard_follow_actuator_regen) comes back
below that close while the plan is still firm. A firm mid-gap close
passes the planner command through; once the close eases under 1.5 the
clip re-engaged and the brake fell off for a few frames.

The latch keeps that pass-through while the plan is still firm. It lives
outside the guard on purpose: `guard_follow_actuator_regen` (and #222, EV
mild settle -0.22, MAX, the -1.5 rail) is untouched, and the latch can
only ever make the command firmer than the guard returned.

Set when a firm close was passed (`lead_midgap_firm_close_hit`) on a plan
at or under ENTER_PLAN. Held until the plan is softer than HOLD_PLAN, the
close collapses under HOLD_CLOSE, MAX_S elapses, the lead is lost, or
LongControl resets.
"""

from openpilot.selfdrive.controls.lib.lead_approach import (
  LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS,
  LEAD_MPC_SOFT_NEAR_M,
  lead_approach_is_rapid,
  lead_firm_alead,
  lead_firm_large_slack_a,
  lead_mid_gap_map_band,
  lead_rising_or_residual_brake,
)

LEAD_FIRM_LATCH_ENTER_PLAN_MS2 = -0.80
LEAD_FIRM_LATCH_HOLD_PLAN_MS2 = -0.50
LEAD_FIRM_LATCH_ENTER_CLOSE_MS = LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS
LEAD_FIRM_LATCH_HOLD_CLOSE_MS = 0.80
LEAD_FIRM_LATCH_MAX_S = 4.0


def lead_midgap_firm_close_hit(v_rel, d_rel, slack, a_lead=None,
                               prev_v_rel=None, a_ego=None, dt=None,
                               should_stop=False) -> bool:
  """True when the mid-gap guard is passing a firm close (not a coast).

  Same inputs as the guard's exclusion that unlocks the planner command at
  close >= 1.5: mid-gap band, firm aLead or #222 rising/residual brake.
  Near bumper, shouldStop, and confirmed rapid already pass raw and do
  not need the latch.
  """
  if should_stop or v_rel is None:
    return False
  v = float(v_rel)
  if v < LEAD_FIRM_LATCH_ENTER_CLOSE_MS or lead_approach_is_rapid(v):
    return False
  if d_rel is not None and float(d_rel) <= LEAD_MPC_SOFT_NEAR_M:
    return False
  if not lead_mid_gap_map_band(slack, d_rel):
    return False
  return lead_firm_alead(a_lead) or lead_rising_or_residual_brake(
    v, prev_v_rel, a_ego, dt,
  )


class FirmBrakeLatch:
  """`update` returns True while the guard's mild-floor clip must stay off.

  Owned by LongControl and stepped once per control frame with its own
  `dt` (not the residual-window span). A lost lead or plan resets it.
  """

  def __init__(self) -> None:
    self.on = False
    self.age = 0.0

  def reset(self) -> None:
    self.on = False
    self.age = 0.0

  def update(self, planner_a, v_rel, hit, dt) -> bool:
    if planner_a is None or v_rel is None:
      self.reset()
      return False
    p = float(planner_a)
    v = float(v_rel)
    if self.on:
      self.age += float(dt)
      if (self.age > LEAD_FIRM_LATCH_MAX_S
          or p > LEAD_FIRM_LATCH_HOLD_PLAN_MS2
          or v < LEAD_FIRM_LATCH_HOLD_CLOSE_MS):
        self.reset()
    if not self.on and hit and p <= LEAD_FIRM_LATCH_ENTER_PLAN_MS2:
      self.on = True
      self.age = 0.0
    return self.on


def apply_firm_brake_latch(guarded_a, raw_a, planner_a, hold, v_rel=None,
                           slack=None, a_lead=None):
  """Keep the planner-following command while the latch holds.

  `guarded_a` is the unchanged guard output and `raw_a` the actuator
  before it. The guard only ever raises `raw_a`, so `min` restores the
  firm command and can never be softer than `guarded_a`. The guard's
  early kinematic cap (firm lead 20-50 m out, not confirmed rapid) is a
  deliberate softening and keeps ownership of the frame, as does any
  frame whose plan is not firm.
  """
  if not hold or guarded_a is None or raw_a is None or planner_a is None:
    return guarded_a
  if float(planner_a) > LEAD_FIRM_LATCH_HOLD_PLAN_MS2:
    return guarded_a
  if (v_rel is not None and not lead_approach_is_rapid(float(v_rel))
      and lead_firm_large_slack_a(v_rel, slack, a_lead) is not None):
    return guarded_a
  return min(float(guarded_a), float(raw_a))
