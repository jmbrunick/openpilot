# Safety Model

Panda-layer invariants for the Pre-AP target. Source of truth is `opendbc_repo/opendbc/safety/modes/tesla_preap.h` — this doc explains the *why*.

## Relay and blocking

```c
{0x488, 0, 4, .check_relay = false, .disable_static_blocking = true}
```

Every Pre-AP TX entry uses both. Pre-AP has no harness relay hardware — the panda connects directly to the car's CAN bus. Standard openpilot assumes a relay that routes CAN between stock AP and panda when openpilot is inactive; setting `check_relay=true` would cause the panda to falsely detect a relay malfunction and block all TX permanently. `disable_static_blocking=true` is required for the same reason.

Tinkla handled this identically via `generic_rx_checks(false)` in the older panda API ("PreAP has no relay", `safety_tesla.h` line 1071, `tesla_unity_betaC3`). The modern API added `check_relay` and `disable_static_blocking` with restrictive defaults, so we explicitly opt out.

## RX checksum and counter validation

```c
{.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true,
          .ignore_checksum = true, .ignore_counter = true}, ...}}
```

Pre-AP EPAS firmware uses a byte-sum checksum, but the exact algorithm has not been verified across every firmware version in the field. A mismatched validation caused a silent 21-second steering dropout during testing — the safety layer silently rejected EPAS messages, the car stopped steering, and no alert fired because the underlying RX check was "valid" from the framework's perspective.

Tinkla's RX checks also disabled checksum/counter validation. Once the checksum algorithm is verified across all in-the-wild EPAS firmware, these can be re-enabled.

All actual safety logic remains active:

- Steering angle + rate limits via `steer_angle_cmd_checks_vm()`
- `controls_allowed` gating on every TX
- Disengage on hands-on override (level >= 2), except during a blinker-latched driver turn (one lamp or held LEFT/RIGHT, flash-latched ~1s, then hand-on until release)
- Disengage on EPAS error codes 6–9, except during that same turn pause
- Disengage on door open, gear out of Drive
- Disengage on stalk cancel (with 600 ms echo filter)
- AEB events blocked from openpilot
- EPB `epasControl` mode validation
- Pedal TX gated by `ENABLE_PEDAL` flag + `get_longitudinal_allowed()`

## Conditional RX checks

`preap_rx_checks[]` is selected by the `ENABLE_PEDAL` flag. The pedal variant adds `0x552` at 50 Hz; the no-pedal variant omits it.

Reason: panda's `safety_tick` computes `timestep = 1e6 / frequency` unconditionally. `frequency=0` means divide-by-zero, which marks the check as lagging and trips `safetyRxChecksInvalid` → controls mismatch. A tester without a pedal hit this on `nap-release` in April 2026 — regression test lives in `test_tesla_preap.py::test_no_pedal_does_not_invalidate_rx_checks`.

## TX whitelist

```c
static const CanMsg PREAP_TX_MSGS[] = {
  {0x488, 0, 4, ...},  // DAS_steeringControl
  {0x2B9, 0, 8, ...},  // DAS_control
  {0x214, 0, 3, ...},  // EPB_epasControl
  {0x551, 0, 6, ...},  // Pedal on bus 0
  {0x551, 2, 6, ...},  // Pedal on bus 2 (alternate wiring)
  {0x45,  0, 8, ...},  // STW_ACTN_RQ (stalk spoof)
};
```

`0x45` is the stalk-action request message. We spoof it to drive stock Tesla CC when no pedal is installed (see [engagement.md](engagement.md)). There is no relay to hide behind, so every TX address must be explicitly listed.

## GTW radar emulation

When `RADAR_EMULATION` is set, the panda forwards chassis-bus traffic with an optional radar-position rewrite. This uses the `rx_all` hook — not `rx` — because it must see every CAN frame, not just the whitelist. Using `rx` here would break radar forwarding silently when messages fell outside `preap_rx_checks[]`.

## Brake architecture

Panda hardcodes `brake_pressed=false`. The framework's generic brake-to-disengage path never fires on pre-AP. This is deliberate:

1. `carstate.py` reads real brake state from `BrakeMessage` CAN
2. Passes to `engagement.process_buttons(real_brake_pressed=...)`
3. On brake rising edge in pedal mode: drops `enableLongControl=False` but keeps `cruiseEnabled=True` (lateral stays active, only pedal drops)
4. `ret.brakePressed = False` suppresses the generic openpilot brake handler from also killing lateral

The driver can always override steering via hands-on level ≥ 2. The panda enforces that, except during a blinker-latched driver turn (lat is already released; wheel torque must not `pcm_cruise_check(false)` or selfdrived will `controlsMismatch` after 2s). It does not enforce brake-to-disengage.

See `test_tesla_preap.py::test_prev_user_brake` for the panda-layer invariant test.

## Blinker-turn latch (panda flash required)

`tesla_preap` keeps `controls_allowed` when a **driver turn** is active so a high-torque corner (EPAS hands-on ≥ 2 / error 6–9) does not `pcm_cruise_check(false)`. That is the same intent as Python `BlinkerLateralHold` / `blinker_turn_blocks_steering_disengage`:

- one GTW lamp XOR (`BC_indicatorLStatus` / `R` == 1), or
- physical stalk `TurnIndLvr_Stat` LEFT/RIGHT, or
- latched through GTW lamp flash gaps (~1s continuous dark ends the turn), then hand-on until release

A stalk **tip** (LEFT/RIGHT then IDLE within 0.40s) is ALC: leftover keep-alive flashes are not a turn. Same-direction stalk during that keep-alive must be held 1.0s to become a turn. Hazards (both lamps) are not a turn. Stalk cancel, door open, and gear out of Drive still drop immediately.

This lives in the panda firmware, not just Python. tesla_preap skips `pcm_cruise_check(false)` and sets `steering_disengage_keep_controls` so `generic_rx_checks` does not drop on the same rising edge.

After pulling this change, **flash the panda** (usual NAP/panda update path: on-device `scons` rebuilds `panda/board` from `opendbc_repo` safety, then reboot so `pandad` compares firmware signatures and flashes). A Python-only update leaves the old firmware dropping `controls_allowed` mid-turn; `controlsMismatch` then appears right after the hold ends.

## Changing safety code

1. Every edit to `tesla_preap.h` / `tesla_preap_blinker.h` needs a test in `test_tesla_preap.py` or `test_tesla_preap_blinker.py`
2. Run `scons opendbc_repo/opendbc/safety/tests/libsafety/libsafety.so` after editing
3. Pre-push: full `pytest test_tesla_preap.py`, then CI (safety tests run on every branch push)
4. If changing RX rate expectations, replay against a real drive log to confirm — see `scripts/nap/analyze_engagement.py` for a starter replay harness
5. Device: rebuild panda firmware and reboot so `pandad` flashes. Confirm no "reflash panda" / signature mismatch in logs.
