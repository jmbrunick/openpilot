"""Shared NAP settings content.

Constants and user-facing text used by the NAP settings panels. Both UI trees
import from here so the safety-critical instruction strings (EPAS flash,
calibration, restore) have a single source.
"""

# Preset values for float/int params exposed as multiple-button selectors.
BRAKE_FACTOR_PRESETS = [0.5, 1.0, 1.5, 2.0]
PEDAL_CAN_BUS_VALUES = [0, 2]

MAP_SPEED_MODES = [0, 1, 2, 3]
MAP_SPEED_MODE_LABELS = ["Off", "Display", "Cap", "Follow"]
MAP_SPEED_OFFSETS_MPH = [-5, 0, 5]
MAP_SPEED_LOOKAHEAD = [0, 1, 2, 3]
MAP_SPEED_LOOKAHEAD_LABELS = ["Off", "Late", "Normal", "Early"]
MAP_SPEED_ACCEL = list(range(1, 11))
MAP_SPEED_ACCEL_LABELS = [str(i) for i in MAP_SPEED_ACCEL]
MAP_SPEED_ACCEL_DEFAULT = 5
MAP_SPEED_ACCEL_DESCRIPTION = (
  "MAX climb / open-road feel (1 lazy → 10 quicker). "
  + "Brake to a lower MAX stays Accel 5. Lead still owns follow."
)
ADAPTIVE_ACCEL_DESCRIPTION = (
  "Softer accel near a lead so you don't overshoot. "
  + "Open road uses Acceleration 1–10."
)
MAP_SPEED_MODE_DESCRIPTION = (
  "OSM posted limit for HUD MAX. Off: no change. Display: LIMIT sign only. "
  + "Cap: never exceed. Follow: track the limit. A stalk set "
  + "holds until the posted limit changes. Cap/Follow need the pedal."
)
MAP_SPEED_OFFSET_DESCRIPTION = (
  "Added to the OSM limit for Cap/Follow (mph)."
)
MAP_SPEED_LOOKAHEAD_DESCRIPTION = (
  "Ease MAX down for a lower limit ahead. Off: wait until GPS is on that way. "
  + "A stalk set holds; lookahead pauses while that set is active. "
  + "A higher limit far ahead never raises MAX. Radar lead still outranks map."
)
FOLLOW_DISTANCE_VALUES = list(range(1, 8))
FOLLOW_DISTANCE_LABELS = [str(i) for i in FOLLOW_DISTANCE_VALUES]
FOLLOW_DISTANCE_DEFAULT = 4
NAP_HYPERMILE = "NAPHypermile"
NAP_HYPERMILE_STEP_DOWN = "NAPHypermileStepDown"
NAP_HYPERMILE_HILL_CLIMB = "NAPHypermileHillClimb"
HYPERMILE_DESCRIPTION = (
  "Default Off. Comfort-biased efficiency — early light ease, not max "
  + "regen. On snaps Adaptive Accel, Cap/Follow, Early lookahead, and "
  + "Accel 1; Off restores. Eco offset is posted-scaled so town 30 stays "
  + "30. Follow Distance stays the stock slider. Lead braking stays on."
)
FOLLOW_DISTANCE_DESCRIPTION = (
  "1 closest, 7 farthest. A slower car ahead eases off farther back "
  + "(more distance, not a harder brake). Behind a lead, a stalk tip "
  + "steps this; a full press still steps MAX."
)
HYPERMILE_STEP_DOWN_DESCRIPTION = (
  "Default Off. Only while Hypermile is On. Larger posted-scale drop "
  + "(0 under 50 mph, −15 at 80). Maps-only; does not invent a drop."
)
HYPERMILE_HILL_CLIMB_DESCRIPTION = (
  "Default On. Only while Hypermile is On. IMU pitch holds grade on "
  + "Accel 1 climbs. Never raises MAX. Lead / MPC brake still wins."
)

NAP_SPEED_SIGN_LOG = "NAPSpeedSignLog"
NAP_DRIVER_LAT_HANDOFF = "NAPDriverLatHandoff"
NAP_DM_SIMULATE_LOOKING = "NAPDmSimulateLooking"
NAP_DM_FALSE_ALERT_IGNORE = "NAPDmFalseAlertIgnore"
NAP_FORCE_OFFROAD = "NAPForceOffroad"
DRIVER_LAT_HANDOFF_DESCRIPTION = (
  "Default On. A light purposeful push frees the wheel. Hands off "
  + "~0.15 s, then it blends back. Turn Off if rumble false-yields. "
  + "Off = stock lat."
)
DM_SIMULATE_LOOKING_DESCRIPTION = (
  "Default On. nap-dev experiment. While engaged, after the look-at-road "
  + "timer has counted down about 1 s, restore awareness on the stock "
  + "attentive path (full soft-reset: no-face, uncertain, phone, pose, "
  + "and eye false nags) and hold until gradual recovery returns "
  + "awareness to full (not a one-frame pulse). Fire time is random in "
  + "the first 3 s of countdown (after 1 s, then within the next 2 s). "
  + "Mutually exclusive with False Alert Ignore: turning this On turns "
  + "that Off. Not a mute of hard cancels: hands-on ≥ 2, stalk cancel, "
  + "door, and reverse still hard-cancel. Always-on DM when not engaged "
  + "is unchanged. Turn Off for stock DM."
)
DM_FALSE_ALERT_IGNORE_DESCRIPTION = (
  "Default Off. Use when Simulate Look is Off and you only want false "
  + "phone/device distraction (phoneProb) ignored on the same random "
  + "1–3 s cadence. Soft-clears only the phone bit so a false device "
  + "“Driver Distracted” can recover without a real glance. Head-pose "
  + "looking-away and eye tracking still drain and alert — this does "
  + "nothing while pose or eye are alarming. Mutually exclusive with "
  + "Simulate Look: turning this On turns that Off (and aborts an "
  + "in-flight wipe). Hands-on ≥ 2, stalk, door, and reverse still "
  + "hard-cancel. Turn Off for stock phone detection."
)
FORCE_OFFROAD_DESCRIPTION = (
  "WARNING: Forces the device offroad and disengages openpilot even while "
  + "moving. On-road, a big Yes/No asks if you are ready to resume steering "
  + "control — No leaves assist as it was. Drive manually after Yes — no "
  + "steering or accel assist while this is on. Unlocks Download US Maps, "
  + "Refresh maps, software install, and other offroad-only NAP actions. "
  + "Default Off. Clears when you toggle Off, Reset to Defaults, reboot, or "
  + "the next time ignition turns on."
)
SPEED_SIGN_LOG_DESCRIPTION = (
  "Log-only MUTCD speed-sign detector on the ROAD camera + GPS. Default Off. "
  + "When On, speedsignd shows a display-only SIGN plate and may append JSONL "
  + "under /data/media/0/nap/speed_signs.jsonl (t, lat, lon, bearing, mph, conf). "
  + "WARNING: onroad YOLO can lag the driving model and cause TAKE CONTROL / "
  + "process timeouts. Heavy detect never runs while openpilot is actively "
  + "controlling (plate shows WAIT) so modeld is not starved. WAIT only means "
  + "OP is commanding actuators and should drop as soon as you cancel — if "
  + "you see WAIT while the UI looks disengaged, that is a bug; report "
  + "swaglog lines matching speedsignd. "
  + "While you are driving manually — moving is OK — detect runs at 1 Hz "
  + "and nice 19 so you can log speed-limit signs. Not gated on park or "
  + "Force Offroad. OSM upload is future work; this is JSONL + HUD only. "
  + "If you see TAKE CONTROL or driving-model lag, turn Logger Off and use "
  + "nap-release. Does not write sqlite, does not change cruise / HUD MAX, "
  + "and does not query osm.org. Stock modelV2 has no speedSign head. Real "
  + "roadside detection needs the compact YOLO ONNX on "
  + "/data/media/0/nap/speed_sign.onnx. If that file is missing, onroad SIGN "
  + "shows NO WT — use Install weights (Wi-Fi, offroad / Force Offroad), or: "
  + "python -m scripts.nap.install_speed_sign_weights."
)

INSTALL_SPEED_SIGN_WEIGHTS_INSTRUCTIONS = """\
Install speed-sign ONNX weights

Downloads the published YOLO ONNX (~43 MB) from GitHub Release
speed-sign-onnx-v1 / speed_sign.onnx and writes
/data/media/0/nap/speed_sign.onnx (SHA-256 checked).

Without this file, Speed Sign Logger On still starts speedsignd, but the
onroad SIGN plate shows NO WT and will not read real roadside signs.

PRECONDITIONS:
  1. Device is offroad / parked (Force Offroad if you are in the car)
  2. Wi-Fi that can reach GitHub Releases
  3. ~50 MiB free on /data

Takes less than a minute. speedsignd retries the ONNX onroad without a
reboot. Then a clear MUTCD R2-1 should light SIGN mph.

Does not write sqlite, does not change cruise / HUD MAX, and does not
query osm.org.

Press START to download."""

# Forwarded 0x45 STW_ACTN_RQ wiper / high-beam. 0 = off (today's stalk).
WIPER_SPEED_VALUES = [0, 1, 2, 3]
WIPER_SPEED_LABELS = ["Off", "Int", "On", "Auto"]
HIGH_LOW_BEAM_VALUES = [0, 1, 2]
HIGH_LOW_BEAM_LABELS = ["Off", "Low", "High"]
WIPER_SPEED_DESCRIPTION = (
  "Rewrite the wiper / collar bits on the forwarded stalk (0x45 STW_ACTN_RQ). "
  + "Off leaves the driver's real stalk nibble alone unless Int/On just "
  + "turned off — then extra-forward rest to cancel latched intermittent. "
  + "Int/On set the high nibble to 1 (TIPWIPE) and hold it — they do not "
  + "spray. Auto does not use TIPWIPE: it overlays WprSw6Posn INTERVAL1 "
  + "(collar=1) and WprWashSw_Psd=0 only when the car is on, gear is "
  + "Drive or Reverse, and the 3X road camera sees a rainy or icy/frosted "
  + "windshield. Hold INTERVAL1 at ~100 Hz (same last-win as high-beam) so "
  + "live stalk Off (collar=0) cannot cancel. While Auto is selected and dry "
  + "(or Park/Neutral), still extra-forward 0x45 rest with collar forced 0 "
  + "and nibble 1 cleared so Pre-AP drops intermittent; a wipe 1→0 sends "
  + "rest immediately. Park and Neutral never Auto-wipe. Default Off — Auto "
  + "is opt-in, not every drive. DAS wiper fields stay 0 (they were ignored "
  + "and caused a controls mismatch). Do not engage NAP and do not pull the "
  + "stalk. No stalk Auto required. No spray. No auto headlights."
)
WIPER_COLLAR_VALUES = [0, 3, 4]
WIPER_COLLAR_LABELS = ["Off", "Collar3", "Collar4"]
WIPER_COLLAR_DESCRIPTION = (
  "Parked experiment only: spoof unused 4-click collar positions on 0x45 "
  + "STW_ACTN_RQ. Off leaves the live stalk (Off=0 Int1=1 Int2=2 Low=5 "
  + "High=6). Collar3/Collar4 brute-force hold WprSw6Posn INTERVAL3=3 or "
  + "INTERVAL4=4 with WprWashSw_Psd=0 — no TIPWIPE 0x10, no wash spray. "
  + "Same last-win as Auto INTERVAL1 / High: extra-forward that 0x45 every "
  + "card frame (~100 Hz, live MC, CRC) so bus-0 Off cannot overwrite 3/4. "
  + "When selected, this overrides camera Auto INTERVAL1. Flicker vs the "
  + "real stalk is acceptable until the ESP32 column gateway — not in this "
  + "tip. After Collar3, cat /data/params/d/NAPWiperCollar must show 3 "
  + "(DBC posn, not UI index 1). cat /data/params/d/NAPWiperCollarStatus "
  + "must change every ~1s: collar=3 tx=N last_d6=3 src=128 err=-. "
  + "candump src 0 is the live stalk. src 128 is our TX. Car on, Force "
  + "Offroad off — card must be running. Default Off. Leave Wiper Control "
  + "Off while testing; camera Auto (#162) is unchanged when this is Off. "
  + "No spray. Do not flash."
)
HIGH_LOW_BEAM_DESCRIPTION = (
  "Pre-AP Model S only, for on-car testing. Default Off. Low is the same as "
  + "Off — the stalk's rest 0x45 (00ff00) is low/cancel, so there is no extra "
  + "nibble to hold. The real stalk continuously sends 00ff04 while high "
  + "beams are held, not a one-shot press. High matches that: keep sending "
  + "00ff04 (nibble 4 held) on the live 0x45, same counter, until you turn "
  + "the setting off, so bus 0 IDLE cannot last-win as a cancel. Off/Low "
  + "returns the real stalk. Off/Low then High is not a one-shot tap. The "
  + "extra-forward is the same 0x45, not a second 0x45. No spray. DAS "
  + "wiper/beam fields stay 0. Car on is enough. Do not send FLASH. No auto "
  + "headlights."
)

# Radar lateral offset bounds (meters). Added to radar yRel in
# radar_interface.py. Negative = shift toward left; positive = toward right.
# ~0.27 is typical for the 3D-printed factory-location mount.
RADAR_OFFSET_MIN = -2.0
RADAR_OFFSET_MAX = 2.0


CALIBRATE_PEDAL_INSTRUCTIONS = """\
NAP Pedal Calibrator

This script calibrates the comma pedal interceptor for your pre-AP Tesla Model S.

PRECONDITIONS:
  1. Car must be ON
  2. Gear must be in NEUTRAL
  3. Brake pedal must be PRESSED and held
  4. Do NOT press the accelerator pedal during calibration

The calibration process will:
  - Detect pedal zero position
  - Detect pedal maximum position
  - Fine-tune the scale factor
  - Validate the calibration values
  - Save calibration to params

Press START when ready to begin calibration."""


CALIBRATE_RADAR_INSTRUCTIONS = """\
NAP Radar Calibrator

This script displays filtered radar points to help align the Bosch radar.

PRECONDITIONS:
  1. Vehicle must be safely parked
  2. Radar must be properly mounted and connected
  3. Place a calibration target 3-10m ahead, centered on the vehicle axis

The calibration display will show:
  - Radar points within 2.5-14.5m ahead
  - Lateral offset from center
  - Adjust radar aim until target shows ~0.0m lateral

Press START when ready to begin."""


TEST_RADAR_INSTRUCTIONS = """\
NAP Radar Test

This script displays live radar data for testing and verification.

The test will show:
  - All detected radar points
  - Distance and relative velocity
  - Track status and confidence

This is useful for:
  - Verifying radar installation
  - Checking radar alignment
  - Debugging radar issues

Press START to begin the radar test."""


FLASH_EPAS_INSTRUCTIONS = """\
EPAS Firmware Flash

STEP 1 — POWER THE CAR ON NOW:
  - Key fob inside the car
  - Foot on the brake
  - Car should be in Park and stay there
  - Do not drive
  (The EPAS ECU only responds when the car is on.)

WARNING: This will modify your steering system firmware!

POWER REQUIREMENTS:
  - 12V battery must be healthy and at a normal charge
  - Do NOT start the flash if the car has been sitting cold
    with marginal voltage
  - Power loss DURING the flash will brick the EPAS module

RISKS:
  - Incorrect firmware can disable power steering
  - Interrupted flash can brick the EPAS module
  - This modification may void warranties

Only proceed if you:
  - Fully understand the implications
  - Have backup EPAS firmware available
  - Are comfortable with the risks involved

Press START only if you accept these risks."""


BACKUP_EPAS_INSTRUCTIONS = """\
EPAS Firmware Backup

STEP 1 — POWER THE CAR ON NOW:
  - Key fob inside the car
  - Foot on the brake
  - Car should be in Park and stay there
  - Do not drive
  (The EPAS ECU only responds when the car is on. If the car
   is not on, the script will time out.)

This action only reads the stock EPAS firmware and saves it.
No flashing or firmware modifications are performed.

Run this before any Flash operation so you have a local backup.

PRECONDITIONS:
  - Stable 12V power
  - Do not power-cycle during extraction

Press START to extract the EPAS firmware backup."""


RESTORE_EPAS_INSTRUCTIONS = """\
EPAS Firmware Restore

STEP 1 — POWER THE CAR ON NOW:
  - Key fob inside the car
  - Foot on the brake
  - Car should be in Park and stay there
  - Do not drive
  (The EPAS ECU only responds when the car is on.)

WARNING: This will reflash your steering system firmware!

POWER REQUIREMENTS:
  - 12V battery must be healthy and at a normal charge
  - Do NOT start the restore if the car has been sitting cold
    with marginal voltage
  - Power loss DURING the restore will brick the EPAS module

This operation:
  - Uses the extracted stock EPAS firmware image
  - May take several minutes to complete
  - Should NOT be interrupted once started

RISKS:
  - Interrupted flash can brick the EPAS module
  - Incorrect image can disable power steering

Only proceed if you:
  - Need to return to stock EPAS firmware
  - Understand and accept the risks

Press START only if you accept these risks."""


DOWNLOAD_US_MAPS_INSTRUCTIONS = """\
Download US OSM speed-limit maps

Fetches a prebuilt sqlite of OpenStreetMap maxspeed ways for the United States
and installs it at /data/media/0/osm/speed_limits.sqlite.

This is the first-install of the full published US pack (not Overpass).
Not stored in git (too large). After flash, run this once over Wi-Fi.

PRECONDITIONS:
  1. Device is offroad / parked
  2. Wi-Fi that can reach GitHub Releases
  3. 850 MiB free on /data (same filesystem as /data/media/0/osm/)
     ~200 MiB zst + ~541 MiB sqlite + 80 MiB margin. Stages in
     /data/media/0/osm/.download/ — not /tmp.

If a previous download died with ENOSPC / "No space left on device":
  rm -f /data/media/0/osm/*.partial /data/media/0/osm/.download/*
  then clear old routes/videos if df -h /data is still short.

Takes a few minutes. mapd reloads onroad within ~15 seconds — no reboot.

For later local updates use Refresh maps (live OSM within 100 miles).

Data is (c) OpenStreetMap contributors (ODbL).
https://www.openstreetmap.org/copyright

Press START to download."""


REFRESH_MAPS_INSTRUCTIONS = """\
Query live OSM within 100 miles and merge into the US pack

Queries live OpenStreetMap speed limits within 100 miles (~160.9 km) of
this car and merges them into the installed US maps. Ways in that radius
are replaced; the rest of the US pack is kept. Tagged OSM maxspeed wins.
In Minnesota, unmarked roads also get statutory estimates (not uploaded
to OSM) so Refresh maps does not wipe those pack fills.

Uses a GNSS fix if one arrives (waits up to 10s), otherwise last stored GPS.
Will not guess a city. If maps are not installed yet, downloads the US pack
first and then overlays the 100-mile extract -- never a 100-mile-only file.

PRECONDITIONS:
  1. Device is offroad / parked
  2. Wi-Fi that can reach Overpass (overpass-api.de)
  3. GPS: wait for a satellite fix, or start openpilot onroad until the GPS
     icon/fix is up for about a minute, then retry
  4. Stages the merge on /data/media/0/osm/.download/ -- not /tmp.
     A previous good sqlite is kept if OSM times out or the merge fails.

Overpass can take several minutes. mapd reloads onroad within ~15s --
no reboot.

Data is (c) OpenStreetMap contributors (ODbL).
https://www.openstreetmap.org/copyright

Press START to refresh."""


ACKNOWLEDGMENTS_INTRO = "Special thanks to the following members. This project wouldn't be possible without you:"

ACKNOWLEDGMENTS_NAMES = [
  "Boggyver and the Tinkla Project",
  "Lukas Loetkolben",
  "Johnmr1",
  "SeriouslySerious",
  "Pod042",
  "1FrostlySlime",
]


def acknowledgments_html() -> str:
  """Render the acknowledgments block as HTML for HtmlRenderer."""
  names_html = "<br>".join(f"<b>{n}</b>" for n in ACKNOWLEDGMENTS_NAMES)
  return f"<p>{ACKNOWLEDGMENTS_INTRO}</p><p>{names_html}</p>"


def acknowledgments_text() -> str:
  """Render the acknowledgments block as plain text."""
  bullets = "\n".join(f"  {n}" for n in ACKNOWLEDGMENTS_NAMES)
  return f"{ACKNOWLEDGMENTS_INTRO}\n\n{bullets}"


def find_preset_index(presets: list, value, default: int = 0) -> int:
  """Find the closest matching preset index for a given value."""
  try:
    return presets.index(value)
  except ValueError:
    return min(range(len(presets)), key=lambda i: abs(presets[i] - value))
