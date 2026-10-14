# Porting NAP (jmbrunick fork) onto a new NAP / comma version

This is a living checklist. Update it in the same PR whenever you add or move a hook into an upstream file.

## Baselines
- Upstream: `NotAutopilot/openpilot` `nap-dev`. This fork's content was branched from it around 2026-08-28 (openpilot 0.11.1, AGNOS 18.4).
- comma: `commaai/openpilot`. The latest tag is v0.11.1. Master says 0.11.2 and needs AGNOS 19.9. Since #38219 (2026-06-21), comma master keeps the code under `openpilot/`.
- opendbc: `.gitmodules` points at `jmbrunick/opendbc`. That pin is NAP's opendbc plus about 61 patches (one-pedal/long, gap-lock CarState, lateral-yield A/B, driverBrakeApplied, regen floor, 0x488 stamping).

## Rules for new code
1. Put new logic in new files: `selfdrive/nap/...`, `selfdrive/{mapd,speedsignd,pathobstacled,nap_dash}`, `system/hardware/nap_*`, `selfdrive/car/tesla/preap_*`. Leave only short hooks in upstream files.
2. Imports: `from openpilot.<pkg> ...`. cereal is the one exception: plain `cereal` until the nested-layout port (`openpilot.cereal` does not exist on 0.11.1).
3. Manager entries: use `openpilot.`-prefixed module strings.
4. No file paths built as `BASEDIR + "/selfdrive/..."`. Resolve paths from `__file__`.
5. No new carState subscribers (msgq reader limit).
6. Steering and longitudinal changes go in separate PRs.

## Port checklist
1. Branch from the new NAP tag or nap-dev, never from comma directly.
2. Rebase the opendbc patch series onto NAP's new opendbc pin. Re-run the panda safety tests.
3. Re-apply the hooks listed below, file by file. Bring new NAP-only files over as they are.
4. Renames in comma master since 0.11.1 that need hook updates:
   - `selfdrive/car/car_specific.py` became `car_events.py` (#38493).
   - `selfdrive/ui/feedback/feedbackd.py` was removed (#38401). Our replacement is `selfdrive/nap/bookmark/bookmarkd.py`.
5. Nested layout (comma >= #38219): `git mv cereal common selfdrive system tools openpilot/`. Then fix the path strings found by `rg '"(selfdrive|system|cereal|common)[/.]'`, including the `.github/ci` mutation definitions and the tests. Switch `import cereal` to `openpilot.cereal`.
6. Regenerate the process replay refs, run Pre-AP and the unit tests (compare against the known-failure baseline), then do a road test on a separate branch before nap-dev.

## Hook sites (upstream files we modify)
These are generated with `git diff -U0 <NAP nap-dev> HEAD`. Each line is a changed hunk and the function it sits in.

### `.gitattributes`
- (top level) (x1)

### `.gitmodules`
- (top level) (x1)

### `cereal/custom.capnp`
- $Cxx.namespace("cereal"); (x1)

### `cereal/log.capnp`
- struct ControlsState @0x97ff69c53601abf1 { (x1)
- struct Event { (x1)
- struct LongitudinalPlan @0xe00b5b3eba12876c { (x1)
- struct OnroadEvent @0xc4fa6047f024e718 { (x1)
- struct RadarState @0x9a185389d6fdd05f { (x1)

### `cereal/services.py`
- _services: dict[str, tuple] = { (x1)

### `common/params_keys.h`
- inline static std::unordered_map<std::string, ParamKeyAttributes> keys = { (x7)

### `opendbc_repo`
- (top level) (x1)

### `selfdrive/car/car_specific.py`
- class CarSpecificEvents: (x5)
- from opendbc.car.interfaces import MAX_CTRL_SPEED (x1)
- from opendbc.car.toyota.values import ToyotaFlags (x1)

### `selfdrive/car/card.py`
- (top level) (x1)
- EventName = log.OnroadEvent.EventName (x1)
- carlog.addHandler(ForwardingHandler(cloudlog)) (x1)
- class Car: (x15)
- from openpilot.selfdrive.pandad import can_capnp_to_list, can_list_to_can_capnp (x1)

### `selfdrive/controls/controlsd.py`
- LaneChangeState = log.LaneChangeState (x1)
- class Controls: (x13)
- from opendbc.car.vehicle_model import VehicleModel (x1)
- from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature (x1)
- from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque (x1)
- from openpilot.selfdrive.controls.lib.longcontrol import LongControl (x1)

### `selfdrive/controls/lib/desire_helper.py`
- DESIRES = { (x1)
- LANE_CHANGE_ARM_TIME = 7.0 (x1)
- LANE_CHANGE_TIME_MAX = 10. (x1)
- LaneChangeDirection = log.LaneChangeDirection (x1)
- class DesireHelper: (x24)
- from openpilot.common.constants import CV (x1)

### `selfdrive/controls/lib/longcontrol.py`
- LongCtrlState = car.CarControl.Actuators.LongControlState (x1)
- class LongControl: (x4)
- from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N (x1)

### `selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py`
- class LongitudinalMpc: (x1)

### `selfdrive/controls/lib/longitudinal_planner.py`
- MIN_ALLOW_THROTTLE_SPEED = 2.5 (x1)
- class LongitudinalPlanner: (x20)
- def get_coast_accel(pitch): (x1)
- def limit_accel_in_turns(v_ego, angle_steers, a_target, CP): (x3)
- from openpilot.common.swaglog import cloudlog (x1)
- from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET (x1)
- from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import ( (x1)
- import math (x1)

### `selfdrive/controls/plannerd.py`
- def main(): (x1)

### `selfdrive/controls/radard.py`
- class LeadTrackAssociation: (x8)
- class RadarD: (x12)
- class Track: (x2)
- def is_association_candidate(v_ego: float, vision_d_rel: float, lead: capnp._Dyn (x1)
- def main() -> None: (x2)
- def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks (x2)
- from openpilot.common.simple_kalman import KF1D (x1)
- import math (x1)

### `selfdrive/modeld/modeld.py`
- def get_action_from_model(model_output: dict[str, np.ndarray], prev_action: log. (x2)
- def main(demo=False): (x4)
- from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper (x1)
- from openpilot.selfdrive.modeld.helpers import usbgpu_present, modeld_pkl_path, (x1)

### `selfdrive/monitoring/dmonitoringd.py`
- def dmonitoringd_thread(): (x2)
- from openpilot.common.realtime import config_realtime_process (x1)

### `selfdrive/monitoring/policy.py`
- class DriverMonitoring: (x11)
- def to_percent(v): (x1)
- from cereal import car, log (x1)
- from math import atan2, radians (x1)
- from openpilot.common.filter_simple import FirstOrderFilter (x1)
- from openpilot.common.transformations.camera import DEVICE_CAMERAS (x1)

### `selfdrive/selfdrived/events.py`
- EVENTS: dict[int, dict[str, Alert | AlertCallbackType]] = { (x4)
- def audio_feedback_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubM (x1)
- def modeld_lagging_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubM (x1)
- def pcm_disable_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMast (x2)
- def personality_changed_alert(CP: car.CarParams, CS: car.CarState, sm: messaging (x1)
- def process_not_running_alert(CP: car.CarParams, CS: car.CarState, sm: messaging (x1)
- from openpilot.common.realtime import DT_CTRL (x1)
- from openpilot.system.hardware import HARDWARE (x1)

### `selfdrive/selfdrived/helpers.py`
- (top level) (x1)
- MIN_LATERAL_ENGAGE_BUFFER = int(1 / DT_CTRL) (x1)
- class ExcessiveActuationCheck: (x5)
- from enum import StrEnum, auto (x2)
- from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX (x1)

### `selfdrive/selfdrived/preap_regen.py`
- REGEN_DEMAND_CLEAR_SPEED = 1.0  # m/s (x1)
- and release are silent. (x1)
- class PreAPChimeState(NamedTuple): (x1)
- class RegenDemandCheck: (x4)
- def update_preap_chimes(*, lat_engaged: bool, long_engaged: bool, (x3)
- from opendbc.car.tesla.preap.interface import get_preap_accel_limits (x1)
- from typing import NamedTuple (x1)

### `selfdrive/selfdrived/selfdrived.py`
- class SelfdriveD: (x32)
- from openpilot.selfdrive.car.car_specific import CarSpecificEvents (x1)
- from openpilot.selfdrive.selfdrived.events import Events, ET (x1)
- from openpilot.system.hardware import HARDWARE (x1)

### `selfdrive/ui/feedback/feedbackd.py`
- (top level) (x1)
- def main(): (x6)
- from openpilot.common.swaglog import cloudlog (x1)
- from openpilot.system.micd import SAMPLE_RATE, SAMPLE_BUFFER (x1)

### `selfdrive/ui/layouts/main.py`
- class MainLayout(Widget): (x1)
- from openpilot.selfdrive.ui.body.layouts.onroad import BodyLayout (x1)

### `selfdrive/ui/layouts/settings/nap.py`
- class NAPLayout(Widget): (x24)
- from opendbc.car.tesla.preap.nap_params import NAPParamKeys, DEFAULTS (x1)
- from openpilot.selfdrive.ui.layouts.settings.nap_content import ( (x4)
- from openpilot.selfdrive.ui.ui_state import ui_state (x1)
- from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog (x1)

### `selfdrive/ui/layouts/settings/nap_content.py`
- CALIBRATE_PEDAL_INSTRUCTIONS = tr_noop("""Keep the car ON for calibration. (x2)
- PEDAL_CAN_BUS_VALUES = [0, 2] (x1)
- Press START only if you accept these risks.""" (x1)
- RADAR_OFFSET_MAX = 2.0 (x1)
- Settings are saved only after calibration succeeds. (x1)
- calibration, restore) have a single source. (x1)
- def find_preset_index(presets: list, value, default: int = 0) -> int: (x1)
- def pedal_calibration_entry_block_reason() -> str | None: (x2)
- def pedal_calibration_entry_enabled() -> bool: (x1)
- or choose Restart device. No automatic restart.""") (x1)

### `selfdrive/ui/layouts/settings/settings.py`
- (top level) (x1)
- class SettingsLayout(Widget): (x6)
- from openpilot.selfdrive.ui.layouts.settings.firehose import FirehoseLayout (x1)
- from openpilot.selfdrive.ui.layouts.settings.toggles import TogglesLayout (x1)

### `selfdrive/ui/mici/layouts/main.py`
- class MiciMainLayout(Scroller): (x1)
- from openpilot.system.ui.lib.application import gui_app (x1)

### `selfdrive/ui/mici/layouts/settings/nap.py`
- class NAPLayoutMici(NavScroller): (x9)
- class RadarSettingsLayoutMici(NavScroller): (x2)
- from opendbc.car.tesla.preap.nap_params import NAPParamKeys (x1)
- from openpilot.selfdrive.ui.layouts.settings.nap_content import ( (x3)
- from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog, Bi (x1)

### `selfdrive/ui/mici/layouts/settings/nap_script.py`
- (top level) (x2)
- def launch_script(title: str, instructions: str, script_module: str) -> None: (x2)
- from openpilot.common.basedir import BASEDIR (x1)
- import os (x1)
- import subprocess (x1)
- pandad/card/controlsd/etc. for the duration. (x1)
- panel that's been shipping on nap-release. (x1)

### `selfdrive/ui/mici/layouts/settings/settings.py`
- (top level) (x1)
- class SettingsLayout(NavScroller): (x3)
- from openpilot.selfdrive.ui.mici.layouts.settings.firehose import FirehoseLayout (x1)
- from openpilot.selfdrive.ui.mici.layouts.settings.nap import NAPLayoutMici (x1)

### `selfdrive/ui/mici/onroad/augmented_road_view.py`
- class AugmentedRoadView(CameraView): (x1)

### `selfdrive/ui/mici/onroad/hud_renderer.py`
- class HudRenderer(Widget): (x8)
- from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar (x1)

### `selfdrive/ui/mici/onroad/model_renderer.py`
- class ModelRenderer(Widget): (x3)

### `selfdrive/ui/mici/widgets/big_multi_value_param.py`
- class BigMultiValueParamToggle(BigMultiToggle): (x1)

### `selfdrive/ui/onroad/augmented_road_view.py`
- class AugmentedRoadView(CameraView): (x1)

### `selfdrive/ui/onroad/hud_renderer.py`
- class HudRenderer(Widget): (x10)
- from openpilot.common.constants import CV (x1)
- from openpilot.selfdrive.ui.onroad.exp_button import ExpButton (x1)

### `selfdrive/ui/onroad/model_renderer.py`
- class ModelRenderer(Widget): (x3)

### `selfdrive/ui/soundd.py`
- class Soundd: (x6)
- from openpilot.system.hardware import HARDWARE (x1)
- if HARDWARE.get_device_type() == "tizi": (x1)

### `selfdrive/ui/ui_state.py`
- class UIState: (x6)
- from openpilot.selfdrive.ui.radar.bosch_status import BoschRadarMonitor, BoschRa (x1)

### `system/hardware/hardwared.py`
- def hardware_thread(end_event, hw_queue) -> None: (x36)
- def hw_state_thread(end_event, hw_queue): (x1)
- from openpilot.system.hardware.fan_controller import FanController (x1)

### `system/manager/process.py`
- class ManagerProcess(ABC): (x2)
- class PythonProcess(ManagerProcess): (x3)

### `system/manager/process_config.py`
- def only_offroad(started: bool, params: Params, CP: car.CarParams) -> bool: (x1)
- from openpilot.system.manager.process import PythonProcess, NativeProcess, Daemo (x1)
- procs = [ (x3)

### `system/ui/lib/wifi_manager.py`
- DEFAULT_TETHERING_PASSWORD = "swagswagcomma" (x1)
- _dbus_call_idx = 0 (x1)
- class WifiManager: (x3)
- except Exception: (x1)

### `system/ui/mici/widgets/script_runner_app.py`
- (top level) (x1)
- class MiciScriptRunnerApp: (x2)
- wires ``on_start`` and ``on_exit`` callbacks and decides what those mean. (x1)

### `system/ui/widgets/script_runner.py`
- class ScriptRunner(Widget): (x5)

## Feature owners
- Speed sign (`selfdrive/speedsignd`, `ui/onroad/speed_sign_hud.py`, `scripts/nap/*speed_sign*`): Speed Sign agent. They add their hook list here.
- Phone dash / 7070 / MCU link (`selfdrive/nap_dash`): Phone MCU Link agent. They add their hook list here.
- Everything else: NAP main agent.
