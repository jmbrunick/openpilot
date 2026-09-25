#pragma once

#include <string>
#include <unordered_map>

#include "cereal/gen/cpp/log.capnp.h"

inline static std::unordered_map<std::string, ParamKeyAttributes> keys = {
    {"AccessToken", {CLEAR_ON_MANAGER_START | DONT_LOG, STRING}},
    {"AdbEnabled", {PERSISTENT, BOOL}},
    {"AlwaysOnDM", {PERSISTENT, BOOL}},
    {"ApiCache_Device", {PERSISTENT, STRING}},
    {"ApiCache_FirehoseStats", {PERSISTENT, JSON}},
    {"AssistNowToken", {PERSISTENT, STRING}},
    {"AthenadPid", {PERSISTENT, INT}},
    {"AthenadUploadQueue", {PERSISTENT, JSON}},
    {"AthenadRecentlyViewedRoutes", {PERSISTENT, STRING}},
    {"BootCount", {PERSISTENT, INT}},
    {"CalibrationParams", {PERSISTENT, BYTES}},
    {"CameraDebugExpGain", {CLEAR_ON_MANAGER_START, STRING}},
    {"CameraDebugExpTime", {CLEAR_ON_MANAGER_START, STRING}},
    {"CarBatteryCapacity", {PERSISTENT, INT}},
    {"CarParams", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BYTES}},
    {"CarParamsCache", {CLEAR_ON_MANAGER_START, BYTES}},
    {"CarParamsPersistent", {PERSISTENT, BYTES}},
    {"CarParamsPrevRoute", {PERSISTENT, BYTES}},
    {"CompletedTrainingVersion", {PERSISTENT, STRING, "0"}},
    {"ControlsReady", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"CurrentBootlog", {PERSISTENT, STRING}},
    {"CurrentRoute", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, STRING}},
    {"DisableLogging", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"DisablePowerDown", {PERSISTENT, BOOL}},
    {"DisableUpdates", {PERSISTENT, BOOL}},
    {"DisengageOnAccelerator", {PERSISTENT, BOOL, "0"}},
    {"DongleId", {PERSISTENT, STRING}},
    {"DoReboot", {CLEAR_ON_MANAGER_START, BOOL}},
    {"DoShutdown", {CLEAR_ON_MANAGER_START, BOOL}},
    {"DoUninstall", {CLEAR_ON_MANAGER_START, BOOL}},
    {"DriverTooDistracted", {CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON, BOOL}},
    {"AlphaLongitudinalEnabled", {PERSISTENT | DEVELOPMENT_ONLY, BOOL}},
    {"ExperimentalMode", {PERSISTENT, BOOL}},
    {"ExperimentalModeConfirmed", {PERSISTENT, BOOL}},
    {"FirmwareQueryDone", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"ForcePowerDown", {PERSISTENT, BOOL}},
    {"GitBranch", {PERSISTENT, STRING}},
    {"GitCommit", {PERSISTENT, STRING}},
    {"GitCommitDate", {PERSISTENT, STRING}},
    {"GitDiff", {PERSISTENT, STRING}},
    {"GithubSshKeys", {PERSISTENT, STRING}},
    {"GithubUsername", {PERSISTENT, STRING}},
    {"GitRemote", {PERSISTENT, STRING}},
    {"GsmApn", {PERSISTENT, STRING}},
    {"GsmMetered", {PERSISTENT, BOOL, "1"}},
    {"GsmRoaming", {PERSISTENT, BOOL}},
    {"HardwareSerial", {PERSISTENT, STRING}},
    {"HasAcceptedTerms", {PERSISTENT, STRING, "0"}},
    {"InstallDate", {PERSISTENT, TIME}},
    {"IsDriverViewEnabled", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsEngaged", {PERSISTENT, BOOL}},
    {"IsLdwEnabled", {PERSISTENT, BOOL}},
    {"IsMetric", {PERSISTENT, BOOL}},
    {"IsOffroad", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsOnroad", {PERSISTENT, BOOL}},
    {"IsRhdDetected", {PERSISTENT, BOOL}},
    {"IsReleaseBranch", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsTakingSnapshot", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsTestedBranch", {CLEAR_ON_MANAGER_START, BOOL}},
    {"JoystickDebugMode", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"LanguageSetting", {PERSISTENT, STRING, "en"}},
    {"LastAthenaPingTime", {CLEAR_ON_MANAGER_START, INT}},
    {"LastGPSPosition", {PERSISTENT, STRING}},
    {"LastManagerExitReason", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastOffroadStatusPacket", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, JSON}},
    {"LastAgnosPowerMonitorShutdown", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastPowerDropDetected", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastUpdateException", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastUpdateRouteCount", {PERSISTENT, INT, "0"}},
    {"LastUpdateTime", {PERSISTENT, TIME}},
    {"LastUpdateUptimeOnroad", {PERSISTENT, FLOAT, "0.0"}},
    {"LiveDelay", {PERSISTENT, BYTES}},
    {"LiveParameters", {PERSISTENT, JSON}},
    {"LiveParametersV2", {PERSISTENT, BYTES}},
    {"LiveTorqueParameters", {PERSISTENT | DONT_LOG, BYTES}},
    {"LocationFilterInitialState", {PERSISTENT, BYTES}},
    {"LateralManeuverMode", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"LongitudinalManeuverMode", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"LongitudinalPersonality", {PERSISTENT, INT, std::to_string(static_cast<int>(cereal::LongitudinalPersonality::STANDARD))}},
    {"NetworkMetered", {PERSISTENT, BOOL}},
    {"ObdMultiplexingChanged", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"ObdMultiplexingEnabled", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"Offroad_CarUnrecognized", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_ConnectivityNeeded", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_ConnectivityNeededPrompt", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_ExcessiveActuation", {PERSISTENT, JSON}},
    {"Offroad_IsTakingSnapshot", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_NeosUpdate", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_NoFirmware", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_Recalibration", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_TemperatureTooHigh", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_UnregisteredHardware", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_UpdateFailed", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_DriverMonitoringUncertain", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"OnroadCycleRequested", {CLEAR_ON_MANAGER_START, BOOL}},
    {"OpenpilotEnabledToggle", {PERSISTENT, BOOL, "1"}},
    {"PandaHeartbeatLost", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"PrimeType", {PERSISTENT, INT}},
    {"RecordAudio", {PERSISTENT, BOOL}},
    {"RecordAudioFeedback", {PERSISTENT, BOOL, "0"}},
    {"RecordFront", {PERSISTENT, BOOL}},
    {"RecordFrontLock", {PERSISTENT, BOOL}},  // for the internal fleet
    {"SecOCKey", {PERSISTENT | DONT_LOG, STRING}},
    {"ShowDebugInfo", {PERSISTENT, BOOL}},
    {"RouteCount", {PERSISTENT, INT, "0"}},
    {"SnoozeUpdate", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"SshEnabled", {PERSISTENT, BOOL}},
    {"UbloxAvailable", {PERSISTENT, BOOL}},
    {"UpdateAvailable", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"UpdateFailedCount", {CLEAR_ON_MANAGER_START, INT}},
    {"UpdaterAvailableBranches", {PERSISTENT, STRING}},
    {"UpdaterCurrentDescription", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterCurrentReleaseNotes", {CLEAR_ON_MANAGER_START, BYTES}},
    {"UpdaterFetchAvailable", {CLEAR_ON_MANAGER_START, BOOL}},
    {"UpdaterNewDescription", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterNewReleaseNotes", {CLEAR_ON_MANAGER_START, BYTES}},
    {"UpdaterState", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterTargetBranch", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterLastFetchTime", {PERSISTENT, TIME}},
    {"UptimeOffroad", {PERSISTENT, FLOAT, "0.0"}},
    {"UptimeOnroad", {PERSISTENT, FLOAT, "0.0"}},
    {"UsbGpuPresent", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"UsbGpuCompiled", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"Version", {PERSISTENT, STRING}},

    // NAP (NotAutopilot) Pre-AP Tesla params
    {"NAPBrakeFactor", {PERSISTENT, FLOAT, "1.0"}},
    {"NAPFollowDistance", {PERSISTENT, INT, "4"}},
    // City (<~50 mph) and highway (>~50 mph) Follow Distance 1–7. Migrated
    // from NAPFollowDistance on first use. Mannerisms always; not Hypermile.
    {"NAPFollowDistanceCity", {PERSISTENT, INT, "4"}},
    {"NAPFollowDistanceHwy", {PERSISTENT, INT, "4"}},
    {"NAPFollowDistanceSplitMigrated", {PERSISTENT, BOOL, "0"}},
    // One-shot Follow Distance HUD. card sets on every stalk persist
    // (including a tip already at 1 or 7). selfdrived consumes and
    // holds the toast ~1.5 s. Not a preference.
    {"NAPFollowHudPending", {CLEAR_ON_MANAGER_START, BOOL, "0"}},
    // Settings → NAP → Driving Mannerisms → Hypermile. Default Off.
    // On: comfort-biased eco-snap (Adaptive Accel, Cap/Follow,
    // Early lookahead, Accel 1 lazy climb) — early light ease, not max
    // regen bite. Restore those knobs on Off. Soft-lat / DM / blinker /
    // stock 1–7 Follow Distance unchanged (Hypermile does not own follow).
    {"NAPHypermile", {PERSISTENT, BOOL, "0"}},
    {"NAPHypermileSaved", {PERSISTENT, STRING}},
    // Opt-in mileage defer. Default Off. Inert unless Hypermile is On.
    // Lowers the Cap/Follow posted target on the same posted scale as eco,
    // larger drop (−15 at 80). Does not stack with eco. Never exceeds posted.
    {"NAPHypermileStepDown", {PERSISTENT, BOOL, "0"}},
    // Hypermile sub-toggle. Default On. Inert unless Hypermile is On.
    // IMU-pitch climb hold + crest/downhill ease. No maps-elevation lookahead.
    // Never raises HUD MAX. Lead / MPC brake still wins.
    {"NAPHypermileHillClimb", {PERSISTENT, BOOL, "1"}},
    {"NAPForcePreAP", {PERSISTENT, BOOL, "1"}},
    {"NAPiBoosterEnabled", {PERSISTENT, BOOL}},
    {"NAPPedalCalibDone", {PERSISTENT, BOOL}},
    {"NAPPedalCalibFactor", {PERSISTENT, FLOAT, "1.0"}},
    {"NAPPedalCalibMax", {PERSISTENT, FLOAT, "99.6"}},
    {"NAPPedalCalibMin", {PERSISTENT, FLOAT, "-3.0"}},
    {"NAPPedalCalibZero", {PERSISTENT, FLOAT, "0.0"}},
    {"NAPPedalCanBus", {PERSISTENT, INT, "2"}},
    {"NAPAdaptiveAccel", {PERSISTENT, BOOL, "1"}},
    // Settings → NAP → Driving Mannerisms → One-Pedal Long. Default Off.
    // Pedal mode: gas rising from rest kicks software long off (silent
    // pause, like brake). After RELEASE, lift/near-zero accel is Tesla
    // regen through interceptor pass-through — not a GAS_COMMAND rewrite.
    // Off = stock gas override + A+B/A3 resume climb.
    {"NAPOnePedalLong", {PERSISTENT, BOOL, "0"}},
    {"NAPPedalEnabled", {PERSISTENT, BOOL}},
    {"NAPPedalProfile", {PERSISTENT, INT, "4"}},
    {"NAPRadarBehindNosecone", {PERSISTENT, BOOL}},
    {"NAPRadarDonorVin", {PERSISTENT, STRING}},
    {"NAPRadarEnabled", {PERSISTENT, BOOL}},
    {"NAPRadarEpasType", {PERSISTENT, INT, "0"}},
    {"NAPRadarHud", {PERSISTENT, BOOL}},
    {"NAPRadarIgnoreHwFail", {PERSISTENT, BOOL}},
    {"NAPRadarOffset", {PERSISTENT, FLOAT, "0.0"}},
    {"NAPRadarPosition", {PERSISTENT, INT, "0"}},
    {"NAPRadarReadVin", {CLEAR_ON_MANAGER_START, BOOL}},
    {"NAPRadarVinReadStatus", {CLEAR_ON_MANAGER_START, STRING}},
    {"NAPScriptRunning", {CLEAR_ON_MANAGER_START, BOOL}},
    // OSM map speed → HUD MAX. Mode: 0=off 1=display 2=cap 3=follow. Lookahead 2=normal. Accel 5=default.
    {"NAPMapSpeedMode", {PERSISTENT, INT, "0"}},
    {"NAPMapSpeedOffsetMph", {PERSISTENT, INT, "0"}},
    {"NAPMapSpeedLookahead", {PERSISTENT, INT, "2"}},
    {"NAPMapSpeedAccel", {PERSISTENT, INT, "5"}},
    {"NAPMapSpeedDbPath", {PERSISTENT, STRING}},
    {"NAPMapSpeedDbRevision", {PERSISTENT, STRING}},
    {"NAPMapSpeedDbSha256", {PERSISTENT, STRING}},
    // Soft wheel lateral handoff. Default ON. Light purposeful push +
    // hands frees the EPS (latActive false). Stay yielded while
    // handsOnLevel >= 1; blend after ~80 ms hands-off. Hard brake
    // during yield fully cancels. Settings can turn Off.
    {"NAPDriverLatHandoff", {PERSISTENT, BOOL, "1"}},
    // Pre-AP DM: while engaged, full looking-path wipe on the stock
    // vision path (no-face / uncertain / phone / pose / eye). After
    // drain past 1.0 s, fire at random in the next 2.0 s (fire in
    // (1.0, 3.0] of that countdown). Hold until awareness recovers.
    // Mutually exclusive with NAPDmFalseAlertIgnore. Triple-tap
    // Settings → NAP. Default Off on nap-release.
    {"NAPDmSimulateLooking", {PERSISTENT, BOOL, "0"}},
    // Pre-AP DM: soft-clear false phone/device distraction only
    // (phoneProb / distracted_types phone). Same 1–3 s cadence as
    // Simulate Look. Pose and eye still drain / alert. Triple-tap
    // Settings → NAP (third item). Mutually exclusive with Simulate
    // Look. Default Off on nap-release. PERSISTENT like Simulate Look
    // (preference, not a session flag).
    {"NAPDmFalseAlertIgnore", {PERSISTENT, BOOL, "0"}},
    // Settings → triple-tap NAP → Force Offroad. Default off. Not persistent: reboot
    // (manager start) and the next ignition ON clear it. Toggle Off and
    // Reset to Defaults also clear. When on, hardwared keeps started=false.
    {"NAPForceOffroad", {CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON, BOOL, "0"}},
    {"TermsVersion", {PERSISTENT, STRING}},
    {"TrainingVersion", {PERSISTENT, STRING}},
};
