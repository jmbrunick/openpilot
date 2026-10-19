// jmbrunick NAP fork params. Included inside the keys initializer in
// common/params_keys.h, so entries here are plain map entries.
// Keep fork-only keys here, not in params_keys.h (smaller upstream diff).
    {"NAPBookmarkRequest", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, INT}},
    {"NAPBookmarkRecording", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, INT}},
    // Stock Follow Distance 1–7. Stalk behind a radar lead writes this
    // (Hypermile On or Off). Driving Mannerisms slider stays visible.
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
    // Lowers the Cap/Follow posted target by a fixed 15 mph (75→60).
    // Does not stack with the eco −5 offset. Never exceeds posted.
    {"NAPHypermileStepDown", {PERSISTENT, BOOL, "0"}},
    // Hypermile sub-toggle. Default On. Inert unless Hypermile is On.
    // IMU-pitch climb hold + crest/downhill ease. No maps-elevation lookahead.
    // Never raises HUD MAX. Lead / MPC brake still wins.
    {"NAPHypermileHillClimb", {PERSISTENT, BOOL, "1"}},
    // Curve-follow (continuous curve term in the longitudinal planner).
    // 0 = off, 1 = shadow (compute + log only; old preview and CurveMaxHold
    // stay in force), 2 = active (replaces the preview, retires the
    // CurveMaxHold MAX cap). Default 1.
    {"NAPCurveFollow", {PERSISTENT, INT, "2"}},  // nap-alpha test: active
    {"NAPSightCap", {PERSISTENT, INT, "1"}},
    // Settings → NAP → Lateral Control. Default On (Pre-AP only). Turn
    // geometry correction: low-speed lag + rear reference offset (m) in
    // modeld's plan sampling time. On also drops the legacy roundabout
    // outer bias. Off = previous lateral behavior.
    {"NAPLatTurnGeom", {PERSISTENT, BOOL, "1"}},
    {"NAPLatRefOffset", {PERSISTENT, FLOAT, "0.35"}},
    {"NAPTurnInDelay", {PERSISTENT, INT, "0"}},
    // Settings → NAP → Driving Mannerisms → Roundabout Steering Assist.
    // Pre-AP, default Off. Near / on a mapped OSM ring, blends the model's
    // curvature toward the lane's circle curvature (map-match confidence
    // weighted; camera keeps lane edges; driver torque overrides as today).
    {"NAPRoundaboutAssist", {PERSISTENT, BOOL, "0"}},
    // mapd → controlsd: fitted ring geometry of the hinted roundabout (JSON).
    {"NAPRoundaboutRing", {CLEAR_ON_MANAGER_START, JSON}},
    // controlsd -> modeld: the ring assist is latched on a ring; DesireHelper ignores stalk tips (no ALC) while true.
    {"NAPRoundaboutLatched", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    // Settings → NAP → Driving Mannerisms → One-Pedal Long. Default Off.
    // Pedal mode: gas rising from rest kicks software long off (silent
    // pause, like brake). After RELEASE, lift/near-zero accel is Tesla
    // regen through interceptor pass-through — not a GAS_COMMAND rewrite.
    // Off = stock gas override + A+B/A3 resume climb.
    {"NAPOnePedalLong", {PERSISTENT, BOOL, "0"}},
    // Settings → NAP → Driving Mannerisms → Gap lock. Default Off.
    // Pedal mode: 2 s engage-stalk hold latches radar dRel (long may be paused).
    {"NAPGapLock", {PERSISTENT, BOOL, "0"}},
    {"NAPRadarIgnoreSensorDirty", {PERSISTENT, BOOL, "1"}},
    // Companion Dash process. Default On. Off stops nap_dash without
    // blocking engage (process is optional / non-critical).
    {"NAPDashEnabled", {PERSISTENT, BOOL, "1"}},
    // OSM map speed → HUD MAX. Mode: 0=off 1=display 2=cap 3=follow. Lookahead 2=normal. Accel 5=default.
    {"NAPMapSpeedMode", {PERSISTENT, INT, "0"}},
    {"NAPMapSpeedOffsetMph", {PERSISTENT, INT, "0"}},
    {"NAPMapSpeedLookahead", {PERSISTENT, INT, "2"}},
    {"NAPMapSpeedAccel", {PERSISTENT, INT, "5"}},
    {"NAPMapSpeedDbPath", {PERSISTENT, STRING}},
    {"NAPMapSpeedDbRevision", {PERSISTENT, STRING}},
    {"NAPMapSpeedDbSha256", {PERSISTENT, STRING}},
    // Pre-AP 0x45 stalk wiper/high-beam. Default 0 = today's forwarded stalk.
    // Wiper UI: 0=off 3=auto. Legacy 1=int / 2=on coerce to Off.
    // Auto overlays collar INTERVAL1 while on, in Drive/Reverse, moving,
    // and the 3X ROAD camera sees rain or ice/frost.
    // Beam: 0/1 leave stalk, 2=high (low nibble 4).
    // DAS wiper/beam fields stay 0. No auto headlights.
    {"NAPWiperSpeed", {PERSISTENT, INT, "0"}},
    // One-shot Wipers Off/Auto HUD. card/body sets on collar 0→1→0 toggle.
    // selfdrived consumes and holds the toast ~2.5 s. Not a preference.
    {"NAPWiperHudPending", {CLEAR_ON_MANAGER_START, BOOL, "0"}},
    // Auto wiper sensitivity 0–4. Mid 2 = baseline. Lower = wipe sooner /
    // more often (more dry). Higher = tolerate more film (more wet).
    {"NAPWiperSensitivity", {PERSISTENT, INT, "2"}},
    // Live Auto-wiper status (~1 Hz Params.put). STRING so unknown-key put
    // cannot fail silently. No CLEAR_ON_ONROAD: loggerd embeds Params in qlog
    // InitData at route start (same snapshot copied into each segment), so an
    // onroad wipe would hide this from Connect qlog digs. Last 1 Hz line must
    // survive into the next snapshot. Still cleared on manager start.
    {"NAPWiperRainStatus", {CLEAR_ON_MANAGER_START, STRING}},
    {"NAPHighLowBeam", {PERSISTENT, INT, "0"}},
    // On-drive MUTCD speed-sign JSONL logger. Default off. Log-only: no sqlite,
    // no vCruise / HUD MAX, no osm.org. Process: speedsignd.
    {"NAPSpeedSignLog", {PERSISTENT, BOOL, "0"}},
    {"NAPSpeedSignConfirmed", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, JSON}},
    // Soft wheel lateral handoff. Default ON. Yield on driver intent
    // (sustained torsion + aligned rate + hands on). Stay yielded while
    // handsOnLevel >= 1; blend after ~80 ms hands-off. Hard brake during
    // yield fully cancels. Settings can turn Off.
    {"NAPDriverLatHandoff", {PERSISTENT, BOOL, "1"}},
    // Low-visibility lateral back-off. Default On. When lane/road-edge
    // confidence collapses with a wide path, or road-camera exposure
    // collapses (sooner if the sun is low and ahead), lateral eases
    // toward the driver and the HUD says Low visibility. Longitudinal
    // stays engaged. Off = stock lateral, no alert.
    {"NAPLowVisBackoff", {PERSISTENT, BOOL, "1"}},
    // After a ~2 s push away from a radar cone line, the soft-handoff
    // re-take holds that lateral offset instead of blending onto the
    // model path. Default On. Longitudinal is unchanged.
    {"NAPConeLineHold", {PERSISTENT, BOOL, "1"}},
    // Log-only cone-line detector (coneLineNAP in the qlog). Default On.
    // Off stops the publish; the hold then sees no line. Does not steer.
    {"NAPConeLineLog", {PERSISTENT, BOOL, "1"}},
    // Log-only radar obstacle detector (pathObstacleNAP). Default On.
    // Animals, people, and solid debris in or beside the lane. Does not
    // brake or steer. Off stops the log and the chime.
    {"NAPObstacleLog", {PERSISTENT, BOOL, "1"}},
    // Camera-first animal/person detector (animalDetectNAP, selfdrive/nap/animal_detect).
    // Default Off. LOG ONLY: no chime, alert, regen, or braking. Little cores, optional process.
    {"NAPAnimalDetect", {PERSISTENT, BOOL, "0"}},
    // Chime when radar and vision agree something solid is in the lane
    // or stepping into it, and for an animal or a person near the road.
    // Default On. A still mailbox or post on the shoulder stays quiet.
    // Does not brake, steer, or change engagement.
    {"NAPObstacleChime", {PERSISTENT, BOOL, "1"}},
    {"NAPObstacleBrake", {PERSISTENT, BOOL, "0"}},
    // Pre-AP DM: while engaged, full looking-path wipe on the stock
    // vision path (no-face / uncertain / phone / pose / eye). After
    // drain past 1.0 s, fire at random in the next 2.0 s (fire in
    // (1.0, 3.0] of that countdown). Hold until awareness recovers.
    // Mutually exclusive with NAPDmFalseAlertIgnore. Triple-tap
    // Settings → NAP. Default On (nap-dev).
    {"NAPDmSimulateLooking", {PERSISTENT, BOOL, "1"}},
    // Pre-AP DM: soft-clear false phone/device distraction only
    // (phoneProb / distracted_types phone). Same 1–3 s cadence as
    // Simulate Look. Pose and eye still drain / alert. Triple-tap
    // Settings → NAP (third item). Mutually exclusive with Simulate
    // Look — default Off on nap-dev so both are not On. PERSISTENT
    // like Simulate Look (preference, not a session flag).
    {"NAPDmFalseAlertIgnore", {PERSISTENT, BOOL, "0"}},
    // Settings → NAP → Force Offroad. Default off. Not persistent: reboot
    // (manager start) and the next ignition ON clear it. Toggle Off and
    // Reset to Defaults also clear. When on, hardwared keeps started=false
    // after the Pre-AP stock-CC handoff reports ready (or a short timeout).
    {"NAPForceOffroad", {CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON, BOOL, "0"}},
    // card → hardwared: stock CC is ENABLED (or handoff gave up). Do not
    // CLEAR_ON_OFFROAD — going offroad is the point. Default off.
    {"NAPForceOffroadHandoffReady", {CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON, BOOL, "0"}},
    // UI Yes on "Ready to resume steering control?". Default off. Handoff
    // and started=false wait for this while onroad. Same clear flags.
    {"NAPForceOffroadConfirmed", {CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON, BOOL, "0"}},
