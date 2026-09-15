# NAP Developer Docs

Documentation for contributors working on NotAutopilot. These are NAP-specific — upstream openpilot docs live in `docs/` (comma.ai).

## Start here

- **[contributing.md](contributing.md)** — branch structure, pre-push checklist, submodules, how to submit bugs
- **[architecture.md](architecture.md)** — Pre-AP Model S design overview: standalone safety model, radar emulation, pedal interceptor
- **[safety-model.md](safety-model.md)** — panda safety invariants for the Pre-AP target
- **[map-speed.md](map-speed.md)** — OSM map speed → HUD MAX / cruise set speed (comma 3X)
- **[force-offroad.md](force-offroad.md)** — Settings → triple-tap NAP → Force Offroad / Go Offline (started=false while moving)
- **[speed-sign-log.md](speed-sign-log.md)** — on-drive MUTCD speed-sign JSONL logger (log-only, default off)
- **[engagement.md](engagement.md)** — stalk FSM, pedal-vs-no-pedal engagement paths, brake behavior, driver-wheel lateral handoff (default On; intent-to-steer yield; emergency hard-brake full cancel; Settings can disable)
- **[hypermile.md](hypermile.md)** — Hypermile eco snap + Hill Climb (nap-dev experimental, default Off). Stock Follow Distance 1–7 is shared; Hypermile does not own follow.
- **[preap-rain-sensor.md](preap-rain-sensor.md)** — Pre-AP Light/Rain: NAP Int is TIPWIPE `0x10`, not INTERVAL1/2; no CAN rain — keep camera Auto unless parked spray proves BCM rain-arm

## Layout

```
openpilot-nap/
├── docs/            # upstream openpilot docs (comma.ai)
├── docs-nap/        # ← you are here
├── opendbc_repo/    # submodule: NotAutopilot/opendbc fork
├── panda/           # submodule: commaai/panda (tracked upstream)
├── selfdrive/
└── ...
```

The Pre-AP car port lives in `opendbc_repo/opendbc/car/tesla/preap/` and the standalone panda safety mode in `opendbc_repo/opendbc/safety/modes/tesla_preap.h`.
