"""nap-release build switches.

nap-release is the drivable backup: only features that were road-tested on
nap-dev. These flags turn off what is not, without deleting code, so the
cereal schema, params, and shared helpers (pathobstacled uses speedsignd.nv12)
stay identical to nap-dev.
"""
NAP_RELEASE = True

# Speed sign reader: speedsignd is not started, its toggles and the SIGN
# plate are hidden. Code, params and the liveSpeedSignNAP struct stay.
SPEED_SIGN_ENABLED = not NAP_RELEASE

# Roundabout Steering Assist (ring curvature blend, NAPRoundaboutAssist):
# forced off and its toggle hidden. Roundabout yield (#290) and MAX restore
# on exit (#289) are part of the tested steering/long stack and stay.
ROUNDABOUT_ASSIST_ENABLED = not NAP_RELEASE

# Log-only outputs that change nothing the driver sees: cone-line wouldSteer
# (published as 0.0). The cone line itself stays because Hold my line uses it.
LOG_ONLY_EXTRAS = not NAP_RELEASE

# Curve follow (NAPCurveFollow): forced off on release, whatever the param says.
# It is untuned for active use (see curve_follow_eval). CurveMaxHold in card
# keeps handling bends exactly as before (it only steps aside in active mode).
CURVE_FOLLOW_ENABLED = not NAP_RELEASE
