# Speed limit source selection: Tesla vs OSM, with manual override

## Problem
Tesla's map limit (0x238) and OSM can disagree; the driver can't tell which source wins or choose.

Motivating case: Tesla says 30, actual is 55, no findable sign to upgrade the overlay, OSM believed correct (e.g. County Road 31 north of DeGraff, Swift County MN).

## A) Source toggle
Overlay setting to switch the active source between Tesla and OSM, per road or globally.

## B) Manual override
Press and hold 2 s on the on-screen speed limit sign (comma overlay, not nav) to open manual entry. Up/down arrows change the limit in 5 mph increments. The value is saved to the overlay for the whole current map-limit stretch (previous map change behind the car to the next one ahead), both directions, and sticks until cleared.

## C) OSM comparison
The same hold shows OSM's number beside Tesla's before overriding.

In the 2 s hold popup, Tesla's and OSM's limits are shown side by side. Tapping directly on either value instantly sets that number as the active limit for the current road segment, with no arrows needed. The arrows remain for fine-tuning in 5 mph steps, and Save still writes the chosen value to the overlay for that segment, forward and backward.

## Optional: confidence rule
A disagreement above a threshold (e.g. 20 mph) auto-falls back to OSM and flags it on the sign review list.

## Notes
- Each part is its own PR, goes through the Pre-AP check, and is not shipped to nap-alpha/nap-dev without Justin's OK.
- Builds on #380, #387, #389.

## Update (Oct 27 2026): manual Save covers the whole map stretch
A manual Save (`NAPSpeedLimitManual`, row `source: manual`) no longer starts at the car. You
usually fix a limit only after it has been wrong for a while, so the row covers the whole current
map-limit stretch: from the previous map limit change **behind** the car to the next map limit
change ahead, in both directions of travel (row `sign_at: manual_stretch`, `back_m` = how far back
it reaches). No carry past either change.

- The walk back also stops at a road change, an ambiguous fork, or 10 km (`CHAIN_MAX_M`).
- If no map data is known for the road, the row covers only the current OSM way (both ends).
- If the stretch can't be rebuilt through the car (e.g. a fork seen only one way), it falls back
  to the old behaviour: start at the car (`sign_at: manual`).
- Confirmed sign overrides are unchanged: they still never extend behind the sign (#387).
