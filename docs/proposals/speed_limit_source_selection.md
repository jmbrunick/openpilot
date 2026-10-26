# Speed limit source selection: Tesla vs OSM, with manual override

## Problem
Tesla's map limit (0x238) and OSM can disagree; the driver can't tell which source wins or choose.

Motivating case: Tesla says 30, actual is 55, no findable sign to upgrade the overlay, OSM believed correct (e.g. County Road 31 north of DeGraff, Swift County MN).

## A) Source toggle
Overlay setting to switch the active source between Tesla and OSM, per road or globally.

## B) Manual override
Press and hold 2 s on the on-screen speed limit sign (comma overlay, not nav) to open manual entry. Up/down arrows change the limit in 5 mph increments. The value is saved to the overlay for that road segment, forward and backward, and sticks until cleared.

## C) OSM comparison
The same hold shows OSM's number beside Tesla's before overriding.

In the 2 s hold popup, Tesla's and OSM's limits are shown side by side. Tapping directly on either value instantly sets that number as the active limit for the current road segment, with no arrows needed. The arrows remain for fine-tuning in 5 mph steps, and Save still writes the chosen value to the overlay for that segment, forward and backward.

## Optional: confidence rule
A disagreement above a threshold (e.g. 20 mph) auto-falls back to OSM and flags it on the sign review list.

## Notes
- Each part is its own PR, goes through the Pre-AP check, and is not shipped to nap-alpha/nap-dev without Justin's OK.
- Builds on #380, #387, #389.
