This repo uses always-active circular scroll with mode lock until finger lift:
- start near 3h => vertical scroll
- start near 6h => horizontal scroll
- pointer remains active elsewhere

The implementation now waits for a fresh X and Y sample after touch-down before deciding the mode, which avoids accidental fallback to cursor mode caused by stale coordinates from the previous touch.

Tap-to-click is implemented in software by converting touch down/up (ABS_Z threshold crossings) into BTN_TOUCH press/release events, then mapping those events through a hold-tap behavior to a left click on quick tap.
