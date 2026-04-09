This repo switches from layer-only edge scroll to always-active circular scroll:
- start near 3h => vertical scroll
- start near 6h => horizontal scroll
Normal pointer remains active elsewhere.

Current status:
- tap-to-click is implemented in software from ABS_Z with a small motion threshold.
- circular scroll stays locked until finger lift once the gesture direction has been decided.
