This repo switches from layer-only edge scroll to always-active circular scroll:
- start near 3h => vertical scroll
- start near 6h => horizontal scroll
Normal pointer remains active elsewhere.

Current limitation:
- tap-to-click is not implemented in this repo yet. In absolute mode, Cirque hardware tap is not available the same way as relative mode, so this needs a separate software tap implementation.
