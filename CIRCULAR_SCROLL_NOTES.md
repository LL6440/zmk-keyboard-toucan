This repo switches from layer-based edge scrolling to an always-on circular gesture:
- start near 3h -> vertical scroll
- start near 6h -> horizontal scroll
- otherwise -> normal pointer

Main tuning values in boards/shields/toucan/toucan.dtsi:
- inner-ring-pct
- sector-half-angle-deg
- activation-angle-deg
- scroll-angle-deg
- invert-vertical / invert-horizontal
