# docs/media

Media assets for the root README. Benchmark charts use measured data; schematics illustrate the architecture and development workflow.

## Light and dark twins

`cerulion-logo`, `development-loop` and `architecture-flow` each ship three files: the theme-aware source (`<name>.svg`, which follows the viewer's operating system and is the `<img>` fallback) and two fixed-palette twins, `<name>-light.svg` and `<name>-dark.svg`, which the README names in a `<picture>` block so GitHub picks by the theme the viewer chose. An SVG inside an `<img>` cannot see the page, so without the twins a light system on a dark GitHub theme shows the light drawing and the wordmark disappears. The twins are generated: edit the source, then run `python3 tools/scripts/build_media_theme_variants.py` (the Lint job runs it with `--check`).

## Assets the root README uses

| File | Used at | What it shows | Source |
|---|---|---|---|
| `cerulion-logo.svg` (`-static` = no motion) | README header | the official Cerulion wordmark, centered, with a short light sweep every three seconds; reduced motion renders the static twin | the website's vector wordmark, geometry preserved |
| `development-loop.svg` (`-static` = same drawing without motion) | top of the README | the development loop as a schematic: run, inspect, capture, re-execute, review; a feedback path turns an incident into a regression test. One token circles the track: it crosses the five stages at one speed and takes the long return path at twice that speed, so each half of the lap takes about eight seconds | drawn from the shipped CLI verbs and docs; a schematic, not a measurement |
| `native-rtt.svg` | Performance, native graph round trips | round-trip p50 lines with p50 to p99 bands, 64 B to 16 MiB, log scales, six series measured by one harness in one posture at a fixed 100 Hz: Cerulion multi-process, Cerulion single-process, ROS 2 Jazzy with `rmw_fastrtps_cpp` defaults, ROS 2 composed with intra-process communication, zenoh shared memory, and ROS 2 Jazzy over `rmw_cerulion` with loaned publishing and loaned takes; recording on for the native series | the first five series: [Native fixed100 campaign](../benchmarks/results/8a84baf25d5d1710-2026-09-16-fixed100-heroes/); the `rmw_cerulion` series: [ROS 2 over rmw_cerulion in the native harness](../benchmarks/results/8a84baf25d5d1710-2026-09-18-fixed100-rmw-cerulion/), its `jazzy-cerulion-loan-k5/results_jazzy_cerulion_shm_loan_be1_chrt0.csv` |
| `platform-rtt.svg` | Performance, across platforms | native round trips on Linux x86_64, NVIDIA Jetson Orin NX and Apple M4, single- and multi-process, recording on | the hero package above plus [Jetson Orin NX campaign](../benchmarks/results/9b0c5fbf0f55dea4-2026-09-17-fixed100-jetson-orin-nx/) and [Apple M4 campaign](../benchmarks/results/0b7bc3994f78e232-2026-09-17-fixed100-apple-m4/) |
| `architecture-flow.svg` (`-static` = same drawing without motion) | How it fits together | robot-side nodes, the graph scheduler, shared memory and capture; netd's separate LAN and WAN planes and the matching desk planes; local mirrors; Studio, the CLI and agents; account ownership and grants authorizing remote access. The animated version walks one cycle in order, about 30 s: the scheduler drives sensing, then perception, then control through shared memory; a frame leaves on demand through netd; the desk request and the account gates light before data crosses; mirrors feed Studio; every frame reaches the recorder and then replay, and the resim path closes the loop. Long arrows share one dot speed, and no arrow takes longer than about three seconds to cross, so the long recording-to-replay arrow does not dominate the cycle | drawn from the shipped components; a schematic |
| `studio-arm.gif` | See the robot in Studio | a screen recording of Studio showing an industrial six-axis arm: the live 3D model driven by its joint states, the discovered ROS 2 and MoveIt topics with liveness and rates, and a controller state plot. A phone recording of the real arm is inset in the empty corner of the scene, so the model and the arm can be seen moving together. All six moves of a one minute session play at real speed, 15.9 s in total. The holds between moves are cut, from half a second after the arm stops to half a second before it starts again, so the plot's time axis jumps at each cut while the arm's pose does not. A label in the scene's top-left corner reads 1x speed, and a fast-forward mark flashes beside it just before and just after each cut | both recorded live and cut at the same timestamps (their motion profiles line up best at zero offset); the Studio window is cropped at its top and bottom edges; the inset, its border, the speed label and the fast-forward mark are composited in editing, and the recordings themselves are otherwise unaltered; the file carries no metadata from either recording |
| `quickstart-record-verify.gif` | Quickstart: run, record and verify | a real terminal recording of the obstacle avoidance quickstart, 53.5 s: both nodes built, the graph validated and run with recording, the bag verified against the same node builds (replay PASS), the safety controller's forward speed changed from 0.3 to 0.25, the node rebuilt, and the same bag verified again (frame-content divergence). Red outlines mark the recording path, the PASS line and the divergence block | recorded on a desk from the shipped example with the commands the README shows, typed at natural speed; one recorder status line is masked; nothing else altered |

## Regenerating the benchmark charts

`native-rtt.svg` and `platform-rtt.svg` are drawn by [`charts/build_benchmark_plots.py`](charts/build_benchmark_plots.py) from the packages under [`docs/benchmarks/results/`](../benchmarks/results/). From the repository root:

```bash
uv run --quiet --python 3.12 --with matplotlib==3.10.6 --with fonttools python docs/media/charts/build_benchmark_plots.py
```

The script rewrites the two SVGs in place, and puts their PNG twins and a manifest of every source, point, floor and output hash under `charts/out/`, which git ignores. With the pinned matplotlib it reproduces the committed SVGs byte for byte, so `git diff docs/media` is the check: an empty diff means the figures still describe the packages.

It refuses to draw rather than draw something unreviewed:

- anything under `docs/benchmarks/results/` that differs from `HEAD` stops it, so a figure always describes a committed package;
- every CSV's sha256 and every plotted p50 and p99 must equal [`charts/expected-chart-points.json`](charts/expected-chart-points.json), the reviewed point ledger, and each figure's point hash must equal [`charts/reviewed-point-hashes.json`](charts/reviewed-point-hashes.json);
- the `rmw_cerulion` series is re-reduced from its 50 raw `.bin` files and checked against [`charts/rmw-same-harness-point-ledger.json`](charts/rmw-same-harness-point-ledger.json), and its `run.json` must record the machine and posture of the stock ROS 2 row it is drawn beside.

Layout uses the macOS system faces (Helvetica Neue and Menlo), so the script runs on macOS. Publishing a new package means updating the ledger files in the same change, after the new points have been reviewed.

[`charts/render-themes.sh`](charts/render-themes.sh) previews a figure the way a GitHub README shows it, in the light and the dark theme. It needs Google Chrome: `docs/media/charts/render-themes.sh native-rtt platform-rtt` writes PNGs under `charts/out/renders/`.
