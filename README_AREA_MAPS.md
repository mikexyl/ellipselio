# Updated EllipseLIO with accumulated-area exports

This branch is based on upstream `6506f46f1947b4ef86cfba402f11f10a6ef520ee`.
It retains persistent-map odometry and adds export-only accumulated-area snapshots,
reliable sensor input, native update diagnostics, and stable-block octree growth.
It rejects enabled temporal submapping, area-cropped odometry, and the separate
research deskew exporter. The original temporal-submap implementation remains on
`dev/multi-robot-lidar-slam-submaps`.

Use this branch (`dev/upstream-ellipselio-octree-growth`) with
[FAST-LIVO2-ROS2](https://github.com/mikexyl/FAST-LIVO2-ROS2/tree/dev/multi-robot-lidar-slam-submaps).
Build with `-DS3E_RESEARCH_SOURCE=/absolute/path/to/FAST-LIVO2-ROS2`; that repository
provides the bounded export writer in `include/research_export.h` and
`src/research_export.cpp`. With the normal ROS2 dependencies installed:

```sh
colcon build --packages-select ellipselio --cmake-args \
  -DS3E_RESEARCH_SOURCE=/absolute/path/to/FAST-LIVO2-ROS2 -DBUILD_TESTING=ON
ctest --test-dir build/ellipselio --output-on-failure
```

Enable `mapping.area_maps.enabled`, keep `mapping.submaps.enabled` and
`mapping.area_maps.odometry` false, and use the wrapper's
`scripts/recent_submaps/run_trial.py --area-maps --persistent-odometry` launcher.
The tested defaults export all native map representatives within an 80 m
horizontal radius every 20 m or 10 s, without restricting point age or height.
Snapshots and ellipsoids use the current IMU anchor frame. They contain processed
map geometry, not full-resolution raw scans; tensor support remains the native
persistent-map support. Shutdown tails are saved for inspection.

Octree leaf backing storage grows in blocks without relocating existing pointers.
The remaining native point-capacity check still fails explicitly rather than
silently discarding history. No correspondence or optimizer thresholds change.

Validation includes octree growth/deletion, area membership and anchor transforms,
state/covariance preservation, immutable packets, bounded-writer backpressure and
shutdown, plus fresh full-length runs of all eight GRACO aerial and six ground
sequences. The wrapper contains the associated multilayer BEV/MapClosures/PCM/CBS
tests, reports, configuration and artifact hashes.
