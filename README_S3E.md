# S3E multi-robot project adaptation

This local branch adds an optional direct synchronized export for the
neighboring `FAST-LIVO2-ROS2/research` pipeline. The frontend is EllipseLIO;
the other repository supplies the existing artifact writer, MapClosures and
distributed PCM/CBS integration.

From the workspace `src` directory:

```bash
bash FAST-LIVO2-ROS2/scripts/build_ellipselio.sh
FAST-LIVO2-ROS2/scripts/s3e_experiment.sh run --stage all \
  --config FAST-LIVO2-ROS2/research/configs/square1-ellipselio-mapclosures-cbs.yaml \
  --resume
```

The build defines `S3E_RESEARCH_SOURCE` and installs in a separate
`.ros2/ellipse-install` overlay. A normal upstream build without that CMake
option omits the exporter. `research.enabled` defaults false; reliable input
QoS is also opt-in. The default estimator equations and adaptive mapping
path are retained.

See [the wrapper's integration documentation](../FAST-LIVO2-ROS2/research/ELLIPSELIO.md)
for timing, coordinate, calibration, image-free artifact and evaluation contracts.
The upstream revision is `171acea502f1122e3043a460d2e6e3a30d8f8246`.

For ellipsoid BEV experiments, optional `research.ellipsoid_stamps_ns` and
`research.ellipsoid_range_m` request complete fitted-map snapshots after tensor
voting. The extra PointCloud2 channel records native centers, geometric
semi-axes and orthonormal axis directions in the current IMU frame. This uses
the same bounded MCAP writer and does not rely on the sparse visualization
markers. Empty fitted maps during initialization are recorded explicitly.
See the wrapper's ellipsoid-map BEV diagnostic for rendering and results.

For a single-pass export, `research.ellipsoid_keyframes: true` selects snapshots
causally from the current LiDAR-updated state, using
`research.keyframe_translation_m` (1 m), `research.keyframe_rotation_deg`
(10 degrees) and `research.keyframe_interval_s` (2 seconds). This is mutually
exclusive with an explicit timestamp schedule and defaults to disabled.
Snapshots are taken after `MapIncremental` under the map lock. The first exported
frame is selected even if it has no fitted ellipsoids yet. Every requested stamp
is the exact current frame timestamp. The overnight adapter independently checks
the native selection against the saved poses before descriptor preparation.
