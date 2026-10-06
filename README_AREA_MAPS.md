# Persistent-map odometry with observation exports

This integration is based on upstream
`6506f46f1947b4ef86cfba402f11f10a6ef520ee`. Odometry always matches against its
persistent native map. There are no temporal windows, successor maps, handovers,
or area-cropped odometry modes.

Bounded accumulated-area exports provide point observations to
[submap_slam](https://github.com/mikexyl/submap_slam). That package creates the
fixed circle submaps used by MapClosures and CBS. The exported areas do not
restrict the odometry map or its correspondences.

Build against the installed `submap_slam_io` package with
`-DELLIPSELIO_AREA_EXPORT=ON`. The maintained deployment instructions and
four-thread launcher are in `submap_slam`; no FAST-LIVO2 source is required.

Enable `mapping.area_maps.enabled` to export native map representatives within
an 80 m horizontal radius every 20 m or 10 s. Observation exports do not restrict
point age or height. Saved points use the current IMU anchor frame and contain
processed map geometry. The backend planner independently uses 40 m circles.
The existing snapshot identifiers and frame/timestamp metadata are preserved
for artifact readers.

The integration also provides reliable sensor input, essential native update
telemetry, and stable-block octree growth. Octree leaf storage grows without
relocating existing pointers. The native point-capacity check fails explicitly
rather than discarding history. Correspondence and optimizer thresholds remain
unchanged.

Unknown mapping parameters are rejected. Retired odometry-mode settings are
not declared or accepted, including when set to false.

Native tests cover octree growth, area membership, anchor transforms, immutable
packets, state/covariance preservation, rejected unknown mapping parameters,
bounded-writer backpressure and shutdown.
