# SuperLIO PCD to Smac 2D Grid Design

## Goal

Convert `/home/nvidia/Super-LIO/src/super_lio/map(室内)/map.pcd` into a static two-dimensional occupancy map that `nav2_map_server` and `SmacPlanner2D` can load. This step only creates map artifacts; it does not start navigation or send motion commands.

## Outputs

- `/home/nvidia/Super-LIO/src/super_lio/map(室内)/smac_map_0p10.pgm`
- `/home/nvidia/Super-LIO/src/super_lio/map(室内)/smac_map_0p10.yaml`

The YAML references the PGM, uses `0.10 m` resolution, preserves the PCD XY origin in the SuperLIO `map` frame, and uses standard Nav2 occupancy thresholds.

## Conversion

The converter fits the dominant near-horizontal lower surface as a floor plane and evaluates point height relative to that plane:

- points from `-0.10 m` through less than `0.10 m` relative to the floor provide free-space evidence;
- points from `0.10 m` through `1.00 m` above the floor are occupied;
- higher points are ignored so ceilings and overhead structures are not projected as obstacles;
- cells without free or occupied evidence remain unknown;
- occupied evidence takes precedence over free evidence.

The first preview performs no gap filling and does not apply the robot inflation radius, so it cannot silently invent free space from a static PCD. Nav2's global costmap inflation layer owns obstacle inflation.

## Validation

Before Smac is connected:

1. Verify the PGM dimensions, YAML origin, and resolution.
2. Load the files with `nav2_map_server` and confirm `/map` is a `nav_msgs/msg/OccupancyGrid` in frame `map`.
3. Inspect the map in RViz for recognizable walls, traversable floor, unknown exterior, and absence of ceiling artifacts.

This first output is a preview for map-quality review. No SCAN mode change, UDP output, or automatic arming behavior is changed.
