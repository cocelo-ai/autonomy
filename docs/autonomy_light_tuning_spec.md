# Elevation-map tuning

Start with `height_map.source: rolling` for control. It keeps a bounded,
pose-following probabilistic local surface and exposes `valid`, `variance`, and
`age` to the policy. A fixed local-memory grid shifts with robot position while
the published `grid.x_length × grid.y_length` output is sampled at robot yaw;
tune only these groups first:

```yaml
rolling_elevation:
  max_radius_m: 2.0
  base_variance: 0.0004
  range_variance_factor: 0.0005
  mahalanobis_threshold: 3.0
  initial_prior:
    footprint_radius_m: 0.30
    ground_distance_m: 0.48
    variance: 0.04
  visibility:
    enabled: true
    max_ray_length_m: 2.0
    min_observation_age_sec: 0.08
  overhang_filter:
    enabled: true
    near_max_above_sensor_m: 0.20
    ramp_start_m: 1.0
    ramp_slope: 0.30
algorithm:
  min_z:
    min_points_per_cell: 2
    obstacle_min_height: 0.035
  frame_aggregation:
    cell_height_percentile: 0.15
```

- Raise `min_points_per_cell` to reject isolated returns; lower it only when
  valid coverage is persistently too small.
- `max_radius_m` is both the input range guard and retained local-memory
  half-width. When position crosses one resolution cell, the grid shifts and
  copies its overlap; only newly exposed strips are invalidated. Rotation does
  not erase the memory grid.
- Raise `range_variance_factor` when distant terrain should contribute less.
- Keep `obstacle_min_height` below the smallest step the policy must detect.
- Set `initial_prior.ground_distance_m` to the calibrated vertical distance from
  `base_link` to nominal ground; keep its variance much larger than sensor
  variance so the first observation dominates the startup footprint posterior.
- `visibility.min_observation_age_sec` is the response delay before a ray may
  erase a stale cell. Keep it below the LiDAR period, but nonzero to protect a
  just-updated edge. `normal_alignment_min` and `height_margin_m` trade cleanup
  speed against grazing-ray conservatism.
- The overhang ramp starts with `near_max_above_sensor_m` and grows by
  `ramp_slope` after `ramp_start_m`. Do not raise it to accommodate terrain
  below the sensor; that admits ceilings and hanging obstacles.

Use `global` only when a persistent PCD is the intended terrain reference.
The global source does not age out; it should not be used as a substitute for
dynamic-obstacle sensing. In Isaac Lab, train with the quality channels and
the same unknown convention as the deployed controller.

For a non-ROS consumer, choose `height_map_output.transport: cyclone_dds`.
It sends the same `float` grid at `publish_rate_hz`, without quality metadata;
use `both` while validating equivalence against ROS `HeightMap.data`.
