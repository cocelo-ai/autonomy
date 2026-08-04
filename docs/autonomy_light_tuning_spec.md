# Elevation-map tuning

Start with `height_map.source: rolling` for control. It keeps a bounded,
probabilistic local surface and exposes `valid`, `variance`, and `age` to the
policy. Tune only these groups first:

```yaml
rolling_elevation:
  max_age_sec: 0.45
  max_radius_m: 2.0
  base_variance: 0.0004
  range_variance_factor: 0.0005
  mahalanobis_threshold: 3.0
  initial_prior:
    footprint_radius_m: 0.30
    ground_distance_m: 0.48
    variance: 0.04
algorithm:
  min_z:
    min_points_per_cell: 2
    obstacle_min_height: 0.035
  frame_aggregation:
    cell_height_percentile: 0.15
```

- Raise `min_points_per_cell` to reject isolated returns; lower it only when
  valid coverage is persistently too small.
- Reduce `max_age_sec` for fast motion or dynamic obstacles; increase it only
  for sparse static terrain.
- Raise `range_variance_factor` when distant terrain should contribute less.
- Keep `obstacle_min_height` below the smallest step the policy must detect.
- Set `initial_prior.ground_distance_m` to the calibrated vertical distance from
  `base_link` to nominal ground; keep its variance much larger than sensor
  variance so the first observation dominates the startup footprint posterior.

Use `global` only when a persistent PCD is the intended terrain reference.
The global source does not age out; it should not be used as a substitute for
dynamic-obstacle sensing. In Isaac Lab, train with the quality channels and
the same unknown convention as the deployed controller.

For a non-ROS consumer, choose `height_map_output.transport: cyclone_dds`.
It sends the same `float` grid at `publish_rate_hz`, without quality metadata;
use `both` while validating equivalence against ROS `HeightMap.data`.
