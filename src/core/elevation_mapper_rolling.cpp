#include "autonomy_light/elevation_mapper.hpp"

namespace autonomy_light {

void ElevationMapper::moveRollingGrid(const Pose2_5D &robot) {
  const int output_radius = static_cast<int>(
      std::ceil(0.5 * std::hypot(config_.grid.x_length, config_.grid.y_length) /
                config_.grid.resolution));
  const int radius =
      std::max(output_radius + 1,
               static_cast<int>(std::ceil(config_.rolling_max_radius_m /
                                          config_.grid.resolution)));
  const int width = 2 * radius + 1;
  const CellKey center = keyFor(robot.x, robot.y);
  const CellKey origin{center.x - radius, center.y - radius};
  if (!rolling_grid_.initialized || rolling_grid_.width != width ||
      rolling_grid_.height != width) {
    rolling_grid_ = {origin, width, width, true,
                     std::vector<Surface>(static_cast<std::size_t>(width) *
                                          static_cast<std::size_t>(width))};
    return;
  }
  const std::int64_t dx = origin.x - rolling_grid_.origin.x;
  const std::int64_t dy = origin.y - rolling_grid_.origin.y;
  if (dx == 0 && dy == 0) {
    return;
  }
  std::vector<Surface> shifted(rolling_grid_.cells.size());
  for (int row = 0; row < width; ++row) {
    for (int col = 0; col < width; ++col) {
      const std::int64_t source_col = static_cast<std::int64_t>(col) + dx;
      const std::int64_t source_row = static_cast<std::int64_t>(row) + dy;
      if (source_col < 0 || source_col >= width || source_row < 0 ||
          source_row >= width) {
        continue;
      }
      shifted[static_cast<std::size_t>(row) * width + col] =
          rolling_grid_.cells[static_cast<std::size_t>(source_row) * width +
                              static_cast<std::size_t>(source_col)];
    }
  }
  rolling_grid_.origin = origin;
  rolling_grid_.cells = std::move(shifted);
}

ElevationMapper::Surface *ElevationMapper::rollingSurface(const CellKey &key) {
  return const_cast<Surface *>(std::as_const(*this).rollingSurface(key));
}

const ElevationMapper::Surface *
ElevationMapper::rollingSurface(const CellKey &key) const {
  if (!rolling_grid_.initialized) {
    return nullptr;
  }
  const std::int64_t col = key.x - rolling_grid_.origin.x;
  const std::int64_t row = key.y - rolling_grid_.origin.y;
  if (col < 0 || col >= rolling_grid_.width || row < 0 ||
      row >= rolling_grid_.height) {
    return nullptr;
  }
  return &rolling_grid_
              .cells[static_cast<std::size_t>(row) * rolling_grid_.width +
                     static_cast<std::size_t>(col)];
}

const ElevationMapper::Surface *
ElevationMapper::nearestRollingSurface(const double x, const double y) const {
  const CellKey center = keyFor(x, y);
  const Surface *best = nullptr;
  double best_distance2 = std::numeric_limits<double>::infinity();
  for (std::int64_t dy = -1; dy <= 1; ++dy) {
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      const CellKey key{center.x + dx, center.y + dy};
      const Surface *surface = rollingSurface(key);
      if (surface == nullptr || !std::isfinite(surface->ground) ||
          !validRolling(*surface, false)) {
        continue;
      }
      const double px =
          (static_cast<double>(key.x) + 0.5) * config_.grid.resolution;
      const double py =
          (static_cast<double>(key.y) + 0.5) * config_.grid.resolution;
      const double distance2 = (px - x) * (px - x) + (py - y) * (py - y);
      if (distance2 < best_distance2) {
        best = surface;
        best_distance2 = distance2;
      }
    }
  }
  return best;
}

} // namespace autonomy_light
