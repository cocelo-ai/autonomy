#pragma once

#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace autonomy_light {

// Runtime representation of the URDF-derived, row-major FOV mask generated
// by scripts/generate_height_fov_mask.py.  Keeping the format deliberately
// small makes the exact same mask usable by the mapper and the DDS bridge.
struct FovMask {
  bool enabled{false};
  std::uint32_t width{0};
  std::uint32_t height{0};
  double resolution{0.0};
  double length_x{0.0};
  double length_y{0.0};
  std::vector<std::uint8_t> values;

  [[nodiscard]] bool inside(const std::size_t row_major_index) const {
    return !enabled || row_major_index < values.size() && values[row_major_index] != 0U;
  }
};

inline FovMask loadFovMask(const std::string &path, const std::uint32_t expected_width,
                           const std::uint32_t expected_height,
                           const double expected_resolution,
                           const double expected_length_x,
                           const double expected_length_y) {
  if (path.empty()) {
    return {};
  }
  std::ifstream input(path);
  if (!input) {
    throw std::invalid_argument("cannot open FOV mask file: " + path);
  }
  std::string magic;
  std::getline(input, magic);
  if (magic != "AUTONOMY_LIGHT_FOV_MASK_V1") {
    throw std::invalid_argument("invalid FOV mask header in: " + path);
  }
  FovMask mask;
  if (!(input >> mask.width >> mask.height >> mask.resolution >> mask.length_x >> mask.length_y)) {
    throw std::invalid_argument("invalid FOV mask geometry in: " + path);
  }
  const auto close = [](const double left, const double right) {
    return std::fabs(left - right) <= 1.0e-6;
  };
  if (mask.width != expected_width || mask.height != expected_height ||
      !close(mask.resolution, expected_resolution) ||
      !close(mask.length_x, expected_length_x) || !close(mask.length_y, expected_length_y)) {
    throw std::invalid_argument("FOV mask geometry does not match height-map geometry: " + path);
  }
  const std::size_t expected_cells = static_cast<std::size_t>(mask.width) * mask.height;
  mask.values.resize(expected_cells);
  for (std::size_t index = 0; index < expected_cells; ++index) {
    int value = -1;
    if (!(input >> value) || (value != 0 && value != 1)) {
      throw std::invalid_argument("invalid FOV mask cell in: " + path);
    }
    mask.values[index] = static_cast<std::uint8_t>(value);
  }
  std::string extra;
  if (input >> extra) {
    throw std::invalid_argument("too many FOV mask cells in: " + path);
  }
  mask.enabled = true;
  return mask;
}

}  // namespace autonomy_light
