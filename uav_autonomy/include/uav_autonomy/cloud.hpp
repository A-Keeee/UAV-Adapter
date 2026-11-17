#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cstring>
#include <map>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdexcept>
#include <vector>

namespace uav_autonomy {
using Vec = Eigen::Vector3d;
using Points = std::vector<Vec>;
struct CloudStats {
  size_t total = 0, finite = 0;
};

// Reads the complete organized cloud, including row padding and either byte order.
// Resource limits reject the entire message; they never select an input prefix.
template <class Consumer>
CloudStats visitXYZ(const sensor_msgs::msg::PointCloud2 &m, Consumer consume) {
  const uint64_t count = uint64_t(m.width) * m.height;
  if (count == 0 && m.data.empty()) return {};
  if (!m.point_step || uint64_t(m.row_step) < uint64_t(m.width) * m.point_step ||
      m.data.size() < uint64_t(m.row_step) * m.height || count > 4000000 ||
      m.data.size() > 256 * 1024 * 1024)
    throw std::runtime_error("Malformed or resource-limit-exceeding PointCloud2");
  std::array<sensor_msgs::msg::PointField, 3> f;
  for (int axis = 0; axis < 3; ++axis) {
    const std::string name = std::string(1, "xyz"[axis]);
    auto it = std::find_if(m.fields.begin(), m.fields.end(),
                           [&](const auto &v) { return v.name == name; });
    if (it == m.fields.end() || it->count != 1 || (it->datatype != 7 && it->datatype != 8) ||
        uint64_t(it->offset) + (it->datatype == 7 ? 4 : 8) > m.point_step)
      throw std::runtime_error("XYZ must be scalar FLOAT32 or FLOAT64 within point_step");
    f[axis] = *it;
  }
  const uint16_t endian = 1;
  const bool swap = (*reinterpret_cast<const uint8_t *>(&endian) == 0) != m.is_bigendian;
  CloudStats stats;
  stats.total = count;
  for (uint32_t row = 0; row < m.height; ++row)
    for (uint32_t col = 0; col < m.width; ++col) {
      Vec p;
      for (int axis = 0; axis < 3; ++axis) {
        size_t n = f[axis].datatype == 7 ? 4 : 8;
        auto ptr =
            m.data.data() + size_t(row) * m.row_step + size_t(col) * m.point_step + f[axis].offset;
        std::array<uint8_t, 8> b{};
        std::copy(ptr, ptr + n, b.begin());
        if (swap) std::reverse(b.begin(), b.begin() + n);
        if (n == 4) {
          float v;
          std::memcpy(&v, b.data(), 4);
          p[axis] = v;
        } else
          std::memcpy(&p[axis], b.data(), 8);
      }
      if (p.allFinite()) {
        ++stats.finite;
        consume(p);
      }
    }
  return stats;
}

inline Points spatialSample(const Points &points, double cell) {
  std::map<std::array<int64_t, 3>, Vec> bins;
  for (const auto &p : points) {
    if (!p.allFinite() || (p.array().abs() > 1e7).any()) continue;
    std::array<int64_t, 3> key;
    for (int i = 0; i < 3; ++i) key[i] = int64_t(std::floor(p[i] / cell));
    // Keep a real measurement endpoint, rather than inventing an averaged ray.
    auto it = bins.find(key);
    if (it == bins.end() || p.squaredNorm() < it->second.squaredNorm()) bins[key] = p;
  }
  Points result;
  result.reserve(bins.size());
  for (const auto &entry : bins) result.push_back(entry.second);
  return result;
}

inline sensor_msgs::msg::PointCloud2 xyzMessage(const Points &points,
                                                const std_msgs::msg::Header &header) {
  sensor_msgs::msg::PointCloud2 m;
  m.header = header;
  m.height = 1;
  m.width = points.size();
  m.point_step = 12;
  m.row_step = m.width * 12;
  m.is_dense = true;
  const uint16_t endian = 1;
  m.is_bigendian = *reinterpret_cast<const uint8_t *>(&endian) == 0;
  for (uint32_t i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = std::string(1, "xyz"[i]);
    f.offset = i * 4;
    f.datatype = 7;
    f.count = 1;
    m.fields.push_back(f);
  }
  m.data.resize(m.row_step);
  for (size_t i = 0; i < points.size(); ++i)
    for (int a = 0; a < 3; ++a) {
      float value = points[i][a];
      std::memcpy(m.data.data() + 12 * i + 4 * a, &value, 4);
    }
  return m;
}
}  // namespace uav_autonomy
