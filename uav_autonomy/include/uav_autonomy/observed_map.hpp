#pragma once
#include <octomap/octomap.h>

#include <climits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "uav_autonomy/cloud.hpp"

namespace uav_autonomy {
using Key = std::array<int, 3>;
inline Vec observedBodyHalfAxes() { return Vec(.50, .50, .35); }
// Offline collision/rotor checks bound the physical X500 by 0.905 of this
// ellipsoid. Account for attitude and translation before using a supported
// stationary reference as a search origin. Horizontal axes are equal, so the
// exact operator norm depends only on tilt, not yaw.
inline double holdEnclosureRadius(const Vec &offset, const Eigen::Matrix3d &rotation) {
  const Vec axes = observedBodyHalfAxes();
  const double sine = std::sqrt(std::max(0.0, 1.0 - rotation(2, 2) * rotation(2, 2)));
  const double k = (axes.x() / axes.z() - axes.z() / axes.x()) * sine;
  const double rotation_norm = .5 * (std::sqrt(4 + k * k) + std::abs(k));
  return .905 * rotation_norm + offset.cwiseQuotient(axes).norm();
}
struct KeyHash {
  size_t operator()(const Key &k) const {
    return size_t(uint32_t(k[0]) * 73856093u) ^ size_t(uint32_t(k[1]) * 19349663u) ^
           size_t(uint32_t(k[2]) * 83492791u);
  }
};
struct Frontier {
  Vec low, high, centroid;
  size_t cells = 0;
};
struct ClearanceEvidence {
  size_t free = 0, occupied = 0, unknown = 0;
  size_t unknown_body = 0, unknown_halo = 0;
  Vec nearest_unknown = Vec::Constant(std::numeric_limits<double>::quiet_NaN());
  Vec nearest_obstacle = Vec::Constant(std::numeric_limits<double>::quiet_NaN());
};
class ObservedMap {
 public:
  ObservedMap(bool rolling, Vec size, double resolution)
      : rolling_(rolling), size_(size), r_(resolution), tree_(resolution) {
    for (int a = 0; a < 3; ++a) dim_[a] = int(std::ceil(size[a] / r_));
    cells_.resize(size_t(dim_[0]) * dim_[1] * dim_[2]);
    center_ = Vec(0, 0, size_.z() / 2);
  }
  void move(const Vec &p) {
    if (!rolling_) return;
    center_ = p;
    for (auto &cell : cells_)
      if (cell.state >= 0 && !inside(point(cell.key))) cell = Cell{};
  }
  bool inside(const Vec &p, double margin = 0) const {
    return ((p - center_).array().abs() < size_.array() / 2 - margin).all();
  }
  Key key(const Vec &p) const {
    return {int(std::floor(p.x() / r_)), int(std::floor(p.y() / r_)), int(std::floor(p.z() / r_))};
  }
  Vec point(const Key &k) const { return Vec(k[0] + .5, k[1] + .5, k[2] + .5) * r_; }
  int status(const Vec &p) const {
    if (!inside(p)) return -1;
    Key k = key(p);
    const auto &c = cells_[index(k)];
    return c.key == k ? c.state : -1;
  }
  void update(const Points &points, const Vec &origin) {
    std::unordered_set<Key, KeyHash> free, occupied;
    for (const Vec &endpoint : points) {
      Vec d = endpoint - origin;
      double alpha = 1;
      for (int a = 0; a < 3; ++a)
        if (std::abs(d[a]) > 1e-9) {
          double edge = center_[a] + std::copysign(size_[a] / 2 - r_ * .02, d[a]);
          alpha = std::min(alpha, (edge - origin[a]) / d[a]);
        }
      if (alpha <= 0) continue;
      Vec end = origin + std::min(1.0, alpha) * d;
      octomap::KeyRay ray;
      if (tree_.computeRayKeys(octomap::point3d(origin.x(), origin.y(), origin.z()),
                               octomap::point3d(end.x(), end.y(), end.z()), ray)) {
        for (const auto &rk : ray) {
          auto v = tree_.keyToCoord(rk);
          Vec p(v.x(), v.y(), v.z());
          if (inside(p)) free.insert(key(p));
        }
      }
      if (alpha >= 1 && inside(endpoint)) occupied.insert(key(endpoint));
      // Clipped endpoints are free traversal, never synthetic obstacles.
      else if (inside(end))
        free.insert(key(end));
    }
    for (const auto &k : free)
      if (!occupied.count(k)) set(k, 0);
    for (const auto &k : occupied) set(k, 1);
    if (!rolling_) tree_.updateInnerOccupancy();
  }
  bool safe(const Vec &p, double radius, bool require_observed = true) const {
    if (!inside(p, radius)) return false;
    // X500 swept ellipsoid including tracking allowance (not sensor FOV).
    // Occupied cells must respect the full obstacle clearance. Unknown cells
    // must not intersect the swept airframe; the surrounding clearance halo
    // stays unknown and is never inserted as free space.
    const Vec body_half_extent = observedBodyHalfAxes();
    Key center = key(p);
    int n = int(std::ceil((radius + r_ * .866) / r_));
    for (int x = -n; x <= n; ++x)
      for (int y = -n; y <= n; ++y)
        for (int z = -n; z <= n; ++z) {
          Vec c = point({center[0] + x, center[1] + y, center[2] + z});
          bool obstacle_overlap = (c - p).norm() <= radius + r_ * .866;
          Vec nearest = ((c - p).array().abs() - r_ * .5).max(0.0).matrix();
          bool body_overlap = (nearest.array() / body_half_extent.array()).square().sum() <= 1;
          if (!obstacle_overlap && !body_overlap) continue;
          int s = status(c);
          if ((s == 1 && obstacle_overlap) || (require_observed && s < 0 && body_overlap))
            return false;
        }
    return true;
  }
  // Additional measured-obstacle reserve for complete forest stopping tubes.
  // Keep the same observed airframe test; no unknown cells are marked free.
  bool stoppingSafe(const Vec &p, double radius) const {
    return safe(p, radius + (rolling_ ? r_ : 0.0));
  }
  ClearanceEvidence evidence(const Vec &p, double radius) const {
    ClearanceEvidence result;
    Key center = key(p);
    int n = int(std::ceil((radius + r_ * .866) / r_));
    double nearest_unknown = INFINITY, nearest_obstacle = INFINITY;
    for (int x = -n; x <= n; ++x)
      for (int y = -n; y <= n; ++y)
        for (int z = -n; z <= n; ++z) {
          Vec c = point({center[0] + x, center[1] + y, center[2] + z});
          double distance = (c - p).norm();
          Vec nearest = ((c - p).array().abs() - r_ * .5).max(0.0).matrix();
          bool body_overlap = (nearest.array() / observedBodyHalfAxes().array()).square().sum() <= 1;
          if (distance > radius + r_ * .866 && !body_overlap) continue;
          int s = status(c);
          if (s < 0) {
            ++result.unknown;
            if (body_overlap)
              ++result.unknown_body;
            else
              ++result.unknown_halo;
            if (distance < nearest_unknown) {
              nearest_unknown = distance;
              result.nearest_unknown = c;
            }
          } else if (s == 1) {
            ++result.occupied;
            if (distance < nearest_obstacle) {
              nearest_obstacle = distance;
              result.nearest_obstacle = c;
            }
          } else
            ++result.free;
        }
    return result;
  }
  Points occupied() const { return matching(1); }
  Points free() const { return matching(0); }
  std::vector<Frontier> frontiers() const {
    const std::array<Key, 6> adjacent{
        {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};
    std::unordered_set<Key, KeyHash> boundary;
    for (const auto &cell : cells_) {
      if (cell.state != 0) continue;
      Vec p = point(cell.key);
      if (!inside(p) || p.z() < .8 || p.z() > 8.2) continue;
      for (const auto &d : adjacent) {
        Vec next = point({cell.key[0] + d[0], cell.key[1] + d[1], cell.key[2] + d[2]});
        if (inside(next) && status(next) < 0) {
          boundary.insert(cell.key);
          break;
        }
      }
    }
    std::vector<Frontier> result;
    while (!boundary.empty()) {
      Key first = *boundary.begin();
      boundary.erase(first);
      std::queue<Key> todo;
      todo.push(first);
      Frontier f;
      f.low = f.high = point(first);
      f.centroid = Vec::Zero();
      Points members;
      while (!todo.empty()) {
        Key cur = todo.front();
        todo.pop();
        Vec p = point(cur);
        members.push_back(p);
        ++f.cells;
        f.centroid += p;
        f.low = f.low.cwiseMin(p);
        f.high = f.high.cwiseMax(p);
        for (const auto &d : adjacent) {
          Key k{cur[0] + d[0], cur[1] + d[1], cur[2] + d[2]};
          auto it = boundary.find(k);
          if (it != boundary.end()) {
            todo.push(k);
            boundary.erase(it);
          }
        }
      }
      if (f.cells >= 12) {
        // A connected boundary can wrap around the whole room. Its bounding
        // box then contains observed interior far from any actual frontier.
        // Split that boundary spatially before scoring viewpoints, and use a
        // real free cell as representative instead of an unsupported mean.
        std::unordered_map<Key, Points, KeyHash> bins;
        for (const auto &p : members)
          bins[{int(std::floor(p.x() / 1.6)), int(std::floor(p.y() / 1.6)),
                int(std::floor(p.z() / 1.6))}].push_back(p);
        for (const auto &entry : bins) {
          const auto &points = entry.second;
          if (points.size() < 12) continue;
          Frontier part;
          part.cells = points.size();
          part.low = part.high = points.front();
          Vec mean = Vec::Zero();
          for (const auto &p : points) {
            part.low = part.low.cwiseMin(p);
            part.high = part.high.cwiseMax(p);
            mean += p;
          }
          mean /= points.size();
          part.centroid = *std::min_element(points.begin(), points.end(), [&](const Vec &a, const Vec &b) {
            return (a - mean).squaredNorm() < (b - mean).squaredNorm();
          });
          result.push_back(part);
        }
      }
    }
    return result;
  }
  size_t capacity() const { return cells_.size(); }
  size_t known() const {
    size_t n = 0;
    for (const auto &c : cells_)
      if (c.state >= 0 && inside(point(c.key))) ++n;
    return n;
  }
  bool save(const std::string &path) const {
    if (rolling_) return false;
    // writeBinary prunes its receiver. Keep snapshots immutable while a mapping
    // worker copies the current tree.
    auto snapshot = tree_;
    return snapshot.writeBinary(path);
  }
  double resolution() const { return r_; }
  Vec center() const { return center_; }
  Vec size() const { return size_; }
  // Explicitly configured launch pad prior, not an inference about a lidar's FOV.
  void launchPad(const Vec &p, double radius, double height) {
    for (double x = -radius; x <= radius; x += r_)
      for (double y = -radius; y <= radius; y += r_)
        for (double z = .05; z <= height; z += r_)
          if (x * x + y * y < radius * radius) set(key(p + Vec(x, y, z)), 0);
  }

 private:
  struct Cell {
    Key key{INT_MIN, INT_MIN, INT_MIN};
    int8_t state = -1;
  };
  bool rolling_;
  Vec size_, center_;
  double r_;
  Key dim_;
  std::vector<Cell> cells_;
  octomap::OcTree tree_;
  size_t index(const Key &k) const {
    auto wrap = [](int a, int n) { return (a % n + n) % n; };
    return (size_t(wrap(k[0], dim_[0])) * dim_[1] + wrap(k[1], dim_[1])) * dim_[2] +
           wrap(k[2], dim_[2]);
  }
  void set(const Key &k, int8_t state) {
    // The static-forest PointCloud2 path in native EGO-v2 retains measured
    // occupancy until ring eviction (fading is disabled). A later ray through
    // another part of a hit voxel must not reopen the execution map alone:
    // sparse scans otherwise alternate free/occupied and admit a stopping
    // point from which neither the planner nor the viewing guard can depart.
    // Keep ray updates for unknown/free cells; move() still evicts old hits.
    if (rolling_ && state == 0) {
      const auto &existing = cells_[index(k)];
      if (existing.key == k && existing.state == 1) return;
    }
    if (!rolling_) {
      Vec p = point(k);
      auto *node = tree_.updateNode(octomap::point3d(p.x(), p.y(), p.z()), state == 1, true);
      state = tree_.isNodeOccupied(node) ? 1 : 0;
    }
    cells_[index(k)] = {k, state};
  }
  Points matching(int state) const {
    Points result;
    for (const auto &c : cells_)
      if (c.state == state && inside(point(c.key))) result.push_back(point(c.key));
    return result;
  }
};
}  // namespace uav_autonomy
