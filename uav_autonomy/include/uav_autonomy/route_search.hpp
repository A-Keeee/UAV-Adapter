#pragma once
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "uav_autonomy/observed_map.hpp"
#include "uav_autonomy/exploration_evidence.hpp"

namespace uav_autonomy {
struct RouteResult {
  Points path;
  size_t clusters = 0;
};

inline RouteResult searchRouteOnLattice(std::shared_ptr<const ObservedMap> map, const Vec &position,
                               const Vec &target, bool frontier,
                               const std::unordered_set<Key, KeyHash> &visited, double radius,
                               const std::string &, double floor, double ceiling,
                               const Points &audited_views, double step) {
  // A* over observed, inflated free cells. The separate EGO backend retains
  // responsibility for continuous trajectory geometry and optimisation.

  // Keep lattice vertices fixed in the world across frontier selection and
  // connection rechecks. The virtual root is the actual state, never an
  // unchecked snap. Every root connector uses the same swept-body/clearance
  // sampling as an ordinary edge.
  const Key start{INT_MAX, INT_MAX, INT_MAX};
  auto p = [&](const Key &x) -> Vec {
    return x == start ? position : Vec(x[0], x[1], x[2]) * step;
  };
  auto edgeSafe = [&](const Vec &from, const Vec &to) {
    int steps = std::max(1, int(std::ceil((to - from).norm() / .05)));
    for (int i = 0; i <= steps; ++i)
      if (!map->safe(from + (to - from) * (double(i) / steps), radius)) return false;
    return true;
  };
  auto frontiers = frontier ? map->frontiers() : std::vector<Frontier>();
  if (!edgeSafe(position, position)) return {{}, frontiers.size()};
  if (frontier) {
    if (frontiers.empty()) return {};
  }
  struct Queue {
    double f;
    Key key;
    bool operator<(const Queue &b) const { return f > b.f; }
  };
  std::priority_queue<Queue> open;
  open.push({0, start});
  std::unordered_map<Key, double, KeyHash> distance;
  distance[start] = 0;
  std::unordered_map<Key, Key, KeyHash> parent;
  std::unordered_set<Key, KeyHash> closed;
  std::unordered_map<Key, bool, KeyHash> safe_nodes;
  Key end = start;
  double best = -1;
  const std::array<Key, 6> neighbours{
      {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};
  size_t expanded = 0;
  while (!open.empty() && expanded < 30000) {
    Key cur = open.top().key;
    open.pop();
    if (!closed.insert(cur).second) continue;
    ++expanded;
    Vec pos = p(cur);
    if (!frontier && (pos - target).norm() < .5 && edgeSafe(pos, target)) {
      end = cur;
      best = 1;
      break;
    }
    // A stationary observation target needs the same one-voxel reserve as
    // generated patrol targets. Safe traversal alone does not make a robust
    // place to stop while later scans refine a shelf surface.
    if (frontier && (pos - position).norm() > .6 &&
        map->safe(pos, radius + map->resolution())) {
      double gain = 0;
      for (const auto &f : frontiers) {
        double distance_to_cluster = (pos.cwiseMax(f.low).cwiseMin(f.high) - pos).norm();
        bool visible = true;
        if (distance_to_cluster < 1.8) {
          int samples = std::max(1, int(std::ceil((f.centroid - pos).norm() / .2)));
          for (int i = 1; i <= samples; ++i)
            if (map->status(pos + (f.centroid - pos) * (double(i) / samples)) == 1) {
              visible = false;
              break;
            }
        }
        if (distance_to_cluster < 1.8 && visible)
          gain += std::min(size_t(40), f.cells) *
                  std::exp(-distance_to_cluster * distance_to_cluster);
      }
      Key cluster{int(std::floor(pos.x() / 1.2)), int(std::floor(pos.y() / 1.2)),
                  int(std::floor(pos.z() / 1.2))};
      double score = (visited.count(cluster) || auditedObservation(pos,audited_views)) ? -1 : gain / (1 + .3 * distance[cur]);
      if (score > best && gain > 1) {
        best = score;
        end = cur;
      }
    }
    std::vector<Key> adjacent;
    if (cur == start) {
      Key base{int(std::floor(position.x() / step)), int(std::floor(position.y() / step)),
               int(std::floor(position.z() / step))};
      for (int x = -1; x <= 2; ++x)
        for (int y = -1; y <= 2; ++y)
          for (int z = -1; z <= 2; ++z)
            adjacent.push_back({base[0] + x, base[1] + y, base[2] + z});
    } else {
      for (const auto &d : neighbours)
        adjacent.push_back({cur[0] + d[0], cur[1] + d[1], cur[2] + d[2]});
    }
    for (const auto &next : adjacent) {
      if (closed.count(next)) continue;
      Vec np = p(next);
      if (np.z() < floor + .1 || np.z() > ceiling - .1)
        continue;
      double g = distance[cur] + (np - pos).norm();
      if (distance.count(next) && distance[next] <= g) continue;
      auto safe = safe_nodes.find(next);
      if (safe == safe_nodes.end()) safe = safe_nodes.emplace(next, map->safe(np, radius)).first;
      if (!safe->second) continue;
      if (!edgeSafe(pos, np)) continue;
      distance[next] = g;
      parent[next] = cur;
      double h = frontier ? 0 : (np - target).norm();
      open.push({g + h, next});
    }
  }
  if (best <= 0) return {{}, frontiers.size()};
  Points result{p(end)};
  while (end != start) {
    auto it = parent.find(end);
    if (it == parent.end()) return {};
    end = it->second;
    result.push_back(p(end));
  }
  std::reverse(result.begin(), result.end());
  if (!frontier && (result.back() - target).norm() > 1e-6) result.push_back(target);
  return {std::move(result), frontiers.size()};
}
// Refine only after the bounded coarse search finds no connection. This
// changes search sampling, never map resolution, obstacle inflation or what
// counts as observed free space. Both passes retain the 30,000-node bound.
inline RouteResult searchRoute(std::shared_ptr<const ObservedMap> map, const Vec &position,
                               const Vec &target, bool frontier,
                               const std::unordered_set<Key, KeyHash> &visited, double radius,
                               const std::string &task, double floor = .8, double ceiling = 8.2,
                               const Points &audited_views = {}) {
  auto result = searchRouteOnLattice(map, position, target, frontier, visited, radius,
                                    task, floor, ceiling, audited_views, .4);
  if (!result.path.empty()) return result;
  return searchRouteOnLattice(map, position, target, frontier, visited, radius,
                              task, floor, ceiling, audited_views, .2);
}
}  // namespace uav_autonomy
