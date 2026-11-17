#pragma once

#include "uav_autonomy/observed_map.hpp"
#include "uav_autonomy/viewing_move.hpp"

namespace uav_autonomy {
// Select a small, separately guarded observation move, independent of EGO's
// native mission trajectory. The returned path ends at the executable target.
inline Points observationViewpoint(std::shared_ptr<const ObservedMap> map,
    const Vec &position, const Vec &hold, const Vec &goal,
    const std::unordered_set<Key, KeyHash> &visited, double radius,
    double floor, double ceiling, double speed, bool prefer_robust_stop = false) {
  Vec direction = goal - position;
  direction.z() = 0;
  if (direction.norm() > .01) direction.normalize();
  else direction = Vec::UnitX();
  std::vector<std::pair<double, Vec>> candidates;
  for (double x = -.8; x <= .81; x += .4)
    for (double y = -.8; y <= .81; y += .4)
      for (double z = -.4; z <= .41; z += .2) {
        Vec candidate = position + Vec(x, y, z);
        if ((candidate - position).norm() < .15 || candidate.z() < floor + .1 ||
            candidate.z() > ceiling - .1 || !map->stoppingSafe(candidate, radius)) continue;
        bool repeated = visited.count(map->key(candidate));
        for (const auto &k : visited)
          if ((candidate - map->point(k)).norm() < .15) { repeated = true; break; }
        if (repeated) continue;
        // Indoor recovery should leave a fragile shelf-side stop, rather
        // than choose another barely safe stop farther into the same pocket.
        // A reserve is a preference: all candidates retain the original full
        // measured curve and braking checks, including fallback candidates.
        const double robust = prefer_robust_stop && map->safe(candidate, radius + map->resolution()) ? 4.0 : 0.0;
        double score = robust + (candidate - position).dot(direction) + .4 * std::abs(z) - .2 * std::abs(y);
        candidates.emplace_back(score, candidate);
      }
  std::sort(candidates.begin(), candidates.end(),
            [](const auto &a, const auto &b) { return a.first > b.first; });
  for (const auto &candidate : candidates) {
    // Score, deduplicate and execute the SAME point. The former A* route's
    // first edge could end at an old observation stop even when its distant
    // endpoint was new, repeatedly moving back into the same unsupported view.
    // Check the complete direct curve and its stopping tubes to the selected
    // point. No inferred FOV or unmeasured free space is used for acceptance.
    const Vec target = candidate.second;
    if (viewingMoveSafe(hold, target, speed, [&](const Vec &p) {
          return p.z() >= floor && p.z() <= ceiling && map->safe(p, radius);
        })) return {position, target};
  }
  return {};
}
}  // namespace uav_autonomy
