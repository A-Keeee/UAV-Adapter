#pragma once

#include <uav_ganyu_control/ganyu.hpp>

namespace uav_autonomy {

// Viewing moves are independent of EGO's native trajectory. Check their entire
// stopping tube before selecting a viewpoint, on the immutable map snapshot in
// the search worker. Runtime checks still use the newest measured map.
template <class Safe>
bool viewingMoveSafe(const uav_ganyu_control::Vec3 &from,
                     const uav_ganyu_control::Vec3 &goal, double speed, Safe safe) {
  using namespace uav_ganyu_control;
  SmoothReference move;
  move.reset(from, 0);
  const double duration = std::max(4.0, 2.2 * (goal - from).norm() / speed);
  move.moveTo(goal, 0, duration);
  try {
    for (double t = 0; t <= duration + .05; t += .05) {
      const auto r = move.sample(t);
      if (!safe(r.p) || r.v.norm() > speed * 1.01 || r.a.norm() > .7 || r.j.norm() > 1.5)
        return false;
      const auto brake = brakingReference(r, t);
      for (double dt = 0; dt < brake.second; dt += .08)
        if (!safe(brake.first.sample(t + dt).p)) return false;
      if (!safe(brake.first.goal())) return false;
    }
  } catch (const std::exception &) {
    return false;
  }
  return safe(goal);
}

}  // namespace uav_autonomy
