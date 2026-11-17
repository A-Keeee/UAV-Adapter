#pragma once
#include "uav_autonomy/cloud.hpp"

namespace uav_autonomy {
inline bool auditedObservation(const Vec &position, const Points &views) {
  for (const auto &view : views)
    if ((position-view).norm()<1.5) return true;
  return false;
}

// Evidence of spatially distinct, low-information observation windows. This
// never certifies map completeness; that remains an independent measurement.
class ExplorationEvidence {
 public:
  bool windowGain(size_t recent_gain, size_t threshold) {
    // Enter low-gain at threshold, leave only at twice that rate. Sparse
    // non-repeating scans fluctuate around the entry threshold; clearing the
    // evidence on each small crossing repeatedly sends us to the same views.
    if (recent_gain/2<threshold || views_.empty()) return false;
    views_.clear();first_known_=0;return true;
  }
  bool recordView(const Vec &position, size_t known) {
    if (auditedObservation(position,views_)) return false;
    if (views_.empty()) first_known_=known;
    views_.push_back(position);return true;
  }
  const Points &views() const {return views_;}
  size_t firstKnown() const {return first_known_;}
  bool exhausted(double window_extent) const {return window_extent>=3. || views_.size()>=3;}
  bool continueConnection(double target_distance, double window_extent) const {
    // Give a progressing, observed connection time to leave an old view.
    // A stationary planner still gets a different frontier, not an endless wait.
    return target_distance>.35 && window_extent>.25;
  }
 private:
  Points views_;
  size_t first_known_=0;
};
}  // namespace uav_autonomy
