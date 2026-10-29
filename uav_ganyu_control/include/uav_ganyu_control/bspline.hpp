#pragma once
#include "uav_ganyu_control/ganyu.hpp"
#include <vector>

namespace uav_ganyu_control {
// Non-rational clamped B-spline with analytic velocity, acceleration and jerk.
// Derivative control polygons also give conservative dynamic bounds.
class BSplineReference {
public:
  void set(int degree, const std::vector<double> &knots, const std::vector<Vec3> &points) {
    if (degree < 3 || degree > 7 || points.size() < static_cast<size_t>(degree+1) ||
        points.size() > 20000 || knots.size() != points.size()+degree+1)
      throw std::runtime_error("Invalid B-spline dimensions");
    for (const auto &p : points) if (!p.allFinite()) throw std::runtime_error("Non-finite control point");
    for (size_t i=0; i<knots.size(); ++i)
      if (!std::isfinite(knots[i]) || (i && knots[i] < knots[i-1])) throw std::runtime_error("Invalid knots");
    const double end = knots[points.size()];
    if (knots.front() != 0 || end <= 0 || end > 3600) throw std::runtime_error("Invalid B-spline duration");
    for (int i=0; i<=degree; ++i)
      if (knots[i] != 0 || knots[knots.size()-1-i] != end) throw std::runtime_error("Spline must be clamped");
    for (size_t i=degree+1; i<points.size(); ++i)
      if (knots[i] <= knots[i-1]) throw std::runtime_error("Interior knots must be simple");
    degree_ = degree; knots_[0] = knots; points_[0] = points; duration_ = end;
    for (int order=1; order<=3; ++order) {
      const auto &k = knots_[order-1]; const auto &p = points_[order-1];
      knots_[order] = std::vector<double>(k.begin()+1, k.end()-1);
      points_[order].clear();
      for (size_t i=0; i+1<p.size(); ++i) {
        const double denominator = k[i+degree-order+2]-k[i+1];
        if (denominator <= 0) throw std::runtime_error("Degenerate derivative knots");
        points_[order].push_back((degree-order+1)*(p[i+1]-p[i])/denominator);
      }
    }
  }
  double duration() const { return duration_; }
  Vec3 goal() const { return points_[0].back(); }
  double derivativeBound(int order) const {
    double result = 0;
    for (const auto &p : points_[order]) result = std::max(result, p.norm());
    return result;
  }
  Reference sample(double elapsed) const {
    Reference r;
    const double t = std::clamp(elapsed, 0.0, duration_);
    r.p = evaluate(0, t);
    // A held trajectory has zero feed-forward after its endpoint.
    if (elapsed >= duration_) return r;
    r.v = evaluate(1, t); r.a = evaluate(2, t); r.j = evaluate(3, t);
    return r;
  }
private:
  Vec3 evaluate(int order, double t) const {
    const int degree = degree_-order;
    const auto &k = knots_[order]; const auto &p = points_[order];
    int span = static_cast<int>(p.size())-1;
    if (t < duration_) span = static_cast<int>(std::upper_bound(k.begin(), k.end(), t)-k.begin())-1;
    span = std::clamp(span, degree, static_cast<int>(p.size())-1);
    std::vector<Vec3> d(p.begin()+span-degree, p.begin()+span+1);
    for (int r=1; r<=degree; ++r) for (int j=degree; j>=r; --j) {
      const int i = span-degree+j;
      const double denominator = k[i+degree-r+1]-k[i];
      const double alpha = denominator > 0 ? (t-k[i])/denominator : 0;
      d[j] = (1-alpha)*d[j-1]+alpha*d[j];
    }
    return d[degree];
  }
  int degree_ = 7;
  double duration_ = 0;
  std::vector<double> knots_[4];
  std::vector<Vec3> points_[4];
};
}  // namespace uav_ganyu_control
