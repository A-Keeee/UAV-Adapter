#pragma once
#include <cmath>
#include <uav_common_msg/msg/native_trajectory.hpp>

#include "uav_autonomy/cloud.hpp"

namespace uav_autonomy {
struct Sample {
  Vec p = Vec::Zero(), v = Vec::Zero(), a = Vec::Zero(), j = Vec::Zero();
};
class NativeTrajectory {
 public:
  using Message = uav_common_msg::msg::NativeTrajectory;
  explicit NativeTrajectory(const Message &m) : msg(m) {
    if (m.header.frame_id != "odom")
      throw std::runtime_error("Native trajectory must use odom ENU");
    auto finite = [](const auto &xs) {
      return std::all_of(xs.begin(), xs.end(), [](double x) { return std::isfinite(x); });
    };
    if (!finite(m.knots) || !finite(m.control_points) || !finite(m.durations) ||
        !finite(m.coefficients))
      throw std::runtime_error("Non-finite trajectory");
    if (m.representation == Message::BSPLINE) {
      const size_t n = m.control_points.size() / 3;
      if (m.degree < 3 || m.degree > 7 || m.control_points.size() % 3 || n <= m.degree ||
          m.knots.size() != n + m.degree + 1 || !std::is_sorted(m.knots.begin(), m.knots.end()))
        throw std::runtime_error("Invalid native B-spline");
      duration_ = m.knots[n] - m.knots[m.degree];
      std::vector<Vec> p;
      for (size_t i = 0; i < n; ++i)
        p.emplace_back(m.control_points[3 * i], m.control_points[3 * i + 1],
                       m.control_points[3 * i + 2]);
      auto k = m.knots;
      for (int d = 0; d < 4; ++d) {
        controls_[d] = p;
        knots_[d] = k;
        if (d == 3) break;
        std::vector<Vec> next;
        int degree = m.degree - d;
        for (size_t i = 0; i + 1 < p.size(); ++i) {
          double span = k[i + degree + 1] - k[i + 1];
          next.push_back(span > 0 ? (degree / span * (p[i + 1] - p[i])).eval() : Vec::Zero());
        }
        p = std::move(next);
        k = std::vector<double>(k.begin() + 1, k.end() - 1);
      }
    } else if (m.representation == Message::POLYNOMIAL) {
      if (m.degree != 5 || m.durations.empty() || m.coefficients.size() != 18 * m.durations.size())
        throw std::runtime_error("Invalid EGO-v2 quintic");
      for (double d : m.durations) {
        if (d <= 0) throw std::runtime_error("Nonpositive segment");
        duration_ += d;
      }
    } else
      throw std::runtime_error("Unknown trajectory representation");
    if (!(duration_ > 0) || duration_ > 300)
      throw std::runtime_error("Invalid trajectory duration");
  }
  double duration() const { return duration_; }
  double start() const { return msg.start_time.sec + msg.start_time.nanosec * 1e-9; }
  Sample sample(double t) const {
    t = std::clamp(t, 0.0, duration_);
    Sample s;
    std::array<Vec *, 4> out{&s.p, &s.v, &s.a, &s.j};
    if (msg.representation == Message::BSPLINE) {
      for (int d = 0; d < 4; ++d)
        *out[d] = deBoor(controls_[d], knots_[d], msg.degree - d, t + msg.knots[msg.degree]);
    } else {
      size_t segment = 0;
      while (segment + 1 < msg.durations.size() && t > msg.durations[segment])
        t -= msg.durations[segment++];
      for (int d = 0; d < 4; ++d)
        for (int axis = 0; axis < 3; ++axis) {
          double value = 0;
          for (int power = 5; power >= d; --power) {
            double c = msg.coefficients[18 * segment + 6 * axis + 5 - power];
            for (int j = 0; j < d; ++j) c *= power - j;
            value = value * t + c;
          }
          (*out[d])[axis] = value;
        }
    }
    return s;
  }
  Message msg;

 private:
  double duration_ = 0;
  std::array<std::vector<Vec>, 4> controls_;
  std::array<std::vector<double>, 4> knots_;
  static Vec deBoor(const std::vector<Vec> &p, const std::vector<double> &u, int degree, double t) {
    int k = degree;
    while (k + 1 < int(p.size()) && t >= u[k + 1]) ++k;
    std::vector<Vec> d;
    for (int j = 0; j <= degree; ++j) d.push_back(p[k - degree + j]);
    for (int r = 1; r <= degree; ++r)
      for (int j = degree; j >= r; --j) {
        int i = k - degree + j;
        double span = u[i + degree - r + 1] - u[i];
        double a = span > 0 ? (t - u[i]) / span : 0;
        d[j] = (1 - a) * d[j - 1] + a * d[j];
      }
    return d[degree];
  }
};
}  // namespace uav_autonomy
