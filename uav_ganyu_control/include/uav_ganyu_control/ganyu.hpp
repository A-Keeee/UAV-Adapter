#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uav_ganyu_control {
using Vec3 = Eigen::Vector3d;

struct State {
  Vec3 p = Vec3::Zero();
  Vec3 v = Vec3::Zero();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity(); // FRD body -> NED world
};
struct Reference {
  Vec3 p = Vec3::Zero();
  Vec3 v = Vec3::Zero();
  Vec3 a = Vec3::Zero();
  Vec3 j = Vec3::Zero();
};
struct Parameters {
  double mass = 2.125307692307692; // x500_base (all links) + OakD-Lite
  double gravity = 9.81;
  Vec3 kp = Vec3::Constant(9.75);
  Vec3 kv = Vec3::Constant(3.6);
  Vec3 ki = Vec3::Constant(0.5);
  double kr = 120.0;
  double hr = 30.0;
  double integral_limit = 5.0;
  double max_thrust = 30.0;
};
struct Command {
  double thrust = 0.0; // N, positive collective thrust along body -Z
  Vec3 omega = Vec3::Zero(); // rad/s, body FRD
};

inline Eigen::Matrix3d skew(const Vec3 &v) {
  Eigen::Matrix3d s;
  s << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return s;
}

// Port of sz_indoor_transport, commit 214d185, custom/quadrotor_ganyu.h.
// The translational / thrust-direction equations, including dFd, are retained.
// ROS time and old-airframe polynomial calibration are outside this numerical core.
class QuadrotorControllerGanYu {
public:
  explicit QuadrotorControllerGanYu(Parameters parameters = {}) : params(parameters) {}
  Parameters params;
  Vec3 integral = Vec3::Zero();
  void resetIntegral() { integral.setZero(); }

  Command update(const State &state, const Reference &ref, double dt, double yaw_reference) {
    if (!state.p.allFinite() || !state.v.allFinite() || !state.R.allFinite() ||
        !ref.p.allFinite() || !ref.v.allFinite() || !ref.a.allFinite() || !ref.j.allFinite() ||
        !std::isfinite(dt) || !std::isfinite(yaw_reference) || dt <= 0 || params.mass <= 0 || params.hr <= 0) {
      throw std::runtime_error("Invalid GanYu input");
    }
    dt = std::clamp(dt, 0.0001, 0.05);
    const Vec3 e3(0, 0, 1);
    const Vec3 zp = state.p - ref.p;
    const Vec3 zv = state.v - ref.v;
    integral += params.ki.cwiseProduct(zp) * dt;
    const double limit = std::max(0.0, params.integral_limit);
    integral = integral.cwiseMax(-limit).cwiseMin(limit);
    const Vec3 u = -params.kp.cwiseProduct(zp) - params.kv.cwiseProduct(zv) - integral;
    const Vec3 Fd = params.mass * (u - params.gravity * e3 + ref.a);
    const double Td = std::max(Fd.norm(), 1e-6);
    const Vec3 r3d = -Fd / Td;
    const Vec3 r3 = state.R * e3;
    const double thrust = Td * r3d.dot(r3);
    const Vec3 F = -thrust * state.R * e3;
    const Vec3 dzv = F / params.mass + params.gravity * e3 - ref.a;
    const Vec3 dFd = params.mass * (-params.kp.cwiseProduct(zv) - params.kv.cwiseProduct(dzv) + ref.j);
    const Vec3 dr3d = skew(r3d) * skew(r3d) * dFd / Td;
    const double yaw = std::atan2(state.R(1, 0), state.R(0, 0));
    // Same yaw P law as the source; target is the heading captured on takeoff.
    const double yaw_error = std::remainder(yaw - yaw_reference, 2.0 * std::acos(-1.0));
    Command out;
    out.omega = -skew(e3) * skew(e3) * (
        state.R.transpose() * skew(r3d) * dr3d
        + params.kr / params.hr * skew(e3) * state.R.transpose() * r3d
        + Td / (params.mass * params.hr) * skew(e3) * state.R.transpose() * (0.5 * zp + zv))
        - 0.1 * yaw_error * e3;
    out.thrust = std::clamp(thrust, 0.0, params.max_thrust);
    if (!std::isfinite(out.thrust) || !out.omega.allFinite()) throw std::runtime_error("Non-finite GanYu output");
    return out;
  }
};

// X500: rotor speed = speed_min + relative_signal * (speed_max - speed_min).
// PX4 inverts THR_MDL_FAC before mapping its motor output to this speed.
// This collective mapping is exact for equal steady rotor speeds, and approximate
// during attitude corrections; it never reuses the original 207 g vehicle's fit.
inline double normalizedThrust(double thrust, double motor_constant, double speed_min,
                               double speed_max, double thrust_model_factor) {
  const double speed = std::sqrt(std::max(thrust, 0.0) / (4.0 * motor_constant));
  const double signal = std::clamp((speed - speed_min) / (speed_max - speed_min), 0.0, 1.0);
  return thrust_model_factor * signal * signal + (1.0 - thrust_model_factor) * signal;
}

// Seventh-order interpolation preserves p, v, a and jerk when a held key retargets.
class SmoothReference {
public:
  void reset(const Vec3 &p, double time) {
    coefficients_.setZero(); coefficients_.row(0) = p.transpose();
    goal_ = p; start_ = time; duration_ = 0;
  }
  Reference sample(double time) const {
    Reference out;
    if (duration_ <= 0 || time >= start_ + duration_) { out.p = goal_; return out; }
    const double s = std::clamp((time - start_) / duration_, 0.0, 1.0);
    for (int n = 0; n < 8; ++n) {
      const Vec3 c = coefficients_.row(n).transpose();
      out.p += c * std::pow(s, n);
      if (n >= 1) out.v += c * n * std::pow(s, n-1) / duration_;
      if (n >= 2) out.a += c * n*(n-1) * std::pow(s, n-2) / std::pow(duration_, 2);
      if (n >= 3) out.j += c * n*(n-1)*(n-2) * std::pow(s, n-3) / std::pow(duration_, 3);
    }
    return out;
  }
  void moveTo(const Vec3 &goal, double time, double duration) {
    const Reference from = sample(time);
    moveFrom(from, goal, time, duration);
  }
  void moveFrom(const Reference &from, const Vec3 &goal, double time, double duration) {
    duration_ = std::max(1.5, duration); start_ = time; goal_ = goal;
    coefficients_.row(0) = from.p.transpose();
    coefficients_.row(1) = (from.v * duration_).transpose();
    coefficients_.row(2) = (from.a * duration_*duration_/2).transpose();
    coefficients_.row(3) = (from.j * std::pow(duration_, 3)/6).transpose();
    Eigen::Matrix4d matrix;
    matrix << 1,1,1,1, 4,5,6,7, 12,20,30,42, 24,60,120,210;
    Eigen::Matrix<double,4,3> rhs;
    rhs.row(0) = goal.transpose() - coefficients_.topRows<4>().colwise().sum();
    rhs.row(1) = -coefficients_.row(1) - 2*coefficients_.row(2) - 3*coefficients_.row(3);
    rhs.row(2) = -2*coefficients_.row(2) - 6*coefficients_.row(3);
    rhs.row(3) = -6*coefficients_.row(3);
    coefficients_.bottomRows<4>() = matrix.partialPivLu().solve(rhs);
  }
  Vec3 goal() const { return goal_; }
private:
  Eigen::Matrix<double,8,3> coefficients_ = Eigen::Matrix<double,8,3>::Zero();
  Vec3 goal_ = Vec3::Zero();
  double start_ = 0, duration_ = 0;
};
} // namespace uav_ganyu_control
