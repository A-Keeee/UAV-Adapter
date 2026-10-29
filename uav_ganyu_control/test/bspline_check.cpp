#include "uav_ganyu_control/bspline.hpp"
#include <iostream>

using namespace uav_ganyu_control;
int main() {
  const Vec3 target(2,-3,1);
  std::vector<Vec3> points(4, Vec3::Zero());
  points.insert(points.end(), 4, target);
  std::vector<double> knots(8, 0); knots.insert(knots.end(), 8, 2);
  BSplineReference spline;
  spline.set(7, knots, points);
  for (double s : {0.0, 0.03, 0.2, 0.49, 0.8, 0.99}) {
    const auto r = spline.sample(2*s);
    const double p = 35*std::pow(s,4)-84*std::pow(s,5)+70*std::pow(s,6)-20*std::pow(s,7);
    const double v = (140*std::pow(s,3)-420*std::pow(s,4)+420*std::pow(s,5)-140*std::pow(s,6))/2;
    const double a = (420*s*s-1680*std::pow(s,3)+2100*std::pow(s,4)-840*std::pow(s,5))/4;
    const double j = (840*s-5040*s*s+8400*std::pow(s,3)-4200*std::pow(s,4))/8;
    if ((r.p-target*p).norm() > 1e-9 || (r.v-target*v).norm() > 1e-9 ||
        (r.a-target*a).norm() > 1e-9 || (r.j-target*j).norm() > 1e-9) return 1;
  }
  auto end = spline.sample(3);
  if ((end.p-target).norm() > 1e-12 || end.v.norm() || end.a.norm() || end.j.norm()) return 2;
  for (const auto &bad : {std::vector<double>{}, std::vector<double>(16, 0)}) {
    try { spline.set(7, bad, points); return 3; } catch (const std::runtime_error &) {}
  }
  std::cout << "B-spline analytic p/v/a/jerk, endpoint hold and malformed input checks passed\n";
  return 0;
}
