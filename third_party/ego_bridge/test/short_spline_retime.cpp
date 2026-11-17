#include <bspline_opt/uniform_bspline.h>

#include <cmath>
#include <iostream>

int main() {
  constexpr double ratio = 1.9;
  for (int count : {6, 7}) {
    Eigen::MatrixXd points(3, count);
    for (int i = 0; i < count; ++i)
      points.col(i) = Eigen::Vector3d(.3 * i, .1 * i * i, 1 + .2 * std::cos(i));
    ego_planner::UniformBspline before(points, 3, .2), after = before;
    const double duration = before.getTimeSum();
    after.lengthenTime(ratio);
    if (std::abs(after.getTimeSum() - duration * ratio) > 1e-10) return 1;
    for (int derivative = 0; derivative <= 3; ++derivative) {
      for (int i = 0; i < 100; ++i) {
        // Avoid internal knots where jerk is discontinuous.
        double t = duration * (i + .12345) / 101;
        auto expected = before.evaluateDeBoorT(t);
        Eigen::VectorXd actual = after.evaluateDeBoorT(t * ratio) * std::pow(ratio, derivative);
        if (!actual.allFinite() || (expected - actual).norm() > 1e-8) return 2;
      }
      if (derivative < 3) {
        before = before.getDerivative();
        after = after.getDerivative();
      }
    }
  }
  std::cout << "Short B-spline retiming preserves geometry and scales p/v/a/j: PASS\n";
}
