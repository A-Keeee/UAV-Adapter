#pragma once

#include <uav_ganyu_control/bspline.hpp>
#include <array>
#include <limits>
#include <string>
#include <vector>

namespace uav_path_planner {
using Vec3 = Eigen::Vector3d;
using Points = std::vector<Vec3>;
struct Box { std::string name; Vec3 low, high; };

class World {
public:
  World(std::vector<Box> boxes, Vec3 low, Vec3 high, double radius=.65, double resolution=.4);
  static World fromSdf(const std::string &file, Vec3 low, Vec3 high, double radius=.65, double resolution=.4);
  bool free(const Vec3 &p, double extra=0) const;
  bool segmentFree(const Vec3 &a, const Vec3 &b, double extra=0) const;
  bool hullFree(const Points &p) const;
  double clearance(const Vec3 &p) const;
  Points astar(const Vec3 &start, const Vec3 &goal) const;
  std::vector<Box> boxes;
  Vec3 low, high;
  double radius, resolution;
};

class Spline {
public:
  Spline() = default;
  Spline(Points points, std::vector<double> knots);
  Vec3 sample(double time, int order=0) const;
  double duration() const { return reference_.duration(); }
  std::array<double,3> bounds() const;
  bool certified(const World &world, double start=0,
                 double end=std::numeric_limits<double>::infinity()) const;
  Points points;
  std::vector<double> knots;
  static constexpr int degree = 7;
private:
  uav_ganyu_control::BSplineReference reference_;
};

struct Plan {
  Points raw;
  Spline spline;
  double astar_length=0, spline_length=0, minimum_clearance=0;
  double planning_resolution=0, local_margin=0;
};
Plan plan(const World &world, const Vec3 &start, const Vec3 &goal,
          const std::array<double,3> &limits={1.0,.7,1.5});
}  // namespace uav_path_planner
