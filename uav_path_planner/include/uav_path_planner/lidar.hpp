#pragma once
#include "uav_path_planner/core.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <map>

namespace uav_path_planner {
Points cloudXYZ(const sensor_msgs::msg::PointCloud2 &msg);
bool unmodeled(const Vec3 &point, const World &world, double tolerance=.1);
struct WallAlignment {
  bool valid=false;
  double yaw=0, rms=0;
  Vec3 translation=Vec3::Zero();
  size_t inliers=0;
};
// Register only against already-known vertical SDF faces; never add map geometry.
WallAlignment alignKnownWalls(const Points &points,const World &world,const Vec3 &pivot);
Points applyAlignment(const Points &points,const Vec3 &pivot,const WallAlignment &alignment);
Plan planLocal(const World &world,const Vec3 &start,const Vec3 &goal,
               const std::array<double,3> &limits={1.0,.7,1.5},double margin=.2);
class LocalObstacles {
public:
  LocalObstacles(double voxel_size=.3, double memory=.6, double margin=.15, size_t max_voxels=6000);
  void update(const Points &points, double stamp);
  void expire(double now);
  void clear() { cells_.clear(); }
  World world(const World &static_world, double now, const Vec3 &origin);
  Points centers(double now);
  double lastStamp() const { return last_stamp_; }
  double voxelSize() const { return voxel_size_; }
  size_t size() const { return cells_.size(); }
private:
  using Cell=std::array<int,3>;
  double voxel_size_, memory_, margin_, last_stamp_=-std::numeric_limits<double>::infinity();
  size_t max_voxels_;
  std::map<Cell,double> cells_;
};
}  // namespace uav_path_planner
