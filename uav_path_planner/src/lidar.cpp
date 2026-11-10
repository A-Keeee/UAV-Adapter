#include "uav_path_planner/lidar.hpp"
#include <cstring>
#include <set>
#include <stdexcept>

namespace uav_path_planner {
Points cloudXYZ(const sensor_msgs::msg::PointCloud2 &msg) {
  if (!msg.width || !msg.height || !msg.point_step || uint64_t(msg.row_step)<uint64_t(msg.width)*msg.point_step
      || msg.data.size()<uint64_t(msg.row_step)*msg.height || uint64_t(msg.width)*msg.height>200000)
    throw std::runtime_error("Empty, truncated or oversized PointCloud2");
  std::array<sensor_msgs::msg::PointField,3> fields;
  const std::array<std::string,3> names{"x","y","z"};
  for (int i=0;i<3;++i) {
    const auto it=std::find_if(msg.fields.begin(),msg.fields.end(),[&](const auto &f){ return f.name==names[i]; });
    if (it==msg.fields.end() || (it->datatype!=7 && it->datatype!=8) || it->count!=1
        || uint64_t(it->offset)+(it->datatype==7?4:8)>msg.point_step)
      throw std::runtime_error("PointCloud2 requires scalar FLOAT32/FLOAT64 XYZ within point_step");
    fields[i]=*it;
  }
  const uint16_t probe=1; const bool host_big=*reinterpret_cast<const uint8_t*>(&probe)==0;
  Points result; result.reserve(uint64_t(msg.width)*msg.height);
  for (uint32_t row=0;row<msg.height;++row) for (uint32_t col=0;col<msg.width;++col) {
    Vec3 p;
    for (int i=0;i<3;++i) {
      const size_t size=fields[i].datatype==7?4:8;
      const auto *data=msg.data.data()+size_t(row)*msg.row_step+size_t(col)*msg.point_step+fields[i].offset;
      std::array<uint8_t,8> bytes{}; std::copy(data,data+size,bytes.begin());
      if (host_big!=msg.is_bigendian) std::reverse(bytes.begin(),bytes.begin()+size);
      if (size==4) { float value; std::memcpy(&value,bytes.data(),4); p[i]=value; }
      else std::memcpy(&p[i],bytes.data(),8);
    }
    result.push_back(p);
  }
  return result;
}
bool unmodeled(const Vec3 &p, const World &world, double tolerance) {
  for (const auto &box:world.boxes)
    if ((p.array()>=box.low.array()-tolerance).all() && (p.array()<=box.high.array()+tolerance).all()) return false;
  return true;
}
namespace {
struct FaceMatch { int axis=-1; double residual=0; };
FaceMatch matchFace(const Vec3 &p,const World &world) {
  FaceMatch result; double best=.45;
  for (const auto &box:world.boxes) {
    if (p.z()<box.low.z()+.1 || p.z()>box.high.z()-.1) continue;
    for (int axis=0;axis<2;++axis) {
      const int tangent=1-axis;
      if (p[tangent]<box.low[tangent]-.1 || p[tangent]>box.high[tangent]+.1) continue;
      for (double face:{box.low[axis],box.high[axis]}) {
        const double residual=p[axis]-face;
        if (std::abs(residual)<best) { best=std::abs(residual); result={axis,residual}; }
      }
    }
  }
  return result;
}
}
WallAlignment alignKnownWalls(const Points &points,const World &world,const Vec3 &pivot) {
  WallAlignment result;
  if (points.size()<80 || !pivot.allFinite()) return result;
  const size_t stride=std::max(size_t(1),points.size()/1800);
  double yaw=0; Vec3 translation=Vec3::Zero();
  for (int iteration=0;iteration<8;++iteration) {
    // A small translation prior also makes one visible wall well conditioned:
    // displacement along that wall stays at its odometry prediction.
    Eigen::Matrix3d hessian=Vec3(1,40,40).asDiagonal();
    Vec3 gradient=Vec3::Zero(); size_t count=0;
    const Eigen::Matrix3d rotation=Eigen::AngleAxisd(yaw,Vec3::UnitZ()).toRotationMatrix();
    for (size_t i=0;i<points.size();i+=stride) {
      if (!points[i].allFinite()) continue;
      const Vec3 rotated=rotation*(points[i]-pivot), p=pivot+rotated+translation;
      const auto match=matchFace(p,world); if (match.axis<0) continue;
      const double weight=std::min(1.0,.06/std::max(1e-9,std::abs(match.residual)));
      const Vec3 jacobian=match.axis==0?Vec3(-rotated.y(),1,0):Vec3(rotated.x(),0,1);
      hessian+=weight*jacobian*jacobian.transpose(); gradient+=weight*jacobian*match.residual; ++count;
    }
    if (count<80) return result;
    Vec3 step=-hessian.ldlt().solve(gradient);
    if (!step.allFinite()) return result;
    step.x()=std::clamp(step.x(),-.04,.04); step.y()=std::clamp(step.y(),-.03,.03); step.z()=std::clamp(step.z(),-.03,.03);
    yaw+=step.x(); translation+=Vec3(step.y(),step.z(),0);
    if (std::abs(yaw)>.25 || translation.head<2>().norm()>.25) return result;
    if (step.norm()<1e-5) break;
  }
  double squared=0; size_t count=0;
  const Eigen::Matrix3d rotation=Eigen::AngleAxisd(yaw,Vec3::UnitZ()).toRotationMatrix();
  for (size_t i=0;i<points.size();i+=stride) {
    const auto match=matchFace(pivot+rotation*(points[i]-pivot)+translation,world);
    if (match.axis>=0 && std::abs(match.residual)<.12) { squared+=match.residual*match.residual; ++count; }
  }
  if (count<80) return result;
  result.valid=true; result.yaw=yaw; result.translation=translation; result.inliers=count;
  result.rms=std::sqrt(squared/count); return result;
}
Points applyAlignment(const Points &points,const Vec3 &pivot,const WallAlignment &alignment) {
  if (!alignment.valid) return points;
  const Eigen::Matrix3d rotation=Eigen::AngleAxisd(alignment.yaw,Vec3::UnitZ()).toRotationMatrix();
  Points result; result.reserve(points.size());
  for (const auto &p:points) result.push_back(pivot+rotation*(p-pivot)+alignment.translation);
  return result;
}
Plan planLocal(const World &world,const Vec3 &start,const Vec3 &goal,
               const std::array<double,3> &limits,double margin) {
  if (!std::isfinite(margin) || margin<0) throw std::runtime_error("Invalid local planning margin");
  if (margin>0) {
    World buffered=world;
    for (auto &box:buffered.boxes) if (box.name=="lidar") { box.low.array()-=margin; box.high.array()+=margin; }
    try { auto result=plan(buffered,start,goal,limits); result.local_margin=margin; return result; }
    catch (const std::runtime_error &) {}  // Retry without the optional buffer, keeping the vehicle radius.
  }
  try { return plan(world,start,goal,limits); }
  catch (const std::runtime_error &) {
    // Subdivide the search grid only. Collision geometry and safety radius stay fixed.
    World fine(world.boxes,world.low,world.high,world.radius,world.resolution/2);
    return plan(fine,start,goal,limits);
  }
}
LocalObstacles::LocalObstacles(double voxel_size, double memory, double margin, size_t max_voxels)
    : voxel_size_(voxel_size), memory_(memory), margin_(margin), max_voxels_(max_voxels) {
  if (!std::isfinite(voxel_size) || voxel_size<=0 || !std::isfinite(memory) || memory<=0
      || !std::isfinite(margin) || margin<0 || !max_voxels || max_voxels>200000)
    throw std::runtime_error("Invalid lidar voxel size, memory, margin or capacity");
}
void LocalObstacles::expire(double now) {
  for (auto it=cells_.begin();it!=cells_.end();) {
    if (now-it->second>memory_) it=cells_.erase(it); else ++it;
  }
}
void LocalObstacles::update(const Points &points, double stamp) {
  if (!std::isfinite(stamp) || stamp<=last_stamp_) throw std::runtime_error("Non-finite or out-of-order lidar timestamp");
  std::set<Cell> incoming;
  for (const auto &p:points) {
    const Vec3 cell=(p/voxel_size_).array().floor();
    if (!p.allFinite() || (cell.array().abs()>1e8).any()) throw std::runtime_error("Invalid lidar point");
    incoming.insert({static_cast<int>(cell.x()),static_cast<int>(cell.y()),static_cast<int>(cell.z())});
  }
  expire(stamp);
  size_t combined=cells_.size(); for (const auto &cell:incoming) if (!cells_.count(cell)) ++combined;
  if (combined>max_voxels_) throw std::runtime_error("Lidar voxel capacity exceeded; refusing incomplete obstacle data");
  for (const auto &cell:incoming) cells_[cell]=stamp;
  last_stamp_=stamp;
}
Points LocalObstacles::centers(double now) {
  expire(now); Points result; result.reserve(cells_.size());
  for (const auto &[cell,stamp]:cells_) {
    (void)stamp; result.emplace_back((cell[0]+.5)*voxel_size_,(cell[1]+.5)*voxel_size_,(cell[2]+.5)*voxel_size_);
  }
  return result;
}
World LocalObstacles::world(const World &static_world, double now, const Vec3 &origin) {
  World result=static_world;
  const double half=voxel_size_/2+margin_;
  for (const auto &p:centers(now)) result.boxes.push_back({"lidar", (p-origin).array()-half,(p-origin).array()+half});
  return result;
}
}  // namespace uav_path_planner
