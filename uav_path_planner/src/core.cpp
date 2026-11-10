#include "uav_path_planner/core.hpp"
#include <tinyxml2.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace uav_path_planner {
namespace {
using tinyxml2::XMLElement;
Vec3 vectorText(const char *text) {
  if (!text) throw std::runtime_error("Missing SDF vector");
  std::istringstream input(text); Vec3 v; std::string extra;
  if (!(input >> v.x() >> v.y() >> v.z()) || !v.allFinite() || input >> extra)
    throw std::runtime_error("Invalid finite SDF vector");
  return v;
}
Vec3 position(const XMLElement *element) {
  const auto *pose = element->FirstChildElement("pose");
  if (!pose) return Vec3::Zero();
  if (pose->Attribute("relative_to")) throw std::runtime_error("relative_to SDF poses are not supported");
  std::istringstream input(pose->GetText() ? pose->GetText() : "");
  Vec3 p, rotation; std::string extra;
  if (!(input >> p.x() >> p.y() >> p.z() >> rotation.x() >> rotation.y() >> rotation.z())
      || !p.allFinite() || !rotation.allFinite() || rotation.cwiseAbs().maxCoeff() > 1e-8 || input >> extra)
    throw std::runtime_error("Map loader requires finite, axis-aligned box poses");
  return p;
}
bool within(const Vec3 &p, const Vec3 &low, const Vec3 &high) {
  return (p.array() >= low.array()).all() && (p.array() <= high.array()).all();
}
double length(const Points &p) {
  double sum=0; for (size_t i=1; i<p.size(); ++i) sum += (p[i]-p[i-1]).norm(); return sum;
}
}  // namespace

World::World(std::vector<Box> input, Vec3 minimum, Vec3 maximum, double r, double res)
    : boxes(std::move(input)), low(minimum), high(maximum), radius(r), resolution(res) {
  if (!low.allFinite() || !high.allFinite() || (high.array() <= low.array()).any()
      || !std::isfinite(r) || r<=0 || !std::isfinite(res) || res<=0
      || (((high-low)/res).array().floor()+1).prod()>2'000'000)
    throw std::runtime_error("Invalid map bounds, radius, resolution or voxel count");
  for (const auto &b : boxes)
    if (!b.low.allFinite() || !b.high.allFinite() || (b.low.array()>b.high.array()).any())
      throw std::runtime_error("Invalid obstacle box");
}
World World::fromSdf(const std::string &file, Vec3 low, Vec3 high, double radius, double resolution) {
  tinyxml2::XMLDocument document;
  if (document.LoadFile(file.c_str()) != tinyxml2::XML_SUCCESS) throw std::runtime_error("Cannot load SDF: "+file);
  const auto *sdf=document.FirstChildElement("sdf");
  const auto *world=sdf ? sdf->FirstChildElement("world") : nullptr;
  if (!world) throw std::runtime_error("SDF must contain a world");
  if (world->FirstChildElement("include")) throw std::runtime_error("Included SDF obstacles must be expanded before planning");
  std::vector<Box> boxes;
  for (auto *model=world->FirstChildElement("model"); model; model=model->NextSiblingElement("model")) {
    const auto *is_static=model->FirstChildElement("static");
    if (!is_static || !is_static->GetText() ||
        (std::string(is_static->GetText())!="true" && std::string(is_static->GetText())!="1")) continue;
    for (auto *link=model->FirstChildElement("link"); link; link=link->NextSiblingElement("link"))
      for (auto *collision=link->FirstChildElement("collision"); collision; collision=collision->NextSiblingElement("collision")) {
        const auto *geometry=collision->FirstChildElement("geometry");
        const auto *box=geometry ? geometry->FirstChildElement("box") : nullptr;
        if (!box) {
          if (geometry && geometry->FirstChildElement("plane")) continue;
          throw std::runtime_error("Unsupported static SDF collision");
        }
        const auto *size_element=box->FirstChildElement("size");
        Vec3 size=vectorText(size_element ? size_element->GetText() : nullptr);
        if ((size.array()<=0).any()) throw std::runtime_error("Invalid SDF box size");
        Vec3 center=position(model)+position(link)+position(collision);
        boxes.push_back({model->Attribute("name") ? model->Attribute("name") : "box", center-size/2, center+size/2});
      }
  }
  if (boxes.empty()) throw std::runtime_error("No static obstacle boxes found");
  return World(std::move(boxes), low, high, radius, resolution);
}
bool World::free(const Vec3 &p, double extra) const {
  if (!p.allFinite() || !within(p, low, high)) return false;
  for (const auto &b : boxes)
    if (within(p, b.low.array()-radius-extra, b.high.array()+radius+extra)) return false;
  return true;
}
bool World::segmentFree(const Vec3 &a, const Vec3 &b, double extra) const {
  // The slab test also checks endpoints inside a box. Avoid scanning boxes twice.
  if (!a.allFinite() || !b.allFinite() || !within(a,low,high) || !within(b,low,high)) return false;
  const Vec3 delta=b-a;
  for (const auto &box : boxes) {
    const Vec3 minimum=box.low.array()-radius-extra, maximum=box.high.array()+radius+extra;
    double near=0, far=1;
    for (int axis=0; axis<3; ++axis) {
      if (std::abs(delta[axis])<1e-12) {
        if (a[axis]<minimum[axis] || a[axis]>maximum[axis]) { near=1; far=0; break; }
      } else {
        const double u=(minimum[axis]-a[axis])/delta[axis], v=(maximum[axis]-a[axis])/delta[axis];
        near=std::max(near, std::min(u,v)); far=std::min(far,std::max(u,v));
      }
    }
    if (near<=far) return false;
  }
  return true;
}
bool World::hullFree(const Points &points) const {
  Vec3 a=points.front(), b=a;
  for (const auto &p : points) { a=a.cwiseMin(p); b=b.cwiseMax(p); }
  if (!within(a,low,high) || !within(b,low,high)) return false;
  for (const auto &box : boxes)
    if ((b.array()>=box.low.array()-radius-1e-7).all()
        && (a.array()<=box.high.array()+radius+1e-7).all()) return false;
  return true;
}
double World::clearance(const Vec3 &p) const {
  double result=std::numeric_limits<double>::infinity();
  for (const auto &box : boxes) result=std::min(result,(box.low-p).cwiseMax(p-box.high).cwiseMax(0).norm());
  return result;
}
Points World::astar(const Vec3 &start, const Vec3 &goal) const {
  if (!free(start) || !free(goal)) throw std::runtime_error("Waypoint inside inflated obstacle or outside map");
  const Eigen::Vector3i shape=((high-low)/resolution).array().floor().cast<int>()+1;
  const int count=shape.prod();
  auto id=[&](const Eigen::Vector3i &p) { return (p.x()*shape.y()+p.y())*shape.z()+p.z(); };
  auto index=[&](int i) { const int z=i%shape.z(); i/=shape.z(); return Eigen::Vector3i(i/shape.y(),i%shape.y(),z); };
  auto inside=[&](const Eigen::Vector3i &p) { return (p.array()>=0).all() && (p.array()<shape.array()).all(); };
  auto point=[&](const Eigen::Vector3i &p)->Vec3 { return low+p.cast<double>()*resolution; };
  std::vector<uint8_t> blocked(count,0), closed(count,0);
  // Occupy centers by the physical clearance only. Every graph edge is checked
  // exactly below; half-diagonal padding would seal otherwise usable passages.
  const double padding=radius;
  for (const auto &box : boxes) {
    const Eigen::Vector3i a=((box.low.array()-padding-low.array())/resolution).ceil().cast<int>().cwiseMax(0);
    const Eigen::Vector3i b=((box.high.array()+padding-low.array())/resolution).floor().cast<int>().matrix().cwiseMin(shape-Eigen::Vector3i::Ones());
    for (int x=a.x(); x<=b.x(); ++x) for (int y=a.y(); y<=b.y(); ++y) for (int z=a.z(); z<=b.z(); ++z)
      blocked[id({x,y,z})]=1;
  }
  auto endpoint=[&](const Vec3 &p) {
    const Eigen::Vector3i center=((p-low)/resolution).array().round().cast<int>();
    double distance=std::numeric_limits<double>::infinity(); int best=-1;
    for (int x=-2; x<=2; ++x) for (int y=-2; y<=2; ++y) for (int z=-2; z<=2; ++z) {
      const Eigen::Vector3i candidate=center+Eigen::Vector3i(x,y,z);
      if (!inside(candidate) || blocked[id(candidate)]) continue;
      const Vec3 q=point(candidate); const double d=(p-q).norm();
      if (d<distance && segmentFree(p,q)) { distance=d; best=id(candidate); }
    }
    if (best<0) throw std::runtime_error("Waypoint cannot connect safely to voxel grid");
    return best;
  };
  const int source=endpoint(start), target=endpoint(goal);
  const Eigen::Vector3i target_index=index(target);
  auto heuristic=[&](int i) { return (index(i)-target_index).cast<double>().norm(); };
  using Item=std::tuple<double,double,int>;
  std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
  std::vector<double> cost(count,std::numeric_limits<double>::infinity());
  std::vector<int> parents(count,-1);
  queue.emplace(heuristic(source),0,source); cost[source]=0;
  while (!queue.empty()) {
    const auto [priority,current_cost,current]=queue.top(); queue.pop(); (void)priority;
    if (closed[current]) continue;
    if (current==target) {
      Points reverse;
      for (int i=current; i>=0; i=parents[i]) reverse.push_back(point(index(i)));
      Points result{start};
      for (auto it=reverse.rbegin(); it!=reverse.rend(); ++it) if ((*it-result.back()).norm()>1e-8) result.push_back(*it);
      if ((goal-result.back()).norm()>1e-8) result.push_back(goal);
      return result;
    }
    closed[current]=1; const auto p=index(current);
    for (int x=-1; x<=1; ++x) for (int y=-1; y<=1; ++y) for (int z=-1; z<=1; ++z) {
      if (!x && !y && !z) continue;
      const Eigen::Vector3i delta(x,y,z), neighbor=p+delta;
      if (!inside(neighbor)) continue;
      const int next=id(neighbor);
      if (blocked[next] || closed[next]) continue;
      const double candidate=current_cost+delta.cast<double>().norm();
      if (candidate<cost[next] && segmentFree(point(p),point(neighbor))) {
        cost[next]=candidate; parents[next]=current;
        queue.emplace(candidate+heuristic(next),candidate,next);
      }
    }
  }
  throw std::runtime_error("A* found no collision-free route");
}

Spline::Spline(Points p, std::vector<double> k) : points(std::move(p)), knots(std::move(k)) {
  reference_.set(degree,knots,points);
}
Vec3 Spline::sample(double time, int order) const {
  const auto r=reference_.sample(time);
  switch (order) { case 0:return r.p; case 1:return r.v; case 2:return r.a; case 3:return r.j; }
  throw std::runtime_error("Derivative order must be between zero and three");
}
std::array<double,3> Spline::bounds() const {
  return {reference_.derivativeBound(1),reference_.derivativeBound(2),reference_.derivativeBound(3)};
}
bool Spline::certified(const World &world, double start, double end) const {
  start=std::clamp(start,0.0,duration()); end=std::clamp(end,start,duration());
  if (end<=start) return world.free(sample(start));
  static const Eigen::Matrix<double,8,8> inverse=[] {
    Eigen::Matrix<double,8,8> basis;
    constexpr std::array<int,8> binomial{1,7,21,35,35,21,7,1};
    for (int i=0;i<8;++i) for (int j=0;j<8;++j) {
      const double t=i/7.0; basis(i,j)=binomial[j]*std::pow(t,j)*std::pow(1-t,7-j);
    }
    return Eigen::Matrix<double,8,8>(basis.inverse());
  }();
  std::function<bool(const Points&,int)> certify=[&](const Points &controls,int depth) {
    if (world.hullFree(controls)) return true;
    if (depth>=10) return false;
    Points work=controls,left{work.front()},right{work.back()};
    for (int n=7;n>0;--n) {
      for (int j=0;j<n;++j) work[j]=(work[j]+work[j+1])/2;
      left.push_back(work.front()); right.push_back(work[n-1]);
    }
    std::reverse(right.begin(),right.end());
    return certify(left,depth+1) && certify(right,depth+1);
  };
  for (size_t i=degree;i<points.size();++i) {
    const double a=std::max(start,knots[i]), b=std::min(end,knots[i+1]);
    if (b<=a) continue;
    Eigen::Matrix<double,8,3> samples;
    for (int j=0;j<8;++j) samples.row(j)=sample(a+(b-a)*j/7.0).transpose();
    const Eigen::Matrix<double,8,3> controls=inverse*samples;
    Points hull; for (int j=0;j<8;++j) hull.emplace_back(controls.row(j).transpose());
    if (!certify(hull,0)) return false;
  }
  return true;
}
Plan plan(const World &world, const Vec3 &start, const Vec3 &goal, const std::array<double,3> &limits) {
  for (double v:limits) if (!std::isfinite(v) || v<=0) throw std::runtime_error("Dynamic limits must be finite and positive");
  if ((goal-start).norm()<1e-5) throw std::runtime_error("Consecutive waypoints must differ");
  Plan result; result.planning_resolution=world.resolution;
  result.raw=world.astar(start,goal); result.astar_length=length(result.raw);
  Points reduced{result.raw.front()}; size_t index=0;
  while (index+1<result.raw.size()) {
    size_t following=index+1;
    for (size_t candidate=result.raw.size()-1;candidate>index;--candidate)
      if (world.segmentFree(result.raw[index],result.raw[candidate],.1)) { following=candidate; break; }
    reduced.push_back(result.raw[following]); index=following;
  }
  for (double spacing:{.8,.4,.2,.1,.05}) {
    Points dense{reduced.front()};
    for (size_t i=1;i<reduced.size();++i) {
      const int n=std::max(1,static_cast<int>(std::ceil((reduced[i]-reduced[i-1]).norm()/spacing)));
      for (int j=1;j<=n;++j) dense.push_back(reduced[i-1]+(reduced[i]-reduced[i-1])*j/n);
    }
    Points points(3,dense.front()); points.insert(points.end(),dense.begin(),dense.end());
    points.insert(points.end(),3,dense.back());
    while (points.size()<8) points.insert(points.begin()+points.size()/2,(points.front()+points.back())/2);
    std::vector<double> knots(8,0);
    for (size_t i=1;i<points.size()-7;++i) knots.push_back(i);
    knots.insert(knots.end(),8,points.size()-7);
    Spline candidate(points,knots);
    if (!candidate.certified(world)) continue;
    const auto bounds=candidate.bounds(); double scale=.05;
    for (int i=0;i<3;++i) scale=std::max(scale,std::pow(bounds[i]/limits[i],1.0/(i+1)));
    for (auto &k:knots) k*=scale*1.01;
    result.spline=Spline(std::move(points),std::move(knots));
    Points sampled; const size_t count=std::max(size_t(50),result.spline.points.size()*2);
    result.minimum_clearance=std::numeric_limits<double>::infinity();
    for (size_t i=0;i<count;++i) {
      sampled.push_back(result.spline.sample(result.spline.duration()*i/(count-1)));
      result.minimum_clearance=std::min(result.minimum_clearance,world.clearance(sampled.back()));
    }
    result.spline_length=length(sampled); return result;
  }
  throw std::runtime_error("Could not certify smoothed spline; no trajectory will be sent");
}
}  // namespace uav_path_planner
