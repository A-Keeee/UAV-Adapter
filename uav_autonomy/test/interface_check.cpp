#include <iostream>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

#include "uav_autonomy/cloud.hpp"
#include "uav_autonomy/observed_map.hpp"
#include "uav_autonomy/trajectory.hpp"
#include "uav_autonomy/viewing_move.hpp"
#include "uav_autonomy/exploration_evidence.hpp"
#include "uav_autonomy/route_search.hpp"
#include "uav_autonomy/observation_viewpoint.hpp"

using namespace uav_autonomy;
void require(bool condition, const std::string &why) {
  if (!condition) throw std::runtime_error(why);
}
int main() {
  try {
    require(visitXYZ(sensor_msgs::msg::PointCloud2{}, [](const Vec &) {}).total == 0,
            "Default empty measurement must remain empty, never clear space");
    for (bool big : {false, true})
      for (int type : {7, 8}) {
        sensor_msgs::msg::PointCloud2 m;
        m.width = 2;
        m.height = 2;
        m.point_step = 32;
        m.row_step = 73;
        m.is_bigendian = big;
        for (int a : {2, 0, 1}) {
          sensor_msgs::msg::PointField f;
          f.name = std::string(1, "xyz"[a]);
          f.offset = 8 * a;
          f.datatype = type;
          f.count = 1;
          m.fields.push_back(f);
        }
        m.data.resize(m.row_step * m.height, 0xCA);
        for (int row = 0; row < 2; ++row)
          for (int col = 0; col < 2; ++col)
            for (int a = 0; a < 3; ++a) {
              double value = 100 * row + 10 * col + a + .25;
              float f = value;
              uint8_t bytes[8]{};
              size_t n = type == 7 ? 4 : 8;
              std::memcpy(bytes, type == 7 ? static_cast<void *>(&f) : static_cast<void *>(&value),
                          n);
              uint16_t probe = 1;
              if (big != (*reinterpret_cast<uint8_t *>(&probe) == 0))
                std::reverse(bytes, bytes + n);
              std::memcpy(m.data.data() + row * m.row_step + col * m.point_step + 8 * a, bytes, n);
            }
        Points points;
        auto stats = visitXYZ(m, [&](const Vec &p) { points.push_back(p); });
        require(stats.total == 4 && stats.finite == 4 &&
                    (points[3] - Vec(110.25, 111.25, 112.25)).norm() < 1e-8,
                "PointCloud2 layout/endian failure");
        m.data.resize(10);
        bool rejected = false;
        try {
          visitXYZ(m, [](const Vec &) {});
        } catch (const std::exception &) {
          rejected = true;
        }
        require(rejected, "Truncated cloud accepted");
      }
    sensor_msgs::msg::PointCloud2 oversized;
    oversized.width = 4000001;
    oversized.height = 1;
    oversized.point_step = 12;
    oversized.row_step = oversized.width * oversized.point_step;
    oversized.data.resize(oversized.row_step);
    bool overload_rejected = false;
    try {
      visitXYZ(oversized, [](const Vec &) {});
    } catch (const std::exception &) {
      overload_rejected = true;
    }
    require(overload_rejected, "Over-budget cloud must be rejected as a whole");
    NativeTrajectory::Message poly;
    poly.header.frame_id = "odom";
    poly.representation = 1;
    poly.degree = 5;
    poly.durations = {2, 3};
    poly.coefficients.resize(36, 0);
    poly.coefficients[4] = 1;
    poly.coefficients[18 + 4] = 1;
    poly.coefficients[18 + 5] = 2;
    NativeTrajectory tr(poly);
    require(std::abs(tr.sample(3).p.x() - 3) < 1e-9 && std::abs(tr.sample(3).v.x() - 1) < 1e-9,
            "Native polynomial piece evaluation failure");
    NativeTrajectory::Message bs;
    bs.header.frame_id = "odom";
    bs.degree = 3;
    bs.knots = {-3, -2, -1, 0, 1, 2, 3, 4, 5};
    for (int i = 0; i < 5; ++i) {
      bs.control_points.push_back(i);
      bs.control_points.push_back(0);
      bs.control_points.push_back(0);
    }
    NativeTrajectory spline(bs);
    require(std::abs(spline.sample(.5).p.x() - 1.5) < 1e-9 &&
                std::abs(spline.sample(.5).v.x() - 1) < 1e-9 && spline.sample(.5).a.norm() < 1e-9,
            "Native non-clamped EGO spline evaluation failure");
    bs.knots = {0, 0, 0, 0, 1, 1, 1, 1};
    bs.control_points = {0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0};
    auto curved = NativeTrajectory(bs).sample(.5);
    require(
        (curved.p - Vec(.75, .5, 0)).norm() < 1e-9 && (curved.v - Vec(0, 1.5, 0)).norm() < 1e-9 &&
            (curved.a - Vec(-6, 0, 0)).norm() < 1e-9 && (curved.j - Vec(0, -12, 0)).norm() < 1e-9,
        "Curved spline derivatives failure");
    poly.durations = {1};
    poly.coefficients.assign(18, 0);
    poly.coefficients[0] = 1;
    poly.coefficients[7] = 1;
    poly.coefficients[14] = 1;
    auto fifth = NativeTrajectory(poly).sample(.5);
    require((fifth.p - Vec(.03125, .0625, .125)).norm() < 1e-9 &&
                (fifth.v - Vec(.3125, .5, .75)).norm() < 1e-9 &&
                (fifth.a - Vec(2.5, 3, 3)).norm() < 1e-9 &&
                (fifth.j - Vec(15, 12, 6)).norm() < 1e-9,
            "Quintic derivatives failure");
    ObservedMap map(true, Vec(20, 20, 8), .2);
    map.move(Vec(0, 0, 1));
    size_t capacity = map.capacity();
    map.update({Vec(30, 0, 1)}, Vec(0, 0, 1));
    require(map.status(Vec(5, 0, 1)) == 0, "Clipped ray lost free traversal");
    require(map.status(Vec(9.9, 0, 1)) != 1, "False obstacle at window boundary");
    require(map.status(Vec(0, 5, 1)) < 0, "Missing return incorrectly marked free");
    for (int x = 0; x < 1000; x += 10) {
      map.move(Vec(x, 0, 1));
      map.update({Vec(x + 5, 0, 1)}, Vec(x, 0, 1));
    }
    require(map.capacity() == capacity && map.status(Vec(0, 0, 1)) < 0,
            "Rolling observation storage grows or retains distant cells");
    map.move(Vec(0, 0, 1));
    require(map.status(Vec(5, 0, 1)) < 0, "Out-of-window observations resurrect after returning");
    // forest/avia/1: an observed tree voxel (29.1, .1, 1.3) blinked
    // between occupied/free as later rays crossed another part of its voxel.
    // EGO-v2 keeps those hits until ring eviction. The execution map must not
    // reopen a stopping corridor which its native planner still blocks.
    ObservedMap static_forest(true, Vec(20, 20, 8), .2);
    const Vec tree(29.1, .1, 1.3), ray_origin(28.5, .1, 1.3);
    static_forest.move(Vec(29.5, 1.8, 1.8));
    static_forest.update({tree}, ray_origin);
    require(static_forest.status(tree) == 1, "Measured tree endpoint was not occupied");
    for (int frame = 0; frame < 20; ++frame)
      static_forest.update({Vec(32.1, .1, 1.3)}, ray_origin);
    require(static_forest.status(tree) == 1,
            "Rolling execution map cleared a static hit retained by the native planner");
    require(static_forest.status(Vec(30.1, .1, 1.3)) == 0,
            "Retaining a static hit prevented measured free-ray updates");
    static_forest.move(Vec(60, 0, 1.8));
    static_forest.move(Vec(29.5, 1.8, 1.8));
    require(static_forest.status(tree) < 0 && static_forest.capacity() == capacity,
            "Retained rolling hits survived window eviction or grew storage");
    ObservedMap body_support(true, Vec(20, 20, 8), .2);
    body_support.move(Vec(0, 0, 1));
    body_support.launchPad(Vec(0, 0, 0), 2, 1.6);
    require(body_support.safe(Vec(0, 0, 1), .65),
            "Clear swept body incorrectly rejected by unknown halo");
    require(body_support.status(Vec(0, 0, 1.7)) < 0, "Unknown halo was relabeled as free");
    require(!body_support.safe(Vec(0, 0, 1.4), .65), "Unknown swept body volume accepted");
    body_support.update({Vec(.7, 0, 1)}, Vec(0, 0, 1));
    require(!body_support.safe(Vec(0, 0, 1), .65), "Known obstacle clearance was relaxed");
    ObservedMap frontier_map(false, Vec(24, 32, 12), .2);
    frontier_map.launchPad(Vec(0, 0, 0), 4., 6.);
    auto frontiers = frontier_map.frontiers();
    require(frontiers.size() >= 8, "Room-sized connected frontier was not spatially subdivided");
    for (const auto &frontier : frontiers) {
      require((frontier.high - frontier.low).maxCoeff() <= 1.6 + 1e-8,
              "Frontier gain still uses a room-sized bounding box");
      require(frontier_map.status(frontier.centroid) == 0,
              "Frontier representative is not an observed free cell");
    }
    // A slow sensor adds a few cells in every 45 s window. Cumulative small
    // additions must not erase evidence gathered from multiple distinct views.
    ExplorationEvidence audit;
    size_t known=10000;
    for (double x : {0.,.6,1.2,1.8,2.4,3.6}) {
      audit.windowGain(400,800);
      audit.recordView(Vec(x,0,3),known);
      known+=400;
    }
    require(audit.views().size()==3 && audit.exhausted(.6),
            "Gradual cumulative gain erased distinct low-gain observation windows");
    // Actual Avia failure trace: modest windows at 982 and 826 cells crossed
    // the 800-cell entry threshold and repeatedly erased two audited views.
    // Hysteresis must keep those views until a substantial recovery occurs.
    require(!audit.windowGain(982,800) && !audit.windowGain(826,800) && audit.views().size()==3,
            "Near-threshold sampling variation erased audited low-gain viewpoints");
    require(audit.windowGain(1600,800) && audit.views().empty(),
            "A real renewed information-gain window did not invalidate exhaustion evidence");
    audit.recordView(Vec(0,0,3),known);
    require(!audit.exhausted(.6),"One low-gain view falsely ended exploration");
    require(audit.continueConnection(5.,.6) && !audit.continueConnection(5.,.01) &&
                !audit.continueConnection(.2,.6),
            "Low-gain audit interrupted a progressing connection or retained a stationary one");
    auto route_map=std::make_shared<ObservedMap>(frontier_map);
    auto first_view=searchRoute(route_map,Vec(0,0,3),Vec::Zero(),true,{},.9,"explore",1.2,5.8);
    require(!first_view.path.empty(),"Observed-room frontier fixture has no reachable viewpoint");
    Points audited_views{first_view.path.back()};
    auto second_view=searchRoute(route_map,Vec(0,0,3),Vec::Zero(),true,{},.9,"explore",1.2,5.8,audited_views);
    require(!second_view.path.empty() && (second_view.path.back()-audited_views.front()).norm()>=1.5-1e-8,
            "Frontier scoring reused the neighbourhood of an exhausted actual view");
    for (const auto &point : second_view.path)
      require(route_map->safe(point,.9),"View diversification left observed collision-checked space");
    // A narrow measured shelf corridor whose centre is at y=.2 cannot be
    // represented by every pose-anchored .4 m lattice. Small hover offsets
    // must not change reachability, or snap the actual state across a wall.
    auto shelf_map = std::make_shared<ObservedMap>(false, Vec(24, 32, 12), .2);
    shelf_map->launchPad(Vec::Zero(), 6., 6.);
    Points shelf_walls;
    for (double x = .1; x < 4.01; x += .2)
      for (double z = .1; z < 6.; z += .2) {
        shelf_walls.emplace_back(x, -.9, z);
        shelf_walls.emplace_back(x, 1.3, z);
      }
    shelf_map->update(shelf_walls, Vec(-2, 0, 3));
    for (const Vec &origin : {Vec(-1.01, .01, 3.01), Vec(-1.02, -.01, 2.99), Vec(-1, 0, 3)}) {
      const Vec target(3, .2, 3);
      const auto route = searchRoute(shelf_map, origin, target, false, {}, .9, "explore", 1.2, 4.8);
      require(!route.path.empty(), "Hover grid phase hid a measured shelf connection");
      require((route.path.front() - origin).norm() < 1e-8 &&
                  (route.path.back() - target).norm() < 1e-8, "Route snapped away from actual endpoints");
      for (size_t i = 1; i < route.path.size(); ++i)
        for (double f = 0; f <= 1.; f += .01)
          require(shelf_map->safe(route.path[i - 1] + f * (route.path[i] - route.path[i - 1]), .9),
                  "Lattice connector crossed unknown body space or obstacle clearance");
    }
    require(route_map->safe(first_view.path.back(), 1.1),
            "Frontier stop omitted its one-voxel obstacle reserve");
    require(holdEnclosureRadius(Vec(0, 0, .02), Eigen::Matrix3d::Identity()) < .98,
            "A contained physical airframe lost a supported hold because of a 2 cm tracking offset");
    require(holdEnclosureRadius(Vec(0, 0, .06), Eigen::Matrix3d::Identity()) > 1,
            "Uncontained tracking offset was allowed to borrow a supported hold");
    auto tilted = Eigen::AngleAxisd(M_PI / 2, Vec::UnitY()).toRotationMatrix();
    require(holdEnclosureRadius(Vec::Zero(), tilted) > 1,
            "Attitude was ignored in the physical-airframe enclosure check");
    // The smooth viewing curve stays inside [0, 1], but a complete stop from
    // its decelerating tail can leave that interval. Curve-only checks miss it.
    require(!viewingMoveSafe(Vec::Zero(), Vec(1, 0, 0), .5, [](const Vec &p) {
              return p.x() >= -1e-9 && p.x() <= 1 + 1e-9;
            }), "Viewing move accepted without a supported complete stopping tube");
    require(viewingMoveSafe(Vec::Zero(), Vec(1, 0, 0), .5, [](const Vec &p) {
              return p.norm() < 2;
            }), "A supported viewing move was rejected");
    // OS1 exploration: a new distant candidate used to be accepted while the
    // executed first A* edge stopped at a previously audited observation cell.
    auto observation_map = std::make_shared<ObservedMap>(false, Vec(24,32,12), .2);
    observation_map->launchPad(Vec::Zero(), 4, 7);
    const Vec observation_origin(.1,.1,3.1);
    std::unordered_set<Key,KeyHash> observed_stops{observation_map->key(observation_origin),
        observation_map->key(observation_origin+Vec(.4,0,0))};
    auto observation = observationViewpoint(observation_map, observation_origin, observation_origin,
        observation_origin+Vec(80,0,0), observed_stops, .9, 2, 6, .5);
    require(!observation.empty(), "No supported alternative observation move selected");
    const Vec executed_observation = observation[std::min(size_t(1),observation.size()-1)];
    require(!observed_stops.count(observation_map->key(executed_observation)),
            "Viewing recovery stops at an already audited cell while scoring a different endpoint");
    require(observation.size()==2 && (executed_observation-observation.back()).norm()<1e-9,
            "Scored observation endpoint differs from the actual stop");
    require(viewingMoveSafe(observation_origin,executed_observation,.5,[&](const Vec &p) {
              return p.z()>=2 && p.z()<=6 && observation_map->safe(p,.9);
            }),"New observation target has an unsupported curve or stopping tube");
    auto unknown_view_map = std::make_shared<ObservedMap>(false,Vec(24,32,12),.2);
    require(observationViewpoint(unknown_view_map,observation_origin,observation_origin,
                Vec(80,0,3),{},.9,2,6,.5).empty(),
            "Recovery manufactured free space for an unobserved observation move");
    // Recorded MID-360 forest failure: a newly hit adjacent surface voxel
    // enclosed the old stop. Synthetic free fill isolates OCCUPIED clearance;
    // unknown handling is separately asserted above and stays strict at runtime.
    nlohmann::json stop_record;
    std::ifstream(std::filesystem::path(__FILE__).parent_path()/"data/mid360_forest_stop.json") >> stop_record;
    auto stop_map = std::make_shared<ObservedMap>(true,Vec(20,20,8),.2);
    stop_map->move(Vec(16,-2,2));
    stop_map->launchPad(Vec(16,-2,0),4,6);
    auto points = [](const auto &data) {
      Points result;
      for (const auto &p : data) result.emplace_back(p[0],p[1],p[2]);
      return result;
    };
    stop_map->update(points(stop_record["old_occupied"]),Vec(16,-2,2));
    auto brake = [&](const char *name) {
      const auto &r = stop_record[name]["reference"];
      auto vec = [&](const char *field) { const auto &p=r[field]; return Vec(p[0],p[1],p[2]); };
      return uav_ganyu_control::brakingReference(
          {vec("position"),vec("velocity"),vec("acceleration"),vec("jerk")},0);
    };
    auto last_stop=brake("last"), earlier_stop=brake("earlier");
    require(stop_map->safe(last_stop.first.goal(),.8),
            "Recorded old stop was not admitted by the original obstacle check");
    require(!stop_map->stoppingSafe(last_stop.first.goal(),.8),
            "Forest stopping guard lacks a measured-surface reserve");
    for (double t=0; t<=earlier_stop.second; t+=.025)
      require(stop_map->stoppingSafe(earlier_stop.first.sample(t).p,.8),
              "Earlier recorded stop cannot retain the forest stopping reserve");
    stop_map->update(points(stop_record["new_occupied"]),Vec(16,-2,2));
    require(!stop_map->safe(last_stop.first.goal(),.8),
            "Fixture no longer reproduces the newly occupied stopping location");
    for (double t=0; t<=earlier_stop.second; t+=.025)
      require(stop_map->safe(earlier_stop.first.sample(t).p,.8),
              "Earlier recorded brake loses original clearance after new returns");
    require(!observationViewpoint(stop_map,last_stop.first.goal(),last_stop.first.goal(),
                Vec(80,0,1.8),{},.8,1.5,2.3,.18).size(),
            "Observation recovery escaped an occupied start by bypassing safety");
    require(!observationViewpoint(stop_map,earlier_stop.first.goal(),earlier_stop.first.goal(),
                Vec(80,0,1.8),{},.8,1.5,2.3,.18).empty(),
            "Earlier stop has no strictly supported observation recovery");
    std::cout << "PointCloud2 layouts, native trajectories, clipped rays and bounded rolling "
                 "storage: PASS\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
