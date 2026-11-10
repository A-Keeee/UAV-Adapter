// Gazebo regression only: inject an obstacle absent from the planner's SDF.
// This program never arms the vehicle and removes only its own unique model.
#include "uav_path_planner/core.hpp"
#include "uav_path_planner/gazebo_service.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nlohmann/json.hpp>
#include <unistd.h>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {
using Json=nlohmann::json;
using Vec3=Eigen::Vector3d;
volatile std::sig_atomic_t interrupted=0;
void stop(int) { interrupted=1; }
double now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
void service(const std::string &name,const std::string &type,const std::string &request) {
  uav_path_planner::gazeboService(name,type,request);
}
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv,rclcpp::InitOptions{},rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,stop); std::signal(SIGTERM,stop);
  auto node=std::make_shared<rclcpp::Node>("lidar_avoidance_check");
  const std::string output=node->declare_parameter<std::string>("output","/tmp/lidar_avoidance_check.json");
  const double timeout=node->declare_parameter<double>("timeout",240);
  Json mission,controller; double controller_at=0; nav_msgs::msg::Path::ConstSharedPtr path;
  auto mission_sub=node->create_subscription<std_msgs::msg::String>("/planning/status",10,
    [&](std_msgs::msg::String::ConstSharedPtr m){ mission=Json::parse(m->data); });
  auto controller_sub=node->create_subscription<std_msgs::msg::String>("/ganyu/status",10,
    [&](std_msgs::msg::String::ConstSharedPtr m){ controller=Json::parse(m->data); controller_at=now(); });
  auto path_sub=node->create_subscription<nav_msgs::msg::Path>("/planning/bspline_path",rclcpp::QoS(1).transient_local(),
    [&](nav_msgs::msg::Path::ConstSharedPtr m){ path=m; });
  (void)mission_sub; (void)controller_sub; (void)path_sub;
  const std::string name="lidar_avoidance_check_"+std::to_string(::getpid());
  bool created=false,moved=false; size_t baseline_stops=0,baseline_replans=0;
  double clearance=std::numeric_limits<double>::infinity(); Vec3 center=Vec3::Zero();
  std::vector<std::string> states;
  Json result={{"passed",false},{"obstacle",name}};
  rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
  try {
    const double deadline=now()+timeout;
    while (now()<deadline && !interrupted) {
      executor.spin_once(std::chrono::milliseconds(100));
      if (mission.is_null() || controller.is_null() || now()-controller_at>1) continue;
      const std::string state=mission.at("state");
      if (states.empty() || states.back()!=state) { states.push_back(state); std::cout<<"Observed state: "<<state<<std::endl; }
      const auto &lidar=mission.at("lidar_avoidance");
      if (state=="TRACKING" && !created && path && path->poses.size()>=160) {
        if (!lidar.value("enabled",false) || !mission.at("completed_waypoints").empty())
          throw std::runtime_error("Start this check before the default MID-360 waypoint mission");
        const double duration=mission.at("plans").at(0).at("duration_s");
        const size_t index=std::clamp<size_t>(std::lround(6/duration*159),1,159);
        const auto &p=path->poses[index].pose.position; center={p.x,p.y,p.z};
        const std::string sdf="<sdf version='1.9'><model name='"+name+"'><static>true</static><pose>"
            +std::to_string(p.x)+" "+std::to_string(p.y)+" "+std::to_string(p.z)+" 0 0 0</pose><link name='obstacle'>"
            "<collision name='box'><geometry><box><size>0.6 0.6 0.6</size></box></geometry></collision>"
            "<visual name='box'><geometry><box><size>0.6 0.6 0.6</size></box></geometry>"
            "<material><diffuse>1 0.25 0.05 1</diffuse></material></visual></link></model></sdf>";
        service("create","gz.msgs.EntityFactory","sdf: "+Json(sdf).dump()); created=true;
        baseline_stops=lidar.at("stops"); baseline_replans=lidar.at("replans");
        result["initial_center_enu_m"]={center.x(),center.y(),center.z()};
        std::cout<<"Inserted 0.6 m obstacle into the active route at "<<center.transpose()<<std::endl;
      }
      if (created) {
        const auto &p=controller.at("position_ned"), &h=controller.at("home_ned");
        const Vec3 actual(p[1].get<double>()-h[1].get<double>(),p[0].get<double>()-h[0].get<double>(),h[2].get<double>()-p[2].get<double>());
        clearance=std::min(clearance,((actual-center).cwiseAbs().array()-.3).max(0).matrix().norm());
      }
      if (created && !moved && !mission.at("completed_waypoints").empty() && lidar.at("replans").get<size_t>()>baseline_replans) {
        center={-3,-2,2.5}; service("set_pose","gz.msgs.Pose","name: \""+name+"\" position { x: -3 y: -2 z: 2.5 } orientation { w: 1 }");
        moved=true; std::cout<<"First waypoint reached around the obstacle; moved it out of the route"<<std::endl;
      }
      if (state=="FAILED" || state=="ABORTING") throw std::runtime_error(mission.value("error",state));
      if (state=="COMPLETE") {
        result["passed"]=created && moved && lidar.at("stops").get<size_t>()>baseline_stops
            && lidar.at("replans").get<size_t>()>baseline_replans && mission.at("succeeded").get<bool>()
            && mission.at("landed_disarmed").get<bool>() && clearance>=mission.at("safety_radius_m").get<double>();
        result["final_mission"]=mission; break;
      }
    }
    if (!result.contains("final_mission")) throw std::runtime_error(interrupted?"Operator stopped regression":"Timed out waiting for completed avoidance mission");
  } catch (const std::exception &e) { result["error"]=e.what(); }
  if (created) {
    try { service("remove","gz.msgs.Entity","name: \""+name+"\" type: MODEL"); }
    catch (const std::exception &e) { result["cleanup_error"]=e.what(); }
  }
  result["states"]=states; result["obstacle_created"]=created; result["obstacle_moved"]=moved;
  result["minimum_estimated_obstacle_clearance_m"]=std::isfinite(clearance)?Json(clearance):Json(nullptr);
  const std::filesystem::path file(output); if (file.has_parent_path()) std::filesystem::create_directories(file.parent_path());
  { std::ofstream out(file); out<<result.dump(2)<<'\n'; }
  std::cout<<result.dump(2)<<std::endl; executor.remove_node(node); node.reset(); rclcpp::shutdown();
  return result.at("passed").get<bool>()?0:1;
}
