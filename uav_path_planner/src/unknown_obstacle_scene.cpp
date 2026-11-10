// Scene-side test driver only. No geometry is sent to the planner or its map.
#include "uav_path_planner/unknown_obstacles.hpp"
#include "uav_path_planner/gazebo_service.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <nlohmann/json.hpp>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace {
using namespace uav_path_planner;
using Json=nlohmann::json;
using String=std_msgs::msg::String;
volatile std::sig_atomic_t interrupted=0;
void stop(int) { interrupted=1; }
double wallTime() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
struct Record {
  ScenarioObstacle obstacle;
  std::string model;
  bool spawned=false,responded=false;
  double spawn_time=0,minimum_clearance=std::numeric_limits<double>::infinity();
};
class Scenario : public rclcpp::Node {
public:
  Scenario() : Node("unknown_obstacle_scene") {
    const auto share=ament_index_cpp::get_package_share_directory("uav_path_planner");
    config_=declare_parameter<std::string>("config",share+"/scenarios/unknown_obstacles.yaml");
    output_=declare_parameter<std::string>("output","/tmp/unknown_obstacles.json");
    for (const auto &obstacle:loadUnknownObstacles(config_)) {
      Record record; record.obstacle=obstacle;
      record.model="unmapped_"+obstacle.name+"_"+std::to_string(::getpid());
      records_.push_back(record);
    }
    status_pub_=create_publisher<String>("/simulation/unknown_obstacles/status",rclcpp::QoS(1).transient_local());
    mission_sub_=create_subscription<String>("/planning/status",10,[this](String::ConstSharedPtr msg) {
      try { mission_=Json::parse(msg->data); mission_at_=wallTime(); }
      catch (const Json::exception &e) { RCLCPP_WARN(get_logger(),"Invalid mission status: %s",e.what()); }
    });
    controller_sub_=create_subscription<String>("/ganyu/status",10,[this](String::ConstSharedPtr msg) {
      try { controller_=Json::parse(msg->data); controller_at_=wallTime(); }
      catch (const Json::exception &e) { RCLCPP_WARN(get_logger(),"Invalid controller status: %s",e.what()); }
    });
    clock_sub_=create_subscription<rosgraph_msgs::msg::Clock>("/clock",rclcpp::SensorDataQoS(),
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg){ sim_time_=msg->clock.sec+msg->clock.nanosec*1e-9; });
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this](){ tick(); });
    RCLCPP_INFO(get_logger(),"Queued %zu Gazebo-only obstacles; each appears after its leg starts tracking",records_.size());
    writeReport();
  }
  void cleanup() {
    for (const auto &r:records_) if (r.spawned) {
      try { gazeboService("remove","gz.msgs.Entity","name: "+Json(r.model).dump()+" type: MODEL"); }
      catch (const std::exception &e) { RCLCPP_WARN(get_logger(),"Obstacle cleanup: %s",e.what()); }
    }
  }
private:
  void spawn(Record &r) {
    const auto &o=r.obstacle;
    std::ostringstream sdf; sdf<<std::setprecision(12);
    sdf<<"<sdf version='1.9'><model name='"<<r.model<<"'><static>true</static><pose>"
       <<o.center.transpose()<<" 0 0 0</pose><link name='obstacle'>"
       <<"<collision name='box'><geometry><box><size>"<<o.size.transpose()<<"</size></box></geometry></collision>"
       <<"<visual name='box'><geometry><box><size>"<<o.size.transpose()<<"</size></box></geometry>"
       <<"<material><ambient>0.8 0.15 0.03 1</ambient><diffuse>1 0.25 0.05 1</diffuse></material></visual>"
       <<"</link></model></sdf>";
    gazeboService("create","gz.msgs.EntityFactory","sdf: "+Json(sdf.str()).dump());
    r.spawned=true; r.spawn_time=sim_time_;
    RCLCPP_WARN(get_logger(),"Leg %zu: spawned %s at ENU (%.2f, %.2f, %.2f); global map unchanged",
                o.leg,r.model.c_str(),o.center.x(),o.center.y(),o.center.z());
  }
  void tick() {
    const double time=wallTime();
    if (time-last_report_>1) { writeReport(); last_report_=time; }
    if (finished_ || mission_.is_null() || controller_.is_null() || time-mission_at_>2 || time-controller_at_>1) return;
    const auto &p=controller_.at("position_ned"), &h=controller_.at("home_ned");
    const Vec3 actual(p[1].get<double>()-h[1].get<double>(),p[0].get<double>()-h[0].get<double>(),h[2].get<double>()-p[2].get<double>());
    const std::string state=mission_.at("state");
    const auto &lidar=mission_.at("lidar_avoidance");
    const size_t leg=mission_.at("completed_waypoints").size()+1;
    for (auto &r:records_) {
      if (!r.spawned && r.obstacle.leg==leg && state=="TRACKING"
          && controller_.value("phase",std::string{})=="ACTIVE" && controller_.value("trajectory_active",false)) {
        if (!lidar.value("enabled",false)) throw std::runtime_error("Unknown-obstacle scenario requires MID-360 avoidance");
        spawn(r); writeReport();
      }
      if (!r.spawned) continue;
      r.minimum_clearance=std::min(r.minimum_clearance,
          ((actual-r.obstacle.center).cwiseAbs()-r.obstacle.size/2).cwiseMax(0).norm());
      for (const auto &event:lidar.at("replan_history"))
        if (!r.responded && event.at("leg").get<size_t>()==r.obstacle.leg && event.at("sim_time").get<double>()>=r.spawn_time) {
          r.responded=true;
          RCLCPP_INFO(get_logger(),"Leg %zu: lidar replanning response observed",r.obstacle.leg);
        }
    }
    if (state=="FAILED" || state=="ABORTING" || state=="COMPLETE") {
      finished_=true; passed_=state=="COMPLETE" && mission_.value("succeeded",false) && mission_.value("landed_disarmed",false);
      const double radius=mission_.at("safety_radius_m");
      for (const auto &r:records_) passed_=passed_ && r.spawned && r.responded && r.minimum_clearance>=radius;
      RCLCPP_INFO(get_logger(),"Unknown-obstacle evaluation finished: %s; objects retained until scene stops",passed_?"PASS":"FAIL");
      writeReport();
    }
  }
  void writeReport() {
    Json obstacles=Json::array();
    for (const auto &r:records_) {
      const auto &o=r.obstacle;
      obstacles.push_back({{"name",r.model},{"leg",o.leg},{"center_enu_m",{o.center.x(),o.center.y(),o.center.z()}},
          {"size_m",{o.size.x(),o.size.y(),o.size.z()}},{"spawned",r.spawned},{"spawn_sim_time_s",r.spawn_time},
          {"replanning_observed",r.responded},{"minimum_estimated_clearance_m",
            std::isfinite(r.minimum_clearance)?Json(r.minimum_clearance):Json(nullptr)}});
    }
    Json report={{"state",finished_?"FINISHED":"WAITING_OR_RUNNING"},{"passed",finished_?Json(passed_):Json(nullptr)},
      {"config",config_},{"global_map_modified",false},{"spawn_policy","after corresponding leg is TRACKING"},
      {"obstacles",obstacles},{"mission",mission_}};
    String msg; msg.data=report.dump(); status_pub_->publish(msg);
    const std::filesystem::path file(output_);
    if (file.has_parent_path()) std::filesystem::create_directories(file.parent_path());
    auto temp=file; temp.replace_extension(".tmp");
    { std::ofstream out(temp); if (!out) throw std::runtime_error("Cannot write unknown obstacle report"); out<<report.dump(2)<<'\n'; }
    std::filesystem::rename(temp,file);
  }
  std::vector<Record> records_;
  std::string config_,output_;
  Json mission_,controller_;
  bool finished_=false,passed_=false;
  double mission_at_=0,controller_at_=0,sim_time_=0,last_report_=0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<String>::SharedPtr status_pub_;
  rclcpp::Subscription<String>::SharedPtr mission_sub_,controller_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv,rclcpp::InitOptions{},rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,stop); std::signal(SIGTERM,stop);
  std::shared_ptr<Scenario> node; int result=0;
  try {
    node=std::make_shared<Scenario>(); rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
    while (rclcpp::ok() && !interrupted) executor.spin_once(std::chrono::milliseconds(100));
    executor.remove_node(node);
  } catch (const std::exception &e) { RCLCPP_ERROR(rclcpp::get_logger("unknown_obstacle_scene"),"%s",e.what()); result=1; }
  if (node) node->cleanup();
  node.reset(); if (rclcpp::ok()) rclcpp::shutdown(); return result;
}
