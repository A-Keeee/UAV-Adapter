#include "uav_path_planner/core.hpp"
#include "uav_path_planner/lidar.hpp"
#include "uav_path_planner/gcs_heartbeat.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <uav_common_msg/msg/b_spline_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>

namespace uav_path_planner {
using Json=nlohmann::json;
using Marker=visualization_msgs::msg::Marker;
using PathMsg=nav_msgs::msg::Path;
using String=std_msgs::msg::String;
namespace fs=std::filesystem;
namespace {
double wallTime() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
Vec3 enu(const Vec3 &p) { return {p.y(),p.x(),-p.z()}; }
Json vectorJson(const Vec3 &p) { return Json::array({p.x(),p.y(),p.z()}); }
Vec3 vectorValue(const Json &j) {
  if (!j.is_array() || j.size()!=3) throw std::runtime_error("Expected finite XYZ feedback");
  Vec3 p(j.at(0).get<double>(),j.at(1).get<double>(),j.at(2).get<double>());
  if (!p.allFinite()) throw std::runtime_error("Non-finite feedback");
  return p;
}
geometry_msgs::msg::Point pointMsg(const Vec3 &p) {
  geometry_msgs::msg::Point result; result.x=p.x(); result.y=p.y(); result.z=p.z(); return result;
}
Json planInfo(const Plan &p) {
  return {{"astar_length_m",p.astar_length},{"spline_length_m",p.spline_length},
          {"duration_s",p.spline.duration()},{"derivative_bounds",p.spline.bounds()},
          {"minimum_sampled_clearance_m",p.minimum_clearance},
          {"planning_resolution_m",p.planning_resolution},{"local_planning_margin_m",p.local_margin},
          {"collision_certificate","all subdivided Bezier hull AABBs free"},{"control_points",p.spline.points.size()}};
}
}  // namespace

class Mission : public rclcpp::Node {
public:
  Mission() : Node("waypoint_mission") {
    const auto share=ament_index_cpp::get_package_share_directory("uav_path_planner");
    auto vecParam=[&](const std::string &name,std::vector<double> value) {
      auto result=param(name,value);
      if (result.size()!=3) throw std::runtime_error(name+" must contain three values");
      return Vec3(result[0],result[1],result[2]);
    };
    const auto world_file=param("world_file",share+"/worlds/walls_demo.sdf");
    const auto minimum=vecParam("map_min",{-15,-16,.8}), maximum=vecParam("map_max",{25,18,4});
    world_=std::make_shared<World>(World::fromSdf(world_file,minimum,maximum,param("safety_radius",.65),param("resolution",.4)));
    obstacles_=world_;
    const auto flat=param("waypoints",std::vector<double>{0,8,2.5,8,8,2.5,8,0,2.5});
    if (flat.empty() || flat.size()%3) throw std::runtime_error("waypoints must be finite ENU triples");
    for (size_t i=0;i<flat.size();i+=3) {
      Vec3 p(flat[i],flat[i+1],flat[i+2]); if (!p.allFinite()) throw std::runtime_error("Non-finite waypoint");
      waypoints_.push_back(p);
    }
    height_=param("takeoff_height",2.5); auto_start_=param("auto_start",false);
    land_at_end_=param("land_at_end",true); result_file_=param("result_file",std::string{});
    limits_={param("max_velocity",1.0),param("max_acceleration",.7),param("max_jerk",1.5)};
    if (!std::isfinite(height_) || height_<.8 || height_>3) throw std::runtime_error("takeoff_height must be between 0.8 and 3 m");
    lidar_enabled_=param("lidar_enabled",false); lidar_topic_=param("lidar_topic",std::string("/lidar/points"));
    lidar_frame_=param("lidar_frame",std::string("mid360_link"));
    lidar_range_=param("lidar_range",8.0); lidar_min_range_=param("lidar_min_range",.45);
    lidar_ground_=param("lidar_ground_height",.15); lidar_timeout_=param("lidar_timeout",.5);
    lookahead_=param("lidar_lookahead",6.0); replan_timeout_=param("replan_timeout",30.0);
    const double memory=param("lidar_memory",.6), voxel=param("lidar_voxel_size",.3), margin=param("lidar_margin",.15);
    const int capacity=param("lidar_max_voxels",6000);
    if (capacity<=0) throw std::runtime_error("lidar_max_voxels must be positive");
    local_=std::make_unique<LocalObstacles>(voxel,memory,margin,static_cast<size_t>(capacity));
    align_yaw_=param("lidar_align_spawn_yaw",true); spawn_yaw_=param("lidar_spawn_yaw_enu",0.0);
    register_walls_=param("lidar_register_known_walls",true);
    replan_margin_=param("lidar_replan_margin",.2);
    if (!std::isfinite(replan_margin_) || replan_margin_<0 || replan_margin_>1)
      throw std::runtime_error("lidar_replan_margin must be between 0 and 1 m");
    yaw_aligned_=!align_yaw_;
    for (double v:{lidar_range_,lidar_min_range_,lidar_ground_,lidar_timeout_,lookahead_,replan_timeout_,spawn_yaw_})
      if (!std::isfinite(v)) throw std::runtime_error("Non-finite lidar parameter");
    if (lidar_min_range_<=0 || lidar_range_<=lidar_min_range_ || lidar_ground_<0 || lidar_timeout_<=0
        || lidar_timeout_>memory || lookahead_<3 || replan_timeout_<=0
        || (lidar_enabled_ && lidar_range_<limits_[0]*lookahead_+world_->radius))
      throw std::runtime_error("Invalid lidar range, timeout, memory or lookahead");
    const auto latched=rclcpp::QoS(1).transient_local();
    key_pub_=create_publisher<String>("/ganyu/keyboard",10);
    alive_pub_=create_publisher<std_msgs::msg::Empty>("/ganyu/keyboard_alive",10);
    trajectory_pub_=create_publisher<uav_common_msg::msg::BSplineTrajectory>("/planning/trajectory",10);
    status_pub_=create_publisher<String>("/planning/status",latched);
    markers_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("/planning/markers",latched);
    astar_pub_=create_publisher<PathMsg>("/planning/astar_path",latched);
    spline_pub_=create_publisher<PathMsg>("/planning/bspline_path",latched);
    actual_pub_=create_publisher<PathMsg>("/planning/actual_path",1);
    reference_pub_=create_publisher<PathMsg>("/planning/tracking_reference",1);
    lidar_marker_pub_=create_publisher<Marker>("/planning/lidar_obstacles",1);
    status_sub_=create_subscription<String>("/ganyu/status",10,[this](String::ConstSharedPtr msg){ onStatus(*msg); });
    debug_sub_=create_subscription<std_msgs::msg::Float64MultiArray>("/ganyu/control_debug",10,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg){ onDebug(*msg); });
    clock_sub_=create_subscription<rosgraph_msgs::msg::Clock>("/clock",rclcpp::SensorDataQoS(),
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
        const double value=msg->clock.sec+msg->clock.nanosec*1e-9;
        if (sim_time_>0 && value<sim_time_-.01) fail("Simulation clock moved backwards");
        sim_time_=value; clock_at_=wallTime();
      });
    if (lidar_enabled_) {
      tf_buffer_=std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_=std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
      cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic_,rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){ pending_cloud_=msg; pending_at_=wallTime(); });
      RCLCPP_INFO(get_logger(),"MID-360 reactive avoidance enabled on %s; no lidar mapping",lidar_topic_.c_str());
    }
    if (param("provide_gcs_heartbeat",true)) {
      try { gcs_=std::make_unique<GcsHeartbeat>(); }
      catch (const std::runtime_error &e) { RCLCPP_WARN(get_logger(),"%s",e.what()); }
    }
    if (!result_file_.empty()) {
      const fs::path file(result_file_); if (file.has_parent_path()) fs::create_directories(file.parent_path());
      auto csv=file; csv.replace_extension(".csv"); trace_.open(csv); trace_<<std::setprecision(12);
      if (!trace_) throw std::runtime_error("Cannot open tracking CSV");
      trace_<<"sim_time,leg,east,north,up,ref_east,ref_north,ref_up,tracking_error_m,wall_clearance_m\n";
    }
    try {
      Vec3 start(0,0,height_);
      if (!world_->free(start)) throw std::runtime_error("Takeoff point is blocked");
      for (const auto &goal:waypoints_) {
        plans_.push_back(plan(*world_,start,goal,limits_)); start=goal;
        RCLCPP_INFO(get_logger(),"Leg %zu: %s",plans_.size(),planInfo(plans_.back()).dump().c_str());
      }
      for (const auto &p:Points{Vec3(0,0,height_),waypoints_.back()}) for (const auto &box:world_->boxes)
        if ((p.head<2>().array()>=box.low.head<2>().array()-world_->radius).all()
            && (p.head<2>().array()<=box.high.head<2>().array()+world_->radius).all()
            && box.low.z()<=p.z()+world_->radius && box.high.z()>=0)
          throw std::runtime_error("Takeoff/landing column intersects obstacle");
      setState(auto_start_?"WAIT_READY":"PREVIEW"); publishPlan();
    } catch (const std::runtime_error &e) { error_=e.what(); setState("FAILED"); done_=true; }
    started_at_=wallTime();
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this](){ tick(); });
  }
  bool done() const { return done_; }
  bool success() const { return succeeded_ || (!auto_start_ && error_.empty()); }
  void interrupt() { interrupted_=true; }
  void close() { writeReport(); }

private:
  template<class T> T param(const std::string &name,const T &value) { return declare_parameter<T>(name,value); }
  builtin_interfaces::msg::Time stamp() const {
    builtin_interfaces::msg::Time t; const auto ns=static_cast<int64_t>(sim_time_*1e9);
    t.sec=static_cast<int32_t>(ns/1000000000); t.nanosec=static_cast<uint32_t>(ns%1000000000); return t;
  }
  void setState(const std::string &state) {
    state_=state; last_action_=0; stable_since_=-1;
    RCLCPP_INFO(get_logger(),"Mission -> %s (leg %zu/%zu)",state.c_str(),std::min(leg_+1,waypoints_.size()),waypoints_.size());
  }
  void key(const std::string &value) { String msg; msg.data=value; key_pub_->publish(msg); last_action_=wallTime(); }
  Vec3 feedback(const std::string &name) const { return vectorValue(controller_.at(name)); }
  bool flag(const std::string &name) const { return controller_.value(name,false); }
  std::string phase() const { return controller_.value("phase",std::string{}); }
  void onStatus(const String &msg) {
    try {
      Json candidate=Json::parse(msg.data);
      for (const auto &name:{"position_ned","velocity_ned","home_ned","target_ned"}) vectorValue(candidate.at(name));
      for (const auto &name:{"armed","landed","ready","preflight","trajectory_active"}) candidate.at(name).get<bool>();
      candidate.at("phase").get<std::string>(); candidate.at("trajectory_id").get<uint64_t>();
      controller_=std::move(candidate); status_at_=wallTime(); have_status_=true;
      if (phase()=="ACTIVE") home_=feedback("home_ned");
      else if (phase()=="LANDING") actual_.push_back(enu(feedback("position_ned")-home_));
    } catch (const std::exception &e) { fail(std::string("Invalid controller status: ")+e.what()); }
  }
  void onDebug(const std_msgs::msg::Float64MultiArray &msg) {
    if (msg.data.size()!=26 || sim_time_==last_trace_) return;
    last_trace_=sim_time_;
    const Vec3 reference=enu(Vec3(msg.data[5],msg.data[6],msg.data[7])-home_);
    const Vec3 actual=enu(Vec3(msg.data[20],msg.data[21],msg.data[22])-home_);
    const double error=(actual-reference).norm(), clearance=world_->clearance(actual);
    if (!actual.allFinite() || !reference.allFinite() || !std::isfinite(error) || !std::isfinite(clearance)) {
      fail("Non-finite tracking feedback"); return;
    }
    ++tracking_samples_; squared_error_+=error*error; max_error_=std::max(max_error_,error);
    min_clearance_=std::min(min_clearance_,clearance); actual_.push_back(actual); references_.push_back(reference);
    if (trace_) trace_<<sim_time_<<','<<leg_<<','<<actual.x()<<','<<actual.y()<<','<<actual.z()<<','
                     <<reference.x()<<','<<reference.y()<<','<<reference.z()<<','<<error<<','<<clearance<<'\n';
    if ((state_=="TRACKING" || state_=="SETTLING" || state_=="BRAKING" || state_=="REPLANNING")
        && (error>.6 || clearance<world_->radius)) fail("Tracking envelope exceeded");
  }
  bool lidarReady(double now) const {
    return cloud_frames_>0 && yaw_aligned_ && now-cloud_at_<=lidar_timeout_
           && sim_time_-local_->lastStamp()>=-.1 && sim_time_-local_->lastStamp()<=lidar_timeout_;
  }
  bool updateLidar(double now) {
    if (!lidar_enabled_ || now-last_lidar_check_<.1) return false;
    last_lidar_check_=now;
    if (pending_cloud_) {
      const auto &msg=*pending_cloud_; const double stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;
      try {
        if (stamp<=0 || stamp<=local_->lastStamp() || sim_time_-stamp<-.1 || sim_time_-stamp>lidar_timeout_
            || now-pending_at_>lidar_timeout_) throw std::runtime_error("Stale, duplicated or out-of-order lidar frame");
        if (msg.header.frame_id!=lidar_frame_) throw std::runtime_error("Unexpected lidar coordinate frame: "+msg.header.frame_id);
        const auto transform=tf_buffer_->lookupTransform("odom",msg.header.frame_id,
            tf2::TimePoint(std::chrono::nanoseconds(int64_t(msg.header.stamp.sec)*1000000000+msg.header.stamp.nanosec)));
        const auto &t=transform.transform.translation; const auto &q=transform.transform.rotation;
        Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
        Vec3 translation(t.x,t.y,t.z);
        if (!translation.allFinite() || !rotation.coeffs().allFinite() || std::abs(rotation.norm()-1)>.05)
          throw std::runtime_error("Invalid lidar TF");
        rotation.normalize();
        if (!yaw_aligned_) {
          if (!have_status_ || flag("armed") || phase()!="IDLE") throw std::runtime_error("Awaiting unarmed spawn heading alignment");
          const auto r=rotation.toRotationMatrix();
          yaw_offset_=std::remainder(spawn_yaw_-std::atan2(r(1,0),r(0,0)),2*std::acos(-1.0));
          yaw_aligned_=true;
          RCLCPP_INFO(get_logger(),"Lidar spawn heading alignment: %.4f rad",yaw_offset_);
        }
        rotation=Eigen::AngleAxisd(yaw_offset_,Vec3::UnitZ())*rotation;
        const Vec3 origin=enu(home_);
        const auto raw=cloudXYZ(msg); Points observed; size_t finite=0;
        for (const auto &p:raw) {
          if (!p.allFinite()) continue;
          ++finite;
          if (p.norm()<lidar_min_range_ || p.norm()>lidar_range_) continue;
          const Vec3 relative=rotation*p+translation+lidar_translation_-origin;
          if (relative.z()>lidar_ground_ && relative.z()<=world_->high.z()+world_->radius) observed.push_back(relative);
        }
        if (!finite) throw std::runtime_error("Lidar frame has no finite returns");
        if (register_walls_) {
          const Vec3 pivot=translation+lidar_translation_-origin;
          const auto alignment=alignKnownWalls(observed,*world_,pivot);
          if (alignment.valid && (lidar_translation_+alignment.translation).head<2>().norm()<=.5) {
            observed=applyAlignment(observed,pivot,alignment);
            yaw_offset_=std::remainder(yaw_offset_+alignment.yaw,2*std::acos(-1.0));
            lidar_translation_+=alignment.translation; alignment_rms_=alignment.rms; alignment_inliers_=alignment.inliers;
            if (std::abs(alignment.yaw)>.01 || alignment.translation.norm()>.03) local_->clear();
          }
        }
        Points points; points.reserve(observed.size());
        for (const auto &p:observed)
          if ((p.array()>=world_->low.array()-world_->radius).all()
              && (p.array()<=world_->high.array()+world_->radius).all() && unmodeled(p,*world_)) points.push_back(p+origin);
        local_->update(points,stamp); ++cloud_frames_; cloud_at_=pending_at_; cloud_error_.clear(); pending_cloud_.reset();
      } catch (const std::exception &e) {
        cloud_error_=e.what();
        if (!lidarReady(now) && now-last_cloud_warning_>2) {
          RCLCPP_WARN(get_logger(),"Lidar data not usable: %s",e.what()); last_cloud_warning_=now;
        }
      }
    }
    obstacles_=std::make_shared<World>(local_->world(*world_,sim_time_,enu(home_)));
    Marker marker; marker.header.frame_id="odom"; marker.header.stamp=stamp();
    marker.ns="lidar_obstacles"; marker.id=0; marker.type=Marker::CUBE_LIST; marker.action=Marker::ADD;
    marker.pose.orientation.w=1; marker.scale.x=marker.scale.y=marker.scale.z=local_->voxelSize();
    marker.color.r=1; marker.color.g=.55; marker.color.b=.05; marker.color.a=.65;
    for (const auto &p:local_->centers(sim_time_)) marker.points.push_back(pointMsg(p));
    lidar_marker_pub_->publish(marker); return true;
  }
  void brake(const std::string &reason,double now) {
    stop_reason_=reason; ++lidar_stops_;
    RCLCPP_WARN(get_logger(),"Lidar stop: %s",reason.c_str());
    setState("BRAKING"); brake_at_=replan_at_=now; brake_sim_time_=sim_time_; key(" ");
  }
  bool lidarGuard(double now,bool updated) {
    if (!lidar_enabled_ || (state_!="TAKEOFF" && state_!="TRACKING" && state_!="SETTLING" && state_!="HOLDING")) return false;
    if (!lidarReady(now)) {
      if (state_=="TAKEOFF") fail("Lidar data lost during takeoff"); else brake("Lidar data/TF timed out",now);
      return true;
    }
    if (!updated) return false;
    if (state_=="TRACKING") {
      const double elapsed=std::max(0.0,sim_time_-spline_start_-.15);
      if (!plans_[leg_].spline.certified(*obstacles_,elapsed,elapsed+lookahead_)) {
        brake("Point cloud intersects upcoming trajectory",now); return true;
      }
    } else if (state_=="TAKEOFF" && phase()=="ACTIVE") {
      const Vec3 actual=enu(feedback("position_ned")-home_);
      World column=*obstacles_; column.low.z()=std::min(0.0,actual.z());
      if (!column.segmentFree(actual,{0,0,height_})) { fail("Lidar obstacle intersects takeoff column"); return true; }
    } else if ((state_=="SETTLING" || state_=="HOLDING") && !obstacles_->free(enu(feedback("position_ned")-home_))) {
      brake("Point cloud entered hover safety radius",now); return true;
    }
    return false;
  }
  bool settled(const Vec3 &target) {
    const bool stable=(enu(feedback("position_ned")-home_)-target).norm()<.15 && feedback("velocity_ned").norm()<.15;
    if (!stable) stable_since_=-1; else if (stable_since_<0) stable_since_=sim_time_;
    return stable_since_>=0 && sim_time_-stable_since_>1;
  }
  bool sendSpline(bool checked=false) {
    const auto &spline=plans_[leg_].spline;
    if (lidar_enabled_ && (!lidarReady(wallTime()) || (!checked && !spline.certified(*obstacles_)))) {
      if (state_!="REPLANNING") brake("Current point cloud blocks the next leg",wallTime());
      return false;
    }
    uav_common_msg::msg::BSplineTrajectory msg;
    msg.header.frame_id="px4_ned"; msg.header.stamp=stamp();
    const auto identity=static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());
    trajectory_id_=std::max(identity,trajectory_id_+1); msg.trajectory_id=trajectory_id_; msg.degree=7;
    const double start=sim_time_+.5; msg.start_time.sec=static_cast<int32_t>(start);
    msg.start_time.nanosec=static_cast<uint32_t>((start-msg.start_time.sec)*1e9);
    msg.knots=spline.knots;
    for (const auto &p:spline.points) {
      const Vec3 value=enu(p)+home_; for (int i=0;i<3;++i) msg.control_points.push_back(value[i]);
    }
    trajectory_pub_->publish(msg); spline_start_=start; spline_end_=start+spline.duration(); sent_at_=wallTime();
    setState("TRACKING"); return true;
  }
  void replanTick(double now) {
    if (phase()!="ACTIVE") { fail("Controller left ACTIVE while avoiding obstacle"); return; }
    if (now-replan_at_>replan_timeout_) { fail("Lidar avoidance could not find a safe route before timeout"); return; }
    if (!lidarReady(now) && now-cloud_at_>10) { fail("Lidar data/TF did not recover within 10 s"); return; }
    if (state_=="BRAKING") {
      if (flag("trajectory_active")) {
        if (now-brake_at_>2) fail("Controller did not acknowledge lidar stop");
      } else if (sim_time_-brake_sim_time_>=2.3 && settled(enu(feedback("target_ned")-home_))) setState("REPLANNING");
      return;
    }
    if (!lidarReady(now)) return;
    const Vec3 start=enu(feedback("target_ned")-home_);
    if (!obstacles_->free(start)) return;
    if (leg_>=plans_.size()) { setState("HOLDING"); return; }
    if ((start-waypoints_[leg_]).norm()<.05) { setState("SETTLING"); settle_at_=now; return; }
    if (future_.valid()) {
      if (future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready) return;
      try {
        auto candidate=future_.get();
        if (!candidate.spline.certified(*obstacles_)) throw std::runtime_error("New observations invalidate candidate route");
        plans_[leg_]=std::move(candidate);
        if (sendSpline(true)) {
          last_replan_error_.clear();
          ++replans_; auto info=planInfo(plans_[leg_]);
          info["leg"]=leg_+1; info["sim_time"]=sim_time_; info["reason"]=stop_reason_;
          replan_history_.push_back(info); publishPlan();
        }
      } catch (const std::runtime_error &e) {
        last_replan_error_=e.what(); writeReplanFailure();
        if (now-last_route_warning_>2) {
          RCLCPP_WARN(get_logger(),"Waiting for a safe lidar route: %s; start ENU=(%.2f,%.2f,%.2f), local voxels=%zu",
                      e.what(),replan_start_.x(),replan_start_.y(),replan_start_.z(),local_->size());
          last_route_warning_=now;
        }
      }
      return;
    }
    if (now-last_replan_>=1) {
      last_replan_=now; const auto snapshot=obstacles_; const auto goal=waypoints_[leg_]; const auto limits=limits_;
      replan_snapshot_=snapshot; replan_start_=start; replan_goal_=goal;
      const double margin=replan_margin_;
      future_=std::async(std::launch::async,[snapshot,start,goal,limits,margin](){ return planLocal(*snapshot,start,goal,limits,margin); });
    }
  }
  void writeReplanFailure() const {
    if (result_file_.empty() || !replan_snapshot_) return;
    Json boxes=Json::array();
    for (const auto &b:replan_snapshot_->boxes) boxes.push_back({{"name",b.name},{"low",vectorJson(b.low)},{"high",vectorJson(b.high)}});
    const Json snapshot={{"error",last_replan_error_},{"sim_time",sim_time_},{"start",vectorJson(replan_start_)},
        {"goal",vectorJson(replan_goal_)},{"map_min",vectorJson(replan_snapshot_->low)},{"map_max",vectorJson(replan_snapshot_->high)},
        {"radius",replan_snapshot_->radius},{"resolution",replan_snapshot_->resolution},{"boxes",boxes},
        {"yaw_alignment_rad",yaw_offset_},{"translation_alignment_m",vectorJson(lidar_translation_)}};
    fs::path file(result_file_); file.replace_extension(".replan_failure.json");
    std::ofstream out(file); out<<snapshot.dump(2)<<'\n';
  }
  PathMsg path(const Points &points,size_t maximum=std::numeric_limits<size_t>::max()) const {
    PathMsg msg; msg.header.frame_id="odom"; msg.header.stamp=stamp();
    const size_t first=points.size()>maximum?points.size()-maximum:0;
    for (size_t i=first;i<points.size();++i) {
      geometry_msgs::msg::PoseStamped pose; pose.header=msg.header; pose.pose.position=pointMsg(points[i]+enu(home_));
      pose.pose.orientation.w=1; msg.poses.push_back(pose);
    }
    return msg;
  }
  void publishPlan() {
    Points raw,smooth;
    for (const auto &p:plans_) {
      raw.insert(raw.end(),p.raw.begin(),p.raw.end());
      for (int i=0;i<160;++i) smooth.push_back(p.spline.sample(p.spline.duration()*i/159));
    }
    astar_pub_->publish(path(raw)); spline_pub_->publish(path(smooth));
    visualization_msgs::msg::MarkerArray markers;
    auto marker=[&](const std::string &ns,int id,int type,const Vec3 &p,const Vec3 &scale,std::array<float,4> color) {
      Marker m; m.header.frame_id="odom"; m.header.stamp=stamp(); m.ns=ns; m.id=id; m.type=type; m.action=Marker::ADD;
      m.pose.position=pointMsg(p+enu(home_)); m.pose.orientation.w=1;
      m.scale.x=scale.x(); m.scale.y=scale.y(); m.scale.z=scale.z();
      m.color.r=color[0]; m.color.g=color[1]; m.color.b=color[2]; m.color.a=color[3]; return m;
    };
    for (size_t i=0;i<world_->boxes.size();++i) {
      const auto &box=world_->boxes[i]; const Vec3 center=(box.low+box.high)/2, size=box.high-box.low;
      markers.markers.push_back(marker("walls",i,Marker::CUBE,center,size,{.65,.68,.72,.22}));
      markers.markers.push_back(marker("inflation",i,Marker::CUBE,center,size.array()+2*world_->radius,{1,.2,.15,.08}));
    }
    for (size_t i=0;i<waypoints_.size();++i) {
      markers.markers.push_back(marker("waypoints",i,Marker::SPHERE,waypoints_[i],Vec3::Constant(.3),{1,.2,.8,1}));
      auto label=marker("labels",i,Marker::TEXT_VIEW_FACING,waypoints_[i]+Vec3(0,0,.6),{0,0,.4},{1,.9,.9,1});
      label.text="WP"+std::to_string(i+1); markers.markers.push_back(label);
    }
    markers_pub_->publish(markers);
  }
  void fail(const std::string &reason) {
    if (done_ || state_=="FAILED" || state_=="ABORTING") return;
    error_=reason; RCLCPP_ERROR(get_logger(),"%s",reason.c_str());
    if (have_status_ && (flag("armed") || phase()=="PRESTREAM" || phase()=="WAIT_ARM" || phase()=="WAIT_OFFBOARD")) {
      setState("ABORTING"); abort_at_=wallTime(); key("l");
    } else { setState("FAILED"); done_=true; }
  }
  Json report() const {
    Json waypoints=Json::array(), plans=Json::array();
    for (const auto &p:waypoints_) waypoints.push_back(vectorJson(p));
    for (const auto &p:plans_) plans.push_back(planInfo(p));
    return {{"state",state_},{"succeeded",succeeded_},{"error",error_},{"implementation","C++"},
      {"map_source","static SDF boxes; no lidar mapping"},{"waypoints_enu",waypoints},
      {"safety_radius_m",world_->radius},{"plans",plans},{"completed_waypoints",passed_},
      {"tracking_samples",tracking_samples_},
      {"tracking_rmse_m",tracking_samples_?Json(std::sqrt(squared_error_/tracking_samples_)):Json(nullptr)},
      {"tracking_max_error_m",tracking_samples_?Json(max_error_):Json(nullptr)},
      {"minimum_measured_wall_clearance_m",tracking_samples_?Json(min_clearance_):Json(nullptr)},
      {"landed_disarmed",have_status_ && flag("landed") && !flag("armed")},
      {"lidar_avoidance",{{"enabled",lidar_enabled_},{"topic",lidar_topic_},{"ready",lidarReady(wallTime())},
          {"accepted_frames",cloud_frames_},{"local_voxels",local_->size()},{"last_error",cloud_error_},
          {"stops",lidar_stops_},{"replans",replans_},{"last_stop_reason",stop_reason_},
          {"yaw_alignment_rad",yaw_offset_},{"translation_alignment_m",vectorJson(lidar_translation_)},
          {"wall_alignment_inliers",alignment_inliers_},{"wall_alignment_rms_m",alignment_rms_},
          {"last_replan_error",last_replan_error_},{"replan_history",replan_history_}}}};
  }
  void writeReport() {
    const auto result=report(); String msg; msg.data=result.dump(); status_pub_->publish(msg);
    if (!result_file_.empty()) {
      fs::path temp(result_file_); temp.replace_extension(".tmp");
      { std::ofstream stream(temp); if (!stream) throw std::runtime_error("Cannot write mission report"); stream<<result.dump(2)<<'\n'; }
      fs::rename(temp,result_file_); trace_.flush();
    }
  }
  void tick() {
    const double now=wallTime(); alive_pub_->publish(std_msgs::msg::Empty{});
    const bool lidar_updated=updateLidar(now); if (gcs_) gcs_->tick(now);
    if (now-last_visual_>1) { publishPlan(); actual_pub_->publish(path(actual_,12000)); reference_pub_->publish(path(references_,12000)); last_visual_=now; }
    if (now-last_report_>1 || done_) { writeReport(); last_report_=now; }
    if (interrupted_) {
      interrupted_=false;
      if (state_=="PREVIEW") { done_=true; return; }
      if (state_=="HOLDING") { key("l"); setState("LANDING"); landing_at_=now; }
      else fail("Operator requested stop");
    }
    if (done_ || state_=="PREVIEW") return;
    if (state_=="ABORTING") {
      if (have_status_ && now-status_at_<1 && phase()=="IDLE" && !flag("armed")) { setState("FAILED"); done_=true; }
      else if (now-abort_at_>45) { error_+="; landing acknowledgment timed out"; done_=true; }
      else if (now-last_action_>1) key("l");
      return;
    }
    if (state_=="WAIT_READY") {
      if (now-started_at_>90) fail("Controller/PX4/lidar not ready within 90 s: "+cloud_error_);
      else if (have_status_ && now-status_at_<1 && controller_.value("reference_source",std::string{})!="planner") fail("Controller must use reference_source:=planner");
      else if (have_status_ && now-status_at_<1 && flag("ready") && flag("preflight") && !flag("armed") && phase()=="IDLE"
               && (!lidar_enabled_ || lidarReady(now))) {
        if (std::abs(feedback("target_ned").z()-feedback("home_ned").z())>.1) { fail("Controller has an unexpected existing reference"); return; }
        key("t"); setState("TAKEOFF"); takeoff_at_=now;
      }
      return;
    }
    if (!have_status_ || now-status_at_>2 || now-clock_at_>2) { fail("Controller feedback or simulation clock timed out"); return; }
    if (phase()=="YIELDED") { error_="PX4 left Offboard; mission yielded control"; setState("FAILED"); done_=true; return; }
    if (lidarGuard(now,lidar_updated)) return;
    if (state_=="TAKEOFF") {
      if (now-takeoff_at_>60) fail("Takeoff did not settle within 60 s");
      else if (phase()=="ACTIVE" && settled({0,0,height_})) sendSpline();
    } else if (state_=="TRACKING") {
      if (phase()!="ACTIVE") fail("Controller left ACTIVE during tracking");
      else if (controller_.at("trajectory_id").get<uint64_t>()!=trajectory_id_) {
        if (now-sent_at_>2) fail("Controller did not accept spline within 2 s");
      } else if (sim_time_>=spline_end_) { setState("SETTLING"); settle_at_=now; }
      else if (!flag("trajectory_active") && sim_time_<spline_end_-.25) fail("Trajectory canceled before completion");
    } else if (state_=="SETTLING") {
      if (settled(waypoints_[leg_])) {
        passed_.push_back(++leg_);
        if (leg_<plans_.size()) sendSpline();
        else if (land_at_end_) { key("l"); setState("LANDING"); landing_at_=now; }
        else setState("HOLDING");
      } else if (now-settle_at_>20) fail("Waypoint did not settle within 20 s");
    } else if (state_=="BRAKING" || state_=="REPLANNING") replanTick(now);
    else if (state_=="LANDING") {
      if (phase()=="IDLE" && !flag("armed") && flag("landed")) { succeeded_=true; setState("COMPLETE"); done_=true; }
      else if (now-landing_at_>45) fail("Landing did not complete within 45 s");
      else if (now-last_action_>1) key("l");
    }
  }

  std::shared_ptr<World> world_,obstacles_;
  std::shared_ptr<const World> replan_snapshot_;
  std::unique_ptr<LocalObstacles> local_;
  std::unique_ptr<GcsHeartbeat> gcs_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::future<Plan> future_;
  std::vector<Plan> plans_;
  Points waypoints_,actual_,references_;
  Vec3 home_=Vec3::Zero();
  Vec3 lidar_translation_=Vec3::Zero(),replan_start_=Vec3::Zero(),replan_goal_=Vec3::Zero();
  Json controller_,replan_history_=Json::array();
  std::vector<size_t> passed_;
  std::ofstream trace_;
  std::string state_="PLANNING",error_,result_file_,lidar_topic_,lidar_frame_,cloud_error_="No lidar data",stop_reason_,last_replan_error_;
  bool auto_start_=false,land_at_end_=true,lidar_enabled_=false,done_=false,succeeded_=false,interrupted_=false,have_status_=false;
  bool align_yaw_=true,yaw_aligned_=false;
  bool register_walls_=true;
  double replan_margin_=.2,alignment_rms_=0,last_route_warning_=0;
  size_t alignment_inliers_=0;
  double height_=2.5,lidar_range_=8,lidar_min_range_=.45,lidar_ground_=.15,lidar_timeout_=.5,lookahead_=6,replan_timeout_=30;
  double spawn_yaw_=0,yaw_offset_=0;
  std::array<double,3> limits_{};
  double sim_time_=0,clock_at_=0,status_at_=0,started_at_=0,last_action_=0,last_visual_=0,last_report_=0;
  double stable_since_=-1,last_trace_=-1,spline_start_=0,spline_end_=0,sent_at_=0,brake_at_=0,brake_sim_time_=0,replan_at_=0;
  double last_replan_=0,last_lidar_check_=0,cloud_at_=0,pending_at_=0,last_cloud_warning_=0;
  double takeoff_at_=0,settle_at_=0,landing_at_=0,abort_at_=0,squared_error_=0,max_error_=0,min_clearance_=std::numeric_limits<double>::infinity();
  size_t leg_=0,tracking_samples_=0,cloud_frames_=0,lidar_stops_=0,replans_=0;
  uint64_t trajectory_id_=0;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_cloud_;
  rclcpp::Publisher<String>::SharedPtr key_pub_,status_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr alive_pub_;
  rclcpp::Publisher<uav_common_msg::msg::BSplineTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<PathMsg>::SharedPtr astar_pub_,spline_pub_,actual_pub_,reference_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<Marker>::SharedPtr lidar_marker_pub_;
  rclcpp::Subscription<String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr debug_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace uav_path_planner

namespace { volatile std::sig_atomic_t stop_requested=0; void signalStop(int) { stop_requested=1; } }
int main(int argc,char **argv) {
  if (argc==3 && std::string(argv[1])=="--takeoff-height") {
    try {
      const auto params=YAML::LoadFile(argv[2])["waypoint_mission"]["ros__parameters"];
      const double height=params["takeoff_height"].as<double>(2.5);
      if (!std::isfinite(height) || height<.8 || height>3) throw std::runtime_error("takeoff_height must be between 0.8 and 3 m");
      std::cout<<height<<'\n'; return 0;
    } catch (const std::exception &e) { std::cerr<<e.what()<<'\n'; return 2; }
  }
  rclcpp::init(argc,argv,rclcpp::InitOptions{},rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,signalStop); std::signal(SIGTERM,signalStop);
  int result=1;
  try {
    auto node=std::make_shared<uav_path_planner::Mission>();
    rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
    while (rclcpp::ok() && !node->done()) {
      if (stop_requested) { stop_requested=0; node->interrupt(); }
      executor.spin_once(std::chrono::milliseconds(100));
    }
    node->close(); result=node->success()?0:1;
    executor.remove_node(node);
  } catch (const std::exception &e) { RCLCPP_ERROR(rclcpp::get_logger("waypoint_mission"),"%s",e.what()); }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
