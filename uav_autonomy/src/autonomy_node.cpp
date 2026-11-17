#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <uav_common_msg/msg/external_reference.hpp>
#include <uav_common_msg/msg/trajectory_feedback.hpp>

#include "uav_autonomy/observed_map.hpp"
#include "uav_autonomy/route_search.hpp"
#include "uav_autonomy/exploration_evidence.hpp"
#include "uav_autonomy/trajectory.hpp"
#include "uav_autonomy/viewing_move.hpp"
#include "uav_autonomy/observation_viewpoint.hpp"
#include "uav_ganyu_control/ganyu.hpp"

using namespace std::chrono_literals;
using namespace uav_autonomy;
using Json = nlohmann::json;

class AutonomyNode : public rclcpp::Node {
 public:
  AutonomyNode() : Node("autonomy"), tf_(get_clock()), listener_(tf_) {
    task_ = declare_parameter<std::string>("task", "forest");
    if (task_ != "forest" && task_ != "explore" && task_ != "patrol")
      throw std::runtime_error("task must be forest, explore or patrol");
    auto_start_ = declare_parameter<bool>("auto_start", false);
    input_ = declare_parameter<std::string>("cloud_topic", "/lidar/points");
    origin_frame_ = declare_parameter<std::string>("sensor_origin_frame", "");
    output_ = declare_parameter<std::string>("output_directory", "/tmp/gazebo_autonomy");
    minimum_clearance_ = declare_parameter<double>("safety_radius", .65);
    uncertainty_margin_ = declare_parameter<double>("obstacle_uncertainty_margin", .25);
    if (minimum_clearance_ < .65 || uncertainty_margin_ < 0)
      throw std::runtime_error("Minimum clearance must be at least 0.65 m and margin nonnegative");
    radius_ = minimum_clearance_ + uncertainty_margin_;
    speed_ = declare_parameter<double>("max_velocity", .5);
    if (radius_ < .65 || speed_ <= 0 || speed_ > 1)
      throw std::runtime_error("Invalid safety radius or speed");
    takeoff_height_ = declare_parameter<double>("takeoff_height", 1.5);
    flight_floor_ = declare_parameter<double>("flight_min_height", task_ == "forest" ? 1.2 : .8);
    flight_ceiling_ = declare_parameter<double>("flight_max_height", task_ == "forest" ? 2.8 : 8.2);
    execution_min_z_ = declare_parameter<double>("execution_min_z", -1e6);
    execution_max_z_ = declare_parameter<double>("execution_max_z", 1e6);
    if (!std::isfinite(flight_floor_) || !std::isfinite(flight_ceiling_) ||
        flight_floor_ < .65 || flight_ceiling_ - flight_floor_ < .5 ||
        takeoff_height_ < flight_floor_ || takeoff_height_ > flight_ceiling_ ||
        !std::isfinite(execution_min_z_) || !std::isfinite(execution_max_z_) || execution_min_z_ >= execution_max_z_)
      throw std::runtime_error("Takeoff and mission heights must fit the configured flight envelope");
    launch_radius_ = declare_parameter<double>("launch_clearance_radius", 0.0);
    launch_height_ = declare_parameter<double>("launch_clearance_height", 0.0);
    if (launch_radius_ < 0 || launch_height_ < 0)
      throw std::runtime_error("Launch clearance must be nonnegative");
    goal_ = vectorParam("goal", {80, 0, 1.8});
    mission_timeout_ = declare_parameter<double>("mission_timeout", 900);
    explore_timeout_ = declare_parameter<double>("exploration_timeout", 600);
    map_ = std::make_shared<ObservedMap>(task_ == "forest",
                                         task_ == "forest" ? Vec(20, 20, 8) : Vec(24, 32, 12), .2);
    std::filesystem::create_directories(output_);
    events_.open(output_ + "/events.jsonl", std::ios::app);
    normalized_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/autonomy/cloud", rclcpp::SensorDataQoS());
    raw_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/autonomy/raw_cloud",
                                                               rclcpp::SensorDataQoS());
    occupied_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/autonomy/occupied", 1);
    free_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/autonomy/observed_free", 1);
    state_pub_ = create_publisher<nav_msgs::msg::Odometry>("/autonomy/planner_odom", 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/autonomy/goal", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/autonomy/global_path", 1);
    reference_pub_ =
        create_publisher<uav_common_msg::msg::ExternalReference>("/autonomy/reference", 10);
    feedback_pub_ = create_publisher<uav_common_msg::msg::TrajectoryFeedback>(
        "/autonomy/trajectory_feedback", 10);
    brake_pub_ = create_publisher<std_msgs::msg::Empty>("/autonomy/brake", 10);
    key_pub_ = create_publisher<std_msgs::msg::String>("/ganyu/keyboard", 10);
    alive_pub_ = create_publisher<std_msgs::msg::Empty>("/ganyu/keyboard_alive", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/autonomy/status", 10);
    limit_pub_ = create_publisher<std_msgs::msg::Float64>("/autonomy/speed_limit", 10);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
          raw_pub_->publish(*m);
          double stamp = rclcpp::Time(m->header.stamp).seconds();
          if (last_input_stamp_ > 0 && stamp > last_input_stamp_ &&
              std::abs(stamp - now().seconds()) < .2) {
            period_ = period_samples_++ ? .85 * period_ + .15 * (stamp - last_input_stamp_)
                                        : stamp - last_input_stamp_;
          }
          last_input_stamp_ = stamp;
          input_frame_ = m->header.frame_id;
          pending_ = m;
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/autonomy/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          if (m->header.frame_id != "odom") return;
          auto &p = m->pose.pose.position;
          position_ = Vec(p.x, p.y, p.z);
          auto &q = m->pose.pose.orientation;
          orientation_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
          auto &v = m->twist.twist.linear;
          velocity_ = orientation_ * Vec(v.x, v.y, v.z);
          yaw_ = std::atan2(orientation_.toRotationMatrix()(1, 0),
                            orientation_.toRotationMatrix()(0, 0));
          odom_stamp_ = rclcpp::Time(m->header.stamp).seconds();

          nav_msgs::msg::Odometry out = *m;
          // EGO's native odometry API expects world velocity despite the nav_msgs convention.
          out.twist.twist.linear.x = velocity_.x();
          out.twist.twist.linear.y = velocity_.y();
          out.twist.twist.linear.z = velocity_.z();
          state_pub_->publish(out);
          if (!have_home_) {
            home_ = position_;
            have_home_ = true;
            if (launch_radius_ > 0 && launch_height_ > 0)
              map_->launchPad(home_, launch_radius_, launch_height_);
          }
        });
    controller_sub_ = create_subscription<std_msgs::msg::String>(
        "/ganyu/status", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {
          try {
            controller_ = Json::parse(m->data);
          } catch (const std::exception &) {
          }
        });
    state_status_sub_ = create_subscription<std_msgs::msg::String>(
        "/autonomy/state_status", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {
          try {
            auto j = Json::parse(m->data);
            if (j.value("reason", "") == "ekf_origin_reset") {
              if ((phase_ == "WAIT_DATA" || phase_ == "TAKEOFF" || phase_ == "WAIT_TAKEOFF_DATA") &&
                  j.value("displacement", 1.0) < .03 && j.value("rotation_jump", 1.0) < .35) {
                // Initial EKF heading alignment invalidates earlier rays. It
                // cannot be carried into the mission map, even at a fixed pose.
                stop("initial_attitude_alignment_reset");
                map_ = std::make_shared<ObservedMap>(task_ == "forest",
                    task_ == "forest" ? Vec(20, 20, 8) : Vec(24, 32, 12), .2);
                if (launch_radius_ > 0 && launch_height_ > 0)
                  map_->launchPad(home_, launch_radius_, launch_height_);
                discard_mapping_result_ = mapping_.valid();
                minimum_cloud_stamp_ = now().seconds();
                valid_clouds_ = 0;
                last_points_ = 0;
                last_valid_cloud_ = 0;
                pending_.reset();
                if (!active()) key("c");
                phase_ = active() ? "WAIT_TAKEOFF_DATA" : "WAIT_DATA";
                event("initial_alignment_map_rebuilt", {{"rotation_jump", j.value("rotation_jump", 0.0)}});
              } else
                fail("localization_origin_reset");
            }
          } catch (const std::exception &) {
          }
        });
    mission_command_sub_ = create_subscription<std_msgs::msg::String>(
        "/autonomy/mission_command", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {
          if (!active() || !dataReady(now().seconds())) {
            event("mission_command_rejected", {{"reason", "vehicle_or_map_not_ready"}});
            return;
          }
          try {
            Json command = Json::parse(m->data);
            if (command.value("action", "") != "patrol")
              throw std::runtime_error("Unsupported mission command");
            stop("operator_patrol_request");
            if (!command.contains("points")) {
              finishExploration("operator_end_exploration");
              return;
            }
            Points points;
            for (const auto &p : command["points"]) {
              if (p.size() != 3) throw std::runtime_error("Patrol points require ENU XYZ");
              Vec point(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
              if (!point.allFinite() || !heightSafe(point) || !map_->safe(point, radius_))
                throw std::runtime_error("Patrol point is outside the execution envelope or lacks measured clearance");
              points.push_back(point);
            }
            if (points.size() < 4) throw std::runtime_error("At least four patrol points required");
            double lowest = points.front().z(), highest = lowest;
            for (size_t i = 0; i < points.size(); ++i) {
              lowest = std::min(lowest, points[i].z());
              highest = std::max(highest, points[i].z());
              for (size_t j = 0; j < i; ++j)
                if ((points[i] - points[j]).norm() < .5)
                  throw std::runtime_error("Patrol points must be distinct by at least 0.5 m");
            }
            if (highest - lowest < .3)
              throw std::runtime_error("Patrol requires different inspection heights");
            requested_patrol_ = std::move(points);
            finishExploration("operator_end_exploration");
          } catch (const std::exception &e) {
            event("mission_command_rejected", {{"reason", e.what()}});
          }
        });
    native_sub_ = create_subscription<uav_common_msg::msg::NativeTrajectory>(
        "/autonomy/native_trajectory", 10,
        [this](uav_common_msg::msg::NativeTrajectory::ConstSharedPtr m) { receiveTrajectory(*m); });
    control_feedback_sub_ = create_subscription<uav_common_msg::msg::TrajectoryFeedback>(
        "/autonomy/controller_feedback", 10,
        [this](uav_common_msg::msg::TrajectoryFeedback::ConstSharedPtr m) {
          if (m->accepted && trajectory_ && m->trajectory_id == last_id_ && pending_native_ack_) {
            feedback(true, false, "controller_tracking", pending_native_ack_);
            pending_native_ack_ = 0;
          }
          if (!m->accepted) {
            stop("controller_" + m->reason);
            if (phase_ == "TURN") {
              turn_initialized_ = false;
              turn_at_ = now().seconds();
            }
          }
        });
    timer_ = create_wall_timer(5ms, [this]() { tick(); });
    event("ready", {{"task", task_},
                    {"preview", !auto_start_},
                    {"map_capacity", map_->capacity()},
                    {"configured_launch_clearance", {launch_radius_, launch_height_}}});
  }

 private:
  Vec vectorParam(const std::string &name, std::vector<double> def) {
    auto p = declare_parameter<std::vector<double>>(name, def);
    if (p.size() != 3) throw std::runtime_error(name + " must contain XYZ");
    return Vec(p[0], p[1], p[2]);
  }
  void event(const std::string &name, Json detail = Json::object()) {
    detail["event"] = name;
    detail["sim_time"] = now().seconds();
    events_ << detail.dump() << '\n';
    events_.flush();
    RCLCPP_INFO(get_logger(), "%s", detail.dump().c_str());
  }
  bool active() const {
    return controller_.value("phase", "") == "ACTIVE" && controller_.value("armed", false);
  }
  void key(const std::string &s) {
    std_msgs::msg::String m;
    m.data = s;
    key_pub_->publish(m);
  }
  void processCloud(double time) {
    if (mapping_.valid()) {
      if (mapping_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
      try {
        auto result = mapping_.get();
        if (discard_mapping_result_) {
          discard_mapping_result_ = false;
          return;
        }
        map_ = std::move(result.map);
        for (auto it = observation_views_.begin(); it != observation_views_.end();) {
          if (!map_->inside(map_->point(*it)))
            it = observation_views_.erase(it);
          else
            ++it;
        }
        last_cloud_ = result.stamp;
        last_points_ = result.points.size();
        finite_ratio_ = result.ratio;
        mapping_ms_ = result.milliseconds;
        input_points_ = result.input_points;
        actual_voxel_ = result.voxel;
        if (phase_ == "OBSERVE" && result.stamp >= observe_start_) {
          observed_bearings_ += result.bearing_sum;
          observed_bearing_count_ += result.bearing_count;
        }
        if (result.points.empty())
          error_ = "empty_measurement";
        else {
          ++valid_clouds_;
          last_valid_cloud_ = result.stamp;
          error_.clear();
          normalized_pub_->publish(xyzMessage(result.points, result.header));
        }
      } catch (const std::exception &e) {
        error_ = e.what();
        event("cloud_rejected", {{"reason", error_}});
      }
    }
    if (!pending_ || !have_home_) return;
    const auto m = pending_;
    const double stamp = rclcpp::Time(m->header.stamp).seconds();
    if (stamp < minimum_cloud_stamp_) {
      pending_.reset();
      return;
    }
    if (m->header.frame_id.empty() || stamp <= 0 || stamp <= last_cloud_) {
      pending_.reset();
      error_ = "cloud_frame_or_timestamp";
      return;
    }
    if (stamp > time + .03) {
      if (stamp > time + 2) {
        pending_.reset();
        error_ = "cloud_clock_mismatch";
      }
      return;
    }
    if (time - stamp > std::max(.5, 4 * period_)) {
      pending_.reset();
      error_ = "stale_cloud";
      return;
    }
    try {
      std::string origin = origin_frame_.empty() ? m->header.frame_id : origin_frame_;
      if (origin == "odom" || origin == "map" || origin == "world" || origin == "earth")
        throw std::runtime_error("World cloud requires sensor_origin_frame");
      auto world_cloud = tf2::transformToEigen(
          tf_.lookupTransform("odom", m->header.frame_id, rclcpp::Time(m->header.stamp)));
      auto world_sensor =
          tf2::transformToEigen(tf_.lookupTransform("odom", origin, rclcpp::Time(m->header.stamp)));
      auto world_body = tf2::transformToEigen(
          tf_.lookupTransform("odom", "base_link", rclcpp::Time(m->header.stamp)));
      // Mapping runs away from the 200 Hz reference timer. Readers retain an
      // immutable snapshot while the next bounded map is built.
      auto previous = map_;
      Vec center = position_;
      mapping_ = std::async(std::launch::async, [m, world_cloud, world_sensor, world_body, previous,
                                                 center, stamp]() {
        auto began = std::chrono::steady_clock::now();
        CloudResult result;
        Points points;
        auto body_world = world_body.inverse(), sensor_world = world_sensor.inverse();
        auto stats = visitXYZ(*m, [&](const Vec &p) {
          Vec w = world_cloud * p, b = body_world * w;
          if ((b.array().abs() < Vec(.39, .39, .21).array()).all()) return;
          Vec ray = world_body.linear().transpose() * (w - world_sensor.translation());
          double horizontal = ray.head<2>().norm();
          if (horizontal > 1e-6) {
            result.bearing_sum.head<2>() += ray.head<2>() / horizontal;
            ++result.bearing_count;
          }
          points.push_back(sensor_world * w);
        });
        double voxel = .15;
        points = spatialSample(points, voxel);
        while (points.size() > 15000) {
          voxel *= 1.5;
          points = spatialSample(points, voxel);
        }
        for (auto &p : points) p = world_sensor * p;
        result.map = std::make_shared<ObservedMap>(*previous);
        result.map->move(center);
        result.map->update(points, world_sensor.translation());
        result.points = std::move(points);
        result.header = m->header;
        result.header.frame_id = "odom";
        result.stamp = stamp;
        result.ratio = stats.total ? double(stats.finite) / stats.total : 0;
        result.input_points = stats.total;
        result.voxel = voxel;
        result.milliseconds =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began)
                .count();
        return result;
      });
      pending_.reset();
    } catch (const tf2::TransformException &) {
      error_ = "waiting_for_sample_time_tf";
    } catch (const std::exception &e) {
      error_ = e.what();
      pending_.reset();
      event("cloud_rejected", {{"reason", error_}});
    }
  }
  bool dataReady(double t) const {
    const double sensor_timeout = std::max(.3, 3.5 * period_);
    const double map_timeout =
        std::max(sensor_timeout, std::min(2.0, sensor_timeout + 2 * mapping_ms_ * .001));
    return !clock_fault_latched_ && (error_.empty() || error_ == "waiting_for_sample_time_tf") && have_home_ &&
           valid_clouds_ >= 3 && t - odom_stamp_ < .3 && last_points_ > 0 &&
           t - last_input_stamp_ < sensor_timeout && t - last_valid_cloud_ < map_timeout;
  }
  bool brakeSafe(const Sample &s, double t, bool reserve = true) const {
    // Forest surface returns can extend into an adjacent voxel during a stop.
    // Leave one extra measured-obstacle voxel along native stopping tubes.
    // Independent recovery moves retain the original complete clearance test
    // so they can leave a base-safe hold after its reserve was consumed.
    auto clear = [&](const Vec &p) {
      return reserve ? nativeStoppingSafe(p) : map_->safe(p, radius_);
    };
    try {
      uav_ganyu_control::Reference r{s.p, s.v, s.a, s.j};
      auto b = uav_ganyu_control::brakingReference(r, t);
      for (double dt = 0; dt <= b.second; dt += .08)
        if (!heightSafe(b.first.sample(t + dt).p) || !clear(b.first.sample(t + dt).p)) {
          support_failure_point_ = b.first.sample(t + dt).p;
          return false;
        }
      if (!heightSafe(b.first.goal()) || !clear(b.first.goal())) {
        support_failure_point_ = b.first.goal();
        return false;
      }
      return true;
    } catch (const std::exception &) {
      support_brake_failed_ = true;
      return false;
    }
  }
  double lowerHeight() const { return std::max(home_.z() + flight_floor_, execution_min_z_); }
  double upperHeight() const { return std::min(home_.z() + flight_ceiling_, execution_max_z_); }
  bool heightSafe(const Vec &p) const { return p.z() >= lowerHeight() && p.z() <= upperHeight(); }
  bool nativeStoppingSafe(const Vec &p) const {
    // The exact brake must stop away from the hard height boundary as well
    // as measured obstacles. This is tracking reserve, not a relaxed envelope.
    const double margin = task_ == "forest" ? .1 : 0.0;
    return p.z() >= lowerHeight() + margin && p.z() <= upperHeight() - margin &&
           map_->stoppingSafe(p, radius_);
  }
  bool supported(const NativeTrajectory &tr, double t) const {
    support_brake_failed_ = false;
    double elapsed = std::max(0.0, t - tr.start());
    // Every executed prefix has an independently checked full stopping manoeuvre.
    for (int i = 0; i <= 3; ++i) {
      double d = i * .05;
      auto s = tr.sample(elapsed + d);
      if (!heightSafe(s.p) || !map_->safe(s.p, radius_)) {
        support_failure_point_ = s.p;
        return false;
      }
      if (!brakeSafe(s, t + d)) return false;
    }
    return true;
  }
  void receiveTrajectory(const uav_common_msg::msg::NativeTrajectory &m) {
    double t = now().seconds();
    // Stop feedback already revoked execution permission. Late packets from
    // that goal must not cancel the next goal during a turn or view selection.
    if (phase_ != "TRANSIT" && phase_ != "EXPLORE" && phase_ != "PATROL" && phase_ != "RETURN" &&
        phase_ != "REPOSITION")
      return;
    const double previous_speed_limit = speed_;
    try {
      if (!active() || !dataReady(t)) throw std::runtime_error("not_ready_to_execute");
      auto tr = std::make_unique<NativeTrajectory>(m);
      if (m.trajectory_id <= last_native_id_ || tr->start() < t - .2 || tr->start() > t + .5)
        throw std::runtime_error("trajectory_id_or_start_time");
      double envelope_until = INFINITY;
      for (double dt = 0; dt <= tr->duration(); dt += .02) {
        auto s = tr->sample(dt);
        if (!heightSafe(s.p) && !std::isfinite(envelope_until)) {
          event("trajectory_envelope_violation", {{"id", m.trajectory_id},
              {"time_from_start", dt}, {"position", {s.p.x(), s.p.y(), s.p.z()}},
              {"height_bounds", {lowerHeight(), upperHeight()}}});
          // EGO-v2 deliberately collision-checks only a receding prefix of
          // non-terminal plans. Retain its exact geometry/time, but expire
          // execution before an invalid future tail; never execute that tail.
          envelope_until = tr->start() + dt - .05;
        }
        if (s.v.norm() > speed_ + 1e-6 || s.a.norm() > .7 + 1e-6 || s.j.norm() > 1.5 + 1e-6) {
          event("trajectory_limit_violation", {{"id", m.trajectory_id},
                                               {"time_from_start", dt},
                                               {"speed", s.v.norm()},
                                               {"acceleration", s.a.norm()},
                                               {"jerk", s.j.norm()},
                                               {"speed_limit", speed_},
                                               {"acceleration_limit", .7},
                                               {"jerk_limit", 1.5}});
          speed_ = std::max(.18, speed_ * .8);
          throw std::runtime_error("trajectory_dynamic_limits_replan_slower");
        }
      }
      if (envelope_until < t + .5)
        throw std::runtime_error("trajectory_outside_flight_envelope");
      if (!supported(*tr, t)) {
        if (support_brake_failed_) {
          speed_ = std::max(.18, speed_ * .8);
          throw std::runtime_error("no_feasible_braking_reference_replan_slower");
        }
        speed_ = std::max(.18, speed_ * .8);
        if (!trajectory_ && ++unsupported_count_ >= 2)
          waitForObservation("trajectory_lacks_observed_braking_corridor");
        throw std::runtime_error("trajectory_lacks_observed_braking_corridor");
      }
      unsupported_count_ = 0;
      last_native_id_ = m.trajectory_id;
      ++last_id_;
      trajectory_ = std::move(tr);
      trajectory_valid_until_ = envelope_until;
      if (std::isfinite(envelope_until))
        event("trajectory_execution_horizon", {{"id", m.trajectory_id}, {"valid_until", envelope_until}});
      pending_native_ack_ = m.trajectory_id;
    } catch (const std::exception &e) {
      // A failed replacement is not a revocation of an already accepted
      // trajectory. Keep its exact reference only while the latest data,
      // execution horizon and complete stopping corridor still support it.
      if (trajectory_) {
        if (speed_ + 1e-6 >= previous_speed_limit && active() && dataReady(t) && t + .2 < trajectory_valid_until_ &&
            t < trajectory_->start() + trajectory_->duration() && supported(*trajectory_, t))
          event("replacement_rejected_current_retained", {{"rejected_id", m.trajectory_id},
              {"retained_native_id", last_native_id_}, {"reason", e.what()}});
        else
          stop(std::string("native_rejected_") + e.what());
      }
      feedback(false, false, e.what(), m.trajectory_id);
    }
  }
  void feedback(bool accepted, bool braking, const std::string &why, uint64_t id = 0) {
    uav_common_msg::msg::TrajectoryFeedback m;
    m.header.stamp = now();
    m.header.frame_id = "odom";
    m.trajectory_id = id ? id : last_id_;
    m.accepted = accepted;
    m.braking = braking;
    m.reason = why;
    feedback_pub_->publish(m);
    if (!accepted) event("trajectory_rejected", {{"id", m.trajectory_id}, {"reason", why}});
  }
  void stop(const std::string &why) {
    pending_native_ack_ = 0;
    if (trajectory_ || (!guard_stopped_ && active())) {
      brake_pub_->publish(std_msgs::msg::Empty());
      trajectory_.reset();
      trajectory_valid_until_ = INFINITY;
      guard_stopped_ = true;
      feedback(false, true, why);
      event("brake", {{"reason", why}});
    }
    if (phase_ != "FAILED_HOVER") reason_ = why;
  }
  void fail(const std::string &why) {
    stop(why);
    phase_ = "FAILED_HOVER";
    reason_ = why;
    saveResult(false);
  }
  void saveResult(bool success) {
    Json j = {{"success", success},
              {"task", task_},
              {"phase", phase_},
              {"reason", reason_},
              {"known_cells", map_->known()},
              {"map_capacity", map_->capacity()},
              {"completed_patrol_points", patrol_index_},
              {"patrol_generation", patrol_generation_},
              {"patrol_replans", patrol_replans_},
              {"patrol_auto_generated", patrol_auto_generated_},
              {"sim_time", now().seconds()},
              {"exploration_end_reason", explore_end_},
              {"goal_enu", {goal_.x(), goal_.y(), goal_.z()}},
              {"home_enu", {home_.x(), home_.y(), home_.z()}},
              {"patrol_points_enu", Json::array()},
              {"coverage_verified", false}};
    for (const auto &p : patrol_) j["patrol_points_enu"].push_back({p.x(), p.y(), p.z()});
    std::ofstream(output_ + "/result.json") << j.dump(2) << '\n';
  }
  void publishReference(const Sample &s, double yaw, double yaw_rate, double t) {
    guard_stopped_ = false;
    uav_common_msg::msg::ExternalReference m;
    m.header.frame_id = "odom";
    m.header.stamp = rclcpp::Time(int64_t(t * 1e9));
    m.trajectory_id = last_id_;
    m.valid_until = rclcpp::Time(int64_t(std::min(t + .12, trajectory_ ? trajectory_valid_until_ : INFINITY) * 1e9));
    auto arr = [](const Vec &p) { return std::array<double, 3>{p.x(), p.y(), p.z()}; };
    m.position = arr(s.p);
    m.velocity = arr(s.v);
    m.acceleration = arr(s.a);
    m.jerk = arr(s.j);
    m.yaw = yaw;
    m.yaw_rate = yaw_rate;
    reference_pub_->publish(m);
  }
  void sendGoal(const Vec &p) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = now();
    msg.pose.position.x = p.x();
    msg.pose.position.y = p.y();
    msg.pose.position.z = p.z();
    msg.pose.orientation.w = 1;
    goal_pub_->publish(msg);
    last_goal_ = p;
    last_goal_time_ = now().seconds();
  }
  Vec controllerGoal() const {
    if (controller_.contains("target_ned") && controller_["target_ned"].size() == 3) {
      const auto &p = controller_["target_ned"];
      return Vec(p[1].get<double>(), p[0].get<double>(), -p[2].get<double>());
    }
    return position_;
  }
  Vec planningOrigin() {
    if (heightSafe(position_) && map_->safe(position_, radius_)) return position_;
    const Vec hold = controllerGoal();
    double enclosure = holdEnclosureRadius(position_ - hold, orientation_.toRotationMatrix());
    if (active() && velocity_.norm() < .08 && (position_ - hold).norm() <= .05 &&
        enclosure <= .98 && heightSafe(hold) && map_->safe(hold, radius_)) {
      event("supported_hold_search_origin", {{"position", {position_.x(), position_.y(), position_.z()}},
          {"hold", {hold.x(), hold.y(), hold.z()}}, {"physical_enclosure_radius", enclosure}});
      return hold;
    }
    return position_;
  }
  void navigate(const Vec &p, const std::string &mission_phase) {
    stop("turn_to_observe_route");
    mission_final_ = p;
    if (task_ != "forest" &&
        (mission_phase == "PATROL" || mission_phase == "RETURN" || mission_phase == "EXPLORE")) {
      resume_phase_ = mission_phase;
      phase_ = "WAIT_ROUTE";
      last_goal_ = p;
      turn_at_ = now().seconds();
      return;
    }
    beginTurn(p, mission_phase);
  }
  void beginTurn(const Vec &guide, const std::string &mission_phase) {
    // A forest observation/recovery ends the previous native attempt. Its
    // wait-for-output interval must not include time spent moving/scanning.
    // Mission time, best-goal progress and observation retry limits stay intact.
    if (task_ == "forest") native_wait_since_ = 0;
    last_goal_ = guide;
    resume_phase_ = mission_phase;
    phase_ = "TURN";
    Vec d = guide - position_;
    turn_yaw_ = d.head<2>().norm() > .1 ? std::atan2(d.y(), d.x()) - observation_bearing_ : yaw_;
    heading_ref_ = yaw_;
    turn_position_ = position_;
    turn_at_ = now().seconds();
    turn_initialized_ = false;
    ++last_id_;
  }
  void waitForObservation(const std::string &why) {
    if (phase_ == "WAIT_OBSERVE" || phase_ == "OBSERVE") return;
    resume_phase_ = phase_;
    stop(why);
    phase_ = "WAIT_OBSERVE";
    turn_at_ = now().seconds();
    if (++observation_attempts_ > 3) {
      fail("insufficient_observation_after_scanning");
      return;
    }
    auto evidence = map_->evidence(support_failure_point_, radius_);
    event("observation_requested",
          {{"reason", why},
           {"attempt", observation_attempts_},
           {"unsupported_position",
            {support_failure_point_.x(), support_failure_point_.y(), support_failure_point_.z()}},
           {"unsupported_height_safe", heightSafe(support_failure_point_)},
           {"unsupported_stopping_safe", nativeStoppingSafe(support_failure_point_)},
           {"unknown_cells", evidence.unknown},
           {"unknown_swept_body_cells", evidence.unknown_body},
           {"nearest_unknown",
            {evidence.nearest_unknown.x(), evidence.nearest_unknown.y(),
             evidence.nearest_unknown.z()}},
           {"occupied_cells", evidence.occupied}});
  }
  void chooseObservationViewpoint() {
    observation_resume_phase_ = task_=="forest" ? "TRANSIT" :
        ((resume_phase_=="PATROL" || resume_phase_=="RETURN") ? resume_phase_ : "EXPLORE");
    stop("change_observation_viewpoint");
    phase_ = "PLAN_OBSERVATION";
    observation_views_.insert(map_->key(position_));
    while (observation_views_.size() > 512) observation_views_.erase(observation_views_.begin());
    auto map = map_;
    Vec position = planningOrigin(), goal = task_=="forest" ? goal_ : mission_final_, hold = controllerGoal();
    auto visited = observation_views_;
    double radius = radius_;
    double floor = lowerHeight(), ceiling = upperHeight();
    view_search_ =
        std::async(std::launch::async, [map, position, goal, hold, visited, radius, floor, ceiling,
                                      speed = speed_, indoor = task_ != "forest"]() -> Points {
          return observationViewpoint(map, position, hold, goal, visited, radius, floor, ceiling, speed, indoor);
        });
  }
  void retireIndoorViewpoint(const std::string &reason = "observation_did_not_support_native_corridor") {
    visited_.insert({int(std::floor(mission_final_.x()/1.2)),
                     int(std::floor(mission_final_.y()/1.2)),
                     int(std::floor(mission_final_.z()/1.2))});
    event("frontier_viewpoint_retired", {{"position",
          {mission_final_.x(),mission_final_.y(),mission_final_.z()}},
          {"reason",reason}});
  }
  void chooseIndoorGoal() {
    stop("frontier_viewpoint_reached");
    phase_ = "PLAN_FRONTIER";
    turn_position_ = controllerGoal();
    auto map = map_;
    auto position = planningOrigin();
    auto visited = visited_;
    auto audited_views = low_gain_.views();
    double radius = radius_;
    std::string task = task_;
    double floor = lowerHeight(), ceiling = upperHeight();
    frontier_search_ = std::async(std::launch::async, [map, position, visited, audited_views, radius, task, floor, ceiling]() {
      return searchRoute(map, position, Vec::Zero(), true, visited, radius, task, floor, ceiling, audited_views);
    });
  }
  void beginObservation(double t) {
    phase_ = "OBSERVE";
    observe_start_ = t;
    observe_yaw_ = yaw_;
    observe_position_ = controllerGoal();
    observed_bearings_.setZero();
    observed_bearing_count_ = 0;
    observed_known_before_ = map_->known();
  }
  void beginObservationMove(double t) {
    // A separate small viewing manoeuvre, not a refit of an EGO trajectory.
    // This lets the airframe leave a conservative/old native-map cell using
    // current measured clearance. The main A->B path remains native MINCO.
    const Vec from = controllerGoal();
    if (!map_->stoppingSafe(last_goal_, radius_)) {
      support_failure_point_ = last_goal_;
      waitForObservation("viewing_target_lost_stopping_reserve");
      return;
    }
    observation_move_.reset(from, t);
    observation_move_duration_ = std::max(4.0, 2.2 * (last_goal_ - from).norm() / speed_);
    observation_move_.moveTo(last_goal_, t, observation_move_duration_);
    observation_move_start_ = t;
    for (double dt = 0; dt <= observation_move_duration_ + .03; dt += .03) {
      auto r = observation_move_.sample(t + dt);
      if (!heightSafe(r.p) || !map_->safe(r.p, radius_) || r.v.norm() > speed_ * 1.01 || r.a.norm() > .7 ||
          r.j.norm() > 1.5) {
        waitForObservation("viewing_manoeuvre_lacks_measured_clearance");
        return;
      }
    }
    phase_ = "OBS_MOVE";
    event("viewing_manoeuvre_started",
          {{"duration", observation_move_duration_},
           {"goal", {last_goal_.x(), last_goal_.y(), last_goal_.z()}}});
  }
  void useIndoorGoal(RouteResult result) {
    Points path = std::move(result.path);
    frontier_count_ = result.clusters;
    event("frontier_search", {{"clusters", frontier_count_},
                              {"path_points", path.size()},
                              {"position", {position_.x(), position_.y(), position_.z()}},
                              {"current_safe", map_->safe(position_, radius_)},
                              {"safe_xp", map_->safe(position_ + Vec(.4, 0, 0), radius_)},
                              {"safe_xn", map_->safe(position_ - Vec(.4, 0, 0), radius_)},
                              {"safe_yp", map_->safe(position_ + Vec(0, .4, 0), radius_)},
                              {"safe_yn", map_->safe(position_ - Vec(0, .4, 0), radius_)}});
    if (path.empty()) {
      if (++no_frontier_ >= 4)
        finishExploration("no_reachable_frontier");
      else {
        phase_ = "WAIT_OBSERVE";
        turn_at_ = now().seconds();
      }
      return;
    }
    // Selecting a path is not physical progress: a later connection recheck
    // may reject it. Reset the exhaustion counter only after measured motion.
    Vec target = path.back();
    nav_msgs::msg::Path out;
    out.header.frame_id = "odom";
    out.header.stamp = now();
    for (const Vec &p : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = out.header;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();
      pose.pose.orientation.w = 1;
      out.poses.push_back(pose);
    }
    path_pub_->publish(out);
    // Use a visible, collision-checked guide point along the global connection.
    Vec guide = path[std::min(size_t(5), path.size() - 1)];
    navigate(guide, "EXPLORE");
    mission_final_ = target;
  }
  void finishExploration(const std::string &why) {
    stop("exploration_end");
    explore_end_ = why;
    patrol_repair_pending_ = false;
    patrol_replans_ = 0;
    phase_ = "WAIT_PATROL_START";
    turn_at_ = now().seconds();
  }
  void buildPatrol() {
    const Vec origin = planningOrigin();
    if (!map_->safe(origin, radius_)) {
      waitForObservation("patrol_start_requires_fresh_observation");
      return;
    }
    if (!patrol_repair_pending_) {
      if (!map_->save(output_ + "/warehouse.bt")) {
        fail("map_save_failed");
        return;
      }
      event("exploration_ended",
            {{"reason", explore_end_}, {"known_cells", map_->known()}, {"map_complete", false}});
    }
    phase_ = "BUILD_PATROL";
    auto map = map_;
    Vec position = origin;
    double radius = radius_;
    turn_position_ = controllerGoal();
    Points requested = std::move(requested_patrol_);
    if (!patrol_repair_pending_) patrol_auto_generated_ = requested.empty();
    double floor = lowerHeight(), ceiling = upperHeight();
    patrol_search_ = std::async(std::launch::async, [map, position, radius, requested, floor, ceiling]() {
      if (!requested.empty()) {
        for (const auto &point : requested)
          if (searchRoute(map, position, point, false, {}, radius, "explore", floor, ceiling).path.empty())
            return Points{};
        return requested;
      }
      Points points;
      Points free = spatialSample(map->free(), .5);
      std::sort(free.begin(), free.end(), [&](const Vec &a, const Vec &b) {
        return (a - position).squaredNorm() < (b - position).squaredNorm();
      });
      size_t attempts = 0;
      for (const Vec &p : free) {
        // Reserve one voxel beyond execution clearance for automatically
        // selected stationary targets as later measurements refine surfaces.
        if (p.z() < floor + .1 || p.z() > ceiling - .1 || (p - position).norm() < 1.5 ||
            !map->safe(p, radius + map->resolution())) continue;
        bool different = true;
        for (const auto &q : points)
          if ((p - q).head<2>().norm() < 1.5 || std::abs(p.z() - q.z()) < .35) different = false;
        if (different && ++attempts <= 40 &&
            !searchRoute(map, points.empty() ? position : points.back(), p, false, {}, radius, "explore", floor, ceiling).path.empty())
          points.push_back(p);
        if (points.size() == 4 || attempts >= 40) break;
      }
      return points;
    });
  }
  void usePatrol(Points points) {
    if (points.size() < 4) {
      fail("insufficient_reachable_patrol_points");
      return;
    }
    if (patrol_generation_ > 0) {
      std::error_code error;
      if (!std::filesystem::copy_file(output_ + "/patrol_points.json",
          output_ + "/patrol_points.revision" + std::to_string(patrol_generation_ - 1) + ".json", error)) {
        fail("patrol_archive_failed");
        return;
      }
    }
    patrol_ = std::move(points);
    patrol_index_ = 0;
    ++patrol_generation_;
    Json pts = Json::array();
    for (const auto &p : patrol_) pts.push_back({p.x(), p.y(), p.z()});
    std::ofstream(output_ + "/patrol_points.json") << pts.dump(2) << '\n';
    if (patrol_repair_pending_)
      event("auto_patrol_replanned", {{"generation", patrol_generation_}, {"points", pts},
          {"completed_points", 0}, {"saved_exploration_map_preserved", true}});
    patrol_repair_pending_ = false;
    support_failure_point_ = Vec::Constant(std::numeric_limits<double>::quiet_NaN());
    force_observation_view_ = false;
    observation_attempts_ = 0;
    observation_views_.clear();
    navigate(patrol_[0], "PATROL");
  }
  bool handleBlockedPatrolTarget() {
    if (task_ == "forest" || patrol_index_ >= patrol_.size()) return false;
    const Vec target = patrol_[patrol_index_];
    if (map_->safe(target, radius_)) return false;
    const auto evidence = map_->evidence(target, radius_);
    // Unknown support can still be resolved by observing. A measured occupied
    // endpoint inside the clearance envelope invalidates the target itself.
    if (evidence.occupied == 0) return false;
    support_failure_point_ = target;
    if (!patrol_auto_generated_) {
      fail("requested_patrol_target_blocked");
      return true;
    }
    if (patrol_replans_ >= 2) {
      fail("auto_patrol_targets_repeatedly_invalidated");
      return true;
    }
    event("auto_patrol_target_invalidated", {{"target", {target.x(), target.y(), target.z()}},
        {"occupied_cells", evidence.occupied}, {"completed_points_discarded", patrol_index_},
        {"previous_generation", patrol_generation_}, {"replan", patrol_replans_ + 1}});
    stop("auto_patrol_target_invalidated");
    ++patrol_replans_;
    patrol_repair_pending_ = true;
    force_observation_view_ = false;
    phase_ = "WAIT_PATROL_START";
    turn_at_ = now().seconds();
    return true;
  }
  void updateExplorationProgress(double t) {
    if (task_ == "forest" || !explore_end_.empty() || phase_ == "TAKEOFF" ||
        phase_ == "WAIT_DATA" || phase_ == "LANDING" || phase_ == "FAILED_HOVER")
      return;
    if (!have_exploration_extent_) {
      exploration_low_ = exploration_high_ = position_;
      have_exploration_extent_ = true;
    }
    exploration_low_ = exploration_low_.cwiseMin(position_);
    exploration_high_ = exploration_high_.cwiseMax(position_);
    if (!exploration_gain_.empty() && t - exploration_gain_.back().first < 1) return;
    size_t known = map_->known();
    exploration_gain_.emplace_back(t, known);
    exploration_positions_.emplace_back(t, position_);
    while (exploration_gain_.size() > 1 && exploration_gain_[1].first <= t - 45)
      exploration_gain_.pop_front();
    while (exploration_positions_.size() > 1 && exploration_positions_[1].first <= t - 45)
      exploration_positions_.pop_front();
    const size_t recent_gain=known-exploration_gain_.front().second;
    const size_t gain_budget=std::max(size_t(800),size_t(known*.002));
    if (t-exploration_gain_.front().first>=45 && low_gain_.windowGain(recent_gain,gain_budget))
      event("exploration_information_gain_recovered", {{"window_seconds",t-exploration_gain_.front().first},
          {"new_known_cells",recent_gain}});
    if (phase_ == "EXPLORE" && t - mission_start_ > 120 && t - exploration_gain_.front().first >= 45 &&
        (exploration_high_ - exploration_low_).norm() > 3 &&
        recent_gain < gain_budget) {
      event("information_gain_stalled",
            {{"window_seconds", t - exploration_gain_.front().first},
             {"new_known_cells", known - exploration_gain_.front().second},
             {"map_complete", false}});
      Vec low = position_, high = position_;
      for (const auto &entry : exploration_positions_) {
        low = low.cwiseMin(entry.second);
        high = high.cwiseMax(entry.second);
      }
      low_gain_.recordView(position_,known);
      event("low_gain_viewpoints", {{"distinct_views", low_gain_.views().size()},
              {"window_spatial_extent_m", (high - low).norm()},
              {"new_known_cells_since_first_view", known - low_gain_.firstKnown()}});
      // Slow execution may not traverse three metres within a single window.
      // Three separate, spatially distinct low-gain windows are an alternative
      // exhaustion criterion; substantial renewed gain resets that audit,
      // with hysteresis to reject near-threshold sampling fluctuations.
      if (low_gain_.exhausted((high-low).norm())) {
        finishExploration("low_observed_information_gain_multiple_viewpoints");
        return;
      }
      // Keep making progress along an observed connection. Repeatedly cancelling
      // it every low-gain window can prevent reaching a genuinely different
      // view. A stationary connection gets a new, spatially unaudited viewpoint.
      const bool continue_connection=low_gain_.continueConnection((position_-mission_final_).norm(),(high-low).norm());
      exploration_gain_.clear();
      exploration_positions_.clear();
      if (continue_connection) {
        event("continue_observed_frontier_connection", {{"target",{mission_final_.x(),mission_final_.y(),mission_final_.z()}},
            {"window_spatial_extent_m",(high-low).norm()}});
        return;
      }
      visited_.insert({int(std::floor(mission_final_.x() / 1.2)),
                       int(std::floor(mission_final_.y() / 1.2)),
                       int(std::floor(mission_final_.z() / 1.2))});
      chooseIndoorGoal();
    }
  }
  void tick() {
    const double t = now().seconds();
    if (last_tick_>0 && t<last_tick_-1e-6) {
      // Pausing is harmless; moving to an earlier clock epoch invalidates all
      // old goals, TF history and execution deadlines. Never resume implicitly.
      last_tick_=t;last_slow_=t-.21;pending_.reset();
      clock_fault_latched_=true;discard_mapping_result_=mapping_.valid();
      error_="simulation_clock_moved_backwards";
      fail("simulation_clock_moved_backwards");
      return;
    }
    if (t <= last_tick_) return;
    const double control_dt = std::clamp(t - last_tick_, .0001, .03);
    last_tick_ = t;
    processCloud(t);
    if (t - last_slow_ > .2) {
      last_slow_ = t;
      alive_pub_->publish(std_msgs::msg::Empty());
      auto evidence = map_->evidence(position_, radius_);
      auto xyz = [](const Vec &v) { return Json::array({v.x(), v.y(), v.z()}); };
      Json j = {{"phase", phase_},
                {"reason", reason_},
                {"cloud_error", error_},
                {"input_frame", input_frame_},
                {"cloud_period", period_},
                {"valid_ratio", finite_ratio_},
                {"mapping_ms", mapping_ms_},
                {"cloud_age", t - last_valid_cloud_},
                {"odom_age", t - odom_stamp_},
                {"points", last_points_},
                {"input_points", input_points_},
                {"spatial_voxel", actual_voxel_},
                {"frequency_hz", 1 / period_},
                {"valid_cloud_frames", valid_clouds_},
                {"data_ready", dataReady(t)},
                {"clock_fault_latched", clock_fault_latched_},
                {"current_safe", map_->safe(position_, radius_)},
                {"planning_speed_limit_mps", speed_},
                {"minimum_obstacle_clearance_m", minimum_clearance_},
                {"obstacle_uncertainty_margin_m", uncertainty_margin_},
                {"planning_obstacle_clearance_m", radius_},
                {"native_stopping_obstacle_reserve_m", task_ == "forest" ? map_->resolution() : 0.0},
                {"native_stopping_height_reserve_m", task_ == "forest" ? .1 : 0.0},
                {"flight_height_envelope_m", {flight_floor_, flight_ceiling_}},
                {"execution_z_bounds", {lowerHeight(), upperHeight()}},
                {"patrol_index", patrol_index_},
                {"patrol_generation", patrol_generation_},
                {"patrol_auto_generated", patrol_auto_generated_},
                {"patrol_replans", patrol_replans_},
                {"observation_bearing_rad", observation_bearing_},
                {"observed_swept_ellipsoid_half_axes_m", {.5, .5, .35}},
                {"controller_hold_position", xyz(controllerGoal())},
                {"controller_hold_safe", map_->safe(controllerGoal(), radius_)},
                {"position", xyz(position_)},
                {"clearance_evidence",
                 {{"free", evidence.free},
                  {"occupied", evidence.occupied},
                  {"unknown", evidence.unknown},
                  {"unknown_swept_body", evidence.unknown_body},
                  {"unknown_halo", evidence.unknown_halo},
                  {"nearest_unknown", xyz(evidence.nearest_unknown)},
                  {"nearest_obstacle", xyz(evidence.nearest_obstacle)}}},
                {"known_cells", map_->known()},
                {"map_capacity", map_->capacity()},
                {"auto_start", auto_start_}};
      std_msgs::msg::String msg;
      msg.data = j.dump();
      status_pub_->publish(msg);
      std_msgs::msg::Float64 limit;
      // v2 constrains the vector norm; Swarm constrains each axis separately.
      // Swarm's feasibility terms are soft and its refinement can overshoot.
      // Reserve headroom in the requested plan, preserving execution limits.
      limit.data = task_ == "forest" ? speed_ : .8 * speed_ / std::sqrt(3.0);
      limit_pub_->publish(limit);
    }
    if (!auto_start_) return;
    if (phase_ == "FAILED_HOVER" || phase_ == "COMPLETE" ||
        phase_ == "COMPLETE_WITH_INCOMPLETE_MAP")
      return;
    if (controller_.value("phase", "") == "YIELDED") {
      fail("operator_left_offboard");
      return;
    }
    if (phase_ == "WAIT_DATA") {
      if (active()) {
        stop("startup_pointcloud_or_state_timeout");
        phase_ = "WAIT_TAKEOFF_DATA";
        return;
      }
      if (dataReady(t) && controller_.value("ready", false) &&
          controller_.value("phase", "") == "IDLE" && !controller_.value("armed", true) &&
          controller_.value("landed", false)) {
        if (!heightSafe(home_ + Vec(0, 0, takeoff_height_)) || (task_ == "forest" && !heightSafe(goal_))) {
          fail("takeoff_or_goal_outside_execution_envelope");
          return;
        }
        key("t");
        takeoff_command_at_ = t;
        takeoff_started_at_ = t;
        guard_stopped_ = false;
        phase_ = "TAKEOFF";
        mission_start_ = t;
        event("takeoff_requested");
      }
      return;
    }
    if ((phase_ == "TAKEOFF" || phase_ == "WAIT_TAKEOFF_DATA") && takeoff_started_at_ > 0) {
      // A request/settled reference and even PX4's landed flag do not prove
      // physical lift-off. Bound the whole attempt, including data-gap
      // resumes, rather than waiting out the hour-long exploration budget.
      const double elapsed = t - takeoff_started_at_;
      const double budget = std::max(45.0, 2 * std::max(12.0, 4.5 * takeoff_height_ + 1) + 10);
      const double lift = position_.z() - home_.z();
      const bool no_liftoff = elapsed > 15 && active() && dataReady(t) && lift < .3;
      if (no_liftoff || elapsed > budget) {
        event("takeoff_progress_timeout", {{"elapsed", elapsed}, {"budget", budget},
              {"measured_lift", lift}, {"requested_lift", takeoff_height_},
              {"controller_phase", controller_.value("phase", "")},
              {"controller_landed", controller_.value("landed", true)},
              {"controller_thrust_N", controller_.value("thrust_N", 0.0)}});
        if (!controller_.value("armed", true)) key("c"); // cancel any delayed arming request
        fail(no_liftoff ? "takeoff_no_liftoff" : "takeoff_did_not_reach_height");
        return;
      }
    }
    if (t - mission_start_ > mission_timeout_) {
      fail("mission_timeout");
      return;
    }
    if (active() && phase_ != "TAKEOFF" && phase_ != "LANDING") {
      double remaining = (position_ - goal_).norm();
      // Turning or shuttling sideways at a blocked passage must not keep an
      // A->B mission alive indefinitely. Indoor exploration has changing goals;
      // forest progress is measured against its best remaining goal distance.
      bool progressed = task_ == "forest" ? remaining < best_goal_distance_ - .25
                                            : (position_ - last_motion_position_).norm() > .25;
      if (last_motion_time_ == 0 || progressed) {
        last_motion_position_ = position_;
        last_motion_time_ = t;
        best_goal_distance_ = std::min(best_goal_distance_, remaining);
        observation_attempts_ = 0;
        no_frontier_ = 0;
      }
      // Four frontier checks may include three full 25 s scans plus settling.
      // Leave time for exhaustion to be reported and the saved map assessed.
      if (t - last_motion_time_ > 120) {
        fail("no_progress_observation_or_execution_blocked");
        return;
      }
    }
    if (phase_ == "TAKEOFF") {
      if (!dataReady(t)) {
        if (active()) {
          stop("takeoff_pointcloud_or_state_timeout");
          phase_ = "WAIT_TAKEOFF_DATA";
        } else {
          key("c");
          phase_ = "WAIT_DATA";
        }
        return;
      }
      if (t - takeoff_command_at_ > .5 && active() &&
          std::abs(position_.z() - home_.z() - takeoff_height_) < .2 &&
          controller_.value("internal_reference_settled", false) && velocity_.norm() < .08) {
        last_goal_time_ = t;
        heading_ref_ = yaw_;
        beginObservation(t);
      }
      return;
    }
    if (phase_ == "WAIT_TAKEOFF_DATA") {
      if (!controller_.value("armed", false) && controller_.value("phase", "") == "IDLE") {
        phase_ = "WAIT_DATA";
        return;
      }
      if (dataReady(t) && active() && velocity_.norm() < .08 &&
          controller_.value("internal_reference_settled", false)) {
        key("r");
        phase_ = "TAKEOFF";
        takeoff_command_at_ = t;
        guard_stopped_ = false;
        event("takeoff_resumed_after_fresh_observation");
      }
      return;
    }
    if (phase_ == "LANDING") {
      if (!controller_.value("armed", true) && controller_.value("landed", false)) {
        bool complete = explore_end_ != "exploration_time_budget";
        phase_ = complete ? "COMPLETE" : "COMPLETE_WITH_INCOMPLETE_MAP";
        reason_ = complete ? "mission_landed" : "exploration_time_budget";
        saveResult(complete);
        event("complete", {{"success", complete}});
      }
      return;
    }
    if (!active()) return;
    if (!dataReady(t)) {
      stop("pointcloud_or_state_timeout");
      if (phase_ == "OBS_MOVE" || phase_ == "OBSERVE" || phase_ == "TURN") {
        if (phase_ == "OBS_MOVE") resume_phase_ = observation_resume_phase_;
        phase_ = "WAIT_OBSERVE";
        turn_at_ = t;
      }
      return;
    }
    updateExplorationProgress(t);
    if (phase_ == "PATROL" && handleBlockedPatrolTarget()) return;
    if (phase_ == "WAIT_OBSERVE") {
      if (t - turn_at_ > 2.5 && velocity_.norm() < .08 &&
          controller_.value("internal_reference_settled", false)) {
        // A measured obstacle or the fixed height envelope cannot be made
        // clear by another stationary scan. Reposition using the existing
        // fully checked viewing manoeuvre instead of spending a full turn
        // at the same blocked hold. Unknown swept airframe space still
        // requires observation. Always scan after the viewing move itself.
        if (task_ == "forest" && observation_attempts_ > 0 &&
            !retry_native_after_view_ && support_failure_point_.allFinite() &&
            !nativeStoppingSafe(support_failure_point_)) {
          const auto evidence = map_->evidence(support_failure_point_, radius_ + map_->resolution());
          const bool height_blocked = support_failure_point_.z() < lowerHeight() + .1 ||
                                      support_failure_point_.z() > upperHeight() - .1;
          if (evidence.unknown_body == 0 && (height_blocked || evidence.occupied > 0)) {
            event("known_stopping_constraint_reposition",
                  {{"height_constraint", height_blocked},
                   {"occupied_cells", evidence.occupied},
                   {"unknown_swept_body_cells", evidence.unknown_body}});
            force_observation_view_ = false;
            chooseObservationViewpoint();
            return;
          }
        }
        beginObservation(t);
      }
      return;
    }
    if (phase_ == "TURN") {
      if (!turn_initialized_) {
        if (velocity_.norm() > .08 || t - turn_at_ < 2.5 ||
            !controller_.value("internal_reference_settled", false))
          return;
        turn_position_ = controllerGoal();
        turn_initialized_ = true;
        last_reference_time_ = t;
      }
      Sample hold;
      hold.p = turn_position_;
      double dt = std::min(.03, t - last_reference_time_);
      last_reference_time_ = t;
      double rate = std::clamp(
          std::remainder(turn_yaw_ - heading_ref_, 2 * M_PI) / std::max(dt, .001), -.35, .35);
      heading_ref_ += rate * dt;
      publishReference(hold, heading_ref_, rate, t);
      if (std::abs(std::remainder(yaw_ - turn_yaw_, 2 * M_PI)) < .15 && std::abs(rate) < .01) {
        phase_ = resume_phase_;
        if (phase_ == "OBS_MOVE")
          beginObservationMove(t);
        else
          sendGoal(last_goal_);
      }
      return;
    }
    if (phase_ == "OBSERVE") {
      Sample hold;
      hold.p = observe_position_;
      const double turn = std::min(2 * M_PI, (t - observe_start_) * .25);
      if (last_id_ == 0) ++last_id_;
      publishReference(hold, observe_yaw_ + turn, turn < 2 * M_PI ? .25 : 0, t);
      if (turn >= 2 * M_PI && std::abs(std::remainder(yaw_ - observe_yaw_, 2 * M_PI)) < .15) {
        double concentration = observed_bearing_count_
                                   ? observed_bearings_.head<2>().norm() / observed_bearing_count_
                                   : 0;
        // This only selects a useful viewing direction from actual returns.
        // It never estimates FOV or clears unobserved directions.
        observation_bearing_ =
            concentration > .25 ? std::atan2(observed_bearings_.y(), observed_bearings_.x()) : 0;
        event("observation_completed", {{"return_direction_rad", observation_bearing_},
                                        {"direction_concentration", concentration},
                                        {"return_rays", observed_bearing_count_},
                                        {"known_cells_before", observed_known_before_},
                                        {"known_cells_after", map_->known()}});
        if (task_ == "forest" && retry_native_after_view_) {
          // This point belonged to a rejected trajectory from the OLD view.
          // It may remain occluded, or leave the rolling window. After a
          // complete guarded move and scan, ask EGO for a fresh connection
          // from measured state instead of demanding visibility of that old
          // point forever. Do not clear map cells or reset retry/watchdog
          // budgets; every new trajectory still needs its own safe prefix
          // and complete stopping corridor.
          event("observation_replan_from_new_view",
                {{"previous_failure_supported", support_failure_point_.allFinite() &&
                    nativeStoppingSafe(support_failure_point_)},
                 {"previous_failure", {support_failure_point_.x(), support_failure_point_.y(),
                                       support_failure_point_.z()}}});
          retry_native_after_view_ = false;
          support_failure_point_ = Vec::Constant(std::numeric_limits<double>::quiet_NaN());
          force_observation_view_ = false;
        }
        bool need_view = force_observation_view_ ||
              (observation_attempts_ > 0 && support_failure_point_.allFinite() &&
               !nativeStoppingSafe(support_failure_point_));
        if (need_view && (task_=="forest" || resume_phase_=="EXPLORE" ||
                          resume_phase_=="PATROL" || resume_phase_=="RETURN")) {
          force_observation_view_ = false;
          if (task_!="forest" && resume_phase_=="EXPLORE") retireIndoorViewpoint();
          chooseObservationViewpoint();
        } else if (task_ == "forest")
          navigate(goal_, "TRANSIT");
        else if (resume_phase_ == "WAIT_PATROL_START") {
          // Resume selection after observing without resetting an automatic
          // repair's bounded retry count or replacing the saved map.
          stop("patrol_selection_resumed_after_observation");
          phase_ = "WAIT_PATROL_START";
          turn_at_ = t;
        }
        else if (resume_phase_ == "PATROL" || resume_phase_ == "RETURN")
          navigate(mission_final_, resume_phase_);
        else
          chooseIndoorGoal();
      }
      return;
    }
    // Planning can take longer than one external-reference validity interval.
    // Retain the stationary turn reference until the first native trajectory
    // arrives; otherwise GanYu times out and cancels the planner's new goal.
    if (!trajectory_ && !guard_stopped_ && phase_ != "OBS_MOVE") {
      Sample hold;
      hold.p = turn_position_;
      publishReference(hold, heading_ref_, 0, t);
    }
    if (phase_ == "WAIT_PATROL_START") {
      if (velocity_.norm() < .08 && t - turn_at_ >= 2.5 &&
          controller_.value("internal_reference_settled", false)) buildPatrol();
      return;
    }
    if (phase_ == "WAIT_ROUTE") {
      if (velocity_.norm() > .08 || t - turn_at_ < 2.5 ||
          !controller_.value("internal_reference_settled", false)) return;
      if (resume_phase_ == "PATROL" && handleBlockedPatrolTarget()) return;
      auto map = map_;
      Vec position = planningOrigin(), target = last_goal_;
      double radius = radius_;
      auto task = task_;
      double floor = lowerHeight(), ceiling = upperHeight();
      phase_ = "PLAN_ROUTE";
      route_search_ = std::async(std::launch::async, [map, position, target, radius, task, floor, ceiling]() {
        return searchRoute(map, position, target, false, {}, radius, task, floor, ceiling);
      });
      return;
    }
    if (phase_ == "PLAN_ROUTE") {
      if (route_search_.valid() && route_search_.wait_for(0ms) == std::future_status::ready) {
        auto result = route_search_.get();
        if (result.path.empty()) {
          event("connection_invalidated", {{"mission_phase", resume_phase_},
                                           {"target", {last_goal_.x(), last_goal_.y(), last_goal_.z()}}});
          if (resume_phase_ == "EXPLORE") {
            // Do not repeatedly select the same unexecuted frontier after a
            // scan or centimetre-scale hover drift invalidates its connector.
            retireIndoorViewpoint("connection_changed_before_execution");
            chooseIndoorGoal();
          }
          else {
            if (resume_phase_ == "PATROL" && handleBlockedPatrolTarget()) return;
            // The failed connection concerns this mission target, rather than
            // an old unsupported point left over from exploration.
            support_failure_point_ = last_goal_;
            force_observation_view_ = true;
            auto resume = resume_phase_;
            waitForObservation("no_observed_connection_rescan");
            resume_phase_ = resume;
          }
        } else
          beginTurn(result.path[std::min(size_t(6), result.path.size() - 1)], resume_phase_);
      }
      return;
    }
    if (phase_ == "PLAN_FRONTIER") {
      if (frontier_search_.valid() && frontier_search_.wait_for(0ms) == std::future_status::ready)
        useIndoorGoal(frontier_search_.get());
      return;
    }
    if (phase_ == "BUILD_PATROL") {
      if (patrol_search_.valid() && patrol_search_.wait_for(0ms) == std::future_status::ready)
        usePatrol(patrol_search_.get());
      return;
    }
    if (phase_ == "PLAN_OBSERVATION") {
      if (view_search_.valid() && view_search_.wait_for(0ms) == std::future_status::ready) {
        auto points = view_search_.get();
        if (points.empty()) {
          if (task_!="forest" && observation_resume_phase_=="EXPLORE") {
            retireIndoorViewpoint("connection_changed_before_execution");
            chooseIndoorGoal();
          } else
            fail("no_observed_safe_observation_viewpoint");
        }
        else {
          auto point = points[std::min(size_t(1), points.size() - 1)];
          event("observation_viewpoint", {{"position", {point.x(), point.y(), point.z()}},
              {"resume_phase", observation_resume_phase_},
              {"mission_goal", {mission_final_.x(), mission_final_.y(), mission_final_.z()}}});
          // This is a temporary viewing move. navigate() assigns a new final
          // task goal; doing that here loses the inspection/home target when
          // the scan resumes PATROL or RETURN.
          beginTurn(point, "OBS_MOVE");
        }
      }
      return;
    }
    if (phase_ == "OBS_MOVE") {
      support_brake_failed_ = false;
      for (int i = 0; i <= 3; ++i) {
        auto r = observation_move_.sample(t + i * .05);
        Sample s;
        s.p = r.p;
        s.v = r.v;
        s.a = r.a;
        s.j = r.j;
        if (!heightSafe(s.p) || !map_->safe(s.p, radius_)) {
          support_failure_point_ = s.p;
          observation_views_.insert(map_->key(last_goal_));
          waitForObservation("viewing_manoeuvre_support_lost");
          return;
        }
        if (!brakeSafe(s, t + i * .05, false)) {
          // Preserve the failing stop position, rather than replacing it with
          // an often-safe point on the viewing curve itself.
          observation_views_.insert(map_->key(last_goal_));
          waitForObservation("viewing_manoeuvre_support_lost");
          return;
        }
      }
      auto r = observation_move_.sample(t);
      Sample s;
      s.p = r.p;
      s.v = r.v;
      s.a = r.a;
      s.j = r.j;
      publishReference(s, heading_ref_, 0, t);
      if (t - observation_move_start_ >= observation_move_duration_ &&
          (position_ - last_goal_).norm() < .15 && velocity_.norm() < .1) {
        stop("observation_viewpoint_reached");
        resume_phase_ = observation_resume_phase_;
        retry_native_after_view_ = task_ == "forest";
        phase_ = "WAIT_OBSERVE";
        turn_at_ = t;
        observation_views_.insert(map_->key(position_));
      }
      return;
    }
    if ((phase_=="TRANSIT" || phase_=="EXPLORE" || phase_=="PATROL" || phase_=="RETURN") && !trajectory_) {
      if (native_wait_since_ == 0) native_wait_since_ = t;
      // A planner can fail before publishing any candidate (for example when
      // its inflated start cell becomes occupied). Candidate rejection alone
      // cannot trigger observation in this case. Settle, scan, then search a
      // measured viewing move; never relax either map's safety constraints.
      if (t - native_wait_since_ > 15 && velocity_.norm() < .08) {
        support_failure_point_ = planningOrigin();
        force_observation_view_ = true;
        native_wait_since_ = 0;
        waitForObservation("native_planner_no_executable_trajectory");
        return;
      }
    } else {
      native_wait_since_ = 0;
    }
    if (trajectory_) {
      if (t + .2 >= trajectory_valid_until_) {
        stop("native_envelope_horizon_replan");
        return;
      }
      if (!supported(*trajectory_, t)) {
        if (support_brake_failed_) {
          speed_ = std::max(.18, speed_ * .8);
          stop("no_feasible_braking_reference_replan_slower");
          return;
        }
        speed_ = std::max(.18, speed_ * .8);
        stop("observation_limited_replan_slower");
        return;
      }
      auto sample = trajectory_->sample(t - trajectory_->start());
      Vec direction =
          trajectory_->sample(std::min(trajectory_->duration(), t - trajectory_->start() + 1)).p -
          sample.p;
      if (sample.v.head<2>().norm() > .05) direction = sample.v;
      double desired = direction.head<2>().norm() > .01
                           ? std::atan2(direction.y(), direction.x()) - observation_bearing_
                           : heading_ref_;
      double delta = std::clamp(std::remainder(desired - heading_ref_, 2 * M_PI), -.4 * control_dt,
                                .4 * control_dt);
      heading_ref_ += delta;
      if (std::abs(std::remainder(yaw_ - desired, 2 * M_PI)) > .35 && sample.v.norm() > .1) {
        auto phase = phase_;
        stop("turn_to_observe_trajectory");
        beginTurn(last_goal_, phase);
        turn_yaw_ = desired;
        return;
      }
      publishReference(sample, heading_ref_, delta / control_dt, t);
      if (t > trajectory_->start() + trajectory_->duration() + .1) {
        if (sample.v.norm() > .01 || sample.a.norm() > .01)
          stop("native_trajectory_ended_without_replacement");
        else {
          turn_position_ = sample.p;
          trajectory_.reset();
        }
      }
    }
    if (t - last_mission_ > .8) {
      last_mission_ = t;
      if (phase_ == "REPOSITION" && (position_ - last_goal_).norm() < .15 &&
          velocity_.norm() < .08) {
        stop("observation_viewpoint_reached");
        phase_ = "WAIT_OBSERVE";
        turn_at_ = t;
        observation_views_.insert(map_->key(position_));
      }
      if (phase_ == "EXPLORE" && (position_ - last_goal_).norm() < .3 && velocity_.norm() < .15) {
        if ((position_ - mission_final_).norm() > .35)
          navigate(mission_final_, "EXPLORE");
        else {
          Key cluster{int(std::floor(mission_final_.x() / 1.2)),
                      int(std::floor(mission_final_.y() / 1.2)),
                      int(std::floor(mission_final_.z() / 1.2))};
          visited_.insert(cluster);
          chooseIndoorGoal();
        }
      }
      if (phase_ == "EXPLORE" && t - mission_start_ > explore_timeout_)
        finishExploration("exploration_time_budget");
      if (phase_ == "TRANSIT" && (position_ - goal_).norm() < .15 && velocity_.norm() < .10) {
        phase_ = "LANDING";
        key("l");
      }
      if (phase_ == "PATROL" && (position_ - patrol_[patrol_index_]).norm() < .15 &&
          velocity_.norm() < .10) {
        if (++patrol_index_ < patrol_.size())
          navigate(patrol_[patrol_index_], "PATROL");
        else {
          navigate(home_ + Vec(0, 0, takeoff_height_), "RETURN");
        }
      } else if ((phase_ == "PATROL" || phase_ == "RETURN") &&
                 (position_ - last_goal_).norm() < .3 && velocity_.norm() < .15 &&
                 (position_ - mission_final_).norm() > .3) {
        auto phase = phase_;
        navigate(mission_final_, phase);
      }
      if (phase_ == "RETURN" && (position_ - (home_ + Vec(0, 0, takeoff_height_))).norm() < .15 &&
          velocity_.norm() < .10) {
        phase_ = "LANDING";
        key("l");
      }
      if (!trajectory_ && t - last_goal_time_ > 3 &&
          (phase_ == "TRANSIT" || phase_ == "EXPLORE" || phase_ == "PATROL" || phase_ == "RETURN"))
        sendGoal(last_goal_);
    }
    if (t - last_visual_ > 3) {
      last_visual_ = t;
      std_msgs::msg::Header h;
      h.frame_id = "odom";
      h.stamp = now();
      occupied_pub_->publish(xyzMessage(map_->occupied(), h));
      free_pub_->publish(xyzMessage(map_->free(), h));
    }
  }
  std::string input_frame_;
  std::string task_, input_, origin_frame_, output_, phase_ = "WAIT_DATA", reason_, error_,
                                                     explore_end_, resume_phase_;
  bool auto_start_, have_home_ = false, guard_stopped_ = false, clock_fault_latched_ = false;
  double minimum_clearance_, uncertainty_margin_, radius_, speed_, takeoff_height_, flight_floor_, flight_ceiling_, execution_min_z_, execution_max_z_, launch_radius_, launch_height_, mission_timeout_,
      explore_timeout_;
  Vec position_ = Vec::Zero(), velocity_ = Vec::Zero(), home_ = Vec::Zero(), goal_,
      last_goal_ = Vec::Zero(), mission_final_ = Vec::Zero();
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
  double yaw_ = 0, heading_ref_ = 0, observe_yaw_ = 0, observe_start_ = 0;
  double turn_yaw_ = 0, last_reference_time_ = 0, turn_at_ = 0;
  bool turn_initialized_ = false;
  Vec turn_position_ = Vec::Zero(), observe_position_ = Vec::Zero();
  double odom_stamp_ = 0, last_cloud_ = 0, last_valid_cloud_ = 0, period_ = .1, finite_ratio_ = 0,
         last_tick_ = 0, last_slow_ = 0;
  double mission_start_ = 0, last_goal_time_ = 0, last_mission_ = 0, last_visual_ = 0;
  double takeoff_command_at_ = 0, takeoff_started_at_ = 0;
  double last_motion_time_ = 0;
  double best_goal_distance_ = INFINITY;
  Vec last_motion_position_ = Vec::Zero();
  size_t last_points_ = 0, patrol_index_ = 0, frontier_count_ = 0;
  int no_frontier_ = 0, observation_attempts_ = 0, unsupported_count_ = 0;
  uint64_t last_id_ = 0, last_native_id_ = 0;
  Points patrol_, requested_patrol_;
  bool patrol_auto_generated_ = false, patrol_repair_pending_ = false;
  size_t patrol_generation_ = 0, patrol_replans_ = 0;
  std::unordered_set<Key, KeyHash> visited_;
  std::future<RouteResult> frontier_search_;
  std::future<RouteResult> route_search_;
  std::future<Points> patrol_search_;
  std::future<Points> view_search_;
  std::unordered_set<Key, KeyHash> observation_views_;
  mutable Vec support_failure_point_ = Vec::Constant(std::numeric_limits<double>::quiet_NaN());
  mutable bool support_brake_failed_ = false;
  uint64_t pending_native_ack_ = 0;
  uav_ganyu_control::SmoothReference observation_move_;
  double observation_move_start_ = 0, observation_move_duration_ = 0;
  double native_wait_since_ = 0;
  bool force_observation_view_ = false;
  bool retry_native_after_view_ = false;
  std::string observation_resume_phase_ = "TRANSIT";
  Json controller_ = Json::object();
  std::ofstream events_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  struct CloudResult {
    std::shared_ptr<ObservedMap> map;
    Points points;
    std_msgs::msg::Header header;
    double stamp = 0, ratio = 0, milliseconds = 0, voxel = 0;
    size_t input_points = 0;
    Vec bearing_sum = Vec::Zero();
    size_t bearing_count = 0;
  };
  std::shared_ptr<ObservedMap> map_;
  std::future<CloudResult> mapping_;
  bool discard_mapping_result_ = false;
  double minimum_cloud_stamp_ = 0;
  double last_input_stamp_ = 0, mapping_ms_ = 0;
  double actual_voxel_ = 0;
  Vec observed_bearings_ = Vec::Zero();
  size_t observed_bearing_count_ = 0, observed_known_before_ = 0;
  double observation_bearing_ = 0;
  std::deque<std::pair<double, size_t>> exploration_gain_;
  std::deque<std::pair<double, Vec>> exploration_positions_;
  Vec exploration_low_ = Vec::Zero(), exploration_high_ = Vec::Zero();
  bool have_exploration_extent_ = false;
  ExplorationEvidence low_gain_;
  size_t input_points_ = 0, valid_clouds_ = 0;
  size_t period_samples_ = 0;
  std::unique_ptr<NativeTrajectory> trajectory_;
  double trajectory_valid_until_ = INFINITY;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr normalized_pub_, raw_pub_,
      occupied_pub_, free_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<uav_common_msg::msg::ExternalReference>::SharedPtr reference_pub_;
  rclcpp::Publisher<uav_common_msg::msg::TrajectoryFeedback>::SharedPtr feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr brake_pub_, alive_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr key_pub_, status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr limit_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_sub_, state_status_sub_,
      mission_command_sub_;
  rclcpp::Subscription<uav_common_msg::msg::NativeTrajectory>::SharedPtr native_sub_;
  rclcpp::Subscription<uav_common_msg::msg::TrajectoryFeedback>::SharedPtr control_feedback_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AutonomyNode>());
  } catch (const std::exception &e) {
    std::fprintf(stderr, "autonomy: %s\n", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
