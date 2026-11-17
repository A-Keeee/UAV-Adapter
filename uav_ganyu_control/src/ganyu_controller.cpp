#include "uav_ganyu_control/ganyu.hpp"
#include "uav_ganyu_control/bspline.hpp"
#include <uav_common_msg/msg/b_spline_trajectory.hpp>
#include <uav_common_msg/msg/external_reference.hpp>
#include <uav_common_msg/msg/trajectory_feedback.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <functional>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

using namespace std::chrono_literals;
using namespace uav_ganyu_control;
namespace px4 = px4_msgs::msg;

struct ControllerLease {
  std::vector<int> descriptors;
  ~ControllerLease() { for (int fd : descriptors) ::close(fd); }
  void acquire(const std::vector<std::string> &outputs) {
    const char *runtime = std::getenv("XDG_RUNTIME_DIR");
    const char *domain = std::getenv("ROS_DOMAIN_ID");
    for (const auto &output : outputs) {
      const auto key = std::hash<std::string>{}(output);
      std::string path = std::string(runtime && *runtime ? runtime : "/tmp") +
        "/gazebo_ganyu_output_" + std::to_string(::getuid()) + "_" +
        std::to_string(domain ? std::strtoul(domain, nullptr, 10) : 0) + "_" + std::to_string(key) + ".lock";
      int fd = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
      if (fd < 0) throw std::runtime_error("PX4 control output lock is unavailable");
      if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
        ::close(fd);
        throw std::runtime_error("Another GanYu controller already owns this PX4 control output");
      }
      descriptors.push_back(fd);
    }
  }
};

class GanYuNode : public rclcpp::Node {
public:
  GanYuNode() : Node("ganyu_controller") {
    prefix_ = declare_parameter<std::string>("px4_namespace", "/px4_1");
    system_id_ = declare_parameter<int>("target_system", 2);
    takeoff_height_ = declare_parameter<double>("takeoff_height", 1.5);
    step_ = declare_parameter<double>("keyboard_step", 0.25);
    external_takeoff_duration_ = declare_parameter<double>("external_takeoff_duration", 12.0);
    reference_source_ = declare_parameter<std::string>("reference_source", "keyboard");
    if (reference_source_ != "keyboard" && reference_source_ != "planner" && reference_source_ != "external")
      throw std::runtime_error("reference_source must be keyboard, planner or external");
    controller_.params.mass = declare_parameter<double>("mass", 2.125307692307692);
    controller_.params.max_thrust = declare_parameter<double>("max_thrust", 30.0);
    motor_k_ = declare_parameter<double>("motor_constant", 8.54858e-6);
    speed_min_ = declare_parameter<double>("motor_speed_min", 150.0);
    speed_max_ = declare_parameter<double>("motor_speed_max", 1000.0);
    thrust_factor_ = declare_parameter<double>("thrust_model_factor", 0.0);
    if (!std::isfinite(external_takeoff_duration_) || external_takeoff_duration_<5 || external_takeoff_duration_>60 ||
        takeoff_height_ < 0.5 || takeoff_height_ > (reference_source_=="external" ? 6 : 3) || step_ <= 0 || step_ > 0.5 ||
        motor_k_ <= 0 || speed_min_ < 0 || speed_max_ <= speed_min_ ||
        thrust_factor_ < 0 || thrust_factor_ > 1 || controller_.params.mass <= 0 ||
        controller_.params.max_thrust < controller_.params.mass * 9.81) {
      throw std::runtime_error("Invalid vehicle / keyboard parameters");
    }
    // Resolve relative names, namespaces and remaps before claiming outputs.
    // target_system does not change the destination of a rates setpoint.
    output_lease_.acquire({
        get_node_topics_interface()->resolve_topic_name(prefix_ + "/fmu/in/vehicle_rates_setpoint"),
        get_node_topics_interface()->resolve_topic_name(prefix_ + "/fmu/in/offboard_control_mode"),
        get_node_topics_interface()->resolve_topic_name(prefix_ + "/fmu/in/vehicle_command")});
    rates_pub_ = create_publisher<px4::VehicleRatesSetpoint>(prefix_ + "/fmu/in/vehicle_rates_setpoint", 10);
    mode_pub_ = create_publisher<px4::OffboardControlMode>(prefix_ + "/fmu/in/offboard_control_mode", 10);
    command_pub_ = create_publisher<px4::VehicleCommand>(prefix_ + "/fmu/in/vehicle_command", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/ganyu/status", 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/ganyu/control_debug", 10);
    reference_debug_pub_ = create_publisher<uav_common_msg::msg::ExternalReference>("/ganyu/control_reference", 10);
    feedback_pub_ = create_publisher<uav_common_msg::msg::TrajectoryFeedback>("/autonomy/controller_feedback", 10);
    const auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<px4::VehicleOdometry>(prefix_ + "/fmu/out/vehicle_odometry", qos,
      [this](px4::VehicleOdometry::ConstSharedPtr msg) {
        if (msg->pose_frame != px4::VehicleOdometry::POSE_FRAME_NED) return;
        Eigen::Quaterniond q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        State candidate;
        candidate.p = Vec3(msg->position[0], msg->position[1], msg->position[2]);
        candidate.v = Vec3(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
        if (!q.coeffs().allFinite() || std::abs(q.norm()-1) > 0.05) return;
        candidate.R = q.normalized().toRotationMatrix();
        if (msg->velocity_frame == px4::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) candidate.v = candidate.R * candidate.v;
        else if (msg->velocity_frame != px4::VehicleOdometry::VELOCITY_FRAME_NED) return;
        if (!candidate.p.allFinite() || !candidate.v.allFinite()) return;
        if (have_odom_ && msg->reset_counter != reset_counter_ && phase_ == "ACTIVE") {
          controller_.resetIntegral();
          RCLCPP_INFO(get_logger(), "EKF reset; integral cleared, reference retained");
        }
        state_ = candidate; reset_counter_ = msg->reset_counter; have_odom_ = true; odom_at_ = wallTime();odom_seen_sim_=sim_time_;
      });
    position_sub_ = create_subscription<px4::VehicleLocalPosition>(prefix_ + "/fmu/out/vehicle_local_position", qos,
      [this](px4::VehicleLocalPosition::ConstSharedPtr msg) {
        position_valid_ = msg->xy_valid && msg->z_valid; position_at_ = wallTime();position_seen_sim_=sim_time_;
      });
    const std::string version = px4::VehicleStatus::MESSAGE_VERSION ? "_v" + std::to_string(px4::VehicleStatus::MESSAGE_VERSION) : "";
    vehicle_sub_ = create_subscription<px4::VehicleStatus>(prefix_ + "/fmu/out/vehicle_status" + version, qos,
      [this](px4::VehicleStatus::ConstSharedPtr msg) { vehicle_ = *msg; have_vehicle_ = true; vehicle_at_ = wallTime();vehicle_seen_sim_=sim_time_; });
    land_sub_ = create_subscription<px4::VehicleLandDetected>(prefix_ + "/fmu/out/vehicle_land_detected", qos,
      [this](px4::VehicleLandDetected::ConstSharedPtr msg) { landed_ = msg->landed; });
    ack_sub_ = create_subscription<px4::VehicleCommandAck>(prefix_ + "/fmu/out/vehicle_command_ack", qos,
      [this](px4::VehicleCommandAck::ConstSharedPtr msg) {
        RCLCPP_INFO(get_logger(), "PX4 ACK command=%u result=%u", msg->command, msg->result);
      });
    clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>("/clock", qos,
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
        onClock(msg->clock.sec + msg->clock.nanosec * 1e-9);
      });
    key_sub_ = create_subscription<std_msgs::msg::String>("/ganyu/keyboard", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) { if (msg->data.size() == 1) onKey(msg->data[0]); });
    heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>("/ganyu/keyboard_alive", 10,
      [this](std_msgs::msg::Empty::ConstSharedPtr) { keyboard_at_ = wallTime(); });
    spline_sub_ = create_subscription<uav_common_msg::msg::BSplineTrajectory>("/planning/trajectory", 10,
      [this](uav_common_msg::msg::BSplineTrajectory::ConstSharedPtr msg) { onSpline(*msg); });
    external_sub_ = create_subscription<uav_common_msg::msg::ExternalReference>("/autonomy/reference", 10,
      [this](uav_common_msg::msg::ExternalReference::ConstSharedPtr msg) { onExternal(*msg); });
    brake_sub_ = create_subscription<std_msgs::msg::Empty>("/autonomy/brake", 10,
      [this](std_msgs::msg::Empty::ConstSharedPtr) { if (reference_source_=="external" && phase_=="ACTIVE") brakeExternal("guard_stop"); });
    timer_ = create_wall_timer(5ms, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "GanYu ready; no automatic arming. T=%.3f kg, expected hover input=%.3f",
                controller_.params.mass, thrustInput(controller_.params.mass*9.81));
  }

private:
  void startTakeoffReference() {
    const double height = std::max(home_.z()-state_.p.z(),
                                  home_.z()-reference_.sample(sim_time_).p.z());
    staged_takeoff_ = reference_source_=="external" && takeoff_height_>3.0 && height<.6;
    // A tall, very slow single ramp spends several seconds on the ground.
    // Lift clear first within the same 0.5 m/s, 0.7 m/s2, 1.5 m/s3 limits.
    const double target_height = staged_takeoff_ ? .8 : takeoff_height_;
    const double duration = staged_takeoff_ ? 4.0 :
                           (reference_source_=="external" ? external_takeoff_duration_ : 5.0);
    reference_.moveTo(home_+Vec3(0,0,-target_height),sim_time_,duration);
    staged_takeoff_end_=sim_time_+duration;
  }
  static double wallTime() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  bool armed() const { return have_vehicle_ && vehicle_.arming_state == px4::VehicleStatus::ARMING_STATE_ARMED; }
  bool feedbackReady() const {
    const double now = wallTime();
    if (reference_source_=="external")
      return have_odom_ && have_vehicle_ && position_valid_ && sim_time_>0 && now-clock_at_<.5 &&
        sim_time_-odom_seen_sim_>=0 && sim_time_-odom_seen_sim_<.3 &&
        sim_time_-position_seen_sim_>=0 && sim_time_-position_seen_sim_<.3 &&
        sim_time_-vehicle_seen_sim_>=0 && sim_time_-vehicle_seen_sim_<3;
    return have_odom_ && have_vehicle_ && position_valid_ && sim_time_ > 0 &&
      now-odom_at_ < 0.3 && now-position_at_ < 0.3 && now-clock_at_ < 0.5 && now-vehicle_at_ < 3;
  }
  void onClock(double next) {
    clock_at_ = wallTime();
    if (reference_source_!="external" || next>=sim_time_-1e-6) { sim_time_=next; return; }
    // A numeric timestamp from the previous clock epoch cannot keep a lease
    // alive. Sample the old curve before changing epochs, then retime only an
    // independent full brake. Normal EGO geometry is never re-fitted.
    const Reference from=external_active_?external_ref_:reference_.sample(sim_time_);
    sim_time_=std::max(0.,next);last_sim_=sim_time_;clock_fault_latched_=true;
    staged_takeoff_=false;external_active_=false;external_yaw_rate_=0;
    if (phase_=="ACTIVE") {
      try {
        reference_=brakingReference(from,sim_time_).first;
        internal_braking_=true;
        externalFeedback(false,true,"simulation_clock_moved_backwards");
      } catch (const std::exception &error) {
        // Do not execute the old epoch's curve if no valid brake exists.
        internal_braking_=false;phase("CLOCK_FAULT");
        externalFeedback(false,false,std::string("clock_reset_brake_failure: ")+error.what());
      }
    } else if (phase_=="PRESTREAM" || phase_=="WAIT_OFFBOARD" || phase_=="WAIT_ARM") {
      reference_.reset(state_.p,sim_time_);phase("IDLE");
      externalFeedback(false,false,"simulation_clock_moved_backwards");
    }
  }
  double thrustInput(double thrust) const { return normalizedThrust(thrust, motor_k_, speed_min_, speed_max_, thrust_factor_); }
  void phase(const std::string &value) {
    phase_ = value; last_command_at_ = 0;
    if (value != "ACTIVE") { spline_active_ = false; external_active_=false; }
    RCLCPP_INFO(get_logger(), "State -> %s", phase_.c_str());
  }
  void command(uint32_t cmd, float p1 = 0, float p2 = 0) {
    px4::VehicleCommand msg{};
    msg.timestamp = get_clock()->now().nanoseconds()/1000;
    msg.command = cmd; msg.param1 = p1; msg.param2 = p2;
    if (cmd == px4::VehicleCommand::VEHICLE_CMD_NAV_LAND) {
      msg.param5 = NAN; msg.param6 = NAN; msg.param7 = NAN;
    }
    msg.target_system = system_id_; msg.target_component = 1;
    msg.source_system = 245; msg.source_component = 1; msg.from_external = true;
    command_pub_->publish(msg); last_command_at_ = wallTime();
  }
  bool exclusiveControl() const {
    for (const auto &topic : {"/fmu/in/trajectory_setpoint", "/fmu/in/vehicle_attitude_setpoint",
                              "/fmu/in/vehicle_thrust_setpoint", "/fmu/in/vehicle_torque_setpoint"}) {
      if (count_publishers(prefix_ + topic) != 0) return false;
    }
    return count_publishers(prefix_ + "/fmu/in/vehicle_rates_setpoint") <= 1 &&
           count_publishers(prefix_ + "/fmu/in/offboard_control_mode") <= 1;
  }
  void requestLand(const char *reason) {
    staged_takeoff_ = false;
    RCLCPP_WARN(get_logger(), "Landing: %s", reason);
    if (!armed()) { controller_.resetIntegral(); phase("IDLE"); return; }
    phase("LANDING"); command(px4::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  }
  void externalFeedback(bool accepted,bool braking,const std::string &reason,uint64_t id=0) {
    uav_common_msg::msg::TrajectoryFeedback m;
    m.header.frame_id="px4_ned"; m.header.stamp=rclcpp::Time(int64_t(sim_time_*1e9));
    m.trajectory_id=id?id:trajectory_id_;m.accepted=accepted;m.braking=braking;m.reason=reason;
    feedback_pub_->publish(m);
  }
  void brakeExternal(const std::string &reason) {
    if (reference_source_!="external" || phase_!="ACTIVE") return;
    if (!external_active_ && internal_braking_) return;
    const Reference from=external_active_?external_ref_:reference_.sample(sim_time_);
    try {reference_=brakingReference(from,sim_time_).first;}
    catch (const std::exception &e) {
      // Retain the last valid reference and report the inability to construct a
      // brake; never silently replace moving derivatives with zero.
      externalFeedback(false,true,std::string("brake_failure: ")+e.what());return;
    }
    staged_takeoff_=false;
    external_active_=false; internal_braking_=true; external_yaw_rate_=0;
    externalFeedback(false,true,reason);
  }
  void onExternal(const uav_common_msg::msg::ExternalReference &m) {
    if (reference_source_!="external" || clock_fault_latched_ || phase_!="ACTIVE" || !feedbackReady() || !armed() ||
        vehicle_.nav_state!=px4::VehicleStatus::NAVIGATION_STATE_OFFBOARD) return;
    const double stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;
    const double expiry=m.valid_until.sec+m.valid_until.nanosec*1e-9;
    auto ned=[](const auto &v){return Vec3(v[1],v[0],-v[2]);};
    Reference ref{ned(m.position),ned(m.velocity),ned(m.acceleration),ned(m.jerk)};
    std::string error;
    if (m.header.frame_id!="odom" || !ref.p.allFinite() || !ref.v.allFinite() || !ref.a.allFinite() || !ref.j.allFinite() ||
        !std::isfinite(m.yaw) || !std::isfinite(m.yaw_rate)) error="invalid_reference";
    else if (stamp<sim_time_-.10 || stamp>sim_time_+.03 || expiry<sim_time_ || expiry>stamp+.25) error="reference_clock_or_expiry";
    else if (m.trajectory_id<trajectory_id_ || stamp<=external_stamp_) error="out_of_order_reference";
    else if (ref.v.norm()>1.+1e-6 || ref.a.norm()>.7+1e-6 || ref.j.norm()>1.5+1e-6 || std::abs(m.yaw_rate)>.5+1e-6) error="reference_dynamic_limits";
    else {
      const Reference previous=external_active_?external_ref_:reference_.sample(sim_time_);
      const double dt=external_active_?std::clamp(stamp-external_stamp_,0.0,.1):0;
      try { (void)brakingReference(ref,sim_time_); }
      catch (const std::exception &) { error="reference_has_no_feasible_brake"; }
      if ((ref.p-previous.p-previous.v*dt).norm()>.12 || (ref.v-previous.v-previous.a*dt).norm()>.2 ||
          (ref.a-previous.a).norm()>.5) error="reference_discontinuity";
    }
    if (!error.empty()) {externalFeedback(false,false,error,m.trajectory_id);return;}
    bool changed=!external_active_ || m.trajectory_id!=trajectory_id_;
    external_ref_=ref; external_stamp_=stamp;external_expiry_=expiry;
    trajectory_id_=m.trajectory_id;external_active_=true;internal_braking_=false;staged_takeoff_=false;
    yaw_reference_=std::remainder(M_PI/2-m.yaw,2*M_PI);external_yaw_rate_=-m.yaw_rate;
    if (changed) externalFeedback(true,false,"tracking");
  }
  void onSpline(const uav_common_msg::msg::BSplineTrajectory &msg) {
    if (reference_source_ != "planner" || phase_ != "ACTIVE" || !feedbackReady() || !armed() ||
        vehicle_.nav_state != px4::VehicleStatus::NAVIGATION_STATE_OFFBOARD ||
        spline_active_ || msg.trajectory_id <= trajectory_id_) return;
    try {
      if (msg.header.frame_id != "px4_ned" || msg.control_points.size()%3)
        throw std::runtime_error("Expected flattened XYZ in px4_ned");
      const double start = msg.start_time.sec + msg.start_time.nanosec*1e-9;
      if (start < sim_time_-0.05 || start > sim_time_+3)
        throw std::runtime_error("Trajectory start time is stale or too far in future");
      std::vector<Vec3> points;
      for (size_t i=0; i<msg.control_points.size(); i+=3)
        points.emplace_back(msg.control_points[i], msg.control_points[i+1], msg.control_points[i+2]);
      BSplineReference candidate;
      candidate.set(msg.degree, msg.knots, points);
      const auto first = candidate.sample(0);
      const auto last = candidate.sample(candidate.duration()-1e-7);
      const auto current = reference_.sample(sim_time_);
      if ((first.p-current.p).norm() > 0.15 || (first.p-state_.p).norm() > 0.35 ||
          current.v.norm() > 0.1 || state_.v.norm() > 0.25 || first.v.norm() > 1e-5 ||
          first.a.norm() > 1e-5 || first.j.norm() > 1e-5 || last.v.norm() > 1e-5 ||
          last.a.norm() > 1e-5 || last.j.norm() > 1e-4)
        throw std::runtime_error("Spline must join a stationary hover with zero endpoint derivatives");
      if (candidate.derivativeBound(1) > 2 || candidate.derivativeBound(2) > 2 || candidate.derivativeBound(3) > 5)
        throw std::runtime_error("Trajectory exceeds controller reference limits (2 m/s, 2 m/s^2, 5 m/s^3)");
      spline_ = candidate; spline_start_ = start; trajectory_id_ = msg.trajectory_id;
      spline_active_ = true;
      RCLCPP_INFO(get_logger(), "Accepted B-spline %lu, duration %.2f s", trajectory_id_, spline_.duration());
    } catch (const std::exception &err) {
      RCLCPP_WARN(get_logger(), "Rejected planner trajectory: %s", err.what());
    }
  }
  void onKey(char key) {
    if (key == 'l' || key == 'x') { requestLand("Keyboard"); return; }
    if (clock_fault_latched_) return; // a new, time-consistent session is required
    if (reference_source_=="external" && key=='c' && phase_=="ACTIVE") {
      brakeExternal("startup_data_lost_after_arming");return;
    }
    if (reference_source_=="external" && key=='c' &&
        (phase_=="PRESTREAM" || phase_=="WAIT_OFFBOARD" || phase_=="WAIT_ARM")) {
      if (landed_) command(px4::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,0);
      reference_.reset(state_.p,sim_time_);internal_braking_=false;phase("IDLE");return;
    }
    if (key == 't') {
      if (phase_ != "IDLE" || armed()) return;
      if (!feedbackReady() || wallTime()-keyboard_at_ > 1) {
        RCLCPP_WARN(get_logger(), "Takeoff rejected: wait for valid position, clock and keyboard"); return;
      }
      if (!exclusiveControl()) { RCLCPP_ERROR(get_logger(), "Another PX4 setpoint publisher exists; takeoff rejected"); return; }
      home_ = state_.p; yaw_reference_ = std::atan2(state_.R(1,0), state_.R(0,0));
      controller_.resetIntegral(); reference_.reset(home_, sim_time_); internal_braking_=false; phase("PRESTREAM");
      stream_start_ = sim_time_; return;
    }
    if (phase_ != "ACTIVE") return;
    if (reference_source_ == "external") {
      if (key==' ') brakeExternal("operator_stop");
      if (key=='r' && !external_active_ && feedbackReady()) {
        internal_braking_=false;
        startTakeoffReference();
      }
      return;
    }
    if (reference_source_ == "planner") {
      if (key == ' ') {
        // Preserve p/v/a/j when the operator cancels a trajectory and brakes.
        const auto ref = spline_active_ ? spline_.sample(sim_time_-spline_start_) : reference_.sample(sim_time_);
        reference_.moveFrom(ref, ref.p, sim_time_, 2.0);
        spline_active_ = false;
      }
      return;
    }
    Vec3 target = reference_.goal();
    if (key == 'w') target.x() += step_; else if (key == 's') target.x() -= step_;
    else if (key == 'a') target.y() -= step_; else if (key == 'd') target.y() += step_;
    else if (key == 'e') target.z() -= step_; else if (key == 'q') target.z() += step_;
    else if (key == ' ') target = state_.p; else return;
    target.x() = std::clamp(target.x(), home_.x()-3, home_.x()+3);
    target.y() = std::clamp(target.y(), home_.y()-3, home_.y()+3);
    target.z() = std::clamp(target.z(), home_.z()-3, home_.z()-0.5);
    reference_.moveTo(target, sim_time_, 2.0);
    RCLCPP_INFO(get_logger(), "Target N/E/up: %.2f %.2f %.2f", target.x()-home_.x(), target.y()-home_.y(), home_.z()-target.z());
  }
  void publishOffboardMode() {
    px4::OffboardControlMode mode{};
    mode.timestamp = get_clock()->now().nanoseconds()/1000; mode.body_rate = true;
    mode_pub_->publish(mode);
  }
  void publishRates(const Command &raw) {
    publishOffboardMode();
    px4::VehicleRatesSetpoint msg{};
    msg.timestamp = get_clock()->now().nanoseconds()/1000;
    msg.roll = std::clamp(raw.omega.x(), -1.0, 1.0);
    msg.pitch = std::clamp(raw.omega.y(), -1.0, 1.0);
    msg.yaw = std::clamp(raw.omega.z(), -0.5, 0.5);
    msg.thrust_body = {0.0f, 0.0f, -static_cast<float>(thrustInput(raw.thrust))};
    rates_pub_->publish(msg); last_output_ = raw;
  }
  void publishStatus() {
    const double now = wallTime();
    if (now-last_status_at_ < 0.2) return;
    last_status_at_ = now;
    auto vec = [](const Vec3 &v) { std::ostringstream s; s << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']'; return s.str(); };
    std_msgs::msg::String msg;
    std::ostringstream s;
    s << "{\"phase\":\"" << phase_ << "\",\"ready\":" << (feedbackReady() ? "true":"false")
      << ",\"preflight\":" << (vehicle_.pre_flight_checks_pass ? "true":"false")
      << ",\"armed\":" << (armed()?"true":"false") << ",\"landed\":" << (landed_?"true":"false")
      << ",\"nav_state\":" << static_cast<int>(vehicle_.nav_state)
      << ",\"reference_source\":\"" << reference_source_ << "\""
      << ",\"clock_fault_latched\":" << (clock_fault_latched_?"true":"false")
      << ",\"trajectory_id\":" << trajectory_id_ << ",\"trajectory_active\":" << ((spline_active_||external_active_)?"true":"false")
      << ",\"internal_reference_settled\":" << ((!external_active_ && (reference_.sample(sim_time_).p-reference_.goal()).norm()<.001 && reference_.sample(sim_time_).v.norm()<.001 && reference_.sample(sim_time_).a.norm()<.001)?"true":"false")
      << ",\"home_ned\":" << vec(home_) << ",\"velocity_ned\":" << vec(state_.v)
      << ",\"position_ned\":" << vec(state_.p) << ",\"target_ned\":" << vec(external_active_?external_ref_.p:(spline_active_ ? spline_.goal() : reference_.goal()))
      << ",\"offset_neu\":" << vec(Vec3(state_.p.x()-home_.x(), state_.p.y()-home_.y(), home_.z()-state_.p.z()))
      << ",\"thrust_N\":" << last_output_.thrust << ",\"throttle\":" << thrustInput(last_output_.thrust) << '}';
    msg.data = s.str(); status_pub_->publish(msg);
  }
  void tick() {
    publishStatus();
    const double now = wallTime();
    if (phase_ == "IDLE" || phase_ == "YIELDED") return;
    if (phase_ == "LANDING") {
      if (!armed() && landed_) { controller_.resetIntegral(); phase("IDLE"); return; }
      if (vehicle_.nav_state != px4::VehicleStatus::NAVIGATION_STATE_AUTO_LAND && now-last_command_at_ > 1)
        command(px4::VehicleCommand::VEHICLE_CMD_NAV_LAND);
      return; // relinquish rate control immediately; PX4 LAND / Offboard-loss protection takes over
    }
    if (phase_ == "PRESTREAM" || phase_ == "WAIT_OFFBOARD" || phase_ == "WAIT_ARM") {
      publishRates(Command{}); // disarmed prestream: zero requested thrust
      if (phase_ == "PRESTREAM" && sim_time_-stream_start_ > 1.5) phase("WAIT_OFFBOARD");
      if (phase_ == "WAIT_OFFBOARD") {
        if (vehicle_.nav_state == px4::VehicleStatus::NAVIGATION_STATE_OFFBOARD) phase("WAIT_ARM");
        else if (now-last_command_at_ > 1) command(px4::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
      }
      if (phase_ == "WAIT_ARM") {
        if (armed()) {
          reference_.reset(state_.p, sim_time_);
          startTakeoffReference();
          controller_.resetIntegral(); last_sim_ = sim_time_; phase("ACTIVE");
        } else if (vehicle_.pre_flight_checks_pass && now-last_command_at_ > 1)
          command(px4::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
      }
      return;
    }
    if (phase_ != "ACTIVE") return;
    if (!armed()) { controller_.resetIntegral(); phase("IDLE"); return; }
    if (vehicle_.nav_state != px4::VehicleStatus::NAVIGATION_STATE_OFFBOARD) { phase("YIELDED"); return; }
    const double dt = sim_time_-last_sim_;
    last_sim_ = sim_time_;
    if (dt <= 0) { publishOffboardMode(); return; }
    try {
      if (staged_takeoff_ && sim_time_ >= staged_takeoff_end_ &&
          (state_.p-reference_.goal()).norm()<.12 && state_.v.norm()<.12) {
        // Begin the remaining climb only after the measured initial lift has
        // settled. Both pieces preserve p/v/a/jerk at their join.
        reference_.moveTo(home_+Vec3(0,0,-takeoff_height_),sim_time_,
                          std::max(8.0,external_takeoff_duration_-4.0));
        staged_takeoff_=false;
      }
      if (external_active_ && (sim_time_>external_expiry_ || !feedbackReady())) brakeExternal("reference_or_state_timeout");
      Reference ref = reference_.sample(sim_time_);
      if (external_active_) ref=external_ref_;
      if (spline_active_) {
        ref = spline_.sample(sim_time_-spline_start_);
        if (sim_time_ >= spline_start_+spline_.duration()) {
          reference_.reset(spline_.goal(), sim_time_);
          spline_active_ = false;
        }
      }
      Command out = controller_.update(state_, ref, dt, yaw_reference_);
      if (reference_source_=="external") out.omega += state_.R.transpose()*Vec3(0,0,external_yaw_rate_);
      publishRates(out);
      if (now-last_debug_at_ > 0.05) {
        std_msgs::msg::Float64MultiArray debug;
        debug.data = {out.thrust, thrustInput(out.thrust), out.omega.x(), out.omega.y(), out.omega.z()};
        for (const Vec3 &v : {ref.p, ref.v, ref.a, ref.j, controller_.integral, state_.p, state_.v})
          for (int i=0; i<3; ++i) debug.data.push_back(v[i]);
        debug_pub_->publish(debug); last_debug_at_ = now;
        uav_common_msg::msg::ExternalReference sampled;
        sampled.header.frame_id="odom";sampled.header.stamp=rclcpp::Time(int64_t(sim_time_*1e9));
        sampled.valid_until=rclcpp::Time(int64_t((sim_time_+.1)*1e9));sampled.trajectory_id=trajectory_id_;
        auto enu=[](const Vec3 &v){return std::array<double,3>{v.y(),v.x(),-v.z()};};
        sampled.position=enu(ref.p);sampled.velocity=enu(ref.v);sampled.acceleration=enu(ref.a);sampled.jerk=enu(ref.j);
        sampled.yaw=M_PI/2-yaw_reference_;sampled.yaw_rate=-external_yaw_rate_;reference_debug_pub_->publish(sampled);
      }
    } catch (const std::exception &err) {
      publishOffboardMode();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "GanYu computation error: %s", err.what());
    }
  }

  std::string prefix_, phase_ = "IDLE", reference_source_;
  int system_id_ = 2;
  double takeoff_height_, step_, motor_k_, speed_min_, speed_max_, thrust_factor_;
  double external_takeoff_duration_;
  bool staged_takeoff_ = false;
  double staged_takeoff_end_ = 0;
  State state_; QuadrotorControllerGanYu controller_; SmoothReference reference_;
  BSplineReference spline_;
  bool spline_active_ = false;
  bool external_active_=false;
  bool internal_braking_=false;
  // Receipt-liveness counters only; acquisition timestamps remain untouched.
  // A paused simulation must not consume the state's simulation-time timeout.
  double odom_seen_sim_=0,position_seen_sim_=0,vehicle_seen_sim_=0;
  Reference external_ref_;
  double external_stamp_=-1,external_expiry_=0,external_yaw_rate_=0;
  uint64_t trajectory_id_ = 0;
  ControllerLease output_lease_;
  double spline_start_ = 0;
  Vec3 home_ = Vec3::Zero(); Command last_output_;
  px4::VehicleStatus vehicle_{};
  bool have_odom_ = false, have_vehicle_ = false, position_valid_ = false, landed_ = true;
  uint8_t reset_counter_ = 0;
  double odom_at_=0, position_at_=0, clock_at_=0, vehicle_at_=0, keyboard_at_=0;
  double sim_time_=0, last_sim_=0, stream_start_=0, yaw_reference_=0;
  bool clock_fault_latched_=false;
  double last_command_at_=0, last_status_at_=0, last_debug_at_=0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4::VehicleRatesSetpoint>::SharedPtr rates_pub_;
  rclcpp::Publisher<px4::OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<px4::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Publisher<uav_common_msg::msg::ExternalReference>::SharedPtr reference_debug_pub_;
  rclcpp::Publisher<uav_common_msg::msg::TrajectoryFeedback>::SharedPtr feedback_pub_;
  rclcpp::Subscription<uav_common_msg::msg::ExternalReference>::SharedPtr external_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr brake_sub_;
  rclcpp::Subscription<px4::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4::VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<px4::VehicleStatus>::SharedPtr vehicle_sub_;
  rclcpp::Subscription<px4::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<px4::VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<uav_common_msg::msg::BSplineTrajectory>::SharedPtr spline_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<GanYuNode>()); }
  catch (const std::exception &err) { std::fprintf(stderr, "GanYu: %s\n", err.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
