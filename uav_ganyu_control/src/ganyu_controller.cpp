#include "uav_ganyu_control/ganyu.hpp"
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

using namespace std::chrono_literals;
using namespace uav_ganyu_control;
namespace px4 = px4_msgs::msg;

class GanYuNode : public rclcpp::Node {
public:
  GanYuNode() : Node("ganyu_controller") {
    prefix_ = declare_parameter<std::string>("px4_namespace", "/px4_1");
    system_id_ = declare_parameter<int>("target_system", 2);
    takeoff_height_ = declare_parameter<double>("takeoff_height", 1.5);
    step_ = declare_parameter<double>("keyboard_step", 0.25);
    controller_.params.mass = declare_parameter<double>("mass", 2.125307692307692);
    controller_.params.max_thrust = declare_parameter<double>("max_thrust", 30.0);
    motor_k_ = declare_parameter<double>("motor_constant", 8.54858e-6);
    speed_min_ = declare_parameter<double>("motor_speed_min", 150.0);
    speed_max_ = declare_parameter<double>("motor_speed_max", 1000.0);
    thrust_factor_ = declare_parameter<double>("thrust_model_factor", 0.0);
    if (takeoff_height_ < 0.5 || takeoff_height_ > 3 || step_ <= 0 || step_ > 0.5 ||
        motor_k_ <= 0 || speed_min_ < 0 || speed_max_ <= speed_min_ ||
        thrust_factor_ < 0 || thrust_factor_ > 1 || controller_.params.mass <= 0 ||
        controller_.params.max_thrust < controller_.params.mass * 9.81) {
      throw std::runtime_error("Invalid vehicle / keyboard parameters");
    }
    rates_pub_ = create_publisher<px4::VehicleRatesSetpoint>(prefix_ + "/fmu/in/vehicle_rates_setpoint", 10);
    mode_pub_ = create_publisher<px4::OffboardControlMode>(prefix_ + "/fmu/in/offboard_control_mode", 10);
    command_pub_ = create_publisher<px4::VehicleCommand>(prefix_ + "/fmu/in/vehicle_command", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/ganyu/status", 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/ganyu/control_debug", 10);
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
        state_ = candidate; reset_counter_ = msg->reset_counter; have_odom_ = true; odom_at_ = wallTime();
      });
    position_sub_ = create_subscription<px4::VehicleLocalPosition>(prefix_ + "/fmu/out/vehicle_local_position", qos,
      [this](px4::VehicleLocalPosition::ConstSharedPtr msg) {
        position_valid_ = msg->xy_valid && msg->z_valid; position_at_ = wallTime();
      });
    const std::string version = px4::VehicleStatus::MESSAGE_VERSION ? "_v" + std::to_string(px4::VehicleStatus::MESSAGE_VERSION) : "";
    vehicle_sub_ = create_subscription<px4::VehicleStatus>(prefix_ + "/fmu/out/vehicle_status" + version, qos,
      [this](px4::VehicleStatus::ConstSharedPtr msg) { vehicle_ = *msg; have_vehicle_ = true; vehicle_at_ = wallTime(); });
    land_sub_ = create_subscription<px4::VehicleLandDetected>(prefix_ + "/fmu/out/vehicle_land_detected", qos,
      [this](px4::VehicleLandDetected::ConstSharedPtr msg) { landed_ = msg->landed; });
    ack_sub_ = create_subscription<px4::VehicleCommandAck>(prefix_ + "/fmu/out/vehicle_command_ack", qos,
      [this](px4::VehicleCommandAck::ConstSharedPtr msg) {
        RCLCPP_INFO(get_logger(), "PX4 ACK command=%u result=%u", msg->command, msg->result);
      });
    clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>("/clock", qos,
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
        sim_time_ = msg->clock.sec + msg->clock.nanosec * 1e-9; clock_at_ = wallTime();
      });
    key_sub_ = create_subscription<std_msgs::msg::String>("/ganyu/keyboard", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) { if (msg->data.size() == 1) onKey(msg->data[0]); });
    heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>("/ganyu/keyboard_alive", 10,
      [this](std_msgs::msg::Empty::ConstSharedPtr) { keyboard_at_ = wallTime(); });
    timer_ = create_wall_timer(5ms, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "GanYu ready; no automatic arming. T=%.3f kg, expected hover input=%.3f",
                controller_.params.mass, thrustInput(controller_.params.mass*9.81));
  }

private:
  static double wallTime() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  bool armed() const { return have_vehicle_ && vehicle_.arming_state == px4::VehicleStatus::ARMING_STATE_ARMED; }
  bool feedbackReady() const {
    const double now = wallTime();
    return have_odom_ && have_vehicle_ && position_valid_ && sim_time_ > 0 &&
      now-odom_at_ < 0.3 && now-position_at_ < 0.3 && now-clock_at_ < 0.5 && now-vehicle_at_ < 3;
  }
  double thrustInput(double thrust) const { return normalizedThrust(thrust, motor_k_, speed_min_, speed_max_, thrust_factor_); }
  void phase(const std::string &value) {
    phase_ = value; last_command_at_ = 0;
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
    RCLCPP_WARN(get_logger(), "Landing: %s", reason);
    if (!armed()) { controller_.resetIntegral(); phase("IDLE"); return; }
    phase("LANDING"); command(px4::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  }
  void onKey(char key) {
    if (key == 'l' || key == 'x') { requestLand("Keyboard"); return; }
    if (key == 't') {
      if (phase_ != "IDLE" || armed()) return;
      if (!feedbackReady() || wallTime()-keyboard_at_ > 1) {
        RCLCPP_WARN(get_logger(), "Takeoff rejected: wait for valid position, clock and keyboard"); return;
      }
      if (!exclusiveControl()) { RCLCPP_ERROR(get_logger(), "Another PX4 setpoint publisher exists; takeoff rejected"); return; }
      home_ = state_.p; yaw_reference_ = std::atan2(state_.R(1,0), state_.R(0,0));
      controller_.resetIntegral(); reference_.reset(home_, sim_time_); phase("PRESTREAM");
      stream_start_ = sim_time_; return;
    }
    if (phase_ != "ACTIVE") return;
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
      << ",\"position_ned\":" << vec(state_.p) << ",\"target_ned\":" << vec(reference_.goal())
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
          reference_.moveTo(home_ + Vec3(0,0,-takeoff_height_), sim_time_, 5.0);
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
      const Reference ref = reference_.sample(sim_time_);
      const Command out = controller_.update(state_, ref, dt, yaw_reference_);
      publishRates(out);
      if (now-last_debug_at_ > 0.05) {
        std_msgs::msg::Float64MultiArray debug;
        debug.data = {out.thrust, thrustInput(out.thrust), out.omega.x(), out.omega.y(), out.omega.z()};
        for (const Vec3 &v : {ref.p, ref.v, ref.a, ref.j, controller_.integral, state_.p, state_.v})
          for (int i=0; i<3; ++i) debug.data.push_back(v[i]);
        debug_pub_->publish(debug); last_debug_at_ = now;
      }
    } catch (const std::exception &err) {
      publishOffboardMode();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "GanYu computation error: %s", err.what());
    }
  }

  std::string prefix_, phase_ = "IDLE";
  int system_id_ = 2;
  double takeoff_height_, step_, motor_k_, speed_min_, speed_max_, thrust_factor_;
  State state_; QuadrotorControllerGanYu controller_; SmoothReference reference_;
  Vec3 home_ = Vec3::Zero(); Command last_output_;
  px4::VehicleStatus vehicle_{};
  bool have_odom_ = false, have_vehicle_ = false, position_valid_ = false, landed_ = true;
  uint8_t reset_counter_ = 0;
  double odom_at_=0, position_at_=0, clock_at_=0, vehicle_at_=0, keyboard_at_=0;
  double sim_time_=0, last_sim_=0, stream_start_=0, yaw_reference_=0;
  double last_command_at_=0, last_status_at_=0, last_debug_at_=0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4::VehicleRatesSetpoint>::SharedPtr rates_pub_;
  rclcpp::Publisher<px4::OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<px4::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Subscription<px4::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4::VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<px4::VehicleStatus>::SharedPtr vehicle_sub_;
  rclcpp::Subscription<px4::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<px4::VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<GanYuNode>()); }
  catch (const std::exception &err) { std::fprintf(stderr, "GanYu: %s\n", err.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
