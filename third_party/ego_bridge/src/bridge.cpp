#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <uav_common_msg/msg/native_trajectory.hpp>
#include <uav_common_msg/msg/trajectory_feedback.hpp>
#ifdef EGO_BACKEND_V2
#include <quadrotor_msgs/msg/goal_set.hpp>
#include <traj_utils/msg/poly_traj.hpp>
using Input = traj_utils::msg::PolyTraj;
using Goal = quadrotor_msgs::msg::GoalSet;
#else
#include <traj_utils/msg/bspline.hpp>
using Input = traj_utils::msg::Bspline;
using Goal = geometry_msgs::msg::PoseStamped;
#endif

class Bridge : public rclcpp::Node {
 public:
  Bridge() : Node("ego_ganyu_bridge") {
    native_ =
        create_publisher<uav_common_msg::msg::NativeTrajectory>("/autonomy/native_trajectory", 10);
    accepted_ = create_publisher<std_msgs::msg::UInt64>("/autonomy/native_accepted", 10);
    permission_ = create_publisher<std_msgs::msg::Bool>("/autonomy/execution_allowed",
                                                        rclcpp::QoS(1).transient_local());
    goal_ = create_publisher<Goal>("/ego/goal", 10);
    trajectory_ =
        create_subscription<Input>("/ego/native_trajectory", 10, [this](Input::ConstSharedPtr in) {
          if (!enabled_ || pending_at_ >= 0 ||
              rclcpp::Time(in->start_time).seconds() < valid_after_ - .001)
            return;
          uav_common_msg::msg::NativeTrajectory out;
          out.header.frame_id = "odom";
          out.header.stamp = now();
          out.trajectory_id = ++sequence_;
          out.start_time = in->start_time;
#ifdef EGO_BACKEND_V2
          out.representation = out.POLYNOMIAL;
          out.degree = in->order;
          for (float d : in->duration) out.durations.push_back(d);
          for (size_t piece = 0; piece < in->duration.size(); ++piece) {
            if (in->coef_x.size() < 6 * (piece + 1) || in->coef_y.size() < 6 * (piece + 1) ||
                in->coef_z.size() < 6 * (piece + 1))
              return;
            for (const auto *axis : {&in->coef_x, &in->coef_y, &in->coef_z})
              for (size_t j = 0; j < 6; ++j) out.coefficients.push_back((*axis)[6 * piece + j]);
          }
#else
      out.representation=out.BSPLINE;out.degree=in->order;out.knots=in->knots;
      for(const auto &p:in->pos_pts) {out.control_points.push_back(p.x);out.control_points.push_back(p.y);out.control_points.push_back(p.z);}
#endif
          forwarded_.emplace_back(out.trajectory_id, uint64_t(in->traj_id));
          while (forwarded_.size() > 8) forwarded_.pop_front();
          native_->publish(out);
        });
    goals_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/autonomy/goal", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
          if (m->header.frame_id != "odom") return;
          pending_ = *m;
          pending_at_ = now().seconds();
          valid_after_ = rclcpp::Time(m->header.stamp).seconds();
          epoch_first_sequence_ = sequence_ + 1;
          enabled_ = true;
          std_msgs::msg::Bool p;
          p.data = true;
          permission_->publish(p);
        });
    feedback_ = create_subscription<uav_common_msg::msg::TrajectoryFeedback>(
        "/autonomy/trajectory_feedback", 10,
        [this](uav_common_msg::msg::TrajectoryFeedback::ConstSharedPtr m) {
          if (m->accepted) {
            for (const auto &ids : forwarded_)
              if (ids.first == m->trajectory_id) {
                std_msgs::msg::UInt64 ack;
                ack.data = ids.second;
                accepted_->publish(ack);
                break;
              }
          }
          if (!m->accepted && m->braking) {
            // Actual guard stop/manual withdrawal: never rearm autonomously.
            enabled_ = false;
            std_msgs::msg::Bool p;
            p.data = false;
            permission_->publish(p);
            pending_at_ = -1;
          } else if (!m->accepted && enabled_ && m->trajectory_id >= epoch_first_sequence_) {
            bool forwarded = false;
            for (const auto &ids : forwarded_)
              if (ids.first == m->trajectory_id) forwarded = true;
            if (!forwarded) return;
            // Reject this candidate and reset the planner to measured state,
            // without revoking the controller's independently guarded old
            // trajectory. Retry the same mission goal after the reset arrives.
            std_msgs::msg::Bool p;
            p.data = false;
            permission_->publish(p);
            pending_at_ = now().seconds();
            valid_after_ = pending_at_;
            epoch_first_sequence_ = sequence_ + 1;
          }
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
      if (pending_at_ < 0 || now().seconds() - pending_at_ < .1) return;
      if (!enabled_) return;
      std_msgs::msg::Bool permission;
      permission.data = true;
      permission_->publish(permission);
      Goal out;
#ifdef EGO_BACKEND_V2
      out.drone_id = 0;
      out.goal = {float(pending_.pose.position.x), float(pending_.pose.position.y),
                  float(pending_.pose.position.z)};
#else
      out=pending_;
#endif
      goal_->publish(out);
      pending_at_ = -1;
    });
    std_msgs::msg::Bool p;
    p.data = false;
    permission_->publish(p);
  }

 private:
  uint64_t sequence_ = 0;
  uint64_t epoch_first_sequence_ = 1;
  double pending_at_ = -1, valid_after_ = 0;
  bool enabled_ = false;
  geometry_msgs::msg::PoseStamped pending_;
  rclcpp::Publisher<uav_common_msg::msg::NativeTrajectory>::SharedPtr native_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr accepted_;
  std::deque<std::pair<uint64_t, uint64_t>> forwarded_;
  rclcpp::Publisher<Goal>::SharedPtr goal_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr permission_;
  rclcpp::Subscription<Input>::SharedPtr trajectory_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goals_;
  rclcpp::Subscription<uav_common_msg::msg::TrajectoryFeedback>::SharedPtr feedback_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bridge>());
  rclcpp::shutdown();
  return 0;
}
