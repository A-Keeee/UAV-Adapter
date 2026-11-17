// #include <ego_ros2/runtime.hpp>
// #include <csignal>
// #include <visualization_msgs/msg/marker.hpp>

// #include <plan_manage/ego_replan_fsm.h>

// using namespace ego_planner;

// void SignalHandler(int signal) {
//   if(ego_ros2::isInitialized() && ego_ros2::isStarted() && ego_ros2::ok() && !ego_ros2::isShuttingDown()){
//     ego_ros2::shutdown();
//   }
// }

// int main(int argc, char **argv) {

//   signal(SIGINT, SignalHandler);
//   signal(SIGTERM,SignalHandler);

//   ego_ros2::init(argc, argv, "ego_planner_node", ego_ros2::init_options::NoSigintHandler);
//   ego_ros2::NodeHandle nh("~");

//   EGOReplanFSM rebo_replan;

//   rebo_replan.init(nh);

//   // ego_ros2::Duration(1.0).sleep();
//   ego_ros2::AsyncSpinner async_spinner(4);
//   async_spinner.start();
//   ego_ros2::waitForShutdown();

//   return 0;
// }

#include <ego_ros2/runtime.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{

  ego_ros2::init(argc, argv, "ego_planner_node");
  ego_ros2::NodeHandle nh("~");

  EGOReplanFSM rebo_replan;

  rebo_replan.init(nh);

  // ego_ros2::Duration(1.0).sleep();
  ego_ros2::spin();

  return 0;
}
