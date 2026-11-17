#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <eigen3/Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ego_ros2/runtime.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <stdlib.h>

using std::vector;
namespace ego_planner
{
  class PlanningVisualization
  {
  private:
    ego_ros2::NodeHandle node;

    ego_ros2::Publisher goal_point_pub;
    ego_ros2::Publisher global_list_pub;
    ego_ros2::Publisher init_list_pub;
    ego_ros2::Publisher optimal_list_pub;
    ego_ros2::Publisher failed_list_pub;
    ego_ros2::Publisher a_star_list_pub;
    ego_ros2::Publisher guide_vector_pub;

    ego_ros2::Publisher intermediate_pt0_pub;
    ego_ros2::Publisher intermediate_pt1_pub;
    ego_ros2::Publisher intermediate_grad0_pub;
    ego_ros2::Publisher intermediate_grad1_pub;
    ego_ros2::Publisher intermediate_grad_smoo_pub;
    ego_ros2::Publisher intermediate_grad_dist_pub;
    ego_ros2::Publisher intermediate_grad_feas_pub;
    ego_ros2::Publisher intermediate_grad_swarm_pub;

  public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    PlanningVisualization(ego_ros2::NodeHandle &nh);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void displayMarkerList(ego_ros2::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id,  bool show_sphere = true);
    void generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale);
    void displayMultiOptimalPathList(vector<vector<Eigen::Vector3d>> optimal_trajs, const double scale);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    void displayFailedList(Eigen::MatrixXd failed_pts, int id);
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void displayArrowList(ego_ros2::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    
    void displayIntermediatePt(std::string type, Eigen::MatrixXd &pts, int id, Eigen::Vector4d color);
    void displayIntermediateGrad(std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color);
    // void displayNewArrow(ego_ros2::Publisher& guide_vector_pub, ego_planner::PolyTrajOptimizer::Ptr optimizer);
  };
} // namespace ego_planner
#endif