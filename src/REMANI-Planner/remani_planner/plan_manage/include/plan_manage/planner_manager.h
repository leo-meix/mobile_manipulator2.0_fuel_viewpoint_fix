
#pragma once

#include <stdlib.h>

#include <optimizer/poly_traj_optimizer.hpp>
#include <traj_utils/DataDisp.h>
#include <plan_env/grid_map.h>
#include <plan_env/map_adapter.h>
#include <plan_env/fuel_frontier_cache.h>
#include <plan_env/fuel_frontier_detector.h>
#include <plan_env/fuel_frontier_target.h>
#include <plan_env/fuel_frontier_target_manager.h>
#include <plan_env/fuel_frontier_tracker.h>
#include <plan_env/fuel_viewpoint_generator.h>
#include "traj_utils/plan_container.hpp"
#include <ros/ros.h>
#include <plan_manage/exploration_goal_provider.h>
#include <plan_manage/frontier_path_finder.h>
#include <plan_manage/planning_visualization.h>
#include "traj_utils/poly_traj_utils.hpp"
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace remani_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class MMPlannerManager
  {
  public:
  
    MMPlannerManager();
    ~MMPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /* main planning interface */
    bool reboundReplan(
        const Eigen::VectorXd &start_pt, const Eigen::VectorXd &start_vel, const Eigen::VectorXd &start_acc, 
        const Eigen::VectorXd &start_jerk, const double start_yaw, const int start_singul, const bool start_gripper, const double trajectory_start_time, 
        const Eigen::VectorXd &end_pt, const Eigen::VectorXd &end_vel ,const Eigen::VectorXd &end_acc, double end_yaw, const bool local_target_gripper,
        const bool flag_polyInit, const bool flag_randomPolyTraj,
        const bool have_local_traj, double &init_time, double &opt_time);
    bool computeInitReferenceState(const Eigen::VectorXd &start_pt, const Eigen::VectorXd &start_vel, 
                                    const Eigen::VectorXd &start_acc, const Eigen::VectorXd &start_jerk,
                                    const double start_yaw, const int start_singul, const bool start_gripper,
                                    const Eigen::VectorXd &local_target_pt, const Eigen::VectorXd &local_target_vel,
                                    const Eigen::VectorXd &local_target_acc, const double local_target_yaw, const bool local_target_gripper,
                                    std::vector<poly_traj::MinSnapOpt<8>> &initMJO_container, 
                                    std::vector<int> &singul_container,
                                    const bool flag_polyInit, const int continous_failures_count);
    bool planGlobalTrajWaypoints(
        const Eigen::VectorXd &start_pos, const double start_yaw, const Eigen::VectorXd &start_vel, const Eigen::VectorXd &start_acc,
        const std::vector<Eigen::VectorXd> &waypoints, const double end_yaw, const Eigen::VectorXd &end_vel, const Eigen::VectorXd &end_acc);
    void getLocalTarget(
        const double planning_horizen, const Eigen::VectorXd &start_pt, const double &start_yaw,
        const Eigen::VectorXd &global_end_pt, const double global_end_yaw,
        Eigen::VectorXd &local_target_pos, Eigen::VectorXd &local_target_vel,Eigen::VectorXd &local_target_acc, bool &reach_horizon);
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);
    void setCurrentBasePose(const Eigen::Vector3d& current_base_pose);
    bool EmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw, const int singul);

    PlanParameters pp_;
    // LocalTrajData local_data_;
    // GlobalTrajData global_data_;
    std::shared_ptr<GridMap> grid_map_;
    TrajContainer traj_container_;
    
    // ros::Publisher obj_pub_; //zx-todo

    PolyTrajOptimizer::Ptr ploy_traj_opt_;
    std::shared_ptr<MMConfig> mm_config_;
    FrontierPathFinder::Ptr frontier_path_finder_;
    ExplorationGoalProvider::Ptr frontier_goal_provider_;
    std::shared_ptr<MapAdapter> fuel_map_adapter_;
    std::shared_ptr<FuelFrontierCache> fuel_frontier_cache_;
    std::shared_ptr<FuelFrontierDetector> fuel_frontier_detector_;
    std::shared_ptr<FuelFrontierClusterer> fuel_frontier_clusterer_;
    std::shared_ptr<FuelFrontierTracker> fuel_frontier_tracker_;
    std::shared_ptr<FuelFrontierTargetManager> fuel_frontier_target_manager_;
    FuelViewpointGenerator::Ptr fuel_viewpoint_generator_;

    bool start_flag_, reach_flag_;
    ros::Time global_start_time_;
    ros::Publisher destory_cmd_pub_;
    double start_time_, reach_time_, average_plan_time_;
    std::vector<double> total_time_;
    std::vector<double> init_time_;
    std::vector<double> opt_time_;

  private:
    /* main planning algorithms & modules */
    
    PlanningVisualization::Ptr visualization_;

    void initFuelFrontierBackend(ros::NodeHandle& nh);
    void fuelFrontierTimerCallback(const ros::TimerEvent& event);
    void updateFuelFrontierWorldClusters(const std::vector<FuelFrontierCluster>& cell_clusters);
    void publishFuelFrontierMarkers();
    visualization_msgs::Marker makeFuelFrontierMarker(const std::string& ns, int id,
                                                      int type, int action) const;

    bool enable_fuel_frontier_debug_{true};
    bool fuel_frontier_print_wait_{true};
    bool frontier_show_raw_cells_{true};
    bool frontier_show_clusters_{true};
    bool frontier_show_boxes_{true};
    bool frontier_target_show_targets_{true};
    bool frontier_target_show_text_{true};
    bool frontier_target_show_top_target_{true};
    bool frontier_target_show_normals_{true};
    int frontier_raw_stride_{2};
    int frontier_max_raw_marker_points_{1500};
    int fuel_frontier_last_raw_displayed_{0};
    double fuel_frontier_update_rate_{3.0};
    double frontier_target_marker_scale_{0.18};
    double frontier_target_text_scale_{0.18};
    double frontier_top_target_marker_scale_{0.28};
    double frontier_target_normal_length_{0.6};
    double frontier_target_normal_shaft_scale_{0.03};
    std::string fuel_frontier_frame_id_{"world"};
    std::vector<FuelFrontierCluster> fuel_frontier_clusters_;
    ros::Timer fuel_frontier_timer_;
    ros::Publisher fuel_frontier_marker_pub_;
    Eigen::Vector3d current_base_pose_{Eigen::Vector3d::Zero()};
    bool current_base_pose_valid_{false};

    int continous_failures_count_{0};

  public:
    typedef unique_ptr<MMPlannerManager> Ptr;

    // !SECTION
  };
} // namespace remani_planner
