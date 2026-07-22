#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <fstream>
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.hpp>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <plan_manage/planning_visualization.h>
#include <quadrotor_msgs/PolynomialTraj.h>
#include <traj_utils/Assignment.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>

#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <sstream>
#include <fstream>
#include <iostream>
using std::vector;

namespace remani_planner
{

  class REMANIReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      // WAIT_GRIPPER,
      EMERGENCY_STOP,
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      NEAREST_FRONTIER = 3
    };
    enum class FrontierPlanResult
    {
      SELECTED,
      TEMPORARILY_UNREACHABLE,
      EXPLORATION_COMPLETE
    };
    
    /* planning utils */
    MMPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    int wpt_id_;
    double no_replan_thresh_, replan_thresh_;
    // double waypoints_[50][4];
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> waypoints_;
    std::vector<double> waypoints_yaw_;
    std::vector<bool> waypoint_gripper_close_;
    bool gripper_flag_; // 是否需要夹爪操作
    int waypoint_num_;
    double planning_horizen_;
    double emergency_time_;
    bool enable_fail_safe_;
    int last_end_id_;
    double replan_trajectory_time_;
    int replan_fail_time_;
    double time_for_gripper_;
    bool global_plan_;
    bool manual_mapping_mode_;
    bool frontier_auto_continue_;
    bool frontier_replan_during_execution_;
    double frontier_replan_period_;
    double frontier_min_commit_time_;
    double frontier_switch_score_margin_;
    double frontier_reach_dist_;
    double frontier_min_select_dist_;
    double frontier_switch_margin_;
    double frontier_goal_backoff_;
    double frontier_path_weight_;
    double frontier_yaw_weight_;
    double frontier_visible_weight_;
    double frontier_unknown_weight_;
    double frontier_arm_motion_weight_;
    double frontier_min_z_;
    double frontier_max_z_;
    double frontier_visited_radius_;
    double frontier_failed_goal_radius_;
    double frontier_failed_goal_timeout_;
    int frontier_memory_max_size_;
    Eigen::VectorXd frontier_mani_config_;

    int mobile_base_dim_, manipulator_dim_, traj_dim_;
    double mobile_base_non_singul_vel_;

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_joint_state_, have_new_target_, have_recv_pre_agent_, have_local_traj_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::VectorXd mm_state_pos_, mm_state_vel_, mm_state_acc_, init_state_; // odometry state
    bool gripper_state_, rcv_gripper_state_;
    int mm_car_singul_;
    Eigen::Quaterniond mm_car_orient_;
    double mm_car_yaw_, mm_car_yaw_rate_;

    Eigen::VectorXd start_pos_, start_vel_, start_acc_, start_jer_; // start state
    int start_singul_;
    double start_yaw_, end_yaw_;
    Eigen::VectorXd end_pt_;                                       // goal state
    Eigen::VectorXd local_target_pt_, local_target_vel_, local_target_acc_;                     // local target state
    int local_target_singul_;

    bool flag_escape_emergency_;
    bool flag_relan_astar_;
    bool try_plan_after_emergency_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, joint_state_sub_, gripper_state_sub_, trigger_sub_, assignment_sub_;
    ros::Publisher replan_pub_, new_pub_, poly_traj_pub_, data_disp_pub_, gripper_cmd_pub_;

    ros::Publisher reached_pub_, start_pub_;

    ros::Time t_last_Astar_;
    ros::Time last_frontier_plan_time_;
    ros::Time next_frontier_retry_time_;
    int frontier_empty_detection_count_;

    int current_frontier_id_;
    bool frontier_goal_locked_;
    ros::Time frontier_goal_lock_time_;
    int current_candidate_id_;
    double current_frontier_score_;
    Eigen::Vector2d current_frontier_viewpoint_;
    bool frontier_target_is_intermediate_;
    struct FrontierGoalMemory
    {
      int frontier_id = -1;
      Eigen::Vector2d pos = Eigen::Vector2d::Zero();
      ros::Time stamp;
      std::string reason;
    };
    std::vector<FrontierGoalMemory> visited_goal_list_;
    std::vector<FrontierGoalMemory> failed_goal_list_;

    std::vector<double> init_time_list_;
    std::vector<double> opt_time_list_;
    std::vector<double> total_time_list_;
    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);           // front-end and back-end method
    bool callEmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw, const int singul); // front-end and back-end method
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(bool flag_use_poly_init);
    FrontierPlanResult planNearestFrontier(bool allow_keep_current = false,
                                           int excluded_frontier_id = -1);
    bool isVisitedFrontierGoal(int frontier_id, const Eigen::Vector2d& pos) const;
    bool isFailedFrontierGoal(int frontier_id, const Eigen::Vector2d& pos) const;
    void trimFrontierGoalMemory(std::vector<FrontierGoalMemory>& memory);
    void addVisitedFrontierGoal(int frontier_id, const Eigen::Vector2d& pos);
    void addFailedFrontierGoal(int frontier_id, const Eigen::Vector2d& pos, const std::string& reason);
    void unlockFrontierGoal(const std::string& reason);
    void logKinoDirection() const;
    
    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, REMANIReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    bool planNextWaypoint(const Eigen::VectorXd next_wp, const double nect_yaw);
    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void mmCarOdomCallback(const nav_msgs::OdometryConstPtr &msg);
    void mmManiOdomCallback(const sensor_msgs::JointStateConstPtr &msg);
    void gripperCallback(const std_msgs::Bool::ConstPtr &msg);
    void sendPolyTrajROSMsg();
    bool frontEndPathSearching();
    bool checkCollision();

  public:
    REMANIReplanFSM(/* args */)
    {
    }
    ~REMANIReplanFSM();


    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace remani_planner

#endif
