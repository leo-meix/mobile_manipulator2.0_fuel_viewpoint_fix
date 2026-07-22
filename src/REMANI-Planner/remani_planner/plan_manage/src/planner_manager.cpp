// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace remani_planner
{

  MMPlannerManager::MMPlannerManager() {}

  MMPlannerManager::~MMPlannerManager()
  {
    std::cout << "destory manager" << std::endl;
    std_msgs::Bool destory_cmd;
    destory_cmd.data = true;
  }

  void MMPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    visualization_ = vis;
    /* read algorithm parameters */
    
    
    nh.param("mm/manipulator_max_vel", pp_.max_mani_vel_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    nh.param("search/time_resolution", pp_.polyTraj_piece_time, -1.0);
    double dist_resolution;
    nh.param("search/dist_resolution", dist_resolution, -1.0);
    pp_.polyTraj_piece_time = dist_resolution / pp_.max_vel_;//计算每个轨迹的时间长度。
    nh.param("manager/drone_id", pp_.drone_id, -1);

    nh.param("fsm/planning_horizon", pp_.planning_horizen_, 5.0);
    bool global_plan;
    nh.param("fsm/global_plan", global_plan, false);
    if(global_plan) pp_.planning_horizen_ = 1e3;//执行全局规划时。
    
    nh.param("mm/mobile_base_dof", pp_.mobile_base_dim_, -1);//移动底盘自由度
    nh.param("mm/manipulator_dof", pp_.manipulator_dim_, -1);//机械臂自由度
    nh.param("mm/mobile_base_non_singul_vel", pp_.mobile_base_non_singul_vel_, -1.0);//非奇异位姿时基座速度
    destory_cmd_pub_ = nh.advertise<std_msgs::Bool>("/mm_controller_node/destory_cmd", 10);
    pp_.traj_dim_ = pp_.mobile_base_dim_ + pp_.manipulator_dim_;//轨迹总维度=底盘维度+机械臂维度
    total_time_.clear();

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    mm_config_.reset(new MMConfig);
    mm_config_->setParam(nh, grid_map_);

    frontier_path_finder_.reset(new FrontierPathFinder);
    frontier_path_finder_->init(nh, grid_map_, mm_config_);

    initFuelFrontierBackend(nh);

    if (!fuel_frontier_target_manager_ || !fuel_viewpoint_generator_) {
      ROS_FATAL("[Exploration] Failed to initialize FUEL frontier backend.");
      throw std::runtime_error("FUEL frontier initialization failed");
    }
    frontier_goal_provider_.reset(new FuelFrontierGoalProvider(
        fuel_frontier_target_manager_,
        fuel_viewpoint_generator_,
        mm_config_));
    ROS_INFO("[Exploration] Using FUEL frontier goal provider.");

    pp_.max_vel_ = mm_config_->getBaseMaxVel();
    pp_.max_acc_ = mm_config_->getBaseMaxAcc();
    
    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh, grid_map_, mm_config_);
    
  }

  void MMPlannerManager::setCurrentBasePose(const Eigen::Vector3d& current_base_pose)
  {
    current_base_pose_ = current_base_pose;
    current_base_pose_valid_ = current_base_pose.allFinite();
  }

  void MMPlannerManager::initFuelFrontierBackend(ros::NodeHandle &nh)
  {
    nh.param("exploration/enable_fuel_frontier_debug", enable_fuel_frontier_debug_, true);
    nh.param("exploration/fuel_frontier_update_rate", fuel_frontier_update_rate_, 3.0);
    nh.param("map_frame", fuel_frontier_frame_id_, std::string("world"));
    nh.param("frontier/show_raw_cells", frontier_show_raw_cells_, true);
    nh.param("frontier/show_clusters", frontier_show_clusters_, true);
    nh.param("frontier/show_boxes", frontier_show_boxes_, true);
    nh.param("frontier/raw_stride", frontier_raw_stride_, 2);
    nh.param("frontier/max_raw_marker_points", frontier_max_raw_marker_points_, 1500);
    nh.param("frontier_target/show_targets", frontier_target_show_targets_, true);
    nh.param("frontier_target/show_target_text", frontier_target_show_text_, true);
    nh.param("frontier_target/show_top_target", frontier_target_show_top_target_, true);
    nh.param("frontier_target/show_normals", frontier_target_show_normals_, true);
    nh.param("frontier_target/target_marker_scale", frontier_target_marker_scale_, 0.18);
    nh.param("frontier_target/text_scale", frontier_target_text_scale_, 0.18);
    nh.param("frontier_target/top_target_marker_scale", frontier_top_target_marker_scale_, 0.28);
    nh.param("frontier_target/normal_length", frontier_target_normal_length_, 0.6);
    nh.param("frontier_target/normal_shaft_scale", frontier_target_normal_shaft_scale_, 0.03);
    fuel_frontier_update_rate_ = std::max(0.1, fuel_frontier_update_rate_);
    frontier_raw_stride_ = std::max(1, frontier_raw_stride_);
    frontier_max_raw_marker_points_ = std::max(0, frontier_max_raw_marker_points_);
    frontier_target_marker_scale_ = std::max(0.01, frontier_target_marker_scale_);
    frontier_target_text_scale_ = std::max(0.01, frontier_target_text_scale_);
    frontier_top_target_marker_scale_ = std::max(0.01, frontier_top_target_marker_scale_);
    frontier_target_normal_length_ = std::max(0.01, frontier_target_normal_length_);
    frontier_target_normal_shaft_scale_ = std::max(0.005, frontier_target_normal_shaft_scale_);

    fuel_map_adapter_.reset(new MapAdapter(grid_map_));
    fuel_frontier_cache_.reset(new FuelFrontierCache);
    fuel_frontier_detector_.reset(new FuelFrontierDetector(fuel_map_adapter_));
    fuel_frontier_detector_->setParams(nh);
    int fuel_cluster_min = 20;
    int fuel_large_cluster_min = 200;
    int fuel_max_split_depth = 3;
    double fuel_min_split_extent_cells = 20.0;
    double fuel_min_split_elongation = 3.0;
    double fuel_min_split_child_ratio = 0.25;
    nh.param("frontier/cluster_min_cells", fuel_cluster_min, 20);
    nh.param("frontier/large_cluster_min_cells", fuel_large_cluster_min, 200);
    nh.param("frontier/max_split_depth", fuel_max_split_depth, 3);
    nh.param("frontier/min_split_extent_cells", fuel_min_split_extent_cells, 20.0);
    nh.param("frontier/min_split_elongation", fuel_min_split_elongation, 3.0);
    nh.param("frontier/min_split_child_ratio", fuel_min_split_child_ratio, 0.25);
    fuel_frontier_clusterer_.reset(new FuelFrontierClusterer(fuel_cluster_min));
    fuel_frontier_clusterer_->setLargeClusterMin(fuel_large_cluster_min);
    fuel_frontier_clusterer_->setMaxSplitDepth(fuel_max_split_depth);
    fuel_frontier_clusterer_->setMinSplitExtentCells(fuel_min_split_extent_cells);
    fuel_frontier_clusterer_->setMinSplitElongation(fuel_min_split_elongation);
    fuel_frontier_clusterer_->setMinSplitChildRatio(fuel_min_split_child_ratio);
    fuel_frontier_tracker_.reset(new FuelFrontierTracker);
    fuel_frontier_tracker_->setParams(nh);
    fuel_frontier_target_manager_.reset(new FuelFrontierTargetManager);
    fuel_frontier_target_manager_->setMap(fuel_map_adapter_);
    fuel_frontier_target_manager_->setParams(nh);
    fuel_viewpoint_generator_.reset(new FuelViewpointGenerator);
    fuel_viewpoint_generator_->init(nh, grid_map_, enable_fuel_frontier_debug_);
    fuel_viewpoint_generator_->setMMConfig<MMConfig>(mm_config_);
    if (frontier_path_finder_) {
      fuel_viewpoint_generator_->setReachabilityChecker(
          [this](const Eigen::Vector3d& start,
                 const Eigen::Vector3d& goal,
                 const double goal_yaw,
                 double& path_cost) {
            std::vector<Eigen::Vector3d> path;
            return frontier_path_finder_ &&
                   frontier_path_finder_->search(start, goal, goal_yaw, path, path_cost);
          });
    }

    if (enable_fuel_frontier_debug_) {
      fuel_frontier_marker_pub_ =
          nh.advertise<visualization_msgs::MarkerArray>("/fuel_frontier/debug_markers", 10);
    }
    fuel_frontier_timer_ = nh.createTimer(ros::Duration(1.0 / fuel_frontier_update_rate_),
                                          &MMPlannerManager::fuelFrontierTimerCallback, this);

    ROS_INFO("[FuelFrontier] initialized, marker_debug=%d",
             enable_fuel_frontier_debug_ ? 1 : 0);
  }

  void MMPlannerManager::fuelFrontierTimerCallback(const ros::TimerEvent& /*event*/)
  {
    if (!fuel_frontier_detector_ || !grid_map_) return;

    if (!grid_map_->hasDepthObservation()) {
      if (fuel_frontier_print_wait_) {
        ROS_WARN_THROTTLE(2.0, "[FuelFrontierDebug] waiting for map observation.");
      }
      return;
    }

    if (!fuel_frontier_detector_->searchUpdatedRegion()) return;

    if (fuel_frontier_cache_ && fuel_map_adapter_) {
      fuel_frontier_cache_->update(fuel_frontier_detector_->rawFrontierCells(),
                                   fuel_frontier_detector_->updatedMin(),
                                   fuel_frontier_detector_->updatedMax(),
                                   *fuel_map_adapter_,
                                   fuel_frontier_detector_->cellConfig());
    }

    fuel_frontier_clusters_.clear();
    if (fuel_frontier_clusterer_) {
      const std::vector<Eigen::Vector3i>& cluster_input =
          fuel_frontier_cache_ ? fuel_frontier_cache_->cells()
                               : fuel_frontier_detector_->rawFrontierCells();
      std::vector<FuelFrontierCluster> cell_clusters =
          fuel_frontier_clusterer_->cluster(cluster_input);
      if (fuel_frontier_tracker_) {
        fuel_frontier_tracker_->update(cell_clusters);
      }
      updateFuelFrontierWorldClusters(cell_clusters);
    }

    Eigen::Vector3d current_base_pose = Eigen::Vector3d::Zero();
    if (current_base_pose_valid_) {
      current_base_pose = current_base_pose_;
    } else {
      ROS_WARN_THROTTLE(
          2.0,
          "[FuelFrontierDebug] base pose unavailable, using zero for target distance.");
    }
    const Eigen::Vector3d robot_pos(current_base_pose.x(), current_base_pose.y(), 0.0);

    if (fuel_frontier_target_manager_) {
      fuel_frontier_target_manager_->updateTargets(
          fuel_frontier_clusters_, robot_pos, grid_map_->getResolution());
    }
    if (fuel_viewpoint_generator_ && fuel_frontier_target_manager_) {
      const Eigen::VectorXd current_mani = mm_config_ ? mm_config_->getManiConfig()
                                                     : Eigen::VectorXd();
      std::vector<double> current_arm_config;
      current_arm_config.reserve(current_mani.size());
      for (int i = 0; i < current_mani.size(); ++i) {
        current_arm_config.push_back(current_mani(i));
      }
      if (mm_config_ && current_base_pose_valid_ &&
          current_mani.size() == mm_config_->getManiDof()) {
        const bool current_collision =
            mm_config_->checkcollision(current_base_pose, current_mani, false);
        const bool current_safe_collision =
            mm_config_->checkcollision(current_base_pose, current_mani, true);
        ROS_INFO_THROTTLE(2.0, "current_pose collision_without_margin=%d",
                          current_collision ? 1 : 0);
        ROS_INFO_THROTTLE(2.0, "current_pose collision_with_margin=%d",
                          current_safe_collision ? 1 : 0);
      }
      fuel_viewpoint_generator_->generate(fuel_frontier_target_manager_->targets(),
                                          current_base_pose, current_arm_config);
      if (enable_fuel_frontier_debug_) {
        fuel_viewpoint_generator_->publishDebugMarkers();
      }
    }
    if (enable_fuel_frontier_debug_) {
      publishFuelFrontierMarkers();
    }
  }

  void MMPlannerManager::updateFuelFrontierWorldClusters(
      const std::vector<FuelFrontierCluster>& cell_clusters)
  {
    fuel_frontier_clusters_.clear();
    fuel_frontier_clusters_.reserve(cell_clusters.size());
    if (!fuel_map_adapter_) return;

    for (const FuelFrontierCluster& cell_cluster : cell_clusters) {
      FuelFrontierCluster world_cluster;
      world_cluster.id = cell_cluster.id;
      world_cluster.filtered_cells = cell_cluster.filtered_cells;
      world_cluster.average.setZero();
      world_cluster.box_min.setConstant(std::numeric_limits<double>::max());
      world_cluster.box_max.setConstant(-std::numeric_limits<double>::max());

      if (world_cluster.filtered_cells.empty()) {
        world_cluster.box_min.setZero();
        world_cluster.box_max.setZero();
        fuel_frontier_clusters_.push_back(world_cluster);
        continue;
      }

      for (const Eigen::Vector3i& cell : world_cluster.filtered_cells) {
        Eigen::Vector3d point;
        fuel_map_adapter_->indexToPos(cell, point);
        world_cluster.average += point;
        world_cluster.box_min = world_cluster.box_min.cwiseMin(point);
        world_cluster.box_max = world_cluster.box_max.cwiseMax(point);
      }
      world_cluster.average /= static_cast<double>(world_cluster.filtered_cells.size());
      fuel_frontier_clusters_.push_back(world_cluster);
    }
  }

  visualization_msgs::Marker MMPlannerManager::makeFuelFrontierMarker(
      const std::string& ns, int id, int type, int action) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = fuel_frontier_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = action;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  void MMPlannerManager::publishFuelFrontierMarkers()
  {
    if (!fuel_frontier_marker_pub_ || !fuel_frontier_detector_) return;

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker clear = makeFuelFrontierMarker(
        "fuel_frontier_raw", 0, visualization_msgs::Marker::CUBE_LIST,
        visualization_msgs::Marker::DELETEALL);
    marker_array.markers.push_back(clear);

    const double resolution = grid_map_ ? grid_map_->getResolution() : 0.05;
    fuel_frontier_last_raw_displayed_ = 0;

    if (frontier_show_raw_cells_) {
      visualization_msgs::Marker raw = makeFuelFrontierMarker(
          "fuel_frontier_raw", 0, visualization_msgs::Marker::CUBE_LIST,
          visualization_msgs::Marker::ADD);
      raw.scale.x = resolution;
      raw.scale.y = resolution;
      raw.scale.z = resolution;
      raw.color.r = 0.05;
      raw.color.g = 0.75;
      raw.color.b = 1.0;
      raw.color.a = 0.45;

      const std::vector<Eigen::Vector3i>* raw_cells =
          fuel_frontier_cache_ ? &fuel_frontier_cache_->cells()
                               : &fuel_frontier_detector_->rawFrontierCells();
      const int raw_total = static_cast<int>(raw_cells->size());
      int sample_step = std::max(1, frontier_raw_stride_);
      if (frontier_max_raw_marker_points_ > 0) {
        const int stride_display_count = (raw_total + sample_step - 1) / sample_step;
        if (stride_display_count > frontier_max_raw_marker_points_) {
          sample_step = std::max(sample_step, (raw_total + frontier_max_raw_marker_points_ - 1) /
                                              frontier_max_raw_marker_points_);
        }
      }

      const int display_limit =
          frontier_max_raw_marker_points_ > 0 ? frontier_max_raw_marker_points_ : raw_total;
      for (int i = 0; i < raw_total && fuel_frontier_last_raw_displayed_ < display_limit;
           i += sample_step) {
        Eigen::Vector3d pt;
        if (!fuel_map_adapter_) continue;
        fuel_map_adapter_->indexToPos((*raw_cells)[i], pt);
        geometry_msgs::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();
        raw.points.push_back(p);
        ++fuel_frontier_last_raw_displayed_;
      }
      marker_array.markers.push_back(raw);
    }

    if (frontier_show_clusters_) {
      visualization_msgs::Marker centers = makeFuelFrontierMarker(
          "fuel_frontier_cluster", 0, visualization_msgs::Marker::SPHERE_LIST,
          visualization_msgs::Marker::ADD);
      centers.scale.x = 0.22;
      centers.scale.y = 0.22;
      centers.scale.z = 0.22;
      centers.color.r = 1.0;
      centers.color.g = 0.35;
      centers.color.b = 0.05;
      centers.color.a = 0.95;

      for (const auto& cluster : fuel_frontier_clusters_) {
        geometry_msgs::Point p;
        p.x = cluster.average.x();
        p.y = cluster.average.y();
        p.z = cluster.average.z();
        centers.points.push_back(p);
      }
      marker_array.markers.push_back(centers);
    }

    if (frontier_show_boxes_) {
      for (const auto& cluster : fuel_frontier_clusters_) {
        visualization_msgs::Marker box = makeFuelFrontierMarker(
            "fuel_frontier_box", cluster.id, visualization_msgs::Marker::CUBE,
            visualization_msgs::Marker::ADD);
        const Eigen::Vector3d box_size =
            (cluster.box_max - cluster.box_min + Eigen::Vector3d(resolution, resolution, resolution))
                .cwiseMax(Eigen::Vector3d(resolution, resolution, resolution));
        const Eigen::Vector3d box_center = 0.5 * (cluster.box_min + cluster.box_max);
        box.pose.position.x = box_center.x();
        box.pose.position.y = box_center.y();
        box.pose.position.z = box_center.z();
        box.scale.x = box_size.x();
        box.scale.y = box_size.y();
        box.scale.z = box_size.z();
        box.color.r = 1.0;
        box.color.g = 0.95;
        box.color.b = 0.10;
        box.color.a = 0.20;
        marker_array.markers.push_back(box);
      }
    }

    if (fuel_frontier_target_manager_) {
      const std::vector<FrontierTarget>& targets = fuel_frontier_target_manager_->targets();

      if (frontier_target_show_targets_) {
        for (const FrontierTarget& target : targets) {
          visualization_msgs::Marker marker = makeFuelFrontierMarker(
              "fuel_frontier_target", target.id, visualization_msgs::Marker::SPHERE,
              visualization_msgs::Marker::ADD);
          marker.pose.position.x = target.center.x();
          marker.pose.position.y = target.center.y();
          marker.pose.position.z = target.center.z();
          marker.scale.x = frontier_target_marker_scale_;
          marker.scale.y = frontier_target_marker_scale_;
          marker.scale.z = frontier_target_marker_scale_;
          marker.color.r = 0.15;
          marker.color.g = 1.0;
          marker.color.b = 0.35;
          marker.color.a = 0.95;
          marker_array.markers.push_back(marker);
        }
      }

      if (frontier_target_show_normals_) {
        for (const FrontierTarget& target : targets) {
          if (target.normal.norm() < 1e-6) continue;

          visualization_msgs::Marker marker = makeFuelFrontierMarker(
              "fuel_frontier_normal", target.id, visualization_msgs::Marker::ARROW,
              visualization_msgs::Marker::ADD);
          marker.scale.x = frontier_target_normal_shaft_scale_;
          marker.scale.y = 2.5 * frontier_target_normal_shaft_scale_;
          marker.scale.z = 3.5 * frontier_target_normal_shaft_scale_;
          marker.color.r = 0.10;
          marker.color.g = 0.85;
          marker.color.b = 1.0;
          marker.color.a = 0.95;

          geometry_msgs::Point start;
          start.x = target.center.x();
          start.y = target.center.y();
          start.z = target.center.z();

          const Eigen::Vector3d end_pos =
              target.center + frontier_target_normal_length_ * target.normal.normalized();
          geometry_msgs::Point end;
          end.x = end_pos.x();
          end.y = end_pos.y();
          end.z = end_pos.z();

          marker.points.push_back(start);
          marker.points.push_back(end);
          marker_array.markers.push_back(marker);
        }
      }

      if (frontier_target_show_text_) {
        char text[128];
        for (const FrontierTarget& target : targets) {
          visualization_msgs::Marker marker = makeFuelFrontierMarker(
              "fuel_frontier_target_text", target.id,
              visualization_msgs::Marker::TEXT_VIEW_FACING,
              visualization_msgs::Marker::ADD);
          marker.pose.position.x = target.center.x();
          marker.pose.position.y = target.center.y();
          marker.pose.position.z = target.center.z() + 0.25;
          marker.scale.z = frontier_target_text_scale_;
          marker.color.r = 1.0;
          marker.color.g = 1.0;
          marker.color.b = 1.0;
          marker.color.a = 0.95;
          std::snprintf(text, sizeof(text), "id:%d\nscore:%.1f\ncells:%d\ndist:%.2f",
                        target.id, target.score, target.cell_num,
                        target.distance_to_robot);
          marker.text = text;
          marker_array.markers.push_back(marker);
        }
      }

      if (frontier_target_show_top_target_ && !targets.empty()) {
        const FrontierTarget& target = targets.front();
        visualization_msgs::Marker marker = makeFuelFrontierMarker(
            "fuel_frontier_top_target", target.id, visualization_msgs::Marker::SPHERE,
            visualization_msgs::Marker::ADD);
        marker.pose.position.x = target.center.x();
        marker.pose.position.y = target.center.y();
        marker.pose.position.z = target.center.z();
        marker.scale.x = frontier_top_target_marker_scale_;
        marker.scale.y = frontier_top_target_marker_scale_;
        marker.scale.z = frontier_top_target_marker_scale_;
        marker.color.r = 1.0;
        marker.color.g = 0.10;
        marker.color.b = 0.10;
        marker.color.a = 0.95;
        marker_array.markers.push_back(marker);
      }
    }

    fuel_frontier_marker_pub_.publish(marker_array);
  }

  bool MMPlannerManager::computeInitReferenceState(const Eigen::VectorXd &start_pt,
                                                    const Eigen::VectorXd &start_vel,
                                                    const Eigen::VectorXd &start_acc,
                                                    const Eigen::VectorXd &start_jerk,
                                                    const double start_yaw,
                                                    const int start_singul,
                                                    const bool start_gripper,
                                                    const Eigen::VectorXd &local_target_pt,
                                                    const Eigen::VectorXd &local_target_vel,
                                                    const Eigen::VectorXd &local_target_acc,
                                                    const double local_target_yaw,
                                                    const bool local_target_gripper,
                                                    std::vector<poly_traj::MinSnapOpt<8>> &initMJO_container,
                                                    std::vector<int> &singul_container,
                                                    const bool flag_polyInit, 
                                                    const int continous_failures_count)
  {
    static bool flag_first_call = true;//记录是否第一次调用
    initMJO_container.clear();
    singul_container.clear();

    /*** case 1: use A* initialization ***/
    if (flag_first_call || flag_polyInit || true){//？？？？？？？？
      // ROS_INFO("get init from search");
      flag_first_call = false;
      /* basic params */
      // std::cout << "computeInit 1\n";
      // Eigen::Matrix<double,4,4> headState, tailState;
      Eigen::MatrixXd headState, tailState;//定义两个双精度矩阵，轨迹起点和终点状态
      headState.resize(pp_.traj_dim_, 4);//调整矩阵大小，行数为轨迹维度，列数为4
      tailState.resize(pp_.traj_dim_, 4);
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;//定义双精度向量

      // int piece_nums;
      // poly_traj::Trajectory traj;
      
      // constexpr double init_of_init_totaldur = 2.0;

      headState.col(0) = start_pt;//起始位置
      headState.col(1) = start_vel;//速度
      headState.col(2) = start_acc;//加速度
      headState.col(3) = start_jerk;//加加速度：描述加速度变化率的物理量

      tailState.col(0) = local_target_pt;
      tailState.col(1) = local_target_vel; 
      tailState.col(2) = local_target_acc;
      tailState.col(3) = Eigen::VectorXd::Zero(pp_.traj_dim_);//创建全零向量，将终点上加加速度在所有维度上设为0

      // std::cout << "headState: \n" << headState << std::endl;
      // std::cout << "tailState: \n" << tailState << std::endl;
      // std::cout << "computeInit 2\n";
      /* step 1: A* search and generate init traj */
      vector<vector<Eigen::VectorXd>> simple_path_container;//存储多条简单路径
      vector<Eigen::Vector2d> simple_path;//存储单条简单路径
      vector<vector<double>> yaw_list_container;//存储多条路径对应的航向角
      yaw_list_container.clear();
      Eigen::Vector2d init_ctrl;
      init_ctrl.setZero();
      // std::cout << "computeInit 3\n";调用A*搜索生成初始轨迹
      int status = ploy_traj_opt_->astarWithMinTraj(headState, tailState, start_yaw, 
                                                    start_singul, start_gripper,//起点奇异状态，起点夹爪状态
                                                    local_target_yaw, local_target_gripper, init_ctrl, 
                                                    continous_failures_count,//连续失败次数
                                                    simple_path_container, yaw_list_container, 
                                                    initMJO_container, singul_container);//最小jerk轨迹优化初始解
      // std::cout << "computeInit 4\n";
      if(status == KinoAstar::NO_PATH || status == KinoAstar::START_COLLISION || status == KinoAstar::GOAL_COLLISION){
        return false;//无路可走、起点碰撞、终点碰撞会导致规划失败
      }

      // finish get init traj

      // show the init simple_path
      vector<vector<Eigen::Vector2d>> path_view;
      vector<Eigen::Vector2d> display_pts;
      std::vector<Eigen::Vector2d> display_point_set;
      std::vector<Eigen::VectorXd> display_simple_path;
      std::vector<double> display_yaw;
      for(unsigned int i = 0; i < simple_path_container.size(); ++i){
        // std::cout << "computeInit 5 " << i << "\n";
        simple_path.clear();
        for(unsigned int j = 0; j < simple_path_container[i].size(); ++j){
          simple_path.push_back((simple_path_container[i])[j].head(2));
          display_simple_path.push_back((simple_path_container[i])[j]);
          display_yaw.push_back((yaw_list_container[i])[j]);
        }
        path_view.push_back(simple_path);

        Eigen::MatrixXd waypoints;
        Eigen::VectorXd time_list;
        Eigen::MatrixXd ctl_points;
        
        Eigen::Vector2d pts;
        waypoints = initMJO_container[i].getInitConstrainPoints(1);
        time_list = initMJO_container[i].get_T1();
        ctl_points = initMJO_container[i].getInitConstrainPoints(8);
        pts = waypoints.col(0).head(2);
        display_pts.push_back(pts);
        for(unsigned int j = 1; j < waypoints.cols(); ++j)
        {
          pts = waypoints.col(j).head(2);
          display_pts.push_back(pts);
        }
        for (int j = 0; j < ctl_points.cols(); ++j)
          display_point_set.push_back(ctl_points.col(j).head(2));
      }
      visualization_->displayAStarList(path_view, 0);
      visualization_->displayInitWaypoints(display_pts, 0.2, 0);
      visualization_->displayInitPathListDebug(display_point_set, 0.1, 0); // show the init traj for debug
      ploy_traj_opt_->displayFrontEndMesh(display_simple_path, display_yaw);
    }
    /*** case 2: initialize from previous optimal trajectory ***/
    else{ // FIXME check replan
      ROS_INFO("get init from local traj");
      // const double local_target_yaw,
      // std::vector<poly_traj::MinSnapOpt<8>> &initMJO_container,
      // std::vector<int> &singul_container,

      if (traj_container_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {//如果没生成过上一条轨迹，直接返回失败，走case 1
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_container_.singul_traj_data.start_time;
      double t_to_lc_end = traj_container_.singul_traj_data.duration - passed_t_on_lctraj;
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_container_.global_traj.glb_t_of_lc_tgt - traj_container_.global_traj.last_glb_t_of_lc_tgt);
      
      if(t_to_lc_end <= 0){
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but previous trajectories are out of date.");
        return false; // need polyInit
      }
      
      int start_piece_num = traj_container_.singul_traj_data.getPieceIdx(passed_t_on_lctraj);
      int singul_traj_num = traj_container_.singul_traj_data.singul_traj.size() - start_piece_num;
      initMJO_container.resize(singul_traj_num);
      singul_container.resize(singul_traj_num);
      int piece_nums;
      for(int i = 0; i < singul_traj_num; ++i){
        double temp_passed_t, temp_glb_t_remain;
        if(i == 0){
          temp_passed_t = passed_t_on_lctraj;
          traj_container_.singul_traj_data.locatePieceIdx(temp_passed_t);
        }else{
          temp_passed_t = 0;
        }
        if(i == singul_traj_num - 1){
          temp_glb_t_remain = t_to_lc_tgt - t_to_lc_end;
        }else{
          temp_glb_t_remain = 0;
        }
        printf("%d i, duration: %lf, passed_t: %lf, glb_t_remain: %lf", i, traj_container_.singul_traj_data.singul_traj[start_piece_num + i].duration, temp_passed_t, temp_glb_t_remain);
        double duration_now = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].duration - temp_passed_t + temp_glb_t_remain;
        // pp_.polyTraj_piece_time = 1.0;
        std::cout << i << " piece time: " << pp_.polyTraj_piece_time << "\n";
        // piece_nums = ceil(duration_now / pp_.polyTraj_piece_time);
        piece_nums = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getPieceNum();
        std::cout << i << " piece num: " << piece_nums << "\n";
        if(piece_nums < 2) piece_nums = 2;
        Eigen::MatrixXd innerPs(pp_.traj_dim_, piece_nums - 1);
        std::cout << "duration: " << duration_now << "\n";
        std::cout << "singul traj duration: " << traj_container_.singul_traj_data.singul_traj[start_piece_num + i].duration << "\n";
        Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, duration_now / piece_nums);
        double t = piece_dur_vec(0);
        for (int j = 0; j < piece_nums - 1; ++j){
          if (t + temp_passed_t < traj_container_.singul_traj_data.singul_traj[start_piece_num + i].duration){
            innerPs.col(j) = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getPos(t + temp_passed_t);
            std::cout << "piece: " << start_piece_num + i << " time: " << t + temp_passed_t << " local: " << innerPs.col(j).head(2).transpose() << "\n";
          }
          else if (t <= duration_now){
            double glb_t = t + temp_passed_t - traj_container_.singul_traj_data.singul_traj[start_piece_num + i].duration + traj_container_.global_traj.last_glb_t_of_lc_tgt - traj_container_.global_traj.global_start_time;
            innerPs.col(j) = traj_container_.global_traj.traj.getPos(glb_t);
            std::cout << "global: " << innerPs.col(j).head(2).transpose() << "\n";
          }
          else{
            ROS_ERROR("Should not happen! x_x 0x88");
          }

          t += piece_dur_vec(j + 1);
        }

        Eigen::MatrixXd headState, tailState;
        headState.resize(pp_.traj_dim_, 4);
        tailState.resize(pp_.traj_dim_, 4);
        if(i == 0){
          headState.block(0, 0, pp_.traj_dim_, 4) << start_pt, start_vel, start_acc, start_jerk;
        }else{
          headState.block(0, 0, pp_.traj_dim_, 4) << traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncPos(0),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncVel(0),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncAcc(0),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncJerk(0);
          // Eigen::VectorXd pos = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getPos(1e-3);
          // Eigen::VectorXd vel = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getVel(1e-3);
          // Eigen::VectorXd acc = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getAcc(1e-3);
          // Eigen::VectorXd jer = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncJerk(1e-3);
        }

        if(i == singul_traj_num - 1){
          tailState.block(0, 0, pp_.traj_dim_, 4) << local_target_pt, local_target_vel, local_target_acc, Eigen::VectorXd::Zero(pp_.traj_dim_);
        }else{
          int PN = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getPieceNum();
          tailState.block(0, 0, pp_.traj_dim_, 4) << traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncPos(PN),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncVel(PN),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncAcc(PN),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncJerk(PN);
        }

        initMJO_container[i].reset(headState, tailState, piece_nums);
        initMJO_container[i].generate(innerPs, piece_dur_vec);
        singul_container[i] = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].singul;
      }
    }

    return true;
  }

  void MMPlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::VectorXd &start_pt, const double &start_yaw,
      const Eigen::VectorXd &global_end_pt, const double global_end_yaw, 
      Eigen::VectorXd &local_target_pos, Eigen::VectorXd &local_target_vel,Eigen::VectorXd &local_target_acc, bool &reach_horizon)
  {
    reach_horizon = true;
    double t;

    traj_container_.global_traj.last_glb_t_of_lc_tgt = traj_container_.global_traj.glb_t_of_lc_tgt;

    double t_step = planning_horizen / 20 / pp_.max_vel_;
    for (t = traj_container_.global_traj.glb_t_of_lc_tgt;
         t < (traj_container_.global_traj.global_start_time + traj_container_.global_traj.duration);
         t += t_step){
      Eigen::VectorXd pos_t = traj_container_.global_traj.traj.getPos(t - traj_container_.global_traj.global_start_time);

      double dist = ((pos_t - start_pt).head(pp_.mobile_base_dim_)).norm();
      if (dist >= planning_horizen){
        bool occ;
        double yaw;
        Eigen::VectorXd vel;
        if ((t - traj_container_.global_traj.global_start_time) >= traj_container_.global_traj.duration){
          yaw = global_end_yaw;
        }else{
          vel = traj_container_.global_traj.traj.getVel(t - traj_container_.global_traj.global_start_time);
          yaw = atan2(vel(1), vel(0));
        }
        occ = mm_config_->checkcollision(Eigen::Vector3d(pos_t(0), pos_t(1), yaw), pos_t.tail(pp_.manipulator_dim_), false);
        if(occ) continue;
        local_target_pos = pos_t;
        traj_container_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    if ((t - traj_container_.global_traj.global_start_time) >= traj_container_.global_traj.duration){ // Last global point
      local_target_pos = global_end_pt;
      traj_container_.global_traj.glb_t_of_lc_tgt = traj_container_.global_traj.global_start_time + traj_container_.global_traj.duration;
      reach_horizon = false;
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_)){
      local_target_vel = Eigen::VectorXd::Zero(pp_.traj_dim_);
      local_target_vel.head(2) = pp_.mobile_base_non_singul_vel_ * Eigen::Vector2d(cos(global_end_yaw), sin(global_end_yaw));
      local_target_acc = Eigen::VectorXd::Zero(pp_.traj_dim_);
    }else{
      local_target_vel = traj_container_.global_traj.traj.getVel(t - traj_container_.global_traj.global_start_time);
      local_target_acc = traj_container_.global_traj.traj.getAcc(t - traj_container_.global_traj.global_start_time);
    }
  }

  bool MMPlannerManager::reboundReplan(//轨迹重规划函数
      const Eigen::VectorXd &start_pt, const Eigen::VectorXd &start_vel, 
      const Eigen::VectorXd &start_acc,const Eigen::VectorXd &start_jerk,
      const double start_yaw, const int start_singul, const bool start_gripper, const double trajectory_start_time, 
      const Eigen::VectorXd &local_target_pt, const Eigen::VectorXd &local_target_vel,
      const Eigen::VectorXd &local_target_acc, double local_target_yaw, const bool local_target_gripper,
      const bool flag_polyInit, const bool flag_randomPolyTraj,
      const bool have_local_traj, double &init_time, double &opt_time)//18个参数
  {
    //每次重规划计数/打印；t_init、t_opt 分别记录“初始化（构造初值）”与“优化”的耗时。
    static int count = 0;
    printf("\033[47;30m\n[replan %d]==============================================\033[0m\n", count++);
    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    /*** STEP 1: INIT ***/
    std::vector<poly_traj::MinSnapOpt<8>> initMJO_container;
    std::vector<int> singul_container;
    if (!computeInitReferenceState(start_pt, start_vel, start_acc, start_jerk, start_yaw, start_singul, start_gripper,
                                   local_target_pt, local_target_vel, local_target_acc, local_target_yaw, local_target_gripper,
                                   initMJO_container, singul_container, 
                                   flag_polyInit, continous_failures_count_)){return false;}//初始化

    Eigen::VectorXd init_len(7);//统计“初值路径长度/时长”（用于日志）
    double init_dura = 0.0;
    init_len.setZero();
    for(unsigned int i = 0; i < initMJO_container.size(); ++i){
      Eigen::MatrixXd cst_pts = initMJO_container[i].getInitConstrainPoints(1);
      Eigen::VectorXd time_list = initMJO_container[i].get_T1();
      Eigen::VectorXd pos1 = cst_pts.col(0), pos2;
      init_dura += time_list.lpNorm<1>();
      for(unsigned int j = 1; j < cst_pts.cols(); ++j){
        pos2 = cst_pts.col(j);
        init_len(0) += (pos2 - pos1).head(2).norm();
        init_len.tail(6) += ((pos2 - pos1).tail(6)).cwiseAbs();
        pos1 = pos2;
      }
    }
    


    if(initMJO_container.size() != singul_container.size()){//一致性检查，每一段轨迹对应一个奇异性标记。
      ROS_ERROR("initMJO_container size = %d != singul_container size = %d", (int)initMJO_container.size(), (int)singul_container.size());
    }
    ploy_traj_opt_->clear_resize_Cps_container(initMJO_container.size());//初始化优化器容器
    std::vector<Eigen::Vector2d> disp_point_set;//可视化点集
    std::vector<Eigen::MatrixXd> iniStates_container;//起始状态容器
    std::vector<Eigen::MatrixXd> finStates_container;//终止状态容器
    std::vector<Eigen::MatrixXd> initInnerPts_container;//内部点容器
    std::vector<Eigen::VectorXd> initT_container;//时间分配容器
    Eigen::MatrixXd cstr_pts;//控制点临时变量
    for(unsigned int i = 0; i < initMJO_container.size(); ++i){//遍历所有轨迹段
      cstr_pts = initMJO_container[i].getInitConstrainPoints(ploy_traj_opt_->get_cps_num_prePiece_());//获取初始轨迹的控制点并设置到优化器中
      ploy_traj_opt_->setControlPoints(i, cstr_pts);

      poly_traj::Trajectory<7> initTraj = initMJO_container[i].getTraj(singul_container[i]);//获取第i段轨迹，考虑奇异性状态

      initT_container.push_back(initTraj.getDurations());//获取该段轨迹个子段的时间长度

      int PN = initTraj.getPieceNum();//获取轨迹段数
      Eigen::MatrixXd all_pos = initTraj.getPositions();//获取所有位置点
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, pp_.traj_dim_, PN - 1);//提取内部点，不含起点和终点
      initInnerPts_container.push_back(innerPts);//存储内部点

      Eigen::MatrixXd headState, tailState;//提取边界状态
      headState.resize(pp_.traj_dim_, 4);//状态矩阵维度：列：位置，速度，加速度，加加速度；行：每个自由度的状态。
      tailState.resize(pp_.traj_dim_, 4);
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0),initTraj.getJuncJerk(0);//起始状态
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN),initTraj.getJuncJerk(PN);//终止状态
      iniStates_container.push_back(headState);//存储起始状态
      finStates_container.push_back(tailState);//存储终止状态
      for (int j = 0; j < cstr_pts.cols(); ++j)//提取控制点前2维（x，y）用作可视化
        disp_point_set.push_back(cstr_pts.col(j).head(2));//第j个控制点的x，y坐标
    }
    visualization_->displayInitCtrlPts(disp_point_set, 0.06, 0);//显示控制点用于调式
    
    t_init = ros::Time::now() - t_start;
    t_start = ros::Time::now();
    
    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;//优化成功的标志
    vector<vector<Eigen::Vector3d>> vis_trajs;//可视化轨迹容器
    
    
    Eigen::MatrixXd opt_waypoints;//优化后的路径点
    Eigen::VectorXd time_list;//时间列表
    vector<vector<Eigen::Vector2d>> his_trajs;//历史轨迹
    
    
    std::vector<Eigen::VectorXd> optT_container, optEECps_container;//优化时间和末端控制点
    std::vector<Eigen::MatrixXd> optWps_container;//优化路径点容器
    std::vector<Eigen::MatrixXd> optCps_container;//优化控制点容器
    //执行lbfgs优化
    flag_success = ploy_traj_opt_->OptimizeTrajectory_lbfgs(iniStates_container, finStates_container,//起始和终止状态
                                                            initInnerPts_container, initT_container,//内部点和初始状态
                                                             singul_container,//奇异性状态
                                                            optCps_container, optWps_container, //输出：控制点和路径点
                                                            optT_container, optEECps_container);//输出：时间和末端控制点
    t_opt = ros::Time::now() - t_start;//计算优化耗时

    // calculate data
    double snap_cost = 0.0, traj_dura = 0.0;//定义加加速度代价和总持续时间
    Eigen::VectorXd traj_len(7);//7维轨迹长度统计
    traj_len.setZero();
    double traj_time;
    Eigen::VectorXd pos1, pos2;
    for(unsigned int i = 0; i < singul_container.size(); ++i){
      snap_cost += (*ploy_traj_opt_->getMinSnapOptContainerPtr())[i].getTrajJerkCost();//累加加加速度代价
      traj_time = (*ploy_traj_opt_->getMinSnapOptContainerPtr())[i].getTraj(singul_container[i]).getTotalDuration();//计算单段轨迹长度并累加
      traj_dura += traj_time;

      pos1 = (*ploy_traj_opt_->getMinSnapOptContainerPtr())[i].getTraj(singul_container[i]).getPos(0.0);//计算轨迹长度（采样积分）
      for(double j = 0.0; j < traj_time + 1.0e-3; j += 0.01){//10ms采样间隔
        pos2 = (*ploy_traj_opt_->getMinSnapOptContainerPtr())[i].getTraj(singul_container[i]).getPos(j);
        traj_len(0) += (pos2 - pos1).head(2).norm();//前2维欧式距离（移动底盘）
        traj_len.tail(6) += ((pos2 - pos1).tail(6)).cwiseAbs();//后6维绝对距离（机械臂关节）
        pos1 = pos2;
      }
    }
    
    if(!flag_success){
      //显示失败的控制点（红色高亮）
      for(unsigned int i = 0; i < optCps_container.size(); ++i){
        visualization_->displayFailedList(optCps_container[i], i);
      }
      continous_failures_count_++;//增加连续失败计数
      return false;
    }
    static double sum_time = 0;//静态变量：累积总时间
    static int count_success = 0;//成功次数
    sum_time += (t_init + t_opt).toSec();//累计本次规划时间
    count_success++;
    init_time = t_init.toSec() * 1000;//初始化时间
    opt_time = t_opt.toSec() * 1000;//优化时间
    cout << "\033[34mtotal time: " << (t_init + t_opt).toSec() * 1000
         << ", init: " << t_init.toSec() * 1000
         << ", optimize: " << t_opt.toSec() * 1000
         << ", avg_time: " << sum_time / count_success * 1000.0
         << ", count_success: " << count_success << "\033[0m"<< endl;//输出性能信息
    average_plan_time_ = sum_time / count_success;//更新平均规划时间

    double traj_start_time;
    if(have_local_traj){//判断是否有局部轨迹，确保时间连续性
      double delta_replan_time = trajectory_start_time - ros::Time::now().toSec();
      if (delta_replan_time > 0) ros::Duration(delta_replan_time).sleep();//等待到预定时间
      traj_start_time = trajectory_start_time;//使用预定开始时间
    }
    else{
      //如果无局部轨迹，则使用当前时间
      traj_start_time = ros::Time::now().toSec();
    }

    traj_container_.singul_traj_data.clearSingulTraj();//清空原有轨迹
    for(unsigned int i = 0; i < singul_container.size(); ++i){
      //逐段添加优化后的轨迹
      traj_container_.singul_traj_data.addSingulTraj((*ploy_traj_opt_->getMinSnapOptContainerPtr())[i].getTraj(singul_container[i]), traj_start_time);
      //更新时间：下一段轨迹从上一段结束时间开始
      traj_start_time = traj_container_.singul_traj_data.singul_traj.back().end_time;
    }
    visualization_->displayOptimalCtrlPts(optCps_container, 0);//绿色显示优化后的控制点

    vector<Eigen::Vector2d> display_pts;
    Eigen::Vector2d pts;
    for(unsigned int i = 0; i < optWps_container.size(); ++i){
      display_pts.clear();
      opt_waypoints = optWps_container[i];//获取第i段路径点
      time_list = optT_container[i];//获取对应时间
      //提取路径点的前2维（x，y）用于显示
      pts = opt_waypoints.col(0).head(2);
      display_pts.push_back(pts);
      for(unsigned int j = 1; j < opt_waypoints.cols(); ++j)
      {
        pts = opt_waypoints.col(j).head(2);
        display_pts.push_back(pts);
      }
      visualization_->displayOptWaypoints(display_pts, 0.2, i);//可视化显示优化路径点
    }
    
    // success. YoY
    continous_failures_count_ = 0;//重置失败次数
    return true;
  }

  bool MMPlannerManager::EmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw,  const int singul){//紧急停止函数
    auto ZERO = Eigen::VectorXd::Zero(pp_.traj_dim_);
    Eigen::MatrixXd headState, tailState;
    headState.resize(pp_.traj_dim_, 4);
    tailState.resize(pp_.traj_dim_, 4);
    headState.block(0, 0, pp_.traj_dim_, 4) << stop_pos, ZERO, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinSnapOpt<8> stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    traj_container_.singul_traj_data.clearSingulTraj();
    traj_container_.singul_traj_data.addSingulTraj(stopMJO.getTraj(singul), ros::Time::now().toSec());

    return true;
  }

  // FIXME singul
  bool MMPlannerManager::planGlobalTrajWaypoints(//全局轨迹规划
      const Eigen::VectorXd &start_pos, const double start_yaw, const Eigen::VectorXd &start_vel, const Eigen::VectorXd &start_acc,
      const std::vector<Eigen::VectorXd> &waypoints, const double end_yaw, const Eigen::VectorXd &end_vel, const Eigen::VectorXd &end_acc)
  {
    int start_singul = 1;
    poly_traj::MinSnapOpt<8> globalMJO;
    Eigen::MatrixXd headState, tailState;
    headState.resize(pp_.traj_dim_, 4);//状态轨迹
    tailState.resize(pp_.traj_dim_, 4);
    headState << start_pos, start_vel, start_acc, Eigen::VectorXd::Zero(pp_.traj_dim_);
    tailState << waypoints.back(), end_vel, end_acc, Eigen::VectorXd::Zero(pp_.traj_dim_);
    Eigen::MatrixXd innerPts;

    if (waypoints.size() > 1)//多路径点：取除最后一个点外的所有点作为内部点
    {
      innerPts.resize(pp_.traj_dim_, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; i++)
        innerPts.col(i) = waypoints[i];
    }
    else
    {
      if (innerPts.size() != 0)//单路径点，无内部点，但检查innerPts是否意外有值
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }
    globalMJO.reset(headState, tailState, waypoints.size());//重置优化器
    double des_vel = pp_.max_vel_ / 1.5;//期望速度=最大速度/1.5
    Eigen::VectorXd time_vec(waypoints.size());
    int try_num = 0;
    do
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        //计算移动底盘所需时间
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).head(pp_.mobile_base_dim_).norm() / des_vel//底盘时间 = 移动时间/期望时间
                               : (waypoints[i] - waypoints[i - 1]).head(pp_.mobile_base_dim_).norm() / des_vel;
        //考虑机械臂各关节的时间约束
        for(int j = 0; j < pp_.manipulator_dim_; ++j){
          double t_temp;
          if(i == 0){
            t_temp = fabs((waypoints[0] - start_pos)[pp_.mobile_base_dim_ + j]) / pp_.max_mani_vel_;//关节角度变化/最大关节速度
          }else{
            t_temp = fabs((waypoints[i] - waypoints[i - 1])[pp_.mobile_base_dim_ + j]) / pp_.max_mani_vel_;
          }
          time_vec(i) = max(time_vec(i), t_temp);//取最大时间，确保移动和操作同步完成。
        }
      }
      globalMJO.generate(innerPts, time_vec);
      // cout << "try_num : " << try_num << endl;
      // cout << "max vel : " << globalMJO.getTraj(start_singul).getMaxVelRate() << endl;
      // cout << "time_vec : " << time_vec.transpose() << endl;
      des_vel /= 1.5;
      try_num++;
    } while (globalMJO.getTraj(start_singul).getMaxVelRate() > pp_.max_vel_ && try_num <= 5);
    auto time_now = ros::Time::now();//保存优化后的轨迹到轨迹容器
    traj_container_.setGlobalTraj(globalMJO.getTraj(start_singul), time_now.toSec());

    return true;
  }
} // namespace remani_planner
