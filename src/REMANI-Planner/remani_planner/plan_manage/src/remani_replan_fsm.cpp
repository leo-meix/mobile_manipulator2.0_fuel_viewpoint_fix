#include <plan_manage/remani_replan_fsm.h>

#include <cmath>
#include <limits>
#include <map>

namespace remani_planner
{
  REMANIReplanFSM::~REMANIReplanFSM(){}
  void REMANIReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    // Exploration must remain idle until RViz publishes a 2D Nav Goal.
    // Leaving this flag uninitialized can make the FSM start autonomously
    // depending on the allocator's previous memory contents.
    have_trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_joint_state_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    try_plan_after_emergency_ = false;
    flag_relan_astar_ = false;
    have_local_traj_ = false;
    replan_fail_time_ = 0;
    current_frontier_id_ = -1;
    frontier_goal_locked_ = false;
    frontier_goal_lock_time_ = ros::Time(0);
    current_candidate_id_ = -1;
    current_frontier_score_ = std::numeric_limits<double>::infinity();
    current_frontier_viewpoint_.setZero();
    frontier_target_is_intermediate_ = false;
    last_frontier_plan_time_ = ros::Time(0);
    next_frontier_retry_time_ = ros::Time(0);
    frontier_empty_detection_count_ = 0;

    /*  fsm param  */
    nh.param("fsm/target_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/replan_trajectory_time", replan_trajectory_time_, 0.0);
    nh.param("fsm/time_for_gripper", time_for_gripper_, -1.0);
    nh.param("fsm/global_plan", global_plan_, false);
    nh.param("fsm/manual_mapping_mode", manual_mapping_mode_, false);
    if(global_plan_) planning_horizen_ = 1.0e3;
    nh.param("frontier/auto_continue", frontier_auto_continue_, true);
    nh.param("frontier/replan_during_execution", frontier_replan_during_execution_, false);
    nh.param("frontier/replan_period", frontier_replan_period_, 1.0);
    nh.param("frontier/min_commit_time", frontier_min_commit_time_, 4.0);
    nh.param("frontier/switch_score_margin", frontier_switch_score_margin_, 1.5);
    nh.param("frontier/reach_dist", frontier_reach_dist_, 0.5);
    nh.param("frontier/min_select_dist", frontier_min_select_dist_, 1.2);
    nh.param("frontier/switch_margin", frontier_switch_margin_, 1.0);
    nh.param("frontier/goal_backoff", frontier_goal_backoff_, 1.0);
    nh.param("frontier/path_cost_weight", frontier_path_weight_, 1.0);
    nh.param("frontier/path_yaw_weight", frontier_yaw_weight_, 0.5);
    nh.param("frontier/path_visible_weight", frontier_visible_weight_, 0.002);
    nh.param("frontier/unknown_gain_weight", frontier_unknown_weight_, 0.005);
    nh.param("frontier/arm_motion_weight", frontier_arm_motion_weight_, 0.05);
    nh.param("frontier/min_z", frontier_min_z_, 0.05);
    nh.param("frontier/max_z", frontier_max_z_, 2.5);
    nh.param("frontier/visited_radius", frontier_visited_radius_, 0.8);
    nh.param("frontier/failed_goal_radius", frontier_failed_goal_radius_, 0.8);
    nh.param("frontier/failed_goal_timeout", frontier_failed_goal_timeout_, 8.0);
    nh.param("frontier/memory_max_size", frontier_memory_max_size_, 100);
    frontier_reach_dist_ = std::max(0.0, frontier_reach_dist_);
    frontier_min_commit_time_ = std::max(0.0, frontier_min_commit_time_);
    frontier_switch_score_margin_ = std::max(0.0, frontier_switch_score_margin_);
    frontier_min_select_dist_ = std::max(0.0, frontier_min_select_dist_);
    frontier_goal_backoff_ = std::max(0.0, frontier_goal_backoff_);
    frontier_path_weight_ = std::max(0.0, frontier_path_weight_);
    frontier_yaw_weight_ = std::max(0.0, frontier_yaw_weight_);
    frontier_visible_weight_ = std::max(0.0, frontier_visible_weight_);
    frontier_unknown_weight_ = std::max(0.0, frontier_unknown_weight_);
    frontier_arm_motion_weight_ = std::max(0.0, frontier_arm_motion_weight_);
    if (frontier_min_z_ > frontier_max_z_) {
      std::swap(frontier_min_z_, frontier_max_z_);
    }
    frontier_visited_radius_ = std::max(0.0, frontier_visited_radius_);
    frontier_failed_goal_radius_ = std::max(0.0, frontier_failed_goal_radius_);
    frontier_failed_goal_timeout_ = std::max(0.0, frontier_failed_goal_timeout_);
    frontier_memory_max_size_ = std::max(1, frontier_memory_max_size_);
    ROS_INFO("[Exploration] nearest frontier memory filters: visited_radius=%.2f, failed_radius=%.2f, failed_timeout=%.2f, memory_max_size=%d",
             frontier_visited_radius_, frontier_failed_goal_radius_,
             frontier_failed_goal_timeout_, frontier_memory_max_size_);

    nh.param("mm/mobile_base_dof", mobile_base_dim_, -1);
    nh.param("mm/manipulator_dof", manipulator_dim_, -1);
    nh.param("mm/mobile_base_non_singul_vel", mobile_base_non_singul_vel_, -1.0);
    

    traj_dim_ = mobile_base_dim_ + manipulator_dim_;//求轨迹总维度

    mm_state_pos_ = Eigen::VectorXd::Zero(traj_dim_);
    mm_state_vel_ = Eigen::VectorXd::Zero(traj_dim_);
    mm_state_acc_ = Eigen::VectorXd::Zero(traj_dim_);
    frontier_mani_config_ = Eigen::VectorXd::Zero(manipulator_dim_);

    gripper_flag_ = true;//加爪控制标志

    start_pos_.resize(traj_dim_);
    start_vel_.resize(traj_dim_);
    start_acc_.resize(traj_dim_);
    start_jer_.resize(traj_dim_);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);

    std::vector<double> frontier_mani_config_param;
    if (!nh.getParam("frontier/explore_manipulator_config", frontier_mani_config_param)) {
      std::vector<double> init_state_param;
      if (nh.getParam("fsm/init_state", init_state_param) &&
          (int)init_state_param.size() >= mobile_base_dim_ + manipulator_dim_) {
        frontier_mani_config_param.assign(
            init_state_param.begin() + mobile_base_dim_,
            init_state_param.begin() + mobile_base_dim_ + manipulator_dim_);
      }
    }
    if ((int)frontier_mani_config_param.size() == manipulator_dim_) {
      for (int i = 0; i < manipulator_dim_; ++i) {
        frontier_mani_config_(i) = frontier_mani_config_param[i] * M_PI / 180.0;
      }
      if (manipulator_dim_ > 0) {
        mm_state_pos_.tail(manipulator_dim_) = frontier_mani_config_;
      }
    } else if (manipulator_dim_ > 0) {
      ROS_WARN("[Exploration] frontier/explore_manipulator_config is not set and fsm/init_state is invalid; "
               "frontier mode will wait for joint_state before planning.");
    }

    waypoints_.clear();
    waypoints_yaw_.clear();
    Eigen::VectorXd wp = Eigen::VectorXd::Zero(traj_dim_);
    double yaw_temp;
    bool gripper_close;
    for (int i = 0; i < waypoint_num_; i++){
      nh.param("fsm/waypoint" + to_string(i) + "_yaw", yaw_temp, -1.0);//加载偏航角
      waypoints_yaw_.push_back(yaw_temp * M_PI / 180.0);

      nh.param("fsm/waypoint" + to_string(i) + "_gripper_close", gripper_close, true);//加载夹爪状态
      waypoint_gripper_close_.push_back(gripper_close);

      std::vector<double> waypoints_temp;
      nh.getParam("fsm/waypoint" + to_string(i), waypoints_temp);//加载路径点
      for(unsigned int j = 0; j < waypoints_temp.size(); j++){
        wp(j) = waypoints_temp[j];
        if((int)j >= mobile_base_dim_) wp(j) = wp(j) * M_PI / 180.0;//角度转弧度
      }
      waypoints_.push_back(wp);
    }
    
    init_time_list_.clear();
    opt_time_list_.clear();
    total_time_list_.clear();

    rcv_gripper_state_ = false;
    gripper_state_ = false;

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new MMPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &REMANIReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.01), &REMANIReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &REMANIReplanFSM::mmCarOdomCallback, this);
    joint_state_sub_ = nh.subscribe("joint_state", 1, &REMANIReplanFSM::mmManiOdomCallback, this);
    gripper_state_sub_ = nh.subscribe("gripper_state", 1, &REMANIReplanFSM::gripperCallback, this);

    poly_traj_pub_ = nh.advertise<quadrotor_msgs::PolynomialTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);

    gripper_cmd_pub_ = nh.advertise<std_msgs::Bool>("gripper_cmd", 100);
    start_pub_ = nh.advertise<std_msgs::Bool>("planning/start", 1);
    reached_pub_ = nh.advertise<std_msgs::Bool>("planning/finish", 1);
    waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &REMANIReplanFSM::waypointCallback, this);

    if (manual_mapping_mode_) {
      ROS_WARN("[FSM] manual_mapping_mode enabled: GridMap/frontier extraction stay active, but REMANI trajectory planning is bypassed.");
    }
    
  }

  void REMANIReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage，开始执行时，立即停止定时器，防止重入，等执行完在开启

    static int fsm_num = 0;//计数调试
    fsm_num++;
    if (fsm_num == 100){
      fsm_num = 0;
      // printFSMExecState();
    }

    if (manual_mapping_mode_) {
      if (have_odom_ && exec_state_ != WAIT_TARGET) {
        changeFSMExecState(WAIT_TARGET, "MANUAL_MAPPING");
      }
      data_disp_.header.stamp = ros::Time::now();
      data_disp_pub_.publish(data_disp_);
      goto force_return;
    }

    switch (exec_state_){
    case INIT:
    {
      if (!have_odom_){
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (target_type_ == TARGET_TYPE::NEAREST_FRONTIER && have_trigger_ && !have_target_ &&
          ros::Time::now() >= next_frontier_retry_time_) {
        const FrontierPlanResult result = planNearestFrontier(false);
        if (result == FrontierPlanResult::EXPLORATION_COMPLETE) {
          have_trigger_ = false;
          ROS_INFO("[Exploration] Finished: confirmed no frontier in 3 consecutive detections.");
        }
      }
      if (!have_target_)//如果没有目标点，则跳出循环。
        goto force_return; // return;
      else{
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ://生成新轨迹状态
    {
      if(try_plan_after_emergency_){//如果是紧急停车状态后发起的重新规划则打印底盘和机械臂的状态信息
        std::cout << "emergency stop mm pos: " << mm_state_pos_.transpose() << std::endl;
        std::cout << "emergency stop mm vel: " << mm_state_vel_.transpose() << std::endl;
        std::cout << "emergency stop mm acc: " << mm_state_acc_.transpose() << std::endl;
        std::cout << "emergency stop mm yaw: " << mm_car_yaw_ << std::endl;
      }
      // std::cout << "gen new traj 1\n";
      have_local_traj_ = false;//清除“已有局部轨迹”标志，表示接下来会生成一条全新的轨迹，而不是在旧轨迹上接续。
      const int global_trial_times = target_type_ == TARGET_TYPE::NEAREST_FRONTIER ? 2 : 10;
      bool success = planFromGlobalTraj(global_trial_times);//调用全局规划函数生成新轨迹
      // std::cout << "gen new traj 2\n";
      if (success){
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        try_plan_after_emergency_ = false;
      }
      else
      {
        if (target_type_ == TARGET_TYPE::NEAREST_FRONTIER) {
          const int failed_frontier_id = current_frontier_id_;
          addFailedFrontierGoal(failed_frontier_id, end_pt_.head<2>(), "global_replan_failed");
          unlockFrontierGoal("global planning failed");
          if (!frontier_auto_continue_) {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            break;
          }
          ROS_WARN("[Exploration] Failed to plan to frontier id=%d, try next frontier.",
                   failed_frontier_id);
          have_target_ = false;
          current_frontier_id_ = -1;
          const FrontierPlanResult result = planNearestFrontier(false, failed_frontier_id);
          if (result == FrontierPlanResult::SELECTED) {
            changeFSMExecState(GEN_NEW_TRAJ, "FRONTIER_FAILOVER");
          } else {
            have_target_ = false;
            changeFSMExecState(WAIT_TARGET, "FSM");
          }
          break;
        }
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      
      if(planFromLocalTraj(flag_relan_astar_)){//局部重规划
        replan_fail_time_ = 0;
        flag_relan_astar_ = false;
        //A*触发机制，每秒最多一次
        if((ros::Time::now() - t_last_Astar_ ).toSec() > 1.0){
          std::cout << "cal front end next time" << std::endl;
          flag_relan_astar_ = true;
          t_last_Astar_ = ros::Time::now();
        }
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else{
        replan_fail_time_++;//失败记数
        flag_relan_astar_ = true;
        t_last_Astar_ = ros::Time::now();
        if(replan_fail_time_ >= 20){
          replan_fail_time_ = 0;
          ROS_ERROR("[FSM]:REPLAN fail over 20 times!!!");
          if (target_type_ == TARGET_TYPE::NEAREST_FRONTIER) {
            addFailedFrontierGoal(current_frontier_id_, end_pt_.head<2>(), "local_replan_failed");
            unlockFrontierGoal("continuous local planning failures");
          }
          changeFSMExecState(WAIT_TARGET, "FSM");//超过20次，切换到等待触发
        }
        else{
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan *///获取当前轨迹信息
      SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
      // LocalTrajData *info = &planner_manager_->traj_container_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;//从轨迹开始到现在的时间
      //关键时间判断
      bool need_to_plan_next = ((t_cur - info->duration) > time_for_gripper_);//判断当前时间是否超过了轨迹时长
      bool need_to_gripper = (t_cur > info->duration + 0.01);//当前时间是否超过轨迹时长，确保机器人到位后再夹爪
      t_cur = min(info->duration, t_cur);//限制不超过轨迹时长
      //位置状态判断
      Eigen::VectorXd pos = info->getPos(t_cur);//当前机器人的位置状态
      bool touch_the_goal = ((local_target_pt_ - end_pt_).norm() < 1e-2);//是否接触到目标点
      bool close_to_no_replan_thresh = ((end_pt_ - pos).head(2).norm() < no_replan_thresh_);//是否接近无重规划阀值//只考虑前2维（底盘位置），忽略了机械臂状态，当接近目标时停止不必要的重规划
      //多路径点任务
      if((target_type_ == TARGET_TYPE::PRESET_TARGET) && close_to_no_replan_thresh){//预设目标模式且接近无重规划阀值
        if((wpt_id_ < waypoint_num_ - 1) && need_to_plan_next){
          ++wpt_id_;//移动到下一个路径点索引
          planNextWaypoint(waypoints_[wpt_id_], waypoints_yaw_[wpt_id_]);//规划到新目标点
          gripper_flag_ = true;//标记需要夹爪操作
        }else if(need_to_gripper && gripper_flag_){
          //机械臂夹爪控制
          std_msgs::Bool gripper_cmd;
          gripper_cmd.data = waypoint_gripper_close_[wpt_id_]; // true: close gripper; false: open
          gripper_cmd_pub_.publish(gripper_cmd);//发布夹爪控制命令

          std::string gripper_cmd_str = waypoint_gripper_close_[wpt_id_] ? "close gripper" : "open gripper";
          ROS_INFO(gripper_cmd_str.c_str());//打印夹爪操作


          // planner_manager_->grid_map_->md_.has_cloud_ = false;

          gripper_flag_ = false;//为了防止重复操作
        }
        //单目标点到达判断
      }else if(t_cur > info->duration - 1e-2 && touch_the_goal){

        if(target_type_ == TARGET_TYPE::NEAREST_FRONTIER && frontier_auto_continue_){
          const bool reached_frontier_goal =
              !frontier_target_is_intermediate_ &&
              (end_pt_.head(2) - mm_state_pos_.head(2)).norm() < frontier_reach_dist_;
          const int completed_frontier_id = current_frontier_id_;
          if (reached_frontier_goal) {
            addVisitedFrontierGoal(completed_frontier_id, end_pt_.head<2>());
            unlockFrontierGoal("frontier reached");
          }
          const int excluded_frontier_id = reached_frontier_goal ? completed_frontier_id : -1;
          const FrontierPlanResult result = planNearestFrontier(false, excluded_frontier_id);
          if(result == FrontierPlanResult::SELECTED){
            changeFSMExecState(GEN_NEW_TRAJ, "FRONTIER");
          }else{
            have_target_ = false;
            if (result == FrontierPlanResult::EXPLORATION_COMPLETE) {
              have_trigger_ = false;
              ROS_INFO("[Exploration] Finished: confirmed no frontier in 3 consecutive detections.");
              unlockFrontierGoal("exploration complete");
            }
            changeFSMExecState(WAIT_TARGET, "FSM");
          }
          goto force_return;
        }
        
        if(target_type_ != TARGET_TYPE::PRESET_TARGET && wpt_id_ >= waypoint_num_ - 1){
          if (target_type_ == TARGET_TYPE::NEAREST_FRONTIER && current_frontier_id_ >= 0 &&
              !frontier_target_is_intermediate_ &&
              (end_pt_.head(2) - mm_state_pos_.head(2)).norm() < frontier_reach_dist_) {
            addVisitedFrontierGoal(current_frontier_id_, end_pt_.head<2>());
            unlockFrontierGoal("frontier reached");
          }
          have_target_ = false;
          have_trigger_ = false;
          /* The navigation task completed */
          std::cout << "reach goal\n";
          changeFSMExecState(WAIT_TARGET, "FSM");//任务完成
          //发布到达消息
          std_msgs::Bool msg;
          msg.data = true;
          reached_pub_.publish(msg);//reached_pub_:发布到达消息
          goto force_return;
        }
        //需要重规划的条件
      }else if(target_type_ == TARGET_TYPE::NEAREST_FRONTIER && frontier_auto_continue_ &&
               frontier_replan_during_execution_ &&
               !close_to_no_replan_thresh && !global_plan_ &&
               (ros::Time::now() - last_frontier_plan_time_).toSec() > frontier_replan_period_){
        if(planNearestFrontier(true) == FrontierPlanResult::SELECTED){
          changeFSMExecState(GEN_NEW_TRAJ, "FRONTIER_REPLAN");
          goto force_return;
        }
      }else if(!close_to_no_replan_thresh && t_cur > replan_thresh_ && (!global_plan_)){
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {
      if(flag_escape_emergency_){ // Avoiding repeated calls
        callEmergencyStop(mm_state_pos_, mm_car_yaw_, mm_car_singul_);
      }
      else{
        if(enable_fail_safe_ && mm_state_vel_.head(2).norm() < 0.1){
          try_plan_after_emergency_ = true;
          have_local_traj_ = false;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }

      flag_escape_emergency_ = false;

      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  void REMANIReplanFSM::checkCollisionCallback(const ros::TimerEvent &e){
    if (manual_mapping_mode_)
      return;

    SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;
    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout()){
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }
    // std::cout << "check 3" << std::endl;
    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    Eigen::VectorXd p_cur = info->getPos(t_cur);
    double t_1_2 = info->duration * 1 / 2;
    double t_2_3 = info->duration * 2 / 3;
    double t_temp;
    bool occ = false;
    // std::cout << "check 4" << std::endl;
    int coll_type;
    for (double t = t_cur; t < info->duration; t += time_step){
      // If t_cur < t_1_2, only the first 2/3 partition of the trajectory is considered valid and will get checked.
      if (t_cur < t_1_2 && t >= t_2_3)
        break;
        
      if (planner_manager_->ploy_traj_opt_->checkCollision(*info, t, coll_type)){
        if(coll_type == 0){
          ROS_WARN("car collision at relative time %f!", t / info->duration);
        }else if (coll_type == 1){
          ROS_WARN("mani collision at relative time %f!", t / info->duration);
        }else if (coll_type == 2){
          ROS_WARN("car-mani collision at relative time %f!", t / info->duration);
        }else if (coll_type == 3){
          ROS_WARN("mani-mani collision at relative time %f!", t / info->duration);
        }
        
        t_temp = t;
        occ = true;
        break;
      }
    }

    if (occ){
      /* Handle the collided case immediately */
      ROS_INFO("Try to replan a safe trajectory");
      if (target_type_ == TARGET_TYPE::NEAREST_FRONTIER) {
        unlockFrontierGoal("current trajectory became infeasible");
      }
      if (planFromLocalTraj(false)){ // Make a chance
        ROS_INFO("Plan success when detect collision.");
        changeFSMExecState(EXEC_TRAJ, "SAFETY");
        return;
      }else{
        // if(planFromLocalTraj(true))
        // {
        //   ROS_INFO("Plan success when detect collision.");
        //   changeFSMExecState(EXEC_TRAJ, "SAFETY");
        //   return;
        // }
        if (t_temp - t_cur < emergency_time_){ // 1.0s of emergency time
          ROS_WARN("Emergency stop! time=%f", t_temp - t_cur);
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        }else{
          ROS_WARN("current traj in collision, replan.");
          if(planFromLocalTraj(true))
          {
            ROS_INFO("Plan success when detect collision.");
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
        return;
      }
    }
  }

  bool REMANIReplanFSM::planNextWaypoint(const Eigen::VectorXd next_wp, const double next_yaw)//为多路径点任务中的下一个目标点生成全局轨迹
  {
    std::vector<Eigen::VectorXd> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    bool success = planner_manager_->planGlobalTrajWaypoints(
        mm_state_pos_, mm_car_yaw_, Eigen::VectorXd::Zero(traj_dim_), Eigen::VectorXd::Zero(traj_dim_),
        one_pt_wps, next_yaw, Eigen::VectorXd::Zero(traj_dim_), Eigen::VectorXd::Zero(traj_dim_));

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = next_wp;//更新目标点
      end_yaw_ = next_yaw;//更新目标偏航角
      have_local_traj_ = false;
      start_singul_ = 0;//重置起始奇异状态

      /*** display ***/
      constexpr double step_size_t = 0.1;//采样间隔0.1秒
      int i_end = floor(planner_manager_->traj_container_.global_traj.duration / step_size_t);//采样点数=轨迹时长/采样间隔
      vector<Eigen::Vector2d> global_traj(i_end);
      for (int i = 0; i < i_end; i++){
        /*取前mobile_base_dim_维（底盘位置）用于2D显示 */
        global_traj[i] = planner_manager_->traj_container_.global_traj.traj.getPos(i * step_size_t).head(mobile_base_dim_);
      }

      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ != WAIT_TARGET && exec_state_ != GEN_NEW_TRAJ)
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(final_goal_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGoalPoint(end_pt_.head(2), Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalTraj(global_traj, 0.05, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

    return success;
  }

  REMANIReplanFSM::FrontierPlanResult REMANIReplanFSM::planNearestFrontier(
      const bool allow_keep_current, const int excluded_frontier_id)
  {
    const auto retryLater = [this]() {
      next_frontier_retry_time_ = ros::Time::now() + ros::Duration(0.75);
      return FrontierPlanResult::TEMPORARILY_UNREACHABLE;
    };
    if (!planner_manager_ || !planner_manager_->frontier_goal_provider_) {
      ROS_WARN("[Exploration] Frontier goal provider is not ready.");
      return retryLater();
    }
    if (!planner_manager_->frontier_goal_provider_->enabled()) {
      ROS_WARN("[Exploration] Frontier goal provider is disabled.");
      return retryLater();
    }
    if (!planner_manager_->grid_map_ || !planner_manager_->grid_map_->hasDepthObservation()) {
      ROS_WARN("[FSM]: no depth observation yet, cannot select nearest frontier.");
      return retryLater();
    }
    if (manipulator_dim_ > 0 && !have_joint_state_) {
      ROS_WARN("[Exploration] Joint state is not ready; skip nearest frontier planning.");
      return retryLater();
    }
    Eigen::Vector3i voxel_num = Eigen::Vector3i::Zero();
    planner_manager_->grid_map_->getVoxelNum(voxel_num);
    if (voxel_num.minCoeff() <= 0) {
      ROS_WARN("[Exploration] Grid map size is not ready; skip nearest frontier planning.");
      return retryLater();
    }

    const Eigen::Vector2d robot_pos = mm_state_pos_.head<2>();
    const Eigen::Vector3d robot_state(mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_);
    ExplorationGoal selected_goal;
    std::vector<ExplorationGoal> feasible_goals;
    std::vector<ExplorationGoal> candidate_goals;
    planner_manager_->frontier_goal_provider_->getGoals(robot_state, candidate_goals);
    bool found_frontier = false;
    int skipped_too_close = 0;
    int skipped_visited = 0;
    int skipped_failed = 0;
    int skipped_no_path = 0;
    int skipped_height = 0;
    int skipped_invalid_arm = 0;
    const double min_select_dist_sq = frontier_min_select_dist_ * frontier_min_select_dist_;

    const auto yawDiff = [](const double a, const double b) {
      return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
    };

    std::map<int, int> attempted_planning_count;
    std::map<int, int> planning_failure_count;
    std::map<int, std::vector<Eigen::Vector2d>> planning_failure_positions;
    const auto recordPlanningFailure = [&](const ExplorationGoal& goal) {
      ++planning_failure_count[goal.frontier_id];
      planning_failure_positions[goal.frontier_id].push_back(goal.viewpoint.head<2>());
    };

    for (const ExplorationGoal& candidate_goal : candidate_goals) {
      if (candidate_goal.frontier_id == excluded_frontier_id) continue;

      if (candidate_goal.frontier_average.z() < frontier_min_z_ ||
          candidate_goal.frontier_average.z() > frontier_max_z_) {
        ++skipped_height;
        continue;
      }

      const Eigen::Vector2d candidate_pos = candidate_goal.viewpoint.head<2>();
      if (!candidate_goal.in_place &&
          (candidate_pos - robot_pos).squaredNorm() < min_select_dist_sq) {
        ++skipped_too_close;
        continue;
      }

      if (isVisitedFrontierGoal(candidate_goal.frontier_id, candidate_pos)) {
        ++skipped_visited;
        continue;
      }
      if (isFailedFrontierGoal(candidate_goal.frontier_id, candidate_pos)) {
        ++skipped_failed;
        continue;
      }

      if (manipulator_dim_ > 0 &&
          static_cast<int>(candidate_goal.arm_config.size()) != manipulator_dim_) {
        ++skipped_invalid_arm;
        ROS_WARN("[Exploration] Candidate frontier id=%d arm_id=%d has invalid arm_config size=%zu, expected=%d.",
                 candidate_goal.frontier_id, candidate_goal.arm_config_id,
                 candidate_goal.arm_config.size(), manipulator_dim_);
        ++attempted_planning_count[candidate_goal.frontier_id];
        recordPlanningFailure(candidate_goal);
        continue;
      }

      double path_len = (candidate_goal.viewpoint.head<2>() - robot_pos).norm();
      if (planner_manager_->frontier_path_finder_ &&
          planner_manager_->frontier_path_finder_->enabled()) {
        std::vector<Eigen::Vector3d> path;
        const Eigen::Vector3d goal_state(
            candidate_goal.viewpoint.x(), candidate_goal.viewpoint.y(), candidate_goal.yaw);
        if (!planner_manager_->frontier_path_finder_->search(
                robot_state, goal_state, candidate_goal.yaw, path, path_len)) {
          ++skipped_no_path;
          ++attempted_planning_count[candidate_goal.frontier_id];
          recordPlanningFailure(candidate_goal);
          continue;
        }
      }

      const double yaw_cost = yawDiff(candidate_goal.yaw, mm_car_yaw_);
      double score = 0.0;
      double arm_motion_cost = 0.0;
      if (manipulator_dim_ > 0 &&
          static_cast<int>(candidate_goal.arm_config.size()) == manipulator_dim_) {
        for (int i = 0; i < manipulator_dim_; ++i) {
          const double dq =
              candidate_goal.arm_config[i] - mm_state_pos_(mobile_base_dim_ + i);
          arm_motion_cost += dq * dq;
        }
        arm_motion_cost = std::sqrt(arm_motion_cost);
      }
      score = frontier_path_weight_ * path_len +
              frontier_yaw_weight_ * yaw_cost -
          frontier_visible_weight_ * static_cast<double>(candidate_goal.visible_num) -
          frontier_unknown_weight_ * static_cast<double>(candidate_goal.unknown_gain) +
          frontier_arm_motion_weight_ * arm_motion_cost;
      ExplorationGoal feasible_goal = candidate_goal;
      feasible_goal.path_cost = path_len;
      feasible_goal.score = score;
      feasible_goals.push_back(feasible_goal);
    }

    found_frontier = !feasible_goals.empty();
    if (found_frontier) selected_goal = feasible_goals.front();

    const auto markPlanningFailedFrontiers = [&]() {
      for (const auto& item : attempted_planning_count) {
        const int frontier_id = item.first;
        if (item.second <= 0 || planning_failure_count[frontier_id] != item.second) continue;
        for (const Eigen::Vector2d& pos : planning_failure_positions[frontier_id]) {
          addFailedFrontierGoal(frontier_id, pos, "all_planning_attempts_failed");
        }
        ROS_WARN("[Exploration] All %d planning attempts failed for frontier id=%d; mark it temporarily failed.",
                 item.second, frontier_id);
      }
    };

    ROS_INFO("[Exploration] frontier candidate filter summary: generated=%zu, skipped_height=%d, skipped_too_close=%d, skipped_visited=%d, skipped_failed=%d, skipped_no_path=%d, skipped_invalid_arm=%d",
             candidate_goals.size(), skipped_height, skipped_too_close, skipped_visited, skipped_failed,
             skipped_no_path, skipped_invalid_arm);

    if (!found_frontier) {
      markPlanningFailedFrontiers();
      size_t raw_frontier_cells = 0;
      if (planner_manager_->fuel_frontier_cache_) {
        raw_frontier_cells = planner_manager_->fuel_frontier_cache_->cells().size();
      } else if (planner_manager_->fuel_frontier_detector_) {
        raw_frontier_cells = planner_manager_->fuel_frontier_detector_->rawFrontierCells().size();
      }
      size_t clusters = 0;
      if (planner_manager_->fuel_frontier_clusterer_) {
        const std::vector<Eigen::Vector3i>& cells = planner_manager_->fuel_frontier_cache_
            ? planner_manager_->fuel_frontier_cache_->cells()
            : planner_manager_->fuel_frontier_detector_->rawFrontierCells();
        clusters = planner_manager_->fuel_frontier_clusterer_->cluster(cells).size();
      }
      const size_t targets = planner_manager_->fuel_frontier_target_manager_
          ? planner_manager_->fuel_frontier_target_manager_->targets().size() : 0;
      if (raw_frontier_cells == 0 && clusters == 0 && targets == 0) {
        ++frontier_empty_detection_count_;
      } else {
        frontier_empty_detection_count_ = 0;
      }
      ROS_WARN("[Exploration] No reachable viewpoint: raw_frontier_cells=%zu, clusters=%zu, targets=%zu, empty_confirmations=%d/3; retrying.",
               raw_frontier_cells, clusters, targets, frontier_empty_detection_count_);
      next_frontier_retry_time_ = ros::Time::now() + ros::Duration(0.75);
      return frontier_empty_detection_count_ >= 3
          ? FrontierPlanResult::EXPLORATION_COMPLETE
          : FrontierPlanResult::TEMPORARILY_UNREACHABLE;
    }
    frontier_empty_detection_count_ = 0;

    if (allow_keep_current && frontier_goal_locked_ && current_frontier_id_ >= 0 && have_target_) {
      last_frontier_plan_time_ = ros::Time::now();
      if (selected_goal.frontier_id == current_frontier_id_) return FrontierPlanResult::TEMPORARILY_UNREACHABLE;
      const double committed_time = (ros::Time::now() - frontier_goal_lock_time_).toSec();
      if (committed_time < frontier_min_commit_time_) return FrontierPlanResult::TEMPORARILY_UNREACHABLE;
      const double score_improvement = current_frontier_score_ - selected_goal.score;
      if (score_improvement < frontier_switch_score_margin_) return FrontierPlanResult::TEMPORARILY_UNREACHABLE;
      unlockFrontierGoal("significantly better frontier");
    }

    last_frontier_plan_time_ = ros::Time::now();

    for (const ExplorationGoal& goal : feasible_goals) {
      ++attempted_planning_count[goal.frontier_id];
      Eigen::VectorXd next_wp = Eigen::VectorXd::Zero(traj_dim_);
      next_wp.head<2>() = goal.viewpoint.head<2>();
      for (int i = 0; i < manipulator_dim_; ++i) {
        next_wp(mobile_base_dim_ + i) = goal.arm_config[i];
      }

      ROS_INFO("[FSM]: try frontier viewpoint id=%d candidate=%d arm_id=%d goal=(%.2f, %.2f), yaw=%.2f, visible=%d, unknown=%d, path_cost=%.2f, score=%.2f",
               goal.frontier_id, goal.candidate_id, goal.arm_config_id,
               goal.viewpoint.x(), goal.viewpoint.y(), goal.yaw,
               goal.visible_num, goal.unknown_gain, goal.path_cost, goal.score);

      if (planNextWaypoint(next_wp, goal.yaw)) {
        frontier_target_is_intermediate_ = false;
        current_frontier_id_ = goal.frontier_id;
        current_candidate_id_ = goal.candidate_id;
        current_frontier_score_ = goal.score;
        current_frontier_viewpoint_ = goal.viewpoint.head<2>();
        frontier_goal_locked_ = true;
        frontier_goal_lock_time_ = ros::Time::now();
        return FrontierPlanResult::SELECTED;
      }

      ROS_WARN("[Exploration] Candidate planning failed: frontier=%d candidate=%d arm_id=%d; try next candidate.",
               goal.frontier_id, goal.candidate_id, goal.arm_config_id);
      recordPlanningFailure(goal);
    }
    markPlanningFailedFrontiers();
    return retryLater();
  }

  void REMANIReplanFSM::unlockFrontierGoal(const std::string& reason)
  {
    if (frontier_goal_locked_) {
      ROS_INFO("[Exploration] Unlock frontier id=%d candidate=%d: %s.",
               current_frontier_id_, current_candidate_id_, reason.c_str());
    }
    frontier_goal_locked_ = false;
    frontier_goal_lock_time_ = ros::Time(0);
    current_frontier_id_ = -1;
    current_candidate_id_ = -1;
    current_frontier_score_ = std::numeric_limits<double>::infinity();
    current_frontier_viewpoint_.setZero();
  }

  bool REMANIReplanFSM::isVisitedFrontierGoal(const int frontier_id,
                                               const Eigen::Vector2d& pos) const
  {
    if (frontier_id < 0 || frontier_visited_radius_ <= 1e-6) return false;
    for (const auto& item : visited_goal_list_) {
      if (item.frontier_id == frontier_id &&
          (pos - item.pos).norm() < frontier_visited_radius_) {
        return true;
      }
    }
    return false;
  }

  bool REMANIReplanFSM::isFailedFrontierGoal(const int frontier_id,
                                              const Eigen::Vector2d& pos) const
  {
    if (frontier_id < 0 || frontier_failed_goal_radius_ <= 1e-6) return false;
    const ros::Time now = ros::Time::now();
    for (const auto& item : failed_goal_list_) {
      if (item.frontier_id != frontier_id) continue;
      if (frontier_failed_goal_timeout_ > 0.0 &&
          (now - item.stamp).toSec() > frontier_failed_goal_timeout_) continue;
      if ((pos - item.pos).norm() < frontier_failed_goal_radius_) return true;
    }
    return false;
  }

  void REMANIReplanFSM::trimFrontierGoalMemory(std::vector<FrontierGoalMemory>& memory)
  {
    while (static_cast<int>(memory.size()) > frontier_memory_max_size_) {
      memory.erase(memory.begin());
    }
  }

  void REMANIReplanFSM::addVisitedFrontierGoal(const int frontier_id, const Eigen::Vector2d& pos)
  {
    if (frontier_id < 0) return;
    if (isVisitedFrontierGoal(frontier_id, pos)) return;
    FrontierGoalMemory visited;
    visited.frontier_id = frontier_id;
    visited.pos = pos;
    visited.stamp = ros::Time::now();
    visited.reason = "reached";
    visited_goal_list_.push_back(visited);
    trimFrontierGoalMemory(visited_goal_list_);
    ROS_INFO("[Exploration] Add visited frontier goal id=%d, pos=(%.2f, %.2f), list_size=%zu",
             frontier_id, pos.x(), pos.y(), visited_goal_list_.size());
  }

  void REMANIReplanFSM::addFailedFrontierGoal(const int frontier_id,
                                              const Eigen::Vector2d& pos,
                                              const std::string& reason)
  {
    if (frontier_id < 0) return;
    if (isFailedFrontierGoal(frontier_id, pos)) return;
    FrontierGoalMemory failed;
    failed.frontier_id = frontier_id;
    failed.pos = pos;
    failed.stamp = ros::Time::now();
    failed.reason = reason;
    failed_goal_list_.push_back(failed);
    trimFrontierGoalMemory(failed_goal_list_);
    ROS_INFO("[Exploration] Add failed frontier goal id=%d, pos=(%.2f, %.2f), reason=%s, list_size=%zu",
             frontier_id, pos.x(), pos.y(), reason.c_str(), failed_goal_list_.size());
  }

  // manual waypoint
  void REMANIReplanFSM::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    
    if (target_type_ == TARGET_TYPE::PRESET_TARGET){
      have_trigger_ = true;
      cout << "Triggered! traget type: " << target_type_ << endl;

      std_msgs::Bool flag_msg;
      flag_msg.data = true;
      planner_manager_->global_start_time_ = ros::Time::now();
      planner_manager_->start_flag_ = true;
      start_pub_.publish(flag_msg);
      wpt_id_ = 0;
      planNextWaypoint(waypoints_[wpt_id_], waypoints_yaw_[wpt_id_]);
      return;
    }

    if (target_type_ == TARGET_TYPE::NEAREST_FRONTIER){
      if (exec_state_ != WAIT_TARGET) {
        ROS_WARN("[Exploration] Ignore nearest-frontier trigger while FSM is not WAIT_TARGET.");
        return;
      }
      have_trigger_ = true;
      cout << "Triggered! target type: " << target_type_ << endl;

      std_msgs::Bool flag_msg;
      flag_msg.data = true;
      planner_manager_->global_start_time_ = ros::Time::now();
      planner_manager_->start_flag_ = true;
      start_pub_.publish(flag_msg);

      const FrontierPlanResult result = planNearestFrontier(false);
      if(result != FrontierPlanResult::SELECTED){
        ROS_WARN("[Exploration] Triggered, but no nearest frontier is available.");
      }
      return;
    }

    if(msg->pose.position.z < -0.1)
      return;
    cout << "Triggered! traget type: " << target_type_ << endl;
    // trigger_ = true;
    init_state_ = mm_state_pos_;
    end_pt_ = Eigen::VectorXd::Zero(traj_dim_);
    
    if(target_type_ == TARGET_TYPE::MANUAL_TARGET){
      end_pt_(0) = msg->pose.position.x;
      end_pt_(1) = msg->pose.position.y;
      end_yaw_ = tf::getYaw(msg->pose.orientation);
    }else{
      ROS_ERROR("wrong target type: %d", target_type_);
      return;
    }
    
    planNextWaypoint(end_pt_, end_yaw_);
  }

  void REMANIReplanFSM::mmCarOdomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    // std::cout << "odom: " << mm_state_pos_.transpose() << "\n";
    mm_state_pos_(0) = msg->pose.pose.position.x;
    mm_state_pos_(1) = msg->pose.pose.position.y;
    mm_car_yaw_ = tf::getYaw(msg->pose.pose.orientation);
    if (planner_manager_) {
      planner_manager_->setCurrentBasePose(
          Eigen::Vector3d(mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_));
    }

    mm_car_orient_.w() = msg->pose.pose.orientation.w;
    mm_car_orient_.x() = msg->pose.pose.orientation.x;
    mm_car_orient_.y() = msg->pose.pose.orientation.y;
    mm_car_orient_.z() = msg->pose.pose.orientation.z;

    mm_state_vel_(0) = msg->twist.twist.linear.x;
    mm_state_vel_(1) = msg->twist.twist.linear.y;
    if(mm_state_vel_.head(2).norm() < mobile_base_non_singul_vel_){
      mm_state_vel_(0) = mobile_base_non_singul_vel_ * cos(mm_car_yaw_);
      mm_state_vel_(1) = mobile_base_non_singul_vel_ * sin(mm_car_yaw_);
      mm_car_singul_ = 1;
    }else{
      Eigen::Vector2d car_head(cos(mm_car_yaw_), sin(mm_car_yaw_));
      mm_car_singul_ = car_head.dot(mm_state_vel_.head<2>()) >= 0.0 ? 1 : -1;
    }

    mm_car_yaw_rate_ = msg->twist.twist.angular.z;

    have_odom_ = true;
  }

  void REMANIReplanFSM::mmManiOdomCallback(const sensor_msgs::JointStateConstPtr &msg){
    if ((int)msg->position.size() < manipulator_dim_) {
      ROS_WARN_THROTTLE(1.0, "[Exploration] joint_state position size %zu < manipulator_dim %d",
                        msg->position.size(), manipulator_dim_);
      return;
    }
    for(int i = 0; i < manipulator_dim_; ++i){
      mm_state_pos_(mobile_base_dim_ + i) = msg->position[i];
      mm_state_vel_(mobile_base_dim_ + i) =
          (int)msg->velocity.size() > i ? msg->velocity[i] : 0.0;
      mm_state_acc_(mobile_base_dim_ + i) =
          (int)msg->effort.size() > i ? msg->effort[i] : 0.0;
    }
    have_joint_state_ = true;
  }

  void REMANIReplanFSM::gripperCallback(const std_msgs::Bool::ConstPtr &msg){
    if(gripper_state_ != msg->data || (!rcv_gripper_state_)){
      rcv_gripper_state_ = true;
      gripper_state_ = msg->data;
      planner_manager_->mm_config_->setGripperPoint(gripper_state_);
    }
  }

  void REMANIReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call){
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void REMANIReplanFSM::printFSMExecState(){
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    static int last_printed_state = -1, dot_nums = 0;

    if (exec_state_ != last_printed_state)
      dot_nums = 0;
    else
      dot_nums++;

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)];

    last_printed_state = exec_state_;

    // some warnings
    if (!have_odom_)
    {
      cout << ", waiting for odom";
    }
    if (!have_target_)
    {
      cout << ", waiting for target";
    }
    if (!have_trigger_)
    {
      cout << ", waiting for trigger";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << ", haven't receive traj from previous drone";
    }

    cout << string(dot_nums, '.') << endl;

    fflush(stdout);
  }

  std::pair<int, REMANIReplanFSM::FSM_EXEC_STATE> REMANIReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void REMANIReplanFSM::sendPolyTrajROSMsg(){
    auto data = &planner_manager_->traj_container_.singul_traj_data;
    
    for(unsigned int i = 0; i < data->singul_traj.size(); ++i){
      quadrotor_msgs::PolynomialTraj msg;
      msg.trajectory_id = data->singul_traj[i].traj_id;
      msg.header.stamp = ros::Time(data->start_time);
      msg.action = msg.ACTION_ADD;
      msg.singul = data->singul_traj[i].singul;
      int piece_num = data->singul_traj[i].traj.getPieceNum();
      for (int j = 0; j < piece_num; ++j)
      {
        quadrotor_msgs::PolynomialMatrix piece;
        piece.num_dim = data->singul_traj[i].traj.getPiece(j).getDim();
        piece.num_order = data->singul_traj[i].traj.getPiece(j).getDegree();
        piece.duration = data->singul_traj[i].traj.getPiece(j).getDuration();
        auto cMat = data->singul_traj[i].traj.getPiece(j).getCoeffMat();
        piece.data.assign(cMat.data(),cMat.data() + cMat.rows()*cMat.cols());
        msg.trajectory.emplace_back(piece);
      }
      poly_traj_pub_.publish(msg);
    }

  }

  bool REMANIReplanFSM::planFromGlobalTraj(const int trial_times /*= 1*/){
    start_pos_ = mm_state_pos_;
    start_vel_ = mm_state_vel_;
    start_acc_.setZero();
    start_jer_.setZero();
    start_yaw_ = mm_car_yaw_;
    start_singul_ = mm_car_singul_;
    bool flag_random_poly_init;
    if(timesOfConsecutiveStateCalls().first == 1) flag_random_poly_init = false;//先用初值，不行再随机
    else flag_random_poly_init = true;
    for(int i = 0; i < trial_times; i++){//10次尝试，成功一次就返回成功，因为重规划是非凸问题
      if(callReboundReplan(true, flag_random_poly_init)){
        logKinoDirection();
        return true;
      }
    }
    return false;
  }

  bool REMANIReplanFSM::planFromLocalTraj(bool flag_use_poly_init){
    SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
    double t_cur = ros::Time::now().toSec() - info->start_time + replan_trajectory_time_;
    t_cur = min(info->duration, t_cur);

    start_pos_     = info->getPos(t_cur);
    start_vel_    = info->getVel(t_cur);
    start_acc_    = info->getAcc(t_cur);
    start_jer_   = info->getJer(t_cur);
    start_singul_ = info->getSingul(t_cur);
    if(start_vel_.norm() >= mobile_base_non_singul_vel_) start_yaw_ = atan2(start_singul_ * start_vel_(1), start_singul_ * start_vel_(0));
    else start_yaw_ = mm_car_yaw_;

    bool success = callReboundReplan(flag_use_poly_init, false);
    if (!success){
      for (int i = 0; i < 1; i++){
        success = callReboundReplan(true, true);
        if (success)
          break;
      }
      if (!success)
      {
        return false;
      }
    }

    logKinoDirection();
    return true;
  }

  void REMANIReplanFSM::logKinoDirection() const
  {
    const auto& trajs = planner_manager_->traj_container_.singul_traj_data.singul_traj;
    if (trajs.empty()) return;
    ROS_INFO("[KinoDirection] start_singul=%d first_singul=%d last_singul=%d",
             start_singul_, trajs.front().singul, trajs.back().singul);
  }

  bool REMANIReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj){
    bool reach_horizon;
    planner_manager_->getLocalTarget(
        planning_horizen_, start_pos_, start_yaw_, end_pt_, end_yaw_,
        local_target_pt_, local_target_vel_, local_target_acc_, reach_horizon);//取局部目标
    bool local_target_gripper;
    if(target_type_ == TARGET_TYPE::NEAREST_FRONTIER){
      local_target_gripper = gripper_state_;
    }else if(reach_horizon){//如果靠近最终目标，则局部目标夹爪状态与最终目标夹爪状态一致
      local_target_gripper = gripper_state_;
    }else{//否则局部目标夹爪状态与预设路径点夹爪状态一致
      local_target_gripper = waypoint_gripper_close_[wpt_id_];
    }
    local_target_acc_.setZero();//局部目标加速度置零
    double local_target_yaw = atan2(local_target_vel_(1), local_target_vel_(0)); //用原先 local_target_vel_ 的方向计算 local_target_yaw
    local_target_vel_.setZero();
    //再把速度幅值强制设为固定的非奇异底盘速度 mobile_base_non_singul_vel_，方向沿 local_target_yaw。给优化器一个“指向前方”的一致、非零速度边界，避免在奇异/停滞附近陷入数值问题。
    local_target_vel_.head(2) = mobile_base_non_singul_vel_ * Eigen::Vector2d(cos(local_target_yaw), sin(local_target_yaw));

    Eigen::VectorXd desired_start_pt, desired_start_vel, desired_start_acc, desired_start_jerk;
    int desired_start_singul;
    double desired_start_yaw;
    double desired_start_time, start_time_dura;
    
    if(have_local_traj_)//有正在执行的局部轨迹，则从这条轨迹上取起始状态
    {
      desired_start_time = ros::Time::now().toSec() + replan_trajectory_time_;//取“从现在起再过 replan_trajectory_time_ 秒”的未来状态作为新轨起点，让轨迹更光滑。
      start_time_dura = desired_start_time - planner_manager_->traj_container_.singul_traj_data.start_time;
      start_time_dura = min(start_time_dura, planner_manager_->traj_container_.singul_traj_data.duration);
      
      desired_start_pt = planner_manager_->traj_container_.singul_traj_data.getPos(start_time_dura);
      desired_start_vel = planner_manager_->traj_container_.singul_traj_data.getVel(start_time_dura);
      if(desired_start_vel.head(2).norm() < mobile_base_non_singul_vel_){//如果x、y速度过小，就按历史 start_singul_ 与 start_yaw_ 注入一个最小非奇异速度：
        desired_start_vel(0) = start_singul_ * mobile_base_non_singul_vel_ * cos(start_yaw_);
        desired_start_vel(1) = start_singul_ * mobile_base_non_singul_vel_ * sin(start_yaw_);
      }
      desired_start_singul = planner_manager_->traj_container_.singul_traj_data.getSingul(start_time_dura);
      desired_start_acc = planner_manager_->traj_container_.singul_traj_data.getAcc(start_time_dura);
      desired_start_jerk = planner_manager_->traj_container_.singul_traj_data.getJer(start_time_dura);
      desired_start_yaw = atan2(desired_start_singul * desired_start_vel(1), desired_start_singul * desired_start_vel(0));
    }else{//没有“正在执行的局部轨迹”（从当前状态起步）
      desired_start_time = ros::Time::now().toSec();
      desired_start_pt = start_pos_;
      desired_start_vel = start_vel_;
      if(desired_start_vel.head(2).norm() < mobile_base_non_singul_vel_){//同样作下限保护
        desired_start_vel(0) = start_singul_ * mobile_base_non_singul_vel_ * cos(start_yaw_);
        desired_start_vel(1) = start_singul_ * mobile_base_non_singul_vel_ * sin(start_yaw_);
      }
      desired_start_acc = start_acc_;
      desired_start_jerk = start_jer_;
      desired_start_yaw = start_yaw_;
      desired_start_singul = start_singul_;
    }
    // std::cout << "desired_start_singul: " << desired_start_singul << std::endl;
    double init_time, opt_time;
    
    bool plan_success = planner_manager_->reboundReplan(//调用一次“完整重规划”
        desired_start_pt, desired_start_vel, desired_start_acc,desired_start_jerk, desired_start_yaw, desired_start_singul, gripper_state_,//起始状态
        desired_start_time, //起始时间
        local_target_pt_, local_target_vel_, local_target_acc_, local_target_yaw, local_target_gripper,//局部目标状态
        (have_new_target_ || flag_use_poly_init),flag_randomPolyTraj, have_local_traj_, //控制规划行为的开关
        init_time, opt_time);//输出规划时间
    have_new_target_ = false;

    if (plan_success){//规划成功后的收尾工作
      //统计耗时
      init_time_list_.push_back(init_time);
      opt_time_list_.push_back(opt_time);
      total_time_list_.push_back(init_time + opt_time);
      //发布轨迹给控制器
      sendPolyTrajROSMsg();//把优化好的多项式轨迹封装成 ROS 消息发出去（上游代码里你用的是 planning/trajectory 话题），控制器/仿真就会“跟着跑”。
      //之后的重规划将接续这条新轨迹
      have_local_traj_ = true;

      //采样并可视化局部轨迹（仅底盘xy）
      int i_end = floor(planner_manager_->traj_container_.singul_traj_data.duration / 0.02);
      std::vector<Eigen::Vector2d> local_path_list;
      Eigen::Vector2d local_traj_pt;
      for(int i = 0; i < i_end; ++i){
        local_traj_pt = planner_manager_->traj_container_.singul_traj_data.getPos(i * 0.02).head(2);
        local_path_list.push_back(local_traj_pt);
      }
      visualization_->displayGlobalTraj(local_path_list, 0.05, 0);//把这串二维点发布成线条
      planner_manager_->ploy_traj_opt_->displayBackEndMesh(planner_manager_->traj_container_.singul_traj_data, false, gripper_state_);
    }

    return plan_success;
  }

  bool REMANIReplanFSM::callEmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw, const int singul){
    std::cout << "\033[31mcall EmergencyStop\033[0m" << std::endl;
    planner_manager_->EmergencyStop(stop_pos, stop_yaw, singul);
    quadrotor_msgs::PolynomialTraj msg;
    msg.action = quadrotor_msgs::PolynomialTraj::ACTION_ABORT;
    poly_traj_pub_.publish(msg);

    return true;
  }

} // namespace remani_planner
