#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <stdexcept>
#include <vector>
#include <string>
#include <unordered_map>
#include <sstream>
#include <cmath>

class TwistToMecanum
{
public:
  TwistToMecanum(ros::NodeHandle& nh)
  {
    // ---------------- 参数（底盘） ----------------
    nh.param("cmd_timeout", cmd_timeout_, 0.2);      // 秒
    nh.param("wheel_radius", r_, 0.07);              // 轮半径
    nh.param("lx", lx_, 0.185);                      // 前后半轴距
    nh.param("ly", ly_, 0.185);                      // 左右半轴距

    nh.param("sign_fl", s_fl_, 1.0);
    nh.param("sign_fr", s_fr_, 1.0);
    nh.param("sign_rl", s_rl_, 1.0);
    nh.param("sign_rr", s_rr_, 1.0);

    nh.param("wz_alpha", wz_alpha_, 0.2);
    nh.param("wz_limit", wz_limit_, 2.0);

    nh.param("gazebo_model_name", gazebo_model_name_, std::string("robot1"));
    nh.param("world_frame", world_frame_, std::string("world"));
    nh.param("gazebo_z", gazebo_z_, 0.2);
    nh.param("initial_x", target_x_, 0.0);
    nh.param("initial_y", target_y_, 0.0);
    nh.param("initial_yaw", target_yaw_, 0.0);
    target_yaw_ = normalizeAngle(target_yaw_);
    filt_x_ = target_x_;
    filt_y_ = target_y_;
    filt_yaw_ = target_yaw_;
    has_base_target_ = true;

    // keep set_model_state path, but with rate-limited filtered injection
    nh.param("base_set_rate", base_set_rate_, 50.0);
    nh.param("base_pose_alpha", base_pose_alpha_, 0.35);
    nh.param("base_yaw_alpha", base_yaw_alpha_, 0.35);
    nh.param("base_apply_twist", base_apply_twist_, false);
    nh.param("base_xy_deadband", base_xy_deadband_, 1e-4);
    nh.param("base_yaw_deadband", base_yaw_deadband_, 1e-4);

    base_set_rate_ = std::max(1.0, base_set_rate_);
    base_pose_alpha_ = std::min(1.0, std::max(0.0, base_pose_alpha_));
    base_yaw_alpha_ = std::min(1.0, std::max(0.0, base_yaw_alpha_));

    set_model_state_client_ =
        nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    ROS_INFO("[bridge] waiting for /gazebo/set_model_state ...");
    set_model_state_client_.waitForExistence(ros::Duration(5.0));

    // 初始化低通滤波状态
    wz_lp_ = 0.0;

    // ---------------- 参数（机械臂） ----------------
    nh.param("arm_publish_rate", arm_publish_rate_, 50.0);       // Hz
    nh.param("arm_time_from_start", arm_time_from_start_, 0.01); // s

    // 关节名：优先从参数读取，也可从 /robot1/joint_controller/joints 兜底
    if (!nh.getParam("arm_joint_names", arm_joint_names_) || arm_joint_names_.empty())
    {
      if (!ros::param::get("/robot1/joint_controller/joints", arm_joint_names_) || arm_joint_names_.empty())
      {
        ROS_ERROR("Cannot get arm_joint_names (param ~arm_joint_names or /robot1/joint_controller/joints).");
        throw std::runtime_error("arm_joint_names empty");
      }
    }

    // ---------------- 机械臂映射参数 ----------------
    const std::size_t n = arm_joint_names_.size();
    arm_index_map_.resize(n);
    arm_sign_.assign(n, 1.0);
    arm_offset_.assign(n, 0.0);

    for (std::size_t j = 0; j < n; ++j) arm_index_map_[j] = static_cast<int>(j);

    nh.getParam("arm_index_map", arm_index_map_);
    nh.getParam("arm_sign", arm_sign_);

    bool has_offset_param = nh.getParam("arm_offset", arm_offset_);
    if (!has_offset_param)
    {
      arm_offset_.assign(n, 0.0);
      if (n >= 4)
      {
        arm_offset_[1] = M_PI / 2.0;
        arm_offset_[3] = M_PI / 2.0;
      }
      ROS_WARN("[bridge] ~arm_offset not provided. Use default offset: [0, +pi/2, 0, +pi/2, 0, 0] (n=%zu)", n);
    }

    if (arm_index_map_.size() != n) {
      ROS_ERROR("arm_index_map size (%zu) != arm_joint_names size (%zu)", arm_index_map_.size(), n);
      throw std::runtime_error("arm_index_map bad size");
    }
    if (arm_sign_.size() != n) {
      ROS_ERROR("arm_sign size (%zu) != arm_joint_names size (%zu)", arm_sign_.size(), n);
      throw std::runtime_error("arm_sign bad size");
    }
    if (arm_offset_.size() != n) {
      ROS_ERROR("arm_offset size (%zu) != arm_joint_names size (%zu)", arm_offset_.size(), n);
      throw std::runtime_error("arm_offset bad size");
    }

    // ---------------- publisher（机械臂轨迹） ----------------
    pub_arm_ = nh.advertise<trajectory_msgs::JointTrajectory>("/robot1/joint_controller/command", 1);
    pub_arm_state_ = nh.advertise<sensor_msgs::JointState>("/mm/mani/joint_state", 1);

    // ---------------- subscriber ----------------
    sub_car_ = nh.subscribe(car_cmd_topic_, 1, &TwistToMecanum::cbCar, this);
    sub_arm_ = nh.subscribe("/mm_controller_node/joint_cmd", 1, &TwistToMecanum::cbArmState, this);
    sub_gz_joint_state_ = nh.subscribe("/robot1/joint_states", 10, &TwistToMecanum::cbGazeboJointState, this);

    // ---------------- timers ----------------
    timer_ = nh.createTimer(ros::Duration(0.05), &TwistToMecanum::onTimer, this);
    base_timer_ = nh.createTimer(ros::Duration(1.0 / base_set_rate_), &TwistToMecanum::onBaseTimer, this);
    arm_timer_ = nh.createTimer(ros::Duration(1.0 / arm_publish_rate_), &TwistToMecanum::onArmTimer, this);

    // ---------------- 初始化状态变量 ----------------
    last_car_stamp_ = ros::Time::now();
    last_arm_stamp_ = ros::Time::now();
    last_cmd_stamp_ = ros::Time::now();
    last_yaw_ = 0.0;
    has_last_yaw_ = false;
    enabled_ = true;

    ROS_INFO("[bridge] arm joints loaded (%zu)", arm_joint_names_.size());

    {
      std::ostringstream oss;
      oss << "[bridge] arm_joint_names = [";
      for (size_t i = 0; i < arm_joint_names_.size(); ++i) {
        oss << arm_joint_names_[i];
        if (i + 1 < arm_joint_names_.size()) oss << ", ";
      }
      oss << "]";
      ROS_INFO("%s", oss.str().c_str());
    }

    ROS_INFO("[bridge] arm_index_map = [%d,%d,%d,%d,%d,%d] (first 6 shown)",
             arm_index_map_.size()>0?arm_index_map_[0]:-1,
             arm_index_map_.size()>1?arm_index_map_[1]:-1,
             arm_index_map_.size()>2?arm_index_map_[2]:-1,
             arm_index_map_.size()>3?arm_index_map_[3]:-1,
             arm_index_map_.size()>4?arm_index_map_[4]:-1,
             arm_index_map_.size()>5?arm_index_map_[5]:-1);

    {
      std::ostringstream oss;
      oss << "[bridge] arm_offset = [";
      for (size_t i = 0; i < arm_offset_.size(); ++i) {
        oss << arm_offset_[i];
        if (i + 1 < arm_offset_.size()) oss << ", ";
      }
      oss << "]";
      ROS_INFO("%s", oss.str().c_str());
    }

    ROS_INFO("[bridge] base set_model_state mode: rate=%.1fHz pose_alpha=%.3f yaw_alpha=%.3f apply_twist=%s",
             base_set_rate_, base_pose_alpha_, base_yaw_alpha_, base_apply_twist_ ? "true" : "false");
    ROS_INFO("[bridge] base command topic: %s", car_cmd_topic_.c_str());
  }

private:
  static double normalizeAngle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static double shortestAngularDistance(double from, double to)
  {
    return normalizeAngle(to - from);
  }

  // ========== 底盘 Twist 回调 ==========
  void cbCar(const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (!enabled_) return;

    last_car_stamp_ = ros::Time::now();

    // REMANI 自定义格式：map/world 下 pose(x,y,yaw) + vel(vx,vy)
    target_x_ = msg->linear.x;
    target_y_ = msg->linear.y;
    target_yaw_ = normalizeAngle(msg->linear.z);
    target_vx_map_ = msg->angular.x;
    target_vy_map_ = msg->angular.y;
    received_base_cmd_ = true;

    if (!has_base_target_) {
      filt_x_ = target_x_;
      filt_y_ = target_y_;
      filt_yaw_ = target_yaw_;
      has_base_target_ = true;
    }
  }

  // fixed-rate filtered base pose injection to reduce physics excitation
  void onBaseTimer(const ros::TimerEvent&)
  {
    if (!enabled_ || !has_base_target_) return;

    // 超时就保持当前滤波状态，不再追目标
    if (received_base_cmd_ && (ros::Time::now() - last_car_stamp_).toSec() > cmd_timeout_) return;

    const double dx = target_x_ - filt_x_;
    const double dy = target_y_ - filt_y_;
    const double dyaw = shortestAngularDistance(filt_yaw_, target_yaw_);

    if (std::abs(dx) > base_xy_deadband_) filt_x_ += base_pose_alpha_ * dx;
    if (std::abs(dy) > base_xy_deadband_) filt_y_ += base_pose_alpha_ * dy;
    if (std::abs(dyaw) > base_yaw_deadband_) filt_yaw_ = normalizeAngle(filt_yaw_ + base_yaw_alpha_ * dyaw);

    gazebo_msgs::SetModelState srv;
    gazebo_msgs::ModelState& state = srv.request.model_state;

    state.model_name = gazebo_model_name_;
    state.reference_frame = world_frame_;

    state.pose.position.x = filt_x_;
    state.pose.position.y = filt_y_;
    state.pose.position.z = gazebo_z_;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, filt_yaw_);
    state.pose.orientation.x = q.x();
    state.pose.orientation.y = q.y();
    state.pose.orientation.z = q.z();
    state.pose.orientation.w = q.w();

    if (base_apply_twist_) {
      state.twist.linear.x = target_vx_map_;
      state.twist.linear.y = target_vy_map_;
    } else {
      state.twist.linear.x = 0.0;
      state.twist.linear.y = 0.0;
    }
    state.twist.linear.z = 0.0;
    state.twist.angular.x = 0.0;
    state.twist.angular.y = 0.0;
    state.twist.angular.z = 0.0;

    if (!set_model_state_client_.exists()) {
      ROS_WARN_THROTTLE(1.0, "[bridge] /gazebo/set_model_state service not available");
      return;
    }

    if (!set_model_state_client_.call(srv)) {
      ROS_WARN_THROTTLE(1.0, "[bridge] failed to call /gazebo/set_model_state");
      return;
    }

    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[bridge] set_model_state failed: %s",
                        srv.response.status_message.c_str());
      return;
    }
  }

  // ========== Gazebo arm state callback: convert back to REMANI order ==========
  void cbGazeboJointState(const sensor_msgs::JointState::ConstPtr& msg)
  {
    if (!msg || msg->name.empty()) return;

    const std::size_t n = arm_joint_names_.size();
    sensor_msgs::JointState out;
    out.header = msg->header;
    out.name.assign(n, std::string());
    out.position.assign(n, 0.0);
    out.velocity.assign(n, 0.0);
    out.effort.assign(n, 0.0);

    std::unordered_map<std::string, std::size_t> idx;
    idx.reserve(msg->name.size());
    for (std::size_t k = 0; k < msg->name.size(); ++k) idx[msg->name[k]] = k;

    for (std::size_t j = 0; j < n; ++j)
    {
      auto it = idx.find(arm_joint_names_[j]);
      if (it == idx.end()) {
        ROS_WARN_THROTTLE(1.0, "[bridge] joint %s not found in /robot1/joint_states", arm_joint_names_[j].c_str());
        return;
      }

      const std::size_t src = it->second;
      const int i = arm_index_map_[j];
      if (i < 0 || i >= static_cast<int>(n)) {
        ROS_WARN_THROTTLE(1.0, "[bridge] arm_index_map[%zu]=%d out of range", j, i);
        return;
      }

      const double s = arm_sign_[j];
      if (std::abs(s) < 1e-9) {
        ROS_WARN_THROTTLE(1.0, "[bridge] arm_sign[%zu] too small", j);
        return;
      }

      out.name[i] = arm_joint_names_[j];
      if (src < msg->position.size()) out.position[i] = (msg->position[src] - arm_offset_[j]) / s;
      if (src < msg->velocity.size()) out.velocity[i] = msg->velocity[src] / s;
      if (src < msg->effort.size()) out.effort[i] = msg->effort[src] / s;
    }

    pub_arm_state_.publish(out);
  }

  // ========== 机械臂 state 回调：只缓存 desired ==========
  void cbArmState(const control_msgs::JointTrajectoryControllerStateConstPtr& msg)
  {
    last_arm_state_ = msg;
    last_arm_stamp_ = ros::Time::now();
  }

  // ========== planning start ==========
  void onStart(const std_msgs::Bool::ConstPtr& msg)
  {
    if (msg->data) {
      enabled_ = true;
      last_car_stamp_ = ros::Time::now();
      last_arm_stamp_ = ros::Time::now();
      ROS_INFO("[bridge] planning start -> ENABLE");
    }
  }

  // ========== watchdog ==========
  void onTimer(const ros::TimerEvent&)
  {
    if (!enabled_) return;

    // if ((ros::Time::now() - last_car_stamp_).toSec() > cmd_timeout_) {
    //   publishStopBase();
    // }
  }

  // ========== 机械臂固定频率发布 ==========
  void onArmTimer(const ros::TimerEvent&)
  {
    if (!enabled_) return;
    if (!last_arm_state_) return;

    if ((ros::Time::now() - last_arm_stamp_).toSec() > cmd_timeout_) {
      return;
    }

    const auto& desired = last_arm_state_->desired;
    const std::size_t n = arm_joint_names_.size();

    if (desired.positions.empty()) return;

    int max_i = -1;
    for (std::size_t j = 0; j < n; ++j) max_i = std::max(max_i, arm_index_map_[j]);
    if (max_i >= static_cast<int>(desired.positions.size())) {
      ROS_WARN_THROTTLE(1.0,
        "[arm] desired.positions size=%zu but max index in arm_index_map is %d",
        desired.positions.size(), max_i);
      return;
    }

    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time(0);
    traj.joint_names = arm_joint_names_;

    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions.resize(n);

    for (std::size_t j = 0; j < n; ++j)
    {
      const int i = arm_index_map_[j];
      pt.positions[j] = arm_sign_[j] * desired.positions[i] + arm_offset_[j];
    }

    if (desired.velocities.size() == desired.positions.size())
    {
      pt.velocities.resize(n);
      for (std::size_t j = 0; j < n; ++j)
      {
        const int i = arm_index_map_[j];
        pt.velocities[j] = arm_sign_[j] * desired.velocities[i];
      }
    }

    pt.time_from_start = ros::Duration(arm_time_from_start_);
    traj.points.push_back(pt);
    pub_arm_.publish(traj);
  }

  // ---------------- ROS ----------------
  ros::Subscriber sub_car_;
  ros::Subscriber sub_arm_;
  ros::Subscriber sub_gz_joint_state_;

  ros::Publisher pub_fl_, pub_fr_, pub_rl_, pub_rr_;
  ros::Publisher pub_arm_;
  ros::Publisher pub_arm_state_;

  ros::Timer timer_;
  ros::Timer base_timer_;
  ros::Timer arm_timer_;

  ros::ServiceClient set_model_state_client_;
  std::string gazebo_model_name_{"robot1"};
  std::string world_frame_{"world"};
  std::string car_cmd_topic_{"/mm_controller_node/car_cmd"};
  double gazebo_z_{0.2};

  double base_set_rate_{50.0};
  double base_pose_alpha_{0.35};
  double base_yaw_alpha_{0.35};
  bool base_apply_twist_{false};
  double base_xy_deadband_{1e-4};
  double base_yaw_deadband_{1e-4};

  // ---------------- state ----------------
  bool enabled_;
  ros::Time last_car_stamp_;
  ros::Time last_arm_stamp_;
  ros::Time last_cmd_stamp_;

  control_msgs::JointTrajectoryControllerStateConstPtr last_arm_state_;

  // ---------------- param（底盘） ----------------
  double last_yaw_;
  bool has_last_yaw_;
  double r_, lx_, ly_;
  double cmd_timeout_;

  double s_fl_, s_fr_, s_rl_, s_rr_;

  double wz_lp_;
  double wz_alpha_;
  double wz_limit_;

  bool has_base_target_{false};
  bool received_base_cmd_{false};
  double target_x_{0.0};
  double target_y_{0.0};
  double target_yaw_{0.0};
  double target_vx_map_{0.0};
  double target_vy_map_{0.0};
  double filt_x_{0.0};
  double filt_y_{0.0};
  double filt_yaw_{0.0};

  // ---------------- param（机械臂） ----------------
  std::vector<std::string> arm_joint_names_;
  double arm_publish_rate_;
  double arm_time_from_start_;

  // 机械臂映射参数
  std::vector<int> arm_index_map_;
  std::vector<double> arm_sign_;
  std::vector<double> arm_offset_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_to_mecanum_bridge");
  ros::NodeHandle nh("~");

  try {
    TwistToMecanum node(nh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("Failed to start bridge: %s", e.what());
  }
  return 0;
}
