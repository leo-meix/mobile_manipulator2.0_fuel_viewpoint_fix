#ifndef _FUEL_VIEWPOINT_GENERATOR_H
#define _FUEL_VIEWPOINT_GENERATOR_H

#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <plan_env/fuel_frontier_target.h>
#include <plan_env/grid_map.h>

struct FuelViewpoint
{
  int target_id = -1;
  int candidate_id = -1;
  int arm_config_id = -1;
  Eigen::Vector3d base_pose = Eigen::Vector3d::Zero();     // x, y, yaw
  Eigen::Vector3d camera_pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond camera_q = Eigen::Quaterniond::Identity();
  std::vector<double> arm_config;
  int visible_frontier_cells = 0;
  int unknown_gain = 0;
  double distance_cost = 0.0;
  double yaw_cost = 0.0;
  double arm_motion_cost = 0.0;
  double path_cost = 0.0;
  double score = 0.0;
  bool collision_free = false;
  bool reachable = true;
  bool in_place = false;
  bool valid = false;
};

struct FuelArmViewConfig
{
  int id = -1;
  std::vector<double> joints;
  Eigen::Vector3d camera_offset = Eigen::Vector3d::Zero();
  double yaw_offset = 0.0;
  double pitch_offset = 0.0;
  Eigen::Vector3d camera_rpy = Eigen::Vector3d::Zero();
  Eigen::Isometry3d T_ee_camera = Eigen::Isometry3d::Identity();
};

struct FuelViewpointStats
{
  int targets = 0;
  int sampled = 0;
  int full_visibility_checked = 0;
  int reject_map = 0;
  int reject_free = 0;
  int reject_clearance = 0;
  int reject_joint = 0;
  int reject_collision = 0;
  int reject_facing = 0;
  int reject_visibility = 0;
  int skipped_visibility_budget = 0;
  int valid_target_count = 0;
  int dormant_target_count = 0;
  int reject_outside_hfov = 0;
  int reject_outside_vfov = 0;
  int reject_occluded = 0;
  int reject_visible_count = 0;
  int reject_visible_fraction = 0;
  int frustum_pass = 0;
  int reject_hfov = 0;
  int reject_vfov = 0;
  int blocked_raw_occupied = 0;
  int blocked_inflated_only = 0;
  int blocked_unknown = 0;
  int out_of_map = 0;
  int ray_visible = 0;
  int reject_gain = 0;
  int reject_reachable = 0;
  int valid = 0;
  int best_target = -1;
  double best_score = 0.0;
  double sampling_ms = 0.0;
  double visibility_ms = 0.0;
  double total_ms = 0.0;
};

class FuelViewpointGenerator
{
public:
  typedef std::shared_ptr<FuelViewpointGenerator> Ptr;
  typedef std::function<bool(const Eigen::Vector3d& start,
                             const Eigen::Vector3d& goal,
                             double goal_yaw,
                             double& path_cost)> ReachabilityChecker;
  typedef std::function<bool(const Eigen::Vector3d& base_pose,
                             const std::vector<double>& joints,
                             const FuelArmViewConfig& config,
                             Eigen::Vector3d& camera_pos,
                             Eigen::Quaterniond& camera_q)> CameraPoseProvider;
  typedef std::function<bool(const Eigen::Vector3d& base_pose,
                             const std::vector<double>& joints,
                             bool safe)> CollisionChecker;

  FuelViewpointGenerator() = default;

  void init(ros::NodeHandle& nh, const GridMap::Ptr& grid_map, bool enable_debug = true);
  void setReachabilityChecker(const ReachabilityChecker& checker);
  void setCameraPoseProvider(const CameraPoseProvider& provider);
  void setCollisionChecker(const CollisionChecker& checker);
  void setArmViewConfigs(const std::vector<FuelArmViewConfig>& configs);

  template <typename MMConfigT>
  void setMMConfig(const std::shared_ptr<MMConfigT>& mm_config)
  {
    mm_config_ready_ = static_cast<bool>(mm_config);
    if (!mm_config) return;

    mani_dof_ = mm_config->getManiDof();
    collision_checker_ = [mm_config](const Eigen::Vector3d& base_pose,
                                     const std::vector<double>& joints,
                                     const bool safe) {
      Eigen::VectorXd joint_vec(joints.size());
      for (int i = 0; i < joint_vec.size(); ++i) joint_vec(i) = joints[i];
      return mm_config->checkcollision(base_pose, joint_vec, safe);
    };

    camera_pose_provider_ =
        [this, mm_config](const Eigen::Vector3d& base_pose,
                          const std::vector<double>& joints,
                          const FuelArmViewConfig& config,
                          Eigen::Vector3d& camera_pos,
                          Eigen::Quaterniond& camera_q) {
          if (static_cast<int>(joints.size()) != mm_config->getManiDof()) return false;

          Eigen::VectorXd joint_vec(joints.size());
          for (int i = 0; i < joint_vec.size(); ++i) joint_vec(i) = joints[i];

          Eigen::Matrix4d T_world_car;
          mm_config->CarState2T(base_pose, T_world_car);

          std::vector<Eigen::Matrix4d> T_joint;
          std::vector<Eigen::Matrix4d> T_joint_grad;
          mm_config->getJointTrans(joint_vec, T_joint, T_joint_grad);

          Eigen::Matrix4d T_world_ee = T_world_car * mm_config->getTq0();
          for (const Eigen::Matrix4d& T : T_joint) T_world_ee *= T;

          Eigen::Isometry3d T_world_ee_iso = Eigen::Isometry3d::Identity();
          T_world_ee_iso.matrix() = T_world_ee;
          const Eigen::Isometry3d T_world_camera = T_world_ee_iso * config.T_ee_camera;

          camera_pos = T_world_camera.translation();
          camera_q = Eigen::Quaterniond(T_world_camera.rotation());
          camera_q.normalize();
          return true;
        };
  }

  std::vector<FuelViewpoint> generate(const std::vector<FrontierTarget>& targets,
                                      const Eigen::Vector3d& robot_pose,
                                      const std::vector<double>& current_arm_config);

  bool hasBestViewpoint() const;
  const FuelViewpoint& bestViewpoint() const;
  const std::vector<FuelViewpoint>& lastCandidates() const;
  const std::vector<int>& activeTargetIds() const;
  const std::vector<int>& dormantTargetIds() const;
  const FuelViewpointStats& lastStats() const;

  // Optical-frame helpers kept public so their coordinate and endpoint
  // contracts can be covered without constructing a ROS map fixture.
  static bool opticalPointInFrustum(const Eigen::Vector3d& rel_cam,
                                    double min_range,
                                    double max_range,
                                    double hfov,
                                    double vfov);
  static std::vector<Eigen::Vector3i> intermediateRayVoxels(
      const Eigen::Vector3d& start_voxel, const Eigen::Vector3d& end_voxel);

  visualization_msgs::MarkerArray makeDebugMarkers() const;
  void publishDebugMarkers() const;

private:
  bool loadArmViewConfigs(ros::NodeHandle& nh);
  bool loadJointLimits(ros::NodeHandle& nh);
  bool simpleCameraPose(const Eigen::Vector3d& base_pose,
                        const FuelArmViewConfig& config,
                        Eigen::Vector3d& camera_pos,
                        Eigen::Quaterniond& camera_q) const;
  std::vector<Eigen::Vector3d> sampleBasePoses(const FrontierTarget& target,
                                               const Eigen::Vector3d& robot_pose,
                                               int& sampled_count) const;
  bool isBaseKnownFree(const Eigen::Vector3d& base_pose) const;
  bool hasUnknownClearance(const Eigen::Vector3d& base_pose) const;
  bool hasObstacleClearance(const Eigen::Vector3d& base_pose) const;
  bool jointsWithinLimits(const std::vector<double>& joints) const;
  bool cameraFacesTarget(const FuelViewpoint& candidate,
                         const FrontierTarget& target,
                         Eigen::Vector3d* camera_forward_world = nullptr,
                         Eigen::Vector3d* to_target_world = nullptr,
                         double* facing_dot = nullptr) const;
  Eigen::Vector3d cameraForward(const Eigen::Quaterniond& camera_q) const;
  void evaluateVisibilityAndGain(const FrontierTarget& target,
                                 FuelViewpoint& candidate,
                                 double& visibility_ms);
  bool rayVisibleToPoint(const Eigen::Vector3d& camera_pos,
                         const Eigen::Vector3d& point);
  bool pointInCameraFrustum(const Eigen::Vector3d& camera_pos,
                            const Eigen::Quaterniond& camera_q,
                            const Eigen::Vector3d& point);
  int countUnknownGain(const FuelViewpoint& candidate) const;
  double scoreCandidate(const FuelViewpoint& candidate) const;
  double yawDiff(double a, double b) const;
  int voxelKey(const Eigen::Vector3i& id) const;
  void appendCandidateMarker(visualization_msgs::MarkerArray& array,
                             int& marker_id,
                             const std::string& ns,
                             const FuelViewpoint& candidate,
                             double r,
                             double g,
                             double b,
                             double a,
                             double scale) const;
  void appendArrowMarker(visualization_msgs::MarkerArray& array,
                         int& marker_id,
                         const std::string& ns,
                         const FuelViewpoint& candidate,
                         double r,
                         double g,
                         double b,
                         double a,
                         double shaft_scale,
                         double length) const;

  GridMap::Ptr grid_map_;
  ros::Publisher debug_marker_pub_;
  std::string frame_id_ = "world";
  std::string debug_marker_topic_ = "/fuel_viewpoint/debug_markers";

  std::vector<FuelArmViewConfig> arm_view_configs_;
  std::vector<double> joint_min_;
  std::vector<double> joint_max_;
  int mani_dof_ = 0;
  bool mm_config_ready_ = false;

  CameraPoseProvider camera_pose_provider_;
  CollisionChecker collision_checker_;
  ReachabilityChecker reachability_checker_;

  double r_min_ = 0.8;
  double r_max_ = 2.4;
  int r_num_ = 5;
  double angle_span_ = 6.283185307179586;
  int angle_num_ = 24;
  int max_candidates_per_target_ = 120;
  int max_valid_per_target_ = 5;
  int max_total_candidates_ = 200;
  int max_evaluated_per_target_ = 120;
  int max_visibility_checks_ = 50;
  int max_visibility_checks_per_target_ = 24;
  double base_check_z_ = 0.15;
  double base_marker_z_ = 0.03;
  double base_footprint_radius_ = 0.32;
  double min_obstacle_distance_ = 0.25;
  double min_unknown_clearance_ = 0.25;
  bool collision_safe_margin_ = true;
  bool debug_show_sampled_ = false;
  bool debug_show_rejected_ = false;
  int debug_max_sampled_ = 50;
  int debug_max_rejected_ = 30;

  int min_visible_frontier_cells_ = 15;
  double min_visible_fraction_ = 0.25;
  int min_unknown_gain_ = 5;
  double max_view_range_ = 5.0;
  double min_view_range_ = 0.10;
  double hfov_ = 1.57;
  double vfov_ = 1.05;
  double gain_ray_step_ = 0.20;
  double gain_yaw_step_ = 0.12;
  double gain_pitch_step_ = 0.12;
  double facing_cos_threshold_ = 0.35;
  bool visibility_unknown_blocks_ray_ = true;

  double weight_gain_ = 1.0;
  double weight_visible_ = 2.0;
  double weight_distance_ = 1.0;
  double weight_yaw_ = 0.2;
  double weight_arm_ = 0.2;
  double weight_path_ = 0.5;

  std::vector<FuelViewpoint> last_candidates_;
  std::vector<FuelViewpoint> debug_candidates_;
  std::vector<int> active_target_ids_;
  std::vector<int> dormant_target_ids_;
  FuelViewpoint best_viewpoint_;
  bool has_best_viewpoint_ = false;
  FuelViewpointStats last_stats_;
  bool first_blocker_logged_ = false;
};

#endif
