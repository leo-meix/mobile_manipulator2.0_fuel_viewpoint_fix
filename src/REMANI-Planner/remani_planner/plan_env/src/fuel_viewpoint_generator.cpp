#include <plan_env/fuel_viewpoint_generator.h>

#include <plan_env/raycast.h>
#include <XmlRpcValue.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <unordered_set>

namespace {

bool readXmlRpcDouble(const XmlRpc::XmlRpcValue& value, double& out)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    out = static_cast<double>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    out = static_cast<int>(value);
    return true;
  }
  return false;
}

bool readXmlRpcDoubleArray(const XmlRpc::XmlRpcValue& value, std::vector<double>& out)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) return false;
  out.clear();
  out.reserve(value.size());
  for (int i = 0; i < value.size(); ++i) {
    double v = 0.0;
    if (!readXmlRpcDouble(value[i], v)) return false;
    out.push_back(v);
  }
  return true;
}

Eigen::Matrix3d rpyToRotation(const Eigen::Vector3d& rpy)
{
  return Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix() *
         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()).toRotationMatrix() *
         Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
}

void updateCameraTransform(FuelArmViewConfig& config)
{
  config.T_ee_camera = Eigen::Isometry3d::Identity();
  config.T_ee_camera.linear() = rpyToRotation(config.camera_rpy);
  config.T_ee_camera.translation() = config.camera_offset;
}

}  // namespace

void FuelViewpointGenerator::init(ros::NodeHandle& nh,
                                  const GridMap::Ptr& grid_map,
                                  const bool enable_debug)
{
  grid_map_ = grid_map;

  nh.param("grid_map/frame_id", frame_id_, std::string("world"));
  nh.param("fuel_viewpoint/debug_marker_topic", debug_marker_topic_,
           std::string("/fuel_viewpoint/debug_markers"));

  nh.param("fuel_viewpoint/r_min", r_min_, 0.8);
  nh.param("fuel_viewpoint/r_max", r_max_, 2.4);
  nh.param("fuel_viewpoint/r_num", r_num_, 5);
  nh.param("fuel_viewpoint/angle_span", angle_span_, 2.0 * M_PI);
  nh.param("fuel_viewpoint/angle_num", angle_num_, 24);
  nh.param("fuel_viewpoint/max_candidates_per_target", max_candidates_per_target_, 120);
  nh.param("fuel_viewpoint/max_valid_per_target", max_valid_per_target_, 5);
  nh.param("fuel_viewpoint/max_total_candidates", max_total_candidates_, 200);
  nh.param("fuel_viewpoint/max_evaluated_per_target", max_evaluated_per_target_, 120);
  nh.param("fuel_viewpoint/max_visibility_checks", max_visibility_checks_, 50);
  nh.param("fuel_viewpoint/max_visibility_checks_per_target",
           max_visibility_checks_per_target_, 24);
  nh.param("fuel_viewpoint/base_check_z", base_check_z_, 0.15);
  nh.param("fuel_viewpoint/base_marker_z", base_marker_z_, 0.03);
  nh.param("fuel_viewpoint/base_footprint_radius", base_footprint_radius_, 0.32);
  nh.param("fuel_viewpoint/min_obstacle_distance", min_obstacle_distance_, 0.25);
  nh.param("fuel_viewpoint/min_unknown_clearance", min_unknown_clearance_, 0.25);
  nh.param("fuel_viewpoint/collision_safe_margin", collision_safe_margin_, false);
  nh.param("fuel_viewpoint/debug_show_sampled", debug_show_sampled_, false);
  nh.param("fuel_viewpoint/debug_show_rejected", debug_show_rejected_, false);
  nh.param("fuel_viewpoint/debug_max_sampled", debug_max_sampled_, 50);
  nh.param("fuel_viewpoint/debug_max_rejected", debug_max_rejected_, 30);

  nh.param("fuel_viewpoint/min_visible_frontier_cells", min_visible_frontier_cells_, 15);
  nh.param("fuel_viewpoint/min_visible_fraction", min_visible_fraction_, 0.25);
  nh.param("fuel_viewpoint/min_unknown_gain", min_unknown_gain_, 5);
  nh.param("fuel_viewpoint/max_view_range", max_view_range_, 5.0);
  nh.param("fuel_viewpoint/min_view_range", min_view_range_, 0.10);
  nh.param("fuel_viewpoint/hfov", hfov_, 1.57);
  nh.param("fuel_viewpoint/vfov", vfov_, 1.05);
  nh.param("fuel_viewpoint/gain_ray_step", gain_ray_step_, 0.20);
  nh.param("fuel_viewpoint/gain_yaw_step", gain_yaw_step_, 0.12);
  nh.param("fuel_viewpoint/gain_pitch_step", gain_pitch_step_, 0.12);
  nh.param("fuel_viewpoint/facing_cos_threshold", facing_cos_threshold_, 0.35);
  nh.param("fuel_viewpoint/visibility_unknown_blocks_ray",
           visibility_unknown_blocks_ray_, true);

  nh.param("fuel_viewpoint/weight_gain", weight_gain_, 1.0);
  nh.param("fuel_viewpoint/weight_visible", weight_visible_, 2.0);
  nh.param("fuel_viewpoint/weight_distance", weight_distance_, 1.0);
  nh.param("fuel_viewpoint/weight_yaw", weight_yaw_, 0.2);
  nh.param("fuel_viewpoint/weight_arm", weight_arm_, 0.2);
  nh.param("fuel_viewpoint/weight_path", weight_path_, 0.5);

  r_num_ = std::max(1, r_num_);
  angle_num_ = std::max(1, angle_num_);
  max_candidates_per_target_ = std::max(1, max_candidates_per_target_);
  max_valid_per_target_ = std::max(1, max_valid_per_target_);
  max_total_candidates_ = std::max(1, max_total_candidates_);
  max_evaluated_per_target_ = std::max(1, max_evaluated_per_target_);
  max_visibility_checks_ = std::max(1, max_visibility_checks_);
  max_visibility_checks_per_target_ =
      std::max(1, max_visibility_checks_per_target_);
  base_footprint_radius_ = std::max(0.0, base_footprint_radius_);
  min_unknown_clearance_ = std::max(0.0, min_unknown_clearance_);
  debug_max_sampled_ = std::max(0, debug_max_sampled_);
  debug_max_rejected_ = std::max(0, debug_max_rejected_);
  gain_ray_step_ = std::max(0.02, gain_ray_step_);
  gain_yaw_step_ = std::max(0.02, gain_yaw_step_);
  gain_pitch_step_ = std::max(0.02, gain_pitch_step_);

  if (r_max_ < r_min_) std::swap(r_min_, r_max_);

  loadArmViewConfigs(nh);
  loadJointLimits(nh);
  if (arm_view_configs_.empty()) {
    FuelArmViewConfig default_config;
    default_config.id = 0;
    if (mani_dof_ > 0) default_config.joints.assign(mani_dof_, 0.0);
    default_config.camera_offset = Eigen::Vector3d(0.0, 0.0, 1.0);
    updateCameraTransform(default_config);
    arm_view_configs_.push_back(default_config);
    ROS_WARN("[FuelViewpointGenerator] arm_view_configs empty; using one simple camera config.");
  }

  if (enable_debug) {
    if (ros::names::clean(debug_marker_topic_).empty()) {
      debug_marker_topic_ = "/fuel_viewpoint/debug_markers";
    }
    debug_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(debug_marker_topic_, 2);
  } else {
    debug_marker_pub_ = ros::Publisher();
  }

  ROS_INFO("[FuelViewpointGenerator] r=[%.2f, %.2f] r_num=%d angle_span=%.2f angle_num=%d max_per_target=%d max_valid_per_target=%d arm_configs=%zu debug=%d",
           r_min_, r_max_, r_num_, angle_span_, angle_num_, max_candidates_per_target_,
           max_valid_per_target_, arm_view_configs_.size(), enable_debug ? 1 : 0);
}

void FuelViewpointGenerator::setReachabilityChecker(const ReachabilityChecker& checker)
{
  reachability_checker_ = checker;
}

void FuelViewpointGenerator::setCameraPoseProvider(const CameraPoseProvider& provider)
{
  camera_pose_provider_ = provider;
}

void FuelViewpointGenerator::setCollisionChecker(const CollisionChecker& checker)
{
  collision_checker_ = checker;
}

void FuelViewpointGenerator::setArmViewConfigs(const std::vector<FuelArmViewConfig>& configs)
{
  arm_view_configs_ = configs;
}

std::vector<FuelViewpoint> FuelViewpointGenerator::generate(
    const std::vector<FrontierTarget>& targets,
    const Eigen::Vector3d& robot_pose,
    const std::vector<double>& current_arm_config)
{
  const auto total_t0 = std::chrono::steady_clock::now();
  last_candidates_.clear();
  debug_candidates_.clear();
  active_target_ids_.clear();
  dormant_target_ids_.clear();
  has_best_viewpoint_ = false;
  best_viewpoint_ = FuelViewpoint();
  last_stats_ = FuelViewpointStats();
  first_blocker_logged_ = false;
  last_stats_.targets = static_cast<int>(targets.size());

  if (!grid_map_) {
    ROS_WARN_THROTTLE(1.0, "[FuelViewpointGenerator] grid_map is not ready.");
    return last_candidates_;
  }

  int next_candidate_id = 0;
  const int target_count = static_cast<int>(std::count_if(
      targets.begin(), targets.end(), [](const FrontierTarget& target) {
        return target.valid && !target.frontier_points.empty();
      }));
  struct FacingStats {
    int checked = 0;
    int rejected = 0;
    double dot_min = std::numeric_limits<double>::infinity();
    double dot_max = -std::numeric_limits<double>::infinity();
    double dot_sum = 0.0;
  };
  std::map<int, FacingStats> facing_stats_by_arm;
  bool logged_first_facing_reject = false;
  bool logged_first_visibility_reject = false;
  enum RejectReason {
    REJECT_MAP,
    REJECT_FREE,
    REJECT_CLEARANCE,
    REJECT_JOINT,
    REJECT_COLLISION,
    REJECT_FACING,
    REJECT_VISIBILITY,
    REJECT_GAIN,
    REJECT_REACHABLE,
    REJECT_REASON_COUNT
  };
  const char* reject_reason_names[REJECT_REASON_COUNT] = {
      "map", "free", "clearance", "joint", "collision", "facing",
      "visibility", "gain", "reachable"};
  FuelViewpoint reject_samples[REJECT_REASON_COUNT];
  bool has_reject_sample[REJECT_REASON_COUNT] = {};
  const auto record_reject_sample = [&](const RejectReason reason,
                                        const FuelViewpoint& candidate) {
    if (!has_reject_sample[reason]) {
      reject_samples[reason] = candidate;
      has_reject_sample[reason] = true;
    }
  };
  const auto store_debug_candidate = [&](const FuelViewpoint& candidate) {
    const int max_debug = debug_max_sampled_ + debug_max_rejected_;
    if (max_debug > 0 && static_cast<int>(debug_candidates_.size()) < max_debug) {
      debug_candidates_.push_back(candidate);
    }
  };
  const auto sampling_t0 = std::chrono::steady_clock::now();
  std::vector<std::pair<int, int>> valid_candidates_by_target;
  valid_candidates_by_target.reserve(target_count);

  for (const FrontierTarget& target : targets) {
    if (!target.valid || target.frontier_points.empty()) continue;

    const int target_visibility_budget =
        std::min(max_visibility_checks_per_target_, max_evaluated_per_target_);
    int target_visibility_checks = 0;
    int target_evaluated = 0;
    int sampled_count = 0;
    std::vector<FuelViewpoint> qualified_candidates;
    std::vector<Eigen::Vector3d> base_samples;
    base_samples.reserve(1 + r_num_ * angle_num_);
    Eigen::Vector3d in_place_pose;
    in_place_pose.x() = robot_pose.x();
    in_place_pose.y() = robot_pose.y();
    in_place_pose.z() = std::atan2(target.center.y() - robot_pose.y(),
                                   target.center.x() - robot_pose.x());
    base_samples.push_back(in_place_pose);
    const std::vector<Eigen::Vector3d> sampled_base_poses =
        sampleBasePoses(target, robot_pose, sampled_count);
    base_samples.insert(base_samples.end(), sampled_base_poses.begin(),
                        sampled_base_poses.end());
    last_stats_.sampled += sampled_count + 1;

    for (size_t base_sample_index = 0; base_sample_index < base_samples.size();
         ++base_sample_index) {
      if (target_evaluated >= max_evaluated_per_target_) break;
      const Eigen::Vector3d& base_pose = base_samples[base_sample_index];

      FuelViewpoint base_candidate;
      base_candidate.target_id = target.id;
      base_candidate.candidate_id = next_candidate_id++;
      base_candidate.base_pose = base_pose;
      base_candidate.in_place = (base_sample_index == 0);
      base_candidate.distance_cost = (base_pose.head<2>() - robot_pose.head<2>()).norm();
      base_candidate.yaw_cost = std::abs(yawDiff(robot_pose.z(), base_pose.z()));

      if (!grid_map_->isInMap(Eigen::Vector2d(base_pose.x(), base_pose.y()))) {
        ++target_evaluated;
        ++last_stats_.reject_map;
        record_reject_sample(REJECT_MAP, base_candidate);
        store_debug_candidate(base_candidate);
        continue;
      }
      if (!isBaseKnownFree(base_pose)) {
        ++target_evaluated;
        ++last_stats_.reject_free;
        record_reject_sample(REJECT_FREE, base_candidate);
        store_debug_candidate(base_candidate);
        continue;
      }
      if (!hasUnknownClearance(base_pose) || !hasObstacleClearance(base_pose)) {
        ++target_evaluated;
        ++last_stats_.reject_clearance;
        record_reject_sample(REJECT_CLEARANCE, base_candidate);
        store_debug_candidate(base_candidate);
        continue;
      }

      for (const FuelArmViewConfig& arm_config : arm_view_configs_) {
        if (target_evaluated >= max_evaluated_per_target_) break;
        ++target_evaluated;

        FuelViewpoint candidate = base_candidate;
        candidate.candidate_id = next_candidate_id++;
        candidate.arm_config_id = arm_config.id;
        candidate.arm_config = arm_config.joints;

        if (!jointsWithinLimits(candidate.arm_config)) {
          ++last_stats_.reject_joint;
          record_reject_sample(REJECT_JOINT, candidate);
          store_debug_candidate(candidate);
          continue;
        }

        if (camera_pose_provider_) {
          if (!camera_pose_provider_(candidate.base_pose, candidate.arm_config, arm_config,
                                     candidate.camera_pos, candidate.camera_q)) {
            ++last_stats_.reject_joint;
            record_reject_sample(REJECT_JOINT, candidate);
            store_debug_candidate(candidate);
            continue;
          }
        } else {
          simpleCameraPose(candidate.base_pose, arm_config, candidate.camera_pos, candidate.camera_q);
        }

        if (collision_checker_ &&
            collision_checker_(candidate.base_pose, candidate.arm_config, collision_safe_margin_)) {
          ++last_stats_.reject_collision;
          record_reject_sample(REJECT_COLLISION, candidate);
          store_debug_candidate(candidate);
          continue;
        }
        candidate.collision_free = true;

        Eigen::Vector3d camera_forward_world;
        Eigen::Vector3d to_target_world;
        double facing_dot = -1.0;
        const bool faces_target = cameraFacesTarget(
            candidate, target, &camera_forward_world, &to_target_world, &facing_dot);
        FacingStats& facing_stats = facing_stats_by_arm[candidate.arm_config_id];
        ++facing_stats.checked;
        facing_stats.dot_min = std::min(facing_stats.dot_min, facing_dot);
        facing_stats.dot_max = std::max(facing_stats.dot_max, facing_dot);
        facing_stats.dot_sum += facing_dot;

        if (!faces_target) {
          ++facing_stats.rejected;
          ++last_stats_.reject_facing;
          record_reject_sample(REJECT_FACING, candidate);
          if (!logged_first_facing_reject) {
            logged_first_facing_reject = true;
            ROS_WARN_STREAM_THROTTLE(
                1.0,
                "[FuelViewpointGenerator] first facing reject"
                    << " target_id=" << candidate.target_id
                    << " candidate_id=" << candidate.candidate_id
                    << " arm_config_id=" << candidate.arm_config_id
                    << " base_pose=(" << candidate.base_pose.x() << ","
                    << candidate.base_pose.y() << "," << candidate.base_pose.z() << ")"
                    << " target_center=(" << target.center.x() << "," << target.center.y()
                    << "," << target.center.z() << ")"
                    << " camera_pos=(" << candidate.camera_pos.x() << ","
                    << candidate.camera_pos.y() << "," << candidate.camera_pos.z() << ")"
                    << " camera_q(x,y,z,w)=(" << candidate.camera_q.x() << ","
                    << candidate.camera_q.y() << "," << candidate.camera_q.z() << ","
                    << candidate.camera_q.w() << ")"
                    << " camera_forward_world=(" << camera_forward_world.x() << ","
                    << camera_forward_world.y() << "," << camera_forward_world.z() << ")"
                    << " to_target_world=(" << to_target_world.x() << ","
                    << to_target_world.y() << "," << to_target_world.z() << ")"
                    << " facing_dot=" << facing_dot
                    << " facing_cos_threshold=" << facing_cos_threshold_);
          }
          store_debug_candidate(candidate);
          continue;
        }

        if (target_visibility_checks >= target_visibility_budget) {
          ++last_stats_.skipped_visibility_budget;
          record_reject_sample(REJECT_VISIBILITY, candidate);
          store_debug_candidate(candidate);
          continue;
        }
        ++target_visibility_checks;
        ++last_stats_.full_visibility_checked;

        evaluateVisibilityAndGain(target, candidate, last_stats_.visibility_ms);
        const double visible_fraction =
            target.frontier_points.empty()
                ? 0.0
                : static_cast<double>(candidate.visible_frontier_cells) /
                      static_cast<double>(target.frontier_points.size());
        if (candidate.visible_frontier_cells < min_visible_frontier_cells_ ||
            visible_fraction < min_visible_fraction_) {
          ++last_stats_.reject_visibility;
          record_reject_sample(REJECT_VISIBILITY, candidate);

          const Eigen::Vector3d target_in_camera =
              candidate.camera_q.inverse() * (target.center - candidate.camera_pos);
          const Eigen::Vector3d forward_axis_camera =
              candidate.camera_q.inverse() * cameraForward(candidate.camera_q);
          const Eigen::Vector3d forward_axis = forward_axis_camera.normalized();
          Eigen::Vector3d right_axis = Eigen::Vector3d::UnitY();
          if (std::abs(forward_axis.dot(right_axis)) > 0.95) {
            right_axis = Eigen::Vector3d::UnitX();
          }
          right_axis =
              (right_axis - forward_axis * forward_axis.dot(right_axis)).normalized();
          const Eigen::Vector3d up_axis = forward_axis.cross(right_axis).normalized();
          const double target_forward = target_in_camera.dot(forward_axis);
          const double horizontal_angle =
              std::atan2(target_in_camera.dot(right_axis), target_forward);
          const double vertical_angle =
              std::atan2(target_in_camera.dot(up_axis), target_forward);
          const bool inside_hfov =
              target_forward > 1e-3 && std::abs(horizontal_angle) <= 0.5 * hfov_;
          const bool inside_vfov =
              target_forward > 1e-3 && std::abs(vertical_angle) <= 0.5 * vfov_;
          const bool target_center_visible =
              inside_hfov && inside_vfov &&
              rayVisibleToPoint(candidate.camera_pos, target.center);

          if (!inside_hfov) {
            ++last_stats_.reject_outside_hfov;
          } else if (!inside_vfov) {
            ++last_stats_.reject_outside_vfov;
          } else if (!target_center_visible) {
            ++last_stats_.reject_occluded;
          } else if (candidate.visible_frontier_cells < min_visible_frontier_cells_) {
            ++last_stats_.reject_visible_count;
          } else {
            ++last_stats_.reject_visible_fraction;
          }

          if (!logged_first_visibility_reject) {
            logged_first_visibility_reject = true;
            ROS_WARN_STREAM_THROTTLE(
                1.0,
                "[FuelViewpointGenerator] first visibility reject"
                    << " target_id=" << candidate.target_id
                    << " candidate_id=" << candidate.candidate_id
                    << " arm_config_id=" << candidate.arm_config_id
                    << " target_center=(" << target.center.x() << "," << target.center.y()
                    << "," << target.center.z() << ")"
                    << " camera_pos=(" << candidate.camera_pos.x() << ","
                    << candidate.camera_pos.y() << "," << candidate.camera_pos.z() << ")"
                    << " target_in_camera=(" << target_in_camera.x() << ","
                    << target_in_camera.y() << "," << target_in_camera.z() << ")"
                    << " horizontal_angle=" << horizontal_angle
                    << " vertical_angle=" << vertical_angle
                    << " hfov/2=" << 0.5 * hfov_
                    << " vfov/2=" << 0.5 * vfov_
                    << " inside_hfov=" << (inside_hfov ? 1 : 0)
                    << " inside_vfov=" << (inside_vfov ? 1 : 0)
                    << " visible_frontier_cells=" << candidate.visible_frontier_cells
                    << " frontier_points=" << target.frontier_points.size()
                    << " visible_fraction=" << visible_fraction
                    << " unknown_gain=" << candidate.unknown_gain);
          }
          store_debug_candidate(candidate);
          continue;
        }

        if (candidate.unknown_gain < min_unknown_gain_) {
          ++last_stats_.reject_gain;
          record_reject_sample(REJECT_GAIN, candidate);
          store_debug_candidate(candidate);
          continue;
        }

        if (reachability_checker_) {
          double path_cost = std::numeric_limits<double>::infinity();
          candidate.reachable = reachability_checker_(robot_pose, candidate.base_pose,
                                                      candidate.base_pose.z(), path_cost);
          candidate.path_cost = path_cost;
          if (!candidate.reachable) {
            ++last_stats_.reject_reachable;
            record_reject_sample(REJECT_REACHABLE, candidate);
            store_debug_candidate(candidate);
            continue;
          }
        } else {
          candidate.reachable = true;
          candidate.path_cost = candidate.distance_cost;
        }

        if (current_arm_config.size() == candidate.arm_config.size()) {
          candidate.arm_motion_cost = 0.0;
          for (size_t i = 0; i < candidate.arm_config.size(); ++i) {
            candidate.arm_motion_cost += std::abs(yawDiff(current_arm_config[i], candidate.arm_config[i]));
          }
        }

        candidate.score = scoreCandidate(candidate);
        candidate.valid = true;
        qualified_candidates.push_back(candidate);
      }
    }
    std::stable_sort(
        qualified_candidates.begin(), qualified_candidates.end(),
        [](const FuelViewpoint& lhs, const FuelViewpoint& rhs) {
          if (lhs.visible_frontier_cells != rhs.visible_frontier_cells) {
            return lhs.visible_frontier_cells > rhs.visible_frontier_cells;
          }
          if (lhs.unknown_gain != rhs.unknown_gain) {
            return lhs.unknown_gain > rhs.unknown_gain;
          }
          return lhs.score > rhs.score;
        });
    if (static_cast<int>(qualified_candidates.size()) > max_valid_per_target_) {
      qualified_candidates.resize(max_valid_per_target_);
    }
    for (const FuelViewpoint& candidate : qualified_candidates) {
      if (static_cast<int>(last_candidates_.size()) >= max_total_candidates_) break;
      last_candidates_.push_back(candidate);
      ++last_stats_.valid;
      if (!has_best_viewpoint_ || candidate.score > best_viewpoint_.score) {
        has_best_viewpoint_ = true;
        best_viewpoint_ = candidate;
        last_stats_.best_target = candidate.target_id;
        last_stats_.best_score = candidate.score;
      }
    }
    const int kept_for_target = static_cast<int>(qualified_candidates.size());
    if (kept_for_target > 0) {
      ++last_stats_.valid_target_count;
      active_target_ids_.push_back(target.id);
    } else {
      dormant_target_ids_.push_back(target.id);
    }
    valid_candidates_by_target.emplace_back(target.id, kept_for_target);
    ROS_INFO("[FuelViewpointGenerator] target_id=%d evaluated=%d qualified=%zu kept=%d",
             target.id, target_evaluated, qualified_candidates.size(), kept_for_target);
  }
  last_stats_.dormant_target_count =
      std::max(0, target_count - last_stats_.valid_target_count);

  const auto sampling_t1 = std::chrono::steady_clock::now();
  last_stats_.sampling_ms =
      std::chrono::duration<double, std::milli>(sampling_t1 - sampling_t0).count() -
      last_stats_.visibility_ms;
  const auto total_t1 = std::chrono::steady_clock::now();
  last_stats_.total_ms =
      std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();

  ROS_INFO("[FuelViewpointGenerator] targets=%d sampled=%d reject_map=%d reject_free=%d reject_clearance=%d reject_joint=%d reject_collision=%d reject_facing=%d reject_visibility=%d visibility_budget_skipped=%d reject_gain=%d reject_reachable=%d full_visibility_checked=%d valid=%d active_targets=%d dormant_targets=%d best_target=%d best_score=%.3f sampling_ms=%.3f visibility_ms=%.3f total_ms=%.3f",
           last_stats_.targets, last_stats_.sampled, last_stats_.reject_map,
           last_stats_.reject_free, last_stats_.reject_clearance,
           last_stats_.reject_joint, last_stats_.reject_collision,
           last_stats_.reject_facing, last_stats_.reject_visibility,
           last_stats_.skipped_visibility_budget,
           last_stats_.reject_gain, last_stats_.reject_reachable,
           last_stats_.full_visibility_checked,
           last_stats_.valid, last_stats_.valid_target_count,
           last_stats_.dormant_target_count,
           last_stats_.best_target, last_stats_.best_score,
           std::max(0.0, last_stats_.sampling_ms), last_stats_.visibility_ms,
           last_stats_.total_ms);

  for (const auto& target_valid : valid_candidates_by_target) {
    ROS_INFO("[FuelViewpointGenerator] target_id=%d valid_candidates=%d",
             target_valid.first, target_valid.second);
  }

  for (const auto& arm_stats : facing_stats_by_arm) {
    const FacingStats& stats = arm_stats.second;
    ROS_INFO("[FuelViewpointGenerator] facing arm_config_id=%d facing_checked=%d reject_facing=%d facing_dot_min=%.6f facing_dot_max=%.6f facing_dot_average=%.6f",
             arm_stats.first, stats.checked, stats.rejected, stats.dot_min,
             stats.dot_max, stats.dot_sum / static_cast<double>(stats.checked));
  }

  ROS_INFO("[FuelViewpointGenerator] visibility_reject_breakdown reject_outside_hfov=%d reject_outside_vfov=%d reject_occluded=%d reject_visible_count=%d reject_visible_fraction=%d",
           last_stats_.reject_outside_hfov, last_stats_.reject_outside_vfov,
           last_stats_.reject_occluded, last_stats_.reject_visible_count,
           last_stats_.reject_visible_fraction);
  ROS_INFO("[FuelViewpointGenerator] optical_visibility unknown_blocks_ray=%d frustum_pass=%d reject_hfov=%d reject_vfov=%d blocked_raw_occupied=%d blocked_inflated_only=%d blocked_unknown=%d out_of_map=%d ray_visible=%d",
           visibility_unknown_blocks_ray_ ? 1 : 0, last_stats_.frustum_pass,
           last_stats_.reject_hfov, last_stats_.reject_vfov,
           last_stats_.blocked_raw_occupied, last_stats_.blocked_inflated_only,
           last_stats_.blocked_unknown, last_stats_.out_of_map,
           last_stats_.ray_visible);

  if (last_stats_.valid == 0) {
    const int reject_counts[REJECT_REASON_COUNT] = {
        last_stats_.reject_map,       last_stats_.reject_free,
        last_stats_.reject_clearance, last_stats_.reject_joint,
        last_stats_.reject_collision, last_stats_.reject_facing,
        last_stats_.reject_visibility, last_stats_.reject_gain,
        last_stats_.reject_reachable};
    int primary_reason = 0;
    for (int i = 1; i < REJECT_REASON_COUNT; ++i) {
      if (reject_counts[i] > reject_counts[primary_reason]) primary_reason = i;
    }
    const char* primary_reason_name = reject_counts[primary_reason] > 0
                                          ? reject_reason_names[primary_reason]
                                          : "none";
    ROS_WARN_THROTTLE(1.0,
                      "[FuelViewpointGenerator] valid=0 primary_reject=%s count=%d",
                      primary_reason_name, reject_counts[primary_reason]);
    std::ostringstream sample_log;
    sample_log << "[FuelViewpointGenerator] valid=0 reject samples:";
    for (int i = 0; i < REJECT_REASON_COUNT; ++i) {
      if (!has_reject_sample[i]) continue;
      const FuelViewpoint& sample = reject_samples[i];
      sample_log << "\n  reason=" << reject_reason_names[i]
                 << " target_id=" << sample.target_id
                 << " candidate_id=" << sample.candidate_id
                 << " base_pose=(" << sample.base_pose.x() << "," << sample.base_pose.y()
                 << "," << sample.base_pose.z() << ")"
                 << " camera_pos=(" << sample.camera_pos.x() << "," << sample.camera_pos.y()
                 << "," << sample.camera_pos.z() << ")"
                 << " arm_config_id=" << sample.arm_config_id;
    }
    ROS_WARN_STREAM_THROTTLE(1.0, sample_log.str());

    if (collision_checker_ && has_reject_sample[REJECT_COLLISION]) {
      const FuelViewpoint& sample = reject_samples[REJECT_COLLISION];
      if (current_arm_config.size() == sample.arm_config.size()) {
        const bool base_with_current_arm_collision =
            collision_checker_(sample.base_pose, current_arm_config, false);
        const bool current_base_with_candidate_arm_collision =
            collision_checker_(robot_pose, sample.arm_config, false);
        const bool full_candidate_collision_no_margin =
            collision_checker_(sample.base_pose, sample.arm_config, false);
        const bool full_candidate_collision_safe_margin =
            collision_checker_(sample.base_pose, sample.arm_config, true);
        ROS_WARN_THROTTLE(
            1.0,
            "candidate target=%d candidate=%d\n"
            "base_with_current_arm_collision=%d\n"
            "current_base_with_candidate_arm_collision=%d\n"
            "full_candidate_collision_no_margin=%d\n"
            "full_candidate_collision_safe_margin=%d",
            sample.target_id, sample.candidate_id,
            base_with_current_arm_collision ? 1 : 0,
            current_base_with_candidate_arm_collision ? 1 : 0,
            full_candidate_collision_no_margin ? 1 : 0,
            full_candidate_collision_safe_margin ? 1 : 0);
      } else {
        ROS_WARN_THROTTLE(
            1.0,
            "[FuelViewpointGenerator] collision breakdown skipped: "
            "current_arm_dof=%zu candidate_arm_dof=%zu",
            current_arm_config.size(), sample.arm_config.size());
      }
    }
  }

  return last_candidates_;
}

bool FuelViewpointGenerator::hasBestViewpoint() const
{
  return has_best_viewpoint_;
}

const FuelViewpoint& FuelViewpointGenerator::bestViewpoint() const
{
  return best_viewpoint_;
}

const std::vector<FuelViewpoint>& FuelViewpointGenerator::lastCandidates() const
{
  return last_candidates_;
}

const std::vector<int>& FuelViewpointGenerator::activeTargetIds() const
{
  return active_target_ids_;
}

const std::vector<int>& FuelViewpointGenerator::dormantTargetIds() const
{
  return dormant_target_ids_;
}

const FuelViewpointStats& FuelViewpointGenerator::lastStats() const
{
  return last_stats_;
}

bool FuelViewpointGenerator::loadArmViewConfigs(ros::NodeHandle& nh)
{
  arm_view_configs_.clear();

  XmlRpc::XmlRpcValue configs;
  if (!nh.getParam("fuel_viewpoint/arm_view_configs", configs) ||
      configs.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return false;
  }

  for (int i = 0; i < configs.size(); ++i) {
    if (configs[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;

    FuelArmViewConfig config;
    if (configs[i].hasMember("id") &&
        configs[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      config.id = static_cast<int>(configs[i]["id"]);
    } else {
      config.id = i;
    }

    std::vector<double> joints_deg;
    if (!configs[i].hasMember("joints") ||
        !readXmlRpcDoubleArray(configs[i]["joints"], joints_deg)) {
      ROS_WARN("[FuelViewpointGenerator] Skip arm view config %d: invalid joints.", i);
      continue;
    }
    config.joints.reserve(joints_deg.size());
    for (const double joint_deg : joints_deg) config.joints.push_back(joint_deg * M_PI / 180.0);

    std::vector<double> translation;
    if (configs[i].hasMember("translation") &&
        readXmlRpcDoubleArray(configs[i]["translation"], translation) &&
        translation.size() >= 3) {
      config.camera_offset =
          Eigen::Vector3d(translation[0], translation[1], translation[2]);
    } else if (configs[i].hasMember("camera_offset") &&
               readXmlRpcDoubleArray(configs[i]["camera_offset"], translation) &&
               translation.size() >= 3) {
      config.camera_offset =
          Eigen::Vector3d(translation[0], translation[1], translation[2]);
    } else {
      config.camera_offset = Eigen::Vector3d(0.0, 0.0, 1.0);
    }

    std::vector<double> rotation_rpy_deg;
    if (configs[i].hasMember("rotation_rpy") &&
        readXmlRpcDoubleArray(configs[i]["rotation_rpy"], rotation_rpy_deg) &&
        rotation_rpy_deg.size() >= 3) {
      config.camera_rpy = Eigen::Vector3d(rotation_rpy_deg[0], rotation_rpy_deg[1],
                                          rotation_rpy_deg[2]) *
                          M_PI / 180.0;
      config.yaw_offset = config.camera_rpy.z();
      config.pitch_offset = config.camera_rpy.y();
    } else {
      double yaw_offset_deg = 0.0;
      if (configs[i].hasMember("yaw_offset")) {
        readXmlRpcDouble(configs[i]["yaw_offset"], yaw_offset_deg);
      }
      config.yaw_offset = yaw_offset_deg * M_PI / 180.0;

      double pitch_offset_deg = 0.0;
      if (configs[i].hasMember("pitch_offset")) {
        readXmlRpcDouble(configs[i]["pitch_offset"], pitch_offset_deg);
      }
      config.pitch_offset = pitch_offset_deg * M_PI / 180.0;
      config.camera_rpy = Eigen::Vector3d(0.0, config.pitch_offset, config.yaw_offset);
    }
    updateCameraTransform(config);

    arm_view_configs_.push_back(config);
  }

  ROS_INFO("[FuelViewpointGenerator] loaded %zu arm view configs.", arm_view_configs_.size());
  return !arm_view_configs_.empty();
}

bool FuelViewpointGenerator::loadJointLimits(ros::NodeHandle& nh)
{
  const bool has_min = nh.getParam("mm/manipulator_min_pos", joint_min_);
  const bool has_max = nh.getParam("mm/manipulator_max_pos", joint_max_);
  if (has_min && has_max && joint_min_.size() == joint_max_.size()) {
    mani_dof_ = static_cast<int>(joint_min_.size());
    return true;
  }
  joint_min_.clear();
  joint_max_.clear();
  return false;
}

bool FuelViewpointGenerator::simpleCameraPose(const Eigen::Vector3d& base_pose,
                                              const FuelArmViewConfig& config,
                                              Eigen::Vector3d& camera_pos,
                                              Eigen::Quaterniond& camera_q) const
{
  Eigen::Isometry3d T_world_base = Eigen::Isometry3d::Identity();
  T_world_base.linear() =
      Eigen::AngleAxisd(base_pose.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T_world_base.translation() = Eigen::Vector3d(base_pose.x(), base_pose.y(), base_check_z_);
  const Eigen::Isometry3d T_world_camera = T_world_base * config.T_ee_camera;
  camera_pos = T_world_camera.translation();
  camera_q = Eigen::Quaterniond(T_world_camera.rotation());
  camera_q.normalize();
  return true;
}

std::vector<Eigen::Vector3d> FuelViewpointGenerator::sampleBasePoses(
    const FrontierTarget& target,
    const Eigen::Vector3d& robot_pose,
    int& sampled_count) const
{
  std::vector<Eigen::Vector3d> samples;
  sampled_count = 0;
  samples.reserve(max_candidates_per_target_);

  Eigen::Vector2d known_dir(-target.normal.x(), -target.normal.y());
  if (known_dir.norm() < 1e-3) {
    known_dir = robot_pose.head<2>() - target.center.head<2>();
  }
  if (known_dir.norm() < 1e-3) {
    ROS_WARN_THROTTLE(1.0,
                      "[FuelViewpointGenerator] target %d has no usable XY normal.",
                      target.id);
    return samples;
  }
  known_dir.normalize();
  const double center_angle = std::atan2(known_dir.y(), known_dir.x());
  const bool full_circle = angle_span_ >= 2.0 * M_PI - 1e-3;

  const auto append_sample = [&](const int ri, const double angle) {
    const double r = r_num_ <= 1
                         ? 0.5 * (r_min_ + r_max_)
                         : r_min_ + (r_max_ - r_min_) * static_cast<double>(ri) /
                                        static_cast<double>(r_num_ - 1);
    const Eigen::Vector2d dir(std::cos(angle), std::sin(angle));
    const Eigen::Vector2d base_xy = target.center.head<2>() + r * dir;
    const double yaw = std::atan2(target.center.y() - base_xy.y(),
                                  target.center.x() - base_xy.x());
    samples.emplace_back(base_xy.x(), base_xy.y(), yaw);
    ++sampled_count;
  };

  if (full_circle) {
    // Interleave radii and use a low-discrepancy angular order.  Even when a
    // per-target evaluation budget is reached, the checked samples still
    // cover the whole circle instead of only one contiguous angular sector.
    const double golden_angle = M_PI * (3.0 - std::sqrt(5.0));
    for (int ai = 0; ai < angle_num_; ++ai) {
      const double angle = center_angle + golden_angle * static_cast<double>(ai);
      for (int ri = 0; ri < r_num_; ++ri) {
        if (static_cast<int>(samples.size()) >= max_candidates_per_target_) {
          return samples;
        }
        append_sample(ri, angle);
      }
    }
    return samples;
  }

  for (int ri = 0; ri < r_num_; ++ri) {
    for (int ai = 0; ai < angle_num_; ++ai) {
      if (static_cast<int>(samples.size()) >= max_candidates_per_target_) break;
      const double angle_offset =
          angle_num_ <= 1
              ? 0.0
              : -0.5 * angle_span_ +
                    angle_span_ * static_cast<double>(ai) /
                        static_cast<double>(angle_num_ - 1);
      const double angle = center_angle + angle_offset;
      append_sample(ri, angle);
    }
  }

  return samples;
}

bool FuelViewpointGenerator::isBaseKnownFree(const Eigen::Vector3d& base_pose) const
{
  const double resolution = std::max(1e-3, grid_map_->getResolution());
  const int radius_voxels =
      static_cast<int>(std::ceil(base_footprint_radius_ / resolution));
  for (int dx = -radius_voxels; dx <= radius_voxels; ++dx) {
    for (int dy = -radius_voxels; dy <= radius_voxels; ++dy) {
      const double offset_x = dx * resolution;
      const double offset_y = dy * resolution;
      if (std::hypot(offset_x, offset_y) > base_footprint_radius_ + 1e-6) continue;
      const Eigen::Vector3d check_pos(base_pose.x() + offset_x,
                                      base_pose.y() + offset_y,
                                      base_check_z_);
      if (!grid_map_->isInMap(check_pos)) return false;
      Eigen::Vector3i id;
      grid_map_->posToIndex(check_pos, id);
      if (!grid_map_->isKnownFree(id) || grid_map_->isInflatedOccupied(check_pos)) {
        return false;
      }
    }
  }
  return true;
}

bool FuelViewpointGenerator::hasUnknownClearance(const Eigen::Vector3d& base_pose) const
{
  if (min_unknown_clearance_ <= 1e-3) return true;
  const double resolution = std::max(1e-3, grid_map_->getResolution());
  const double check_radius = base_footprint_radius_ + min_unknown_clearance_;
  const int radius_voxels = static_cast<int>(std::ceil(check_radius / resolution));
  for (int dx = -radius_voxels; dx <= radius_voxels; ++dx) {
    for (int dy = -radius_voxels; dy <= radius_voxels; ++dy) {
      const double offset_x = dx * resolution;
      const double offset_y = dy * resolution;
      if (std::hypot(offset_x, offset_y) > check_radius + 1e-6) continue;
      const Eigen::Vector3d check_pos(base_pose.x() + offset_x,
                                      base_pose.y() + offset_y,
                                      base_check_z_);
      if (!grid_map_->isInMap(check_pos)) return false;
      Eigen::Vector3i id;
      grid_map_->posToIndex(check_pos, id);
      if (grid_map_->getOccupancy(id) == GridMap::UNKNOWN) return false;
    }
  }
  return true;
}

bool FuelViewpointGenerator::hasObstacleClearance(const Eigen::Vector3d& base_pose) const
{
  if (min_obstacle_distance_ <= 1e-3) return true;
  const Eigen::Vector3d check_pos(base_pose.x(), base_pose.y(), base_check_z_);
  return grid_map_->getDistance(check_pos) >= min_obstacle_distance_;
}

bool FuelViewpointGenerator::jointsWithinLimits(const std::vector<double>& joints) const
{
  if (joints.empty()) return mani_dof_ <= 0;
  if (mani_dof_ > 0 && static_cast<int>(joints.size()) != mani_dof_) return false;
  if (joint_min_.size() != joints.size() || joint_max_.size() != joints.size()) return true;
  for (size_t i = 0; i < joints.size(); ++i) {
    if (joints[i] < joint_min_[i] - 1e-6 || joints[i] > joint_max_[i] + 1e-6) return false;
  }
  return true;
}

bool FuelViewpointGenerator::cameraFacesTarget(const FuelViewpoint& candidate,
                                               const FrontierTarget& target,
                                               Eigen::Vector3d* camera_forward_world,
                                               Eigen::Vector3d* to_target_world,
                                               double* facing_dot) const
{
  const Eigen::Vector3d to_target = target.center - candidate.camera_pos;
  const Eigen::Vector3d camera_forward = cameraForward(candidate.camera_q).normalized();
  const Eigen::Vector3d to_target_normalized =
      to_target.norm() < 1e-3 ? Eigen::Vector3d::Zero() : to_target.normalized();
  const double dot = camera_forward.dot(to_target_normalized);
  if (camera_forward_world) *camera_forward_world = camera_forward;
  if (to_target_world) *to_target_world = to_target_normalized;
  if (facing_dot) *facing_dot = dot;
  if (to_target.norm() < 1e-3) return false;
  return dot >= facing_cos_threshold_;
}

Eigen::Vector3d FuelViewpointGenerator::cameraForward(const Eigen::Quaterniond& camera_q) const
{
  return camera_q * Eigen::Vector3d::UnitZ();
}

void FuelViewpointGenerator::evaluateVisibilityAndGain(const FrontierTarget& target,
                                                       FuelViewpoint& candidate,
                                                       double& visibility_ms)
{
  const auto t0 = std::chrono::steady_clock::now();

  candidate.visible_frontier_cells = 0;
  for (const Eigen::Vector3d& point : target.frontier_points) {
    if (!pointInCameraFrustum(candidate.camera_pos, candidate.camera_q, point)) continue;
    if (rayVisibleToPoint(candidate.camera_pos, point)) ++candidate.visible_frontier_cells;
  }

  candidate.unknown_gain = countUnknownGain(candidate);

  const auto t1 = std::chrono::steady_clock::now();
  visibility_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
}

bool FuelViewpointGenerator::rayVisibleToPoint(const Eigen::Vector3d& camera_pos,
                                               const Eigen::Vector3d& point)
{
  if (!grid_map_->isInMap(camera_pos) || !grid_map_->isInMap(point)) {
    ++last_stats_.out_of_map;
    return false;
  }

  const Eigen::Vector3d half(0.5, 0.5, 0.5);
  const double resolution = grid_map_->getResolution();
  const std::vector<Eigen::Vector3i> cells =
      intermediateRayVoxels(camera_pos / resolution, point / resolution);
  for (const Eigen::Vector3i& id : cells) {
    const Eigen::Vector3d check_pos = (id.cast<double>() + half) * resolution;
    if (!grid_map_->isInMap(check_pos)) {
      ++last_stats_.out_of_map;
      return false;
    }

    const int raw = grid_map_->getOccupancy(id);
    const int inflated = grid_map_->getInflateOccupancy(check_pos);
    if (inflated == 1 && raw != GridMap::OCCUPIED) {
      ++last_stats_.blocked_inflated_only;
    }
    const bool blocked = raw == GridMap::OCCUPIED ||
                         (visibility_unknown_blocks_ray_ && raw == GridMap::UNKNOWN);
    if (blocked) {
      if (raw == GridMap::OCCUPIED) ++last_stats_.blocked_raw_occupied;
      else ++last_stats_.blocked_unknown;
      if (!first_blocker_logged_) {
        first_blocker_logged_ = true;
        ROS_WARN_STREAM("[FuelViewpointGenerator] first optical ray blocker"
                        << " index=(" << id.x() << "," << id.y() << "," << id.z() << ")"
                        << " position=(" << check_pos.x() << "," << check_pos.y()
                        << "," << check_pos.z() << ")"
                        << " raw_occupancy=" << raw
                        << " inflated_occupancy=" << inflated
                        << " distance_to_endpoint=" << (point - check_pos).norm());
      }
      return false;
    }
  }

  ++last_stats_.ray_visible;
  return true;
}

bool FuelViewpointGenerator::pointInCameraFrustum(const Eigen::Vector3d& camera_pos,
                                                  const Eigen::Quaterniond& camera_q,
                                                  const Eigen::Vector3d& point)
{
  const Eigen::Vector3d rel_world = point - camera_pos;
  const double range = rel_world.norm();
  if (range < min_view_range_ || range > max_view_range_) return false;

  const Eigen::Vector3d rel_cam = camera_q.inverse() * rel_world;
  const double forward = rel_cam.z();
  if (forward <= 1e-3) return false;
  const double horizontal = std::atan2(rel_cam.x(), rel_cam.z());
  const double vertical = std::atan2(-rel_cam.y(), rel_cam.z());
  if (std::abs(horizontal) > 0.5 * hfov_) {
    ++last_stats_.reject_hfov;
    return false;
  }
  if (std::abs(vertical) > 0.5 * vfov_) {
    ++last_stats_.reject_vfov;
    return false;
  }
  ++last_stats_.frustum_pass;
  return true;
}

int FuelViewpointGenerator::countUnknownGain(const FuelViewpoint& candidate) const
{
  std::unordered_set<int> unknown_cells;
  const Eigen::Vector3d forward = candidate.camera_q * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d right = candidate.camera_q * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d up = candidate.camera_q * (-Eigen::Vector3d::UnitY());

  for (double yaw = -0.5 * hfov_; yaw <= 0.5 * hfov_ + 1e-6; yaw += gain_yaw_step_) {
    for (double pitch = -0.5 * vfov_; pitch <= 0.5 * vfov_ + 1e-6; pitch += gain_pitch_step_) {
      Eigen::Vector3d dir =
          std::cos(pitch) * std::cos(yaw) * forward +
          std::cos(pitch) * std::sin(yaw) * right +
          std::sin(pitch) * up;
      dir.normalize();

      for (double range = min_view_range_; range <= max_view_range_; range += gain_ray_step_) {
        const Eigen::Vector3d check_pos = candidate.camera_pos + range * dir;
        if (!grid_map_->isInMap(check_pos)) break;
        if (grid_map_->isKnownOccupied(check_pos)) break;
        if (grid_map_->isUnknown(check_pos)) {
          Eigen::Vector3i id;
          grid_map_->posToIndex(check_pos, id);
          unknown_cells.insert(voxelKey(id));
        }
      }
    }
  }

  return static_cast<int>(unknown_cells.size());
}

bool FuelViewpointGenerator::opticalPointInFrustum(const Eigen::Vector3d& rel_cam,
                                                   const double min_range,
                                                   const double max_range,
                                                   const double hfov,
                                                   const double vfov)
{
  const double range = rel_cam.norm();
  if (range < min_range || range > max_range || rel_cam.z() <= 1e-3) return false;
  const double horizontal = std::atan2(rel_cam.x(), rel_cam.z());
  const double vertical = std::atan2(-rel_cam.y(), rel_cam.z());
  return std::abs(horizontal) <= 0.5 * hfov && std::abs(vertical) <= 0.5 * vfov;
}

std::vector<Eigen::Vector3i> FuelViewpointGenerator::intermediateRayVoxels(
    const Eigen::Vector3d& start_voxel, const Eigen::Vector3d& end_voxel)
{
  std::vector<Eigen::Vector3i> cells;
  RayCaster raycaster;
  if (!raycaster.setInput(start_voxel, end_voxel)) return cells;
  Eigen::Vector3d ray_pt;
  const Eigen::Vector3i start_id = start_voxel.array().floor().cast<int>();
  const Eigen::Vector3i end_id = end_voxel.array().floor().cast<int>();
  while (raycaster.step(ray_pt)) {
    const Eigen::Vector3i id = ray_pt.cast<int>();
    if (id == start_id || id == end_id) continue;
    cells.push_back(id);
  }
  return cells;
}

double FuelViewpointGenerator::scoreCandidate(const FuelViewpoint& candidate) const
{
  return weight_gain_ * static_cast<double>(candidate.unknown_gain) +
         weight_visible_ * static_cast<double>(candidate.visible_frontier_cells) -
         weight_distance_ * candidate.distance_cost -
         weight_yaw_ * candidate.yaw_cost -
         weight_arm_ * candidate.arm_motion_cost -
         weight_path_ * candidate.path_cost;
}

double FuelViewpointGenerator::yawDiff(const double a, const double b) const
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

int FuelViewpointGenerator::voxelKey(const Eigen::Vector3i& id) const
{
  return id.x() * 73856093 ^ id.y() * 19349663 ^ id.z() * 83492791;
}

visualization_msgs::MarkerArray FuelViewpointGenerator::makeDebugMarkers() const
{
  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker clear;
  clear.header.frame_id = frame_id_;
  clear.header.stamp = ros::Time::now();
  clear.action = visualization_msgs::Marker::DELETEALL;
  array.markers.push_back(clear);

  int marker_id = 0;
  for (const FuelViewpoint& candidate : last_candidates_) {
    appendCandidateMarker(array, marker_id, "fuel_base_viewpoint_valid", candidate,
                          0.10, 0.95, 0.25, 0.95, 0.13);
    appendArrowMarker(array, marker_id, "fuel_camera_view_valid", candidate,
                      0.10, 0.95, 0.25, 0.90, 0.025, 0.45);
  }

  int shown_sampled = 0;
  int shown_rejected = 0;
  for (const FuelViewpoint& candidate : debug_candidates_) {
    if (candidate.collision_free && candidate.visible_frontier_cells > 0) {
      if (!debug_show_sampled_ || shown_sampled >= debug_max_sampled_) continue;
      appendCandidateMarker(array, marker_id, "fuel_viewpoint_sampled", candidate,
                            1.0, 0.85, 0.05, 0.70, 0.10);
      ++shown_sampled;
    } else {
      if (!debug_show_rejected_ || shown_rejected >= debug_max_rejected_) continue;
      appendCandidateMarker(array, marker_id, "fuel_viewpoint_rejected", candidate,
                            1.0, 0.10, 0.10, 0.70, 0.10);
      ++shown_rejected;
    }
  }

  if (has_best_viewpoint_) {
    appendArrowMarker(array, marker_id, "fuel_camera_view_best", best_viewpoint_,
                      0.15, 1.0, 0.20, 1.0, 0.055, 0.85);
  }

  return array;
}

void FuelViewpointGenerator::publishDebugMarkers() const
{
  if (!debug_marker_pub_) return;
  debug_marker_pub_.publish(makeDebugMarkers());
}

void FuelViewpointGenerator::appendCandidateMarker(visualization_msgs::MarkerArray& array,
                                                   int& marker_id,
                                                   const std::string& ns,
                                                   const FuelViewpoint& candidate,
                                                   const double r,
                                                   const double g,
                                                   const double b,
                                                   const double a,
                                                   const double scale) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = marker_id++;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = candidate.base_pose.x();
  marker.pose.position.y = candidate.base_pose.y();
  // This marker represents the planar mobile-base goal, not the camera pose.
  marker.pose.position.z = base_marker_z_;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  array.markers.push_back(marker);
}

void FuelViewpointGenerator::appendArrowMarker(visualization_msgs::MarkerArray& array,
                                               int& marker_id,
                                               const std::string& ns,
                                               const FuelViewpoint& candidate,
                                               const double r,
                                               const double g,
                                               const double b,
                                               const double a,
                                               const double shaft_scale,
                                               const double length) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = marker_id++;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = shaft_scale;
  marker.scale.y = 2.4 * shaft_scale;
  marker.scale.z = 3.2 * shaft_scale;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  const Eigen::Vector3d start = candidate.camera_pos;
  const Eigen::Vector3d end = start + length * cameraForward(candidate.camera_q).normalized();
  geometry_msgs::Point p0;
  p0.x = start.x();
  p0.y = start.y();
  p0.z = start.z();
  geometry_msgs::Point p1;
  p1.x = end.x();
  p1.y = end.y();
  p1.z = end.z();
  marker.points.push_back(p0);
  marker.points.push_back(p1);
  array.markers.push_back(marker);
}
