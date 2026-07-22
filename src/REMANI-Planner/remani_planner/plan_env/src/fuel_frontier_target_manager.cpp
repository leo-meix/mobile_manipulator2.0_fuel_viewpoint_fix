#include <plan_env/fuel_frontier_target_manager.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace {

void fillFallbackNormalFromRobot(FrontierTarget& target,
                                 const Eigen::Vector3d& robot_pos)
{
  if (target.normal.head<2>().norm() >= 1e-6) return;

  const Eigen::Vector2d fallback_normal =
      target.center.head<2>() - robot_pos.head<2>();
  if (fallback_normal.norm() > 1e-6) {
    const Eigen::Vector2d normal_xy = fallback_normal.normalized();
    target.normal = Eigen::Vector3d(normal_xy.x(), normal_xy.y(), 0.0);
  } else {
    target.valid = false;
  }
}

}  // namespace

FuelFrontierTargetManager::FuelFrontierTargetManager()
{
}

void FuelFrontierTargetManager::setParams(ros::NodeHandle& nh)
{
  nh.param("frontier_target/min_cell_num", min_cell_num_, 20);
  nh.param("frontier_target/min_box_size", min_box_size_, 0.05);
  nh.param("frontier_target/max_distance", max_distance_, 100.0);
  nh.param("frontier_target/distance_weight", distance_weight_, 1.0);
  nh.param("frontier_target/sort_by_score", sort_by_score_, true);
  nh.param("frontier_target/use_volume_gain", use_volume_gain_, false);

  min_cell_num_ = std::max(0, min_cell_num_);
  min_box_size_ = std::max(0.0, min_box_size_);
  max_distance_ = std::max(0.0, max_distance_);
}

void FuelFrontierTargetManager::setMap(const std::shared_ptr<MapAdapter>& map)
{
  map_ = map;
}

bool FuelFrontierTargetManager::updateTargets(
    const std::vector<FuelFrontierCluster>& clusters,
    const Eigen::Vector3d& robot_pos,
    double map_resolution)
{
  const auto t0 = std::chrono::steady_clock::now();

  targets_.clear();
  targets_.reserve(clusters.size());

  int dropped = 0;
  for (const FuelFrontierCluster& cluster : clusters) {
    FrontierTarget target = clusterToTarget(cluster, robot_pos, map_resolution);
    if (!isTargetValid(target)) {
      ++dropped;
      continue;
    }
    targets_.push_back(target);
  }

  sortTargets();

  const auto t1 = std::chrono::steady_clock::now();
  last_update_time_ms_ =
      std::chrono::duration<double, std::milli>(t1 - t0).count();

  int top_id = -1;
  double top_score = 0.0;
  double top_dist = 0.0;
  if (!targets_.empty()) {
    top_id = targets_.front().id;
    top_score = targets_.front().score;
    top_dist = targets_.front().distance_to_robot;
  }

  ROS_INFO_THROTTLE(
      1.0,
      "[FuelFrontierTargetManager] targets=%zu from_clusters=%zu dropped=%d top_id=%d top_score=%.3f top_dist=%.3f time=%.3f ms",
      targets_.size(), clusters.size(), dropped, top_id, top_score, top_dist,
      last_update_time_ms_);

  return !targets_.empty();
}

bool FuelFrontierTargetManager::updateTargets(const std::vector<FrontierCluster>& clusters,
                                              const Eigen::Vector3d& robot_pos,
                                              double map_resolution)
{
  const auto t0 = std::chrono::steady_clock::now();

  targets_.clear();
  targets_.reserve(clusters.size());

  int dropped = 0;
  for (const FrontierCluster& cluster : clusters) {
    FrontierTarget target = clusterToTarget(cluster, robot_pos, map_resolution);
    if (!isTargetValid(target)) {
      ++dropped;
      continue;
    }
    targets_.push_back(target);
  }

  sortTargets();

  const auto t1 = std::chrono::steady_clock::now();
  last_update_time_ms_ =
      std::chrono::duration<double, std::milli>(t1 - t0).count();

  int top_id = -1;
  double top_score = 0.0;
  double top_dist = 0.0;
  if (!targets_.empty()) {
    top_id = targets_.front().id;
    top_score = targets_.front().score;
    top_dist = targets_.front().distance_to_robot;
  }

  ROS_INFO_THROTTLE(
      1.0,
      "[FuelFrontierTargetManager] targets=%zu from_clusters=%zu dropped=%d top_id=%d top_score=%.3f top_dist=%.3f time=%.3f ms",
      targets_.size(), clusters.size(), dropped, top_id, top_score, top_dist,
      last_update_time_ms_);

  return !targets_.empty();
}

const std::vector<FrontierTarget>& FuelFrontierTargetManager::targets() const
{
  return targets_;
}

int FuelFrontierTargetManager::targetCount() const
{
  return static_cast<int>(targets_.size());
}

double FuelFrontierTargetManager::lastUpdateTimeMs() const
{
  return last_update_time_ms_;
}

FrontierTarget FuelFrontierTargetManager::clusterToTarget(
    const FuelFrontierCluster& cluster,
    const Eigen::Vector3d& robot_pos,
    double map_resolution) const
{
  FrontierTarget target;
  target.id = cluster.id;
  target.average = cluster.average;
  target.box_min = cluster.box_min;
  target.box_max = cluster.box_max;
  target.box_size = cluster.box_max - cluster.box_min;
  // Match FUEL's semantics: viewpoints are sampled around the mean of the
  // frontier cells.  The bounding-box center can lie in an obstacle or in an
  // empty part of an irregular frontier and is only suitable for RViz boxes.
  target.center = cluster.average;
  target.cell_num = static_cast<int>(cluster.filtered_cells.size());
  target.valid = true;
  fillFrontierGeometry(cluster.filtered_cells, target);
  fillFallbackNormalFromRobot(target, robot_pos);

  const double resolution = std::max(0.0, map_resolution);
  if (use_volume_gain_) {
    target.coarse_gain =
        static_cast<double>(target.cell_num) * resolution * resolution * resolution;
  } else {
    target.coarse_gain = static_cast<double>(target.cell_num);
  }
  target.information_gain = target.coarse_gain;

  // The exploration goal is a planar mobile-base pose.  Height reachability
  // is checked later using the arm configuration and the real camera FK.
  target.distance_to_robot =
      (target.center.head<2>() - robot_pos.head<2>()).norm();
  target.score = target.coarse_gain - distance_weight_ * target.distance_to_robot;
  return target;
}

FrontierTarget FuelFrontierTargetManager::clusterToTarget(
    const FrontierCluster& cluster,
    const Eigen::Vector3d& robot_pos,
    double map_resolution) const
{
  FrontierTarget target;
  target.id = cluster.id;
  target.average = cluster.average;
  target.box_min = cluster.box_min;
  target.box_max = cluster.box_max;
  target.box_size = cluster.box_max - cluster.box_min;
  target.center = cluster.average;
  target.cell_num = static_cast<int>(cluster.cells.size());
  target.frontier_points = cluster.points;
  target.valid = true;
  fillFrontierGeometry(cluster.cells, target);
  fillFallbackNormalFromRobot(target, robot_pos);

  const double resolution = std::max(0.0, map_resolution);
  if (use_volume_gain_) {
    target.coarse_gain =
        static_cast<double>(target.cell_num) * resolution * resolution * resolution;
  } else {
    target.coarse_gain = static_cast<double>(target.cell_num);
  }
  target.information_gain = target.coarse_gain;

  target.distance_to_robot =
      (target.center.head<2>() - robot_pos.head<2>()).norm();
  target.score = target.coarse_gain - distance_weight_ * target.distance_to_robot;
  return target;
}

void FuelFrontierTargetManager::fillFrontierGeometry(
    const std::vector<Eigen::Vector3i>& cells,
    FrontierTarget& target) const
{
  if (!map_ || !map_->valid()) return;

  static const Eigen::Vector3i dirs[4] = {
      Eigen::Vector3i(1, 0, 0), Eigen::Vector3i(-1, 0, 0),
      Eigen::Vector3i(0, 1, 0), Eigen::Vector3i(0, -1, 0)};

  target.frontier_points.clear();
  target.frontier_points.reserve(cells.size());
  Eigen::Vector2d normal_sum = Eigen::Vector2d::Zero();

  for (const Eigen::Vector3i& cell : cells) {
    if (!map_->isInBox(cell)) continue;

    Eigen::Vector3d frontier_pos;
    map_->indexToPos(cell, frontier_pos);
    target.frontier_points.push_back(frontier_pos);

    for (int i = 0; i < 4; ++i) {
      const Eigen::Vector3i nbr = cell + dirs[i];
      if (!map_->isInBox(nbr)) continue;
      if (map_->getOccupancy(nbr) != GridMap::UNKNOWN) continue;

      Eigen::Vector3d unknown_pos;
      map_->indexToPos(nbr, unknown_pos);
      normal_sum += (unknown_pos - frontier_pos).head<2>();
    }
  }

  if (normal_sum.norm() > 1e-6) {
    const Eigen::Vector2d normal_xy = normal_sum.normalized();
    target.normal = Eigen::Vector3d(normal_xy.x(), normal_xy.y(), 0.0);
  }
}

bool FuelFrontierTargetManager::isTargetValid(const FrontierTarget& target) const
{
  if (target.cell_num < min_cell_num_) return false;
  if (target.box_size.norm() < min_box_size_) return false;
  if (target.distance_to_robot > max_distance_) return false;
  if (!target.center.allFinite()) return false;
  if (!target.average.allFinite()) return false;
  if (!target.normal.allFinite()) return false;
  if (target.normal.head<2>().norm() < 1e-6) return false;
  if (!std::isfinite(target.score)) return false;
  if ((target.box_size.array() < 0.0).any()) return false;
  return target.valid;
}

void FuelFrontierTargetManager::sortTargets()
{
  if (!sort_by_score_) return;

  std::sort(targets_.begin(), targets_.end(),
            [](const FrontierTarget& lhs, const FrontierTarget& rhs) {
              return lhs.score > rhs.score;
            });
}
