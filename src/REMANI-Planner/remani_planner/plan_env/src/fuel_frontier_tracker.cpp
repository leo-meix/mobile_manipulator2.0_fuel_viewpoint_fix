#include <plan_env/fuel_frontier_tracker.h>

#include <algorithm>
#include <chrono>
#include <cmath>

void FuelFrontierTracker::setParams(ros::NodeHandle& nh)
{
  nh.param("frontier_tracker/match_distance", match_distance_, 0.8);
  nh.param("frontier_tracker/nearest_cell_distance", nearest_cell_distance_voxels_, 2.0);
  nh.param("frontier_tracker/nearest_cell_distance_voxels",
           nearest_cell_distance_voxels_, nearest_cell_distance_voxels_);
  nh.param("frontier_tracker/distance_weight", distance_weight_, 1.0);
  nh.param("frontier_tracker/size_weight", size_weight_, 1.0);

  match_distance_ = std::max(0.0, match_distance_);
  nearest_cell_distance_voxels_ = std::max(0.0, nearest_cell_distance_voxels_);
  distance_weight_ = std::max(0.0, distance_weight_);
  size_weight_ = std::max(0.0, size_weight_);
}

void FuelFrontierTracker::update(std::vector<FuelFrontierCluster>& clusters)
{
  const auto t0 = std::chrono::steady_clock::now();
  const int previous_count = static_cast<int>(previous_clusters_.size());

  std::vector<MatchCandidate> candidates;
  candidates.reserve(clusters.size() * previous_clusters_.size());

  for (size_t i = 0; i < clusters.size(); ++i) {
    clusters[i].id = -1;
    for (size_t j = 0; j < previous_clusters_.size(); ++j) {
      if (!canMatch(clusters[i], previous_clusters_[j])) continue;

      MatchCandidate candidate;
      candidate.new_index = static_cast<int>(i);
      candidate.old_index = static_cast<int>(j);
      candidate.cost = matchCost(clusters[i], previous_clusters_[j]);
      candidates.push_back(candidate);
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const MatchCandidate& lhs, const MatchCandidate& rhs) {
              return lhs.cost < rhs.cost;
            });

  std::vector<char> new_matched(clusters.size(), 0);
  std::vector<char> old_matched(previous_clusters_.size(), 0);
  int matched = 0;

  for (const MatchCandidate& candidate : candidates) {
    if (new_matched[candidate.new_index] || old_matched[candidate.old_index]) continue;

    clusters[candidate.new_index].id = previous_clusters_[candidate.old_index].id;
    new_matched[candidate.new_index] = 1;
    old_matched[candidate.old_index] = 1;
    ++matched;
  }

  int created = 0;
  for (FuelFrontierCluster& cluster : clusters) {
    if (cluster.id >= 0) continue;
    cluster.id = next_id_++;
    ++created;
  }

  const int removed = previous_count - matched;
  previous_clusters_ = clusters;

  const auto t1 = std::chrono::steady_clock::now();
  const double time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  ROS_INFO_THROTTLE(
      1.0,
      "[FuelFrontierTracker] clusters=%zu prev=%zu matched=%d created=%d removed=%d next_id=%d time_ms=%.3f",
      clusters.size(), static_cast<size_t>(previous_count), matched, created, removed,
      next_id_, time_ms);
}

void FuelFrontierTracker::clear()
{
  previous_clusters_.clear();
  next_id_ = 0;
}

bool FuelFrontierTracker::canMatch(const FuelFrontierCluster& cur,
                                   const FuelFrontierCluster& prev) const
{
  if (prev.id < 0) return false;
  const double center_dist = (cur.average - prev.average).norm();
  if (center_dist > match_distance_) return false;

  return boxesOverlap(cur, prev) || nearestCellsClose(cur, prev);
}

bool FuelFrontierTracker::boxesOverlap(const FuelFrontierCluster& cur,
                                       const FuelFrontierCluster& prev) const
{
  return cur.box_min.x() <= prev.box_max.x() && cur.box_max.x() >= prev.box_min.x() &&
         cur.box_min.y() <= prev.box_max.y() && cur.box_max.y() >= prev.box_min.y() &&
         cur.box_min.z() <= prev.box_max.z() && cur.box_max.z() >= prev.box_min.z();
}

bool FuelFrontierTracker::nearestCellsClose(const FuelFrontierCluster& cur,
                                            const FuelFrontierCluster& prev) const
{
  if (cur.filtered_cells.empty() || prev.filtered_cells.empty()) return false;

  const int threshold =
      static_cast<int>(std::ceil(nearest_cell_distance_voxels_));
  const int threshold_sq = threshold * threshold;
  for (const Eigen::Vector3i& cur_cell : cur.filtered_cells) {
    for (const Eigen::Vector3i& prev_cell : prev.filtered_cells) {
      if ((cur_cell - prev_cell).squaredNorm() <= threshold_sq) {
        return true;
      }
    }
  }

  return false;
}

double FuelFrontierTracker::matchCost(const FuelFrontierCluster& cur,
                                      const FuelFrontierCluster& prev) const
{
  const double center_cost = (cur.average - prev.average).norm();
  const double cur_size = static_cast<double>(cur.filtered_cells.size());
  const double prev_size = static_cast<double>(prev.filtered_cells.size());
  const double size_norm = std::max(1.0, std::max(cur_size, prev_size));
  const double size_cost = std::abs(cur_size - prev_size) / size_norm;

  return distance_weight_ * center_cost + size_weight_ * size_cost;
}
