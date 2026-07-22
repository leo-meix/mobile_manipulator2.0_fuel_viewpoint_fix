#include <plan_env/fuel_frontier_detector.h>

#include <algorithm>

FuelFrontierDetector::FuelFrontierDetector(const std::shared_ptr<MapAdapter>& map)
  : map_(map)
{
}

void FuelFrontierDetector::setParams(ros::NodeHandle& nh)
{
  nh.param("frontier/search_expand_voxels", search_expand_voxels_, 3);
  nh.param("frontier/ground_z_min", cell_config_.min_z, 0.05);
  nh.param("frontier/use_vertical_neighbors", cell_config_.use_vertical_neighbors, false);
  nh.param("frontier/reject_inflated_cells", cell_config_.reject_inflated_cells, true);
  nh.param("frontier/min_obstacle_clearance", cell_config_.min_obstacle_clearance, 0.0);
  nh.param("frontier/print_debug", print_debug_, true);

  double max_z_param = -1.0;
  if (nh.getParam("frontier/max_z", max_z_param)) {
    cell_config_.max_z = max_z_param;
    cell_config_.use_max_z_filter = cell_config_.max_z > 0.0;
  } else if (map_ && map_->valid()) {
    Eigen::Vector3d origin, size;
    map_->getRegion(origin, size);
    cell_config_.max_z = origin.z() + size.z();
    cell_config_.use_max_z_filter = true;
  } else {
    cell_config_.max_z = -1.0;
    cell_config_.use_max_z_filter = false;
  }

  search_expand_voxels_ = std::max(0, search_expand_voxels_);
  cell_config_.min_obstacle_clearance =
      std::max(0.0, cell_config_.min_obstacle_clearance);
}

bool FuelFrontierDetector::searchFrontiers()
{
  return searchUpdatedRegion();
}

bool FuelFrontierDetector::searchUpdatedRegion()
{
  const ros::WallTime t0 = ros::WallTime::now();//计时

  raw_frontier_points_.clear();
  raw_frontier_cells_.clear();//清空数据
  last_search_time_ms_ = 0.0;

  if (!isReady()) {
    ROS_WARN("[FuelFrontierDetector] map is not ready.");
    return false;
  }

  Eigen::Vector3i min_id, max_id;
  map_->getUpdatedBoxIndex(min_id, max_id, false);

  const Eigen::Vector3i expand(search_expand_voxels_, search_expand_voxels_,
                               search_expand_voxels_);
  min_id -= expand;
  max_id += expand;
  map_->boundIndex(min_id);
  map_->boundIndex(max_id);
  updated_min_ = min_id;
  updated_max_ = max_id;

  int free_voxels = 0;
  int unknown_voxels = 0;
  int occupied_voxels = 0;

  for (int x = min_id.x(); x <= max_id.x(); ++x) {
    for (int y = min_id.y(); y <= max_id.y(); ++y) {
      for (int z = min_id.z(); z <= max_id.z(); ++z) {
        const Eigen::Vector3i idx(x, y, z);

        const int occ = map_->getOccupancy(idx);
        if (occ == GridMap::FREE) {
          ++free_voxels;
        } else if (occ == GridMap::UNKNOWN) {
          ++unknown_voxels;
        } else {
          ++occupied_voxels;
        }

        if (!isFrontierCell(idx)) continue;

        Eigen::Vector3d pos;
        map_->indexToPos(idx, pos);
        raw_frontier_cells_.push_back(idx);
        raw_frontier_points_.push_back(pos);
      }
    }
  }

  last_search_time_ms_ = (ros::WallTime::now() - t0).toSec() * 1000.0;

  ROS_INFO_THROTTLE(1.0,
      "[FuelFrontierDetector] searchUpdatedRegion: time=%.1f ms | raw_pts=%zu | "
      "free=%d unknown=%d occ=%d",
      last_search_time_ms_,
      raw_frontier_points_.size(), free_voxels, unknown_voxels, occupied_voxels);

  if (print_debug_) {
    ROS_INFO("[FuelFrontierDetector] range=[%d %d %d]~[%d %d %d] free=%d unknown=%d occ=%d raw=%zu time=%.2f ms",
             min_id.x(), min_id.y(), min_id.z(), max_id.x(), max_id.y(), max_id.z(),
             free_voxels, unknown_voxels, occupied_voxels, raw_frontier_points_.size(),
             last_search_time_ms_);
  }

  return true;
}

bool FuelFrontierDetector::isFrontierCell(const Eigen::Vector3i& idx) const
{
  return map_ && isFuelFrontierCell(idx, *map_, cell_config_);
}

const FuelFrontierCellConfig& FuelFrontierDetector::cellConfig() const
{
  return cell_config_;
}

const std::vector<Eigen::Vector3d>& FuelFrontierDetector::rawFrontierPoints() const
{
  return raw_frontier_points_;
}

const std::vector<Eigen::Vector3i>& FuelFrontierDetector::rawFrontierCells() const
{
  return raw_frontier_cells_;
}

const Eigen::Vector3i& FuelFrontierDetector::updatedMin() const
{
  return updated_min_;
}

const Eigen::Vector3i& FuelFrontierDetector::updatedMax() const
{
  return updated_max_;
}

int FuelFrontierDetector::rawFrontierCount() const
{
  return static_cast<int>(raw_frontier_points_.size());
}

double FuelFrontierDetector::lastSearchTimeMs() const
{
  return last_search_time_ms_;
}

bool FuelFrontierDetector::isReady()
{
  if (!map_ || !map_->valid()) return false;

  map_->getVoxelNum(voxel_num_);
  return voxel_num_.x() > 0 && voxel_num_.y() > 0 && voxel_num_.z() > 0;
}
