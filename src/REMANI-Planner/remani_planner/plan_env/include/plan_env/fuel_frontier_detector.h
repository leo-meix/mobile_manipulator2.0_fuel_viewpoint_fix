#ifndef _FUEL_FRONTIER_DETECTOR_H
#define _FUEL_FRONTIER_DETECTOR_H

#include <Eigen/Eigen>
#include <plan_env/fuel_frontier_cell.h>
#include <plan_env/map_adapter.h>
#include <ros/ros.h>

#include <memory>
#include <vector>

struct FrontierCluster
{
  int id = -1;
  std::vector<Eigen::Vector3i> cells;
  std::vector<Eigen::Vector3d> points;
  Eigen::Vector3d average = Eigen::Vector3d::Zero();
  Eigen::Vector3d box_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d box_max = Eigen::Vector3d::Zero();
};

class FuelFrontierDetector
{
public:
  explicit FuelFrontierDetector(const std::shared_ptr<MapAdapter>& map);
  ~FuelFrontierDetector() = default;

  void setParams(ros::NodeHandle& nh);

  bool searchFrontiers();
  bool searchUpdatedRegion();
  bool isFrontierCell(const Eigen::Vector3i& idx) const;
  const FuelFrontierCellConfig& cellConfig() const;

  const std::vector<Eigen::Vector3d>& rawFrontierPoints() const;
  const std::vector<Eigen::Vector3i>& rawFrontierCells() const;
  const Eigen::Vector3i& updatedMin() const;
  const Eigen::Vector3i& updatedMax() const;
  int rawFrontierCount() const;
  double lastSearchTimeMs() const;

private:
  bool isReady();

  std::shared_ptr<MapAdapter> map_;
  Eigen::Vector3i voxel_num_ = Eigen::Vector3i::Zero();

  int search_expand_voxels_ = 3;
  FuelFrontierCellConfig cell_config_;
  bool print_debug_ = true;

  std::vector<Eigen::Vector3i> raw_frontier_cells_;
  std::vector<Eigen::Vector3d> raw_frontier_points_;
  Eigen::Vector3i updated_min_ = Eigen::Vector3i::Zero();
  Eigen::Vector3i updated_max_ = Eigen::Vector3i::Zero();
  double last_search_time_ms_ = 0.0;
};

#endif
