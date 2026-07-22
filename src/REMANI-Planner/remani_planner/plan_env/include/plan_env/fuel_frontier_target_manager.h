#ifndef _FUEL_FRONTIER_TARGET_MANAGER_H
#define _FUEL_FRONTIER_TARGET_MANAGER_H

#include <Eigen/Eigen>
#include <plan_env/fuel_frontier_detector.h>
#include <plan_env/map_adapter.h>
#include <plan_env/fuel_frontier_target.h>
#include <ros/ros.h>

#include <memory>
#include <vector>

class FuelFrontierTargetManager
{
public:
  FuelFrontierTargetManager();

  void setParams(ros::NodeHandle& nh);
  void setMap(const std::shared_ptr<MapAdapter>& map);
  bool updateTargets(const std::vector<FuelFrontierCluster>& clusters,
                     const Eigen::Vector3d& robot_pos,
                     double map_resolution);
  bool updateTargets(const std::vector<FrontierCluster>& clusters,
                     const Eigen::Vector3d& robot_pos,
                     double map_resolution);

  const std::vector<FrontierTarget>& targets() const;
  int targetCount() const;
  double lastUpdateTimeMs() const;

private:
  FrontierTarget clusterToTarget(const FuelFrontierCluster& cluster,
                                 const Eigen::Vector3d& robot_pos,
                                 double map_resolution) const;
  FrontierTarget clusterToTarget(const FrontierCluster& cluster,
                                 const Eigen::Vector3d& robot_pos,
                                 double map_resolution) const;
  void fillFrontierGeometry(const std::vector<Eigen::Vector3i>& cells,
                            FrontierTarget& target) const;
  bool isTargetValid(const FrontierTarget& target) const;
  void sortTargets();

  int min_cell_num_ = 20;
  double min_box_size_ = 0.05;
  double max_distance_ = 100.0;
  double distance_weight_ = 1.0;
  bool sort_by_score_ = true;
  bool use_volume_gain_ = false;

  std::shared_ptr<MapAdapter> map_;
  std::vector<FrontierTarget> targets_;
  double last_update_time_ms_ = 0.0;
};

#endif
