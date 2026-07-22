#ifndef _FUEL_FRONTIER_TRACKER_H
#define _FUEL_FRONTIER_TRACKER_H

#include <Eigen/Eigen>
#include <plan_env/fuel_frontier_target.h>
#include <ros/ros.h>

#include <vector>

class FuelFrontierTracker
{
public:
  FuelFrontierTracker() = default;

  void setParams(ros::NodeHandle& nh);
  void update(std::vector<FuelFrontierCluster>& clusters);
  void clear();

private:
  struct MatchCandidate
  {
    int new_index = -1;
    int old_index = -1;
    double cost = 0.0;
  };

  bool canMatch(const FuelFrontierCluster& cur,
                const FuelFrontierCluster& prev) const;
  bool boxesOverlap(const FuelFrontierCluster& cur,
                    const FuelFrontierCluster& prev) const;
  bool nearestCellsClose(const FuelFrontierCluster& cur,
                         const FuelFrontierCluster& prev) const;
  double matchCost(const FuelFrontierCluster& cur,
                   const FuelFrontierCluster& prev) const;

  std::vector<FuelFrontierCluster> previous_clusters_;
  int next_id_ = 0;
  double match_distance_ = 0.8;
  double nearest_cell_distance_voxels_ = 2.0;
  double distance_weight_ = 1.0;
  double size_weight_ = 1.0;
};

#endif
