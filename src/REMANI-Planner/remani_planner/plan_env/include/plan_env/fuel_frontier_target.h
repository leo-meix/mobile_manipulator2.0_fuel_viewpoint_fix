#ifndef _FUEL_FRONTIER_TARGET_H
#define _FUEL_FRONTIER_TARGET_H

#include <Eigen/Eigen>

#include <vector>

struct FrontierTarget
{
  int id = -1;
  // Geometric center for scoring/debug only; candidate viewpoints should be planned separately.
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d average = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d box_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d box_max = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> frontier_points;
  Eigen::Vector3d box_size = Eigen::Vector3d::Zero();
  int cell_num = 0;
  // Rough frontier-size gain. Final gain must be recomputed at candidate camera poses.
  double coarse_gain = 0.0;
  double information_gain = 0.0;
  double distance_to_robot = 0.0;
  double score = 0.0;
  bool valid = true;
};

struct FuelFrontierCluster
{
  int id = -1;
  std::vector<Eigen::Vector3i> filtered_cells;
  Eigen::Vector3d average = Eigen::Vector3d::Zero();
  Eigen::Vector3d box_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d box_max = Eigen::Vector3d::Zero();
};

class FuelFrontierClusterer
{
public:
  FuelFrontierClusterer() = default;
  explicit FuelFrontierClusterer(int cluster_min);

  void setClusterMin(int cluster_min);
  int clusterMin() const;
  void setLargeClusterMin(int large_cluster_min);
  int largeClusterMin() const;
  void setMaxSplitDepth(int max_split_depth);
  int maxSplitDepth() const;
  void setMinSplitExtentCells(double min_split_extent_cells);
  double minSplitExtentCells() const;
  void setMinSplitElongation(double min_split_elongation);
  double minSplitElongation() const;
  void setMinSplitChildRatio(double min_split_child_ratio);
  double minSplitChildRatio() const;

  std::vector<FuelFrontierCluster> cluster(
      const std::vector<Eigen::Vector3i>& input_cells);

private:
  void computeClusterInfo(FuelFrontierCluster& cluster) const;
  void splitClusterRecursive(const FuelFrontierCluster& cluster,
                             int depth,
                             std::vector<FuelFrontierCluster>& output,
                             int& split_count,
                             bool& max_split_depth_reached) const;
  bool splitClusterByPca(const FuelFrontierCluster& cluster,
                         FuelFrontierCluster& negative_cluster,
                         FuelFrontierCluster& positive_cluster) const;
  int effectiveLargeClusterMin() const;
  double clusterMaxXyExtent(const FuelFrontierCluster& cluster) const;

  int cluster_min_ = 1;
  int large_cluster_min_ = 200;
  int max_split_depth_ = 3;
  double min_split_extent_cells_ = 20.0;
  double min_split_elongation_ = 3.0;
  double min_split_child_ratio_ = 0.25;
};

#endif
