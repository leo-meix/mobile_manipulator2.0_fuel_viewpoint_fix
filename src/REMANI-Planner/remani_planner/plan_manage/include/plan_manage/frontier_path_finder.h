#pragma once

#include <Eigen/Eigen>
#include <array>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

#include <mm_config/mm_config.hpp>
#include <plan_env/grid_map.h>
#include <ros/ros.h>

namespace remani_planner
{

class FrontierPathFinder
{
public:
  using Ptr = std::shared_ptr<FrontierPathFinder>;

  FrontierPathFinder() = default;

  void init(ros::NodeHandle& nh,
            const GridMap::Ptr& grid_map,
            const std::shared_ptr<MMConfig>& mm_config);

  bool enabled() const { return enable_; }

  bool search(const Eigen::Vector3d& start,
              const Eigen::Vector3d& goal,
              double goal_yaw,
              std::vector<Eigen::Vector3d>& path,
              double& path_len);

private:
  struct Node
  {
    Eigen::Vector2i id = Eigen::Vector2i::Zero();
    double g = 0.0;
    double f = 0.0;
  };

  struct NodeCompare
  {
    bool operator()(const Node& a, const Node& b) const { return a.f > b.f; }
  };

  int toKey(const Eigen::Vector2i& id) const;
  bool isStateFree(const Eigen::Vector2i& id, double yaw) const;
  bool lineSearch(const Eigen::Vector3d& start,
                  const Eigen::Vector3d& goal,
                  double goal_yaw,
                  std::vector<Eigen::Vector3d>& path,
                  double& path_len) const;
  void reconstructPath(const Eigen::Vector2i& start_id,
                       const Eigen::Vector2i& goal_id,
                       const std::unordered_map<int, Eigen::Vector2i>& parent,
                       const Eigen::Vector3d& start,
                       const Eigen::Vector3d& goal,
                       std::vector<Eigen::Vector3d>& path,
                       double& path_len) const;
  double pathLength(const std::vector<Eigen::Vector3d>& path) const;

  GridMap::Ptr grid_map_;
  std::shared_ptr<MMConfig> mm_config_;

  bool enable_ = true;
  double max_search_time_ = 0.05;
  double search_margin_ = 2.0;
  double line_check_resolution_ = 0.10;
  double yaw_check_weight_ = 0.5;
};

}  // namespace remani_planner
