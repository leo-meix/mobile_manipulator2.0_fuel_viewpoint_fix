#pragma once

#include <Eigen/Eigen>

#include <memory>
#include <vector>

#include <mm_config/mm_config.hpp>
#include <plan_env/fuel_frontier_target_manager.h>
#include <plan_env/fuel_viewpoint_generator.h>

namespace remani_planner
{

struct ExplorationGoal
{
  int frontier_id = -1;
  int candidate_id = -1;
  int arm_config_id = -1;
  Eigen::Vector3d viewpoint = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  std::vector<double> arm_config;
  Eigen::Vector3d frontier_average = Eigen::Vector3d::Zero();
  int visible_num = 0;
  int unknown_gain = 0;
  double path_cost = 0.0;
  double score = 0.0;
  bool in_place = false;
};

class ExplorationGoalProvider
{
public:
  using Ptr = std::shared_ptr<ExplorationGoalProvider>;
  virtual ~ExplorationGoalProvider() = default;

  virtual bool getGoals(const Eigen::Vector3d& robot_state,
                        std::vector<ExplorationGoal>& goals) = 0;
  virtual bool enabled() const = 0;
};

class FuelFrontierGoalProvider : public ExplorationGoalProvider
{
public:
  FuelFrontierGoalProvider(
      const std::shared_ptr<FuelFrontierTargetManager>& target_manager,
      const FuelViewpointGenerator::Ptr& viewpoint_generator,
      const remani_planner::MMConfig::Ptr& mm_config);

  bool getGoals(const Eigen::Vector3d& robot_state,
                std::vector<ExplorationGoal>& goals) override;
  bool enabled() const override;

private:
  const FrontierTarget* findTarget(int target_id) const;

  std::shared_ptr<FuelFrontierTargetManager> target_manager_;
  FuelViewpointGenerator::Ptr viewpoint_generator_;
  remani_planner::MMConfig::Ptr mm_config_;
};

}  // namespace remani_planner
