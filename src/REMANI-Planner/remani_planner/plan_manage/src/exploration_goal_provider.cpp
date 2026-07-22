#include <plan_manage/exploration_goal_provider.h>

#include <algorithm>

namespace remani_planner
{

namespace
{

ExplorationGoal fromFuelViewpoint(const FuelViewpoint& viewpoint,
                                  const FrontierTarget* target)
{
  ExplorationGoal goal;
  goal.frontier_id = viewpoint.target_id;
  goal.candidate_id = viewpoint.candidate_id;
  goal.arm_config_id = viewpoint.arm_config_id;
  goal.viewpoint = Eigen::Vector3d(viewpoint.base_pose.x(),
                                   viewpoint.base_pose.y(),
                                   0.0);
  goal.yaw = viewpoint.base_pose.z();
  goal.arm_config = viewpoint.arm_config;
  if (target) {
    goal.frontier_average = target->average;
  }
  goal.visible_num = viewpoint.visible_frontier_cells;
  goal.unknown_gain = viewpoint.unknown_gain;
  goal.path_cost = viewpoint.path_cost;
  goal.score = viewpoint.score;
  goal.in_place = viewpoint.in_place;
  return goal;
}

}  // namespace

FuelFrontierGoalProvider::FuelFrontierGoalProvider(
    const std::shared_ptr<FuelFrontierTargetManager>& target_manager,
    const FuelViewpointGenerator::Ptr& viewpoint_generator,
    const remani_planner::MMConfig::Ptr& mm_config)
  : target_manager_(target_manager),
    viewpoint_generator_(viewpoint_generator),
    mm_config_(mm_config)
{
}

bool FuelFrontierGoalProvider::enabled() const
{
  return target_manager_ && viewpoint_generator_;
}

bool FuelFrontierGoalProvider::getGoals(const Eigen::Vector3d& robot_state,
                                        std::vector<ExplorationGoal>& goals)
{
  goals.clear();
  if (!enabled()) return false;

  const std::vector<FrontierTarget>& targets = target_manager_->targets();
  if (targets.empty()) return false;

  std::vector<double> current_arm_config;
  if (mm_config_) {
    const Eigen::VectorXd current_mani = mm_config_->getManiConfig();
    current_arm_config.reserve(current_mani.size());
    for (int i = 0; i < current_mani.size(); ++i) {
      current_arm_config.push_back(current_mani(i));
    }
  }

  viewpoint_generator_->generate(targets, robot_state, current_arm_config);
  const std::vector<FuelViewpoint>& candidates = viewpoint_generator_->lastCandidates();

  goals.reserve(candidates.size());
  for (const FuelViewpoint& candidate : candidates) {
    if (!candidate.valid) continue;
    goals.push_back(fromFuelViewpoint(candidate, findTarget(candidate.target_id)));
  }

  std::stable_sort(goals.begin(), goals.end(),
                   [](const ExplorationGoal& lhs, const ExplorationGoal& rhs) {
                     return lhs.score > rhs.score;
                   });
  return !goals.empty();
}

const FrontierTarget* FuelFrontierGoalProvider::findTarget(const int target_id) const
{
  if (!target_manager_) return nullptr;
  const std::vector<FrontierTarget>& targets = target_manager_->targets();
  const auto iter = std::find_if(
      targets.begin(), targets.end(),
      [target_id](const FrontierTarget& target) { return target.id == target_id; });
  return iter == targets.end() ? nullptr : &(*iter);
}

}  // namespace remani_planner
