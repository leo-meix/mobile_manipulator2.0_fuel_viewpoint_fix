#include <plan_manage/frontier_path_finder.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace remani_planner
{

void FrontierPathFinder::init(ros::NodeHandle& nh,
                              const GridMap::Ptr& grid_map,
                              const std::shared_ptr<MMConfig>& mm_config)
{
  grid_map_ = grid_map;
  mm_config_ = mm_config;

  nh.param("frontier/path_check_enable", enable_, true);
  nh.param("frontier/path_max_search_time", max_search_time_, 0.05);
  nh.param("frontier/path_search_margin", search_margin_, 2.0);
  nh.param("frontier/path_check_resolution", line_check_resolution_, 0.10);
  nh.param("frontier/path_yaw_check_weight", yaw_check_weight_, 0.5);
}

bool FrontierPathFinder::search(const Eigen::Vector3d& start,
                                const Eigen::Vector3d& goal,
                                const double goal_yaw,
                                std::vector<Eigen::Vector3d>& path,
                                double& path_len)
{
  path.clear();
  path_len = std::numeric_limits<double>::infinity();

  if (!grid_map_ || !mm_config_) return false;
  const Eigen::Vector2d start_xy = start.head<2>();
  const Eigen::Vector2d goal_xy = goal.head<2>();
  if (!grid_map_->isInMap(start_xy) || !grid_map_->isInMap(goal_xy)) return false;

  if (lineSearch(start, goal, goal_yaw, path, path_len)) return true;

  Eigen::Vector2i start_id = grid_map_->pos2dToIndex(start.head<2>());
  Eigen::Vector2i goal_id = grid_map_->pos2dToIndex(goal.head<2>());
  grid_map_->boundIndex(start_id);
  grid_map_->boundIndex(goal_id);

  if (!isStateFree(start_id, start.z()) || !isStateFree(goal_id, goal_yaw)) return false;

  Eigen::Vector2d min_pos = start.head<2>().cwiseMin(goal.head<2>()) -
                            Eigen::Vector2d(search_margin_, search_margin_);
  Eigen::Vector2d max_pos = start.head<2>().cwiseMax(goal.head<2>()) +
                            Eigen::Vector2d(search_margin_, search_margin_);
  Eigen::Vector2i min_id = grid_map_->pos2dToIndex(min_pos);
  Eigen::Vector2i max_id = grid_map_->pos2dToIndex(max_pos);
  grid_map_->boundIndex(min_id);
  grid_map_->boundIndex(max_id);

  std::priority_queue<Node, std::vector<Node>, NodeCompare> open;
  std::unordered_map<int, double> g_score;
  std::unordered_map<int, Eigen::Vector2i> parent;
  std::unordered_map<int, bool> closed;

  const auto heuristic = [&](const Eigen::Vector2i& id) {
    return (grid_map_->index2dToPos(id) - grid_map_->index2dToPos(goal_id)).norm();
  };

  const int start_key = toKey(start_id);
  g_score[start_key] = 0.0;
  open.push({start_id, 0.0, heuristic(start_id)});

  const ros::Time t0 = ros::Time::now();
  const std::array<Eigen::Vector2i, 8> neighbors = {
      Eigen::Vector2i(1, 0), Eigen::Vector2i(-1, 0), Eigen::Vector2i(0, 1), Eigen::Vector2i(0, -1),
      Eigen::Vector2i(1, 1), Eigen::Vector2i(1, -1), Eigen::Vector2i(-1, 1), Eigen::Vector2i(-1, -1)};

  while (!open.empty() && ros::ok()) {
    if ((ros::Time::now() - t0).toSec() > max_search_time_) return false;

    const Node cur = open.top();
    open.pop();
    const int cur_key = toKey(cur.id);
    if (closed[cur_key]) continue;
    closed[cur_key] = true;

    if (cur.id == goal_id) {
      reconstructPath(start_id, goal_id, parent, start, goal, path, path_len);
      return true;
    }

    const double cur_yaw = yaw_check_weight_ > 1e-3 ? goal_yaw : start.z();
    for (const auto& offset : neighbors) {
      Eigen::Vector2i next_id = cur.id + offset;
      if (!grid_map_->isInMap(next_id)) continue;
      if (next_id.x() < min_id.x() || next_id.x() > max_id.x() ||
          next_id.y() < min_id.y() || next_id.y() > max_id.y()) {
        continue;
      }
      if (!isStateFree(next_id, cur_yaw)) continue;

      const int next_key = toKey(next_id);
      if (closed[next_key]) continue;

      const double step_cost = offset.cast<double>().norm() * grid_map_->getResolution();
      const double tentative_g = cur.g + step_cost;
      const auto g_it = g_score.find(next_key);
      if (g_it != g_score.end() && tentative_g >= g_it->second) continue;

      parent[next_key] = cur.id;
      g_score[next_key] = tentative_g;
      open.push({next_id, tentative_g, tentative_g + heuristic(next_id)});
    }
  }

  return false;
}

int FrontierPathFinder::toKey(const Eigen::Vector2i& id) const
{
  return id.x() * 73856093 ^ id.y() * 19349663;
}

bool FrontierPathFinder::isStateFree(const Eigen::Vector2i& id, const double yaw) const
{
  if (!grid_map_->isInMap(id)) return false;
  const Eigen::Vector2d pos = grid_map_->index2dToPos(id);
  Eigen::Vector3d car_state(pos.x(), pos.y(), yaw);
  double min_dist = 0.0;
  return !mm_config_->checkCarObsCollision(car_state, false, false, min_dist);
}

bool FrontierPathFinder::lineSearch(const Eigen::Vector3d& start,
                                    const Eigen::Vector3d& goal,
                                    const double goal_yaw,
                                    std::vector<Eigen::Vector3d>& path,
                                    double& path_len) const
{
  const Eigen::Vector2d delta = goal.head<2>() - start.head<2>();
  const double dist = delta.norm();
  if (dist < 1e-3) {
    path = {start, goal};
    path_len = 0.0;
    return true;
  }

  const int sample_num =
      std::max(2, static_cast<int>(std::ceil(dist / std::max(0.02, line_check_resolution_))));
  for (int i = 0; i <= sample_num; ++i) {
    const double s = static_cast<double>(i) / static_cast<double>(sample_num);
    const Eigen::Vector2d pos = start.head<2>() + s * delta;
    const double yaw = start.z() + s * (goal_yaw - start.z());
    double min_dist = 0.0;
    if (!grid_map_->isInMap(pos) ||
        mm_config_->checkCarObsCollision(Eigen::Vector3d(pos.x(), pos.y(), yaw), false, false, min_dist)) {
      return false;
    }
  }

  path = {start, goal};
  path_len = dist;
  return true;
}

void FrontierPathFinder::reconstructPath(const Eigen::Vector2i& start_id,
                                         const Eigen::Vector2i& goal_id,
                                         const std::unordered_map<int, Eigen::Vector2i>& parent,
                                         const Eigen::Vector3d& start,
                                         const Eigen::Vector3d& goal,
                                         std::vector<Eigen::Vector3d>& path,
                                         double& path_len) const
{
  std::vector<Eigen::Vector2i> ids;
  Eigen::Vector2i cur = goal_id;
  ids.push_back(cur);
  while (cur != start_id) {
    const auto it = parent.find(toKey(cur));
    if (it == parent.end()) break;
    cur = it->second;
    ids.push_back(cur);
  }
  std::reverse(ids.begin(), ids.end());

  path.clear();
  path.push_back(start);
  for (size_t i = 1; i + 1 < ids.size(); ++i) {
    const Eigen::Vector2d p2d = grid_map_->index2dToPos(ids[i]);
    path.emplace_back(p2d.x(), p2d.y(), goal.z());
  }
  path.push_back(goal);
  path_len = pathLength(path);
}

double FrontierPathFinder::pathLength(const std::vector<Eigen::Vector3d>& path) const
{
  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    len += (path[i].head<2>() - path[i - 1].head<2>()).norm();
  }
  return len;
}

}  // namespace remani_planner
