#ifndef _FUEL_FRONTIER_CELL_H
#define _FUEL_FRONTIER_CELL_H

#include <Eigen/Eigen>
#include <plan_env/map_adapter.h>

struct FuelFrontierCellConfig
{
  double min_z = 0.05;
  double max_z = -1.0;
  double min_obstacle_clearance = 0.0;
  bool use_max_z_filter = false;
  bool use_vertical_neighbors = false;
  bool reject_inflated_cells = true;
};

inline bool isFuelFrontierCell(const Eigen::Vector3i& idx,
                               const MapAdapter& map,
                               const FuelFrontierCellConfig& config)
{
  if (!map.isInBox(idx)) return false;
  if (map.getOccupancy(idx) != GridMap::FREE) return false;

  Eigen::Vector3d pos;
  map.indexToPos(idx, pos);
  if (pos.z() < config.min_z) return false;
  if (config.use_max_z_filter && pos.z() > config.max_z) return false;
  if (config.reject_inflated_cells && map.isInflatedOccupied(pos)) return false;
  if (config.min_obstacle_clearance > 0.0 &&
      map.getDistance(pos) < config.min_obstacle_clearance) {
    return false;
  }

  static const Eigen::Vector3i dirs[6] = {
      Eigen::Vector3i(1, 0, 0), Eigen::Vector3i(-1, 0, 0),
      Eigen::Vector3i(0, 1, 0), Eigen::Vector3i(0, -1, 0),
      Eigen::Vector3i(0, 0, 1), Eigen::Vector3i(0, 0, -1)};
  const int neighbor_count = config.use_vertical_neighbors ? 6 : 4;

  for (int i = 0; i < neighbor_count; ++i) {
    const Eigen::Vector3i nbr = idx + dirs[i];
    if (!map.isInBox(nbr)) continue;
    if (map.getOccupancy(nbr) == GridMap::UNKNOWN) return true;
  }

  return false;
}

#endif
