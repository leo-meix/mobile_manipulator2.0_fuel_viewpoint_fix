#ifndef _FUEL_FRONTIER_CACHE_H
#define _FUEL_FRONTIER_CACHE_H

#include <Eigen/Eigen>
#include <plan_env/fuel_frontier_cell.h>
#include <plan_env/map_adapter.h>

#include <vector>

class FuelFrontierCache
{
public:
  void update(const std::vector<Eigen::Vector3i>& new_cells,
              const Eigen::Vector3i& updated_min,
              const Eigen::Vector3i& updated_max,
              const MapAdapter& map,
              const FuelFrontierCellConfig& config);

  const std::vector<Eigen::Vector3i>& cells() const;
  void clear();

private:
  bool isInBox(const Eigen::Vector3i& idx,
               const Eigen::Vector3i& box_min,
               const Eigen::Vector3i& box_max) const;

  std::vector<Eigen::Vector3i> cells_;
};

#endif
