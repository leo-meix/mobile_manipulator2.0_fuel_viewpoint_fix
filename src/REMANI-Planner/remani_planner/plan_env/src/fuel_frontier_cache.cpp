#include <plan_env/fuel_frontier_cache.h>

#include <algorithm>
#include <unordered_set>

void FuelFrontierCache::update(const std::vector<Eigen::Vector3i>& new_cells,
                               const Eigen::Vector3i& updated_min,
                               const Eigen::Vector3i& updated_max,
                               const MapAdapter& map,
                               const FuelFrontierCellConfig& config)
{
  if (!map.valid()) {
    clear();
    return;
  }

  Eigen::Vector3i box_min = updated_min - Eigen::Vector3i(2, 2, 2);
  Eigen::Vector3i box_max = updated_max + Eigen::Vector3i(2, 2, 2);
  map.boundIndex(box_min);
  map.boundIndex(box_max);

  std::vector<Eigen::Vector3i> merged_cells;
  merged_cells.reserve(cells_.size() + new_cells.size());
  std::unordered_set<int> used_addresses;
  used_addresses.reserve((cells_.size() + new_cells.size()) * 2);

  for (const Eigen::Vector3i& cell : cells_) {
    if (isInBox(cell, box_min, box_max)) continue;
    if (!isFuelFrontierCell(cell, map, config)) continue;

    const int address = map.toAddress(cell);
    if (address == GridMap::INVALID_IDX) continue;
    if (used_addresses.insert(address).second) {
      merged_cells.push_back(cell);
    }
  }

  for (const Eigen::Vector3i& cell : new_cells) {
    if (!isFuelFrontierCell(cell, map, config)) continue;

    const int address = map.toAddress(cell);
    if (address == GridMap::INVALID_IDX) continue;
    if (used_addresses.insert(address).second) {
      merged_cells.push_back(cell);
    }
  }

  cells_.swap(merged_cells);
}

const std::vector<Eigen::Vector3i>& FuelFrontierCache::cells() const
{
  return cells_;
}

void FuelFrontierCache::clear()
{
  cells_.clear();
}

bool FuelFrontierCache::isInBox(const Eigen::Vector3i& idx,
                                const Eigen::Vector3i& box_min,
                                const Eigen::Vector3i& box_max) const
{
  return idx.x() >= box_min.x() && idx.x() <= box_max.x() &&
         idx.y() >= box_min.y() && idx.y() <= box_max.y() &&
         idx.z() >= box_min.z() && idx.z() <= box_max.z();
}
