#include <plan_env/map_adapter.h>

MapAdapter::MapAdapter(const GridMap::Ptr& grid_map) : grid_map_(grid_map) {}

void MapAdapter::setMap(const GridMap::Ptr& grid_map)
{
  grid_map_ = grid_map;
}

GridMap::Ptr MapAdapter::getMap() const
{
  return grid_map_;
}

bool MapAdapter::valid() const
{
  return static_cast<bool>(grid_map_);
}

double MapAdapter::getResolution() const
{
  if (!grid_map_) return 0.0;
  return grid_map_->getResolution();
}

Eigen::Vector3d MapAdapter::getOrigin() const
{
  if (!grid_map_) return Eigen::Vector3d::Zero();
  return grid_map_->getOrigin();
}

void MapAdapter::getRegion(Eigen::Vector3d& origin, Eigen::Vector3d& size) const
{
  if (!grid_map_) {
    origin.setZero();
    size.setZero();
    return;
  }

  grid_map_->getRegion(origin, size);
}

void MapAdapter::getVoxelNum(Eigen::Vector3i& voxel_num) const
{
  if (!grid_map_) {
    voxel_num.setZero();
    return;
  }

  grid_map_->getVoxelNum(voxel_num);
}

void MapAdapter::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) const
{
  if (!grid_map_) {
    id.setConstant(GridMap::INVALID_IDX);
    return;
  }

  grid_map_->posToIndex(pos, id);
}

void MapAdapter::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) const
{
  if (!grid_map_ || !grid_map_->isInMap(id)) {
    pos.setZero();
    return;
  }

  grid_map_->indexToPos(id, pos);
}

bool MapAdapter::isInMap(const Eigen::Vector3d& pos) const
{
  return grid_map_ && grid_map_->isInMap(pos);
}

bool MapAdapter::isInMap(const Eigen::Vector3i& idx) const
{
  return grid_map_ && grid_map_->isInMap(idx);
}

bool MapAdapter::isInBox(const Eigen::Vector3d& pos) const
{
  return isInMap(pos);
}

bool MapAdapter::isInBox(const Eigen::Vector3i& idx) const
{
  return isInMap(idx);
}

void MapAdapter::boundIndex(Eigen::Vector3i& id) const
{
  if (!grid_map_) {
    id.setConstant(GridMap::INVALID_IDX);
    return;
  }

  grid_map_->boundIndex(id);
}

int MapAdapter::getOccupancy(const Eigen::Vector3d& pos) const
{
  if (!grid_map_ || !grid_map_->isInMap(pos)) return GridMap::OCCUPIED;
  return grid_map_->getOccupancy(pos);
}

int MapAdapter::getOccupancy(const Eigen::Vector3i& idx) const
{
  if (!grid_map_ || !grid_map_->isInMap(idx)) return GridMap::OCCUPIED;
  return grid_map_->getOccupancy(idx);
}

int MapAdapter::getInflateOccupancy(const Eigen::Vector3d& pos) const
{
  if (!grid_map_ || !grid_map_->isInMap(pos)) return 1;
  return grid_map_->getInflateOccupancy(pos);
}

double MapAdapter::getDistance(const Eigen::Vector3d& pos) const
{
  if (!grid_map_ || !grid_map_->isInMap(pos)) return 0.0;
  return grid_map_->getDistance(pos);
}

bool MapAdapter::isKnownFree(const Eigen::Vector3i& idx) const
{
  return grid_map_ && grid_map_->isInMap(idx) && grid_map_->isKnownFree(idx);
}

bool MapAdapter::isKnownOccupied(const Eigen::Vector3i& idx) const
{
  return !grid_map_ || !grid_map_->isInMap(idx) || grid_map_->isKnownOccupied(idx);
}

bool MapAdapter::isKnownOccupied(const Eigen::Vector3d& pos) const
{
  return !grid_map_ || !grid_map_->isInMap(pos) || grid_map_->isKnownOccupied(pos);
}

bool MapAdapter::isUnknown(const Eigen::Vector3i& idx) const
{
  return grid_map_ && grid_map_->isInMap(idx) && grid_map_->isUnknown(idx);
}

bool MapAdapter::isUnknown(const Eigen::Vector3d& pos) const
{
  return grid_map_ && grid_map_->isInMap(pos) && grid_map_->isUnknown(pos);
}

bool MapAdapter::isInflatedOccupied(const Eigen::Vector3d& pos) const
{
  return !grid_map_ || grid_map_->isInflatedOccupied(pos);
}

int MapAdapter::toAddress(const Eigen::Vector3i& idx) const
{
  if (!grid_map_ || !grid_map_->isInMap(idx)) return GridMap::INVALID_IDX;
  return grid_map_->toAddress(idx);
}

void MapAdapter::getUpdatedBoxIndex(Eigen::Vector3i& min_id, Eigen::Vector3i& max_id,
                                    bool reset) const
{
  if (!grid_map_) {
    min_id.setZero();
    max_id.setZero();
    return;
  }

  grid_map_->getUpdatedBoxIndex(min_id, max_id, reset);
}

void MapAdapter::getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax) const
{
  if (!grid_map_) {
    bmin.setZero();
    bmax.setZero();
    return;
  }

  grid_map_->getUpdatedBox(bmin, bmax);
}
