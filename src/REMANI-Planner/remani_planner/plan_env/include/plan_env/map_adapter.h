#ifndef _MAP_ADAPTER_H
#define _MAP_ADAPTER_H

#include <Eigen/Eigen>
#include <plan_env/grid_map.h>

class MapAdapter
{
public:
  explicit MapAdapter(const GridMap::Ptr& grid_map = GridMap::Ptr());
  ~MapAdapter() = default;

  void setMap(const GridMap::Ptr& grid_map);
  GridMap::Ptr getMap() const;
  bool valid() const;

  double getResolution() const;
  Eigen::Vector3d getOrigin() const;
  void getRegion(Eigen::Vector3d& origin, Eigen::Vector3d& size) const;
  void getVoxelNum(Eigen::Vector3i& voxel_num) const;

  void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) const;
  void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) const;

  bool isInMap(const Eigen::Vector3d& pos) const;
  bool isInMap(const Eigen::Vector3i& idx) const;
  bool isInBox(const Eigen::Vector3d& pos) const;
  bool isInBox(const Eigen::Vector3i& idx) const;
  void boundIndex(Eigen::Vector3i& id) const;

  int getOccupancy(const Eigen::Vector3d& pos) const;
  int getOccupancy(const Eigen::Vector3i& idx) const;
  int getInflateOccupancy(const Eigen::Vector3d& pos) const;
  double getDistance(const Eigen::Vector3d& pos) const;

  bool isKnownFree(const Eigen::Vector3i& idx) const;
  bool isKnownOccupied(const Eigen::Vector3i& idx) const;
  bool isKnownOccupied(const Eigen::Vector3d& pos) const;
  bool isUnknown(const Eigen::Vector3i& idx) const;
  bool isUnknown(const Eigen::Vector3d& pos) const;
  bool isInflatedOccupied(const Eigen::Vector3d& pos) const;

  int toAddress(const Eigen::Vector3i& idx) const;

  void getUpdatedBoxIndex(Eigen::Vector3i& min_id, Eigen::Vector3i& max_id,
                          bool reset = false) const;
  void getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax) const;

private:
  GridMap::Ptr grid_map_;
};

#endif
