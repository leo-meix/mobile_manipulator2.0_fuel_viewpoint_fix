#include <plan_env/fuel_frontier_target.h>

#include <ros/ros.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <unordered_set>

namespace {

struct Vector3iHash
{
  std::size_t operator()(const Eigen::Vector3i& v) const
  {
    std::size_t seed = 0;
    hashCombine(seed, v.x());
    hashCombine(seed, v.y());
    hashCombine(seed, v.z());
    return seed;
  }

  static void hashCombine(std::size_t& seed, int value)
  {
    const std::size_t h = std::hash<int>()(value);
    seed ^= h + 0x9e3779b9u + (seed << 6) + (seed >> 2);
  }
};

struct Vector3iEqual
{
  bool operator()(const Eigen::Vector3i& lhs, const Eigen::Vector3i& rhs) const
  {
    return lhs.x() == rhs.x() && lhs.y() == rhs.y() && lhs.z() == rhs.z();
  }
};

typedef std::unordered_set<Eigen::Vector3i, Vector3iHash, Vector3iEqual> CellSet;

}  // namespace

FuelFrontierClusterer::FuelFrontierClusterer(int cluster_min)
{
  setClusterMin(cluster_min);
}

void FuelFrontierClusterer::setClusterMin(int cluster_min)
{
  cluster_min_ = std::max(1, cluster_min);
  large_cluster_min_ = std::max(std::max(2 * cluster_min_, 200), large_cluster_min_);
}

int FuelFrontierClusterer::clusterMin() const
{
  return cluster_min_;
}

void FuelFrontierClusterer::setLargeClusterMin(int large_cluster_min)
{
  large_cluster_min_ = std::max(1, large_cluster_min);
}

int FuelFrontierClusterer::largeClusterMin() const
{
  return large_cluster_min_;
}

void FuelFrontierClusterer::setMaxSplitDepth(int max_split_depth)
{
  max_split_depth_ = std::max(0, max_split_depth);
}

int FuelFrontierClusterer::maxSplitDepth() const
{
  return max_split_depth_;
}

void FuelFrontierClusterer::setMinSplitExtentCells(double min_split_extent_cells)
{
  min_split_extent_cells_ = std::max(0.0, min_split_extent_cells);
}

double FuelFrontierClusterer::minSplitExtentCells() const
{
  return min_split_extent_cells_;
}

void FuelFrontierClusterer::setMinSplitElongation(double min_split_elongation)
{
  min_split_elongation_ = std::max(1.0, min_split_elongation);
}

double FuelFrontierClusterer::minSplitElongation() const
{
  return min_split_elongation_;
}

void FuelFrontierClusterer::setMinSplitChildRatio(double min_split_child_ratio)
{
  min_split_child_ratio_ = std::max(0.0, std::min(0.49, min_split_child_ratio));
}

double FuelFrontierClusterer::minSplitChildRatio() const
{
  return min_split_child_ratio_;
}

std::vector<FuelFrontierCluster> FuelFrontierClusterer::cluster(
    const std::vector<Eigen::Vector3i>& input_cells)
{
  const auto t0 = std::chrono::steady_clock::now();

  std::vector<FuelFrontierCluster> clusters;
  if (input_cells.empty()) {
    ROS_INFO("[FuelFrontierClusterer] input_cells=0 grown_clusters=0 dropped_small=0 accepted_clusters=0 max_cluster_cells=0 clusters_before_split=0 clusters_after_split=0 split_count=0 max_split_depth_reached=0 time_ms=0.000");
    return clusters;
  }

  CellSet unvisited;
  unvisited.reserve(input_cells.size() * 2);
  for (const Eigen::Vector3i& cell : input_cells) {
    unvisited.insert(cell);
  }

  int grown_clusters = 0;
  int dropped_small = 0;
  int max_cluster_cells = 0;
  std::queue<Eigen::Vector3i> queue;

  while (!unvisited.empty()) {
    const Eigen::Vector3i seed = *unvisited.begin();
    unvisited.erase(seed);

    FuelFrontierCluster cluster;
    queue.push(seed);

    while (!queue.empty()) {
      const Eigen::Vector3i cur = queue.front();
      queue.pop();
      cluster.filtered_cells.push_back(cur);

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) continue;

            const Eigen::Vector3i nbr = cur + Eigen::Vector3i(dx, dy, dz);
            CellSet::iterator iter = unvisited.find(nbr);
            if (iter == unvisited.end()) continue;

            queue.push(*iter);
            unvisited.erase(iter);
          }
        }
      }
    }

    ++grown_clusters;
    max_cluster_cells = std::max(
        max_cluster_cells, static_cast<int>(cluster.filtered_cells.size()));

    if (static_cast<int>(cluster.filtered_cells.size()) < cluster_min_) {
      ++dropped_small;
      continue;
    }

    cluster.id = -1;
    computeClusterInfo(cluster);
    clusters.push_back(cluster);
  }

  const int clusters_before_split = static_cast<int>(clusters.size());
  std::vector<FuelFrontierCluster> split_clusters;
  split_clusters.reserve(clusters.size());

  int split_count = 0;
  bool max_split_depth_reached = false;
  for (const FuelFrontierCluster& cluster : clusters) {
    splitClusterRecursive(cluster, 0, split_clusters, split_count,
                          max_split_depth_reached);
  }

  for (FuelFrontierCluster& cluster : split_clusters) {
    cluster.id = -1;
  }
  clusters.swap(split_clusters);

  const auto t1 = std::chrono::steady_clock::now();
  const double time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  ROS_INFO(
      "[FuelFrontierClusterer] input_cells=%zu grown_clusters=%d dropped_small=%d accepted_clusters=%zu max_cluster_cells=%d clusters_before_split=%d clusters_after_split=%zu split_count=%d max_split_depth_reached=%d time_ms=%.3f",
      input_cells.size(), grown_clusters, dropped_small, clusters.size(),
      max_cluster_cells, clusters_before_split, clusters.size(), split_count,
      max_split_depth_reached ? 1 : 0, time_ms);

  return clusters;
}

void FuelFrontierClusterer::splitClusterRecursive(
    const FuelFrontierCluster& cluster,
    int depth,
    std::vector<FuelFrontierCluster>& output,
    int& split_count,
    bool& max_split_depth_reached) const
{
  if (static_cast<int>(cluster.filtered_cells.size()) < effectiveLargeClusterMin()) {
    output.push_back(cluster);
    return;
  }
  if (clusterMaxXyExtent(cluster) < min_split_extent_cells_) {
    output.push_back(cluster);
    return;
  }

  if (depth >= max_split_depth_) {
    max_split_depth_reached = true;
    output.push_back(cluster);
    return;
  }

  FuelFrontierCluster negative_cluster;
  FuelFrontierCluster positive_cluster;
  if (!splitClusterByPca(cluster, negative_cluster, positive_cluster)) {
    output.push_back(cluster);
    return;
  }

  ++split_count;
  splitClusterRecursive(negative_cluster, depth + 1, output, split_count,
                        max_split_depth_reached);
  splitClusterRecursive(positive_cluster, depth + 1, output, split_count,
                        max_split_depth_reached);
}

bool FuelFrontierClusterer::splitClusterByPca(
    const FuelFrontierCluster& cluster,
    FuelFrontierCluster& negative_cluster,
    FuelFrontierCluster& positive_cluster) const
{
  if (static_cast<int>(cluster.filtered_cells.size()) < 2 * cluster_min_) {
    return false;
  }

  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const Eigen::Vector3i& cell : cluster.filtered_cells) {
    mean += cell.head<2>().cast<double>();
  }
  mean /= static_cast<double>(cluster.filtered_cells.size());

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const Eigen::Vector3i& cell : cluster.filtered_cells) {
    const Eigen::Vector2d diff = cell.head<2>().cast<double>() - mean;
    covariance += diff * diff.transpose();
  }
  covariance /= static_cast<double>(cluster.filtered_cells.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return false;
  }

  const Eigen::Vector2d eigenvalues = solver.eigenvalues();
  if (!eigenvalues.allFinite() || eigenvalues.maxCoeff() <= 1e-9) {
    return false;
  }
  const double lambda_min = std::max(0.0, eigenvalues.minCoeff());
  const double lambda_max = eigenvalues.maxCoeff();
  const double elongation =
      lambda_min <= 1e-9 ? std::numeric_limits<double>::infinity() : lambda_max / lambda_min;
  if (elongation < min_split_elongation_) {
    return false;
  }

  Eigen::Vector2d axis = solver.eigenvectors().col(1);
  if (!axis.allFinite() || axis.squaredNorm() <= 1e-9) {
    return false;
  }
  axis.normalize();

  negative_cluster.filtered_cells.clear();
  positive_cluster.filtered_cells.clear();
  negative_cluster.filtered_cells.reserve(cluster.filtered_cells.size());
  positive_cluster.filtered_cells.reserve(cluster.filtered_cells.size());

  for (const Eigen::Vector3i& cell : cluster.filtered_cells) {
    const double projection = (cell.head<2>().cast<double>() - mean).dot(axis);
    if (projection < 0.0) {
      negative_cluster.filtered_cells.push_back(cell);
    } else {
      positive_cluster.filtered_cells.push_back(cell);
    }
  }

  if (negative_cluster.filtered_cells.empty() || positive_cluster.filtered_cells.empty()) {
    return false;
  }
  if (static_cast<int>(negative_cluster.filtered_cells.size()) < cluster_min_ ||
      static_cast<int>(positive_cluster.filtered_cells.size()) < cluster_min_) {
    return false;
  }
  const double parent_size = static_cast<double>(cluster.filtered_cells.size());
  const double min_child_size = static_cast<double>(
      std::min(negative_cluster.filtered_cells.size(), positive_cluster.filtered_cells.size()));
  if (min_child_size / parent_size < min_split_child_ratio_) {
    return false;
  }

  computeClusterInfo(negative_cluster);
  computeClusterInfo(positive_cluster);
  return true;
}

int FuelFrontierClusterer::effectiveLargeClusterMin() const
{
  return std::max(2 * cluster_min_, large_cluster_min_);
}

double FuelFrontierClusterer::clusterMaxXyExtent(const FuelFrontierCluster& cluster) const
{
  if (cluster.filtered_cells.empty()) return 0.0;

  int min_x = cluster.filtered_cells.front().x();
  int max_x = min_x;
  int min_y = cluster.filtered_cells.front().y();
  int max_y = min_y;

  for (const Eigen::Vector3i& cell : cluster.filtered_cells) {
    min_x = std::min(min_x, cell.x());
    max_x = std::max(max_x, cell.x());
    min_y = std::min(min_y, cell.y());
    max_y = std::max(max_y, cell.y());
  }

  return static_cast<double>(std::max(max_x - min_x, max_y - min_y));
}

void FuelFrontierClusterer::computeClusterInfo(FuelFrontierCluster& cluster) const
{
  cluster.average.setZero();
  cluster.box_min.setZero();
  cluster.box_max.setZero();

  if (cluster.filtered_cells.empty()) return;

  cluster.box_min.setConstant(std::numeric_limits<double>::max());
  cluster.box_max.setConstant(-std::numeric_limits<double>::max());

  for (const Eigen::Vector3i& cell : cluster.filtered_cells) {
    const Eigen::Vector3d point = cell.cast<double>();
    cluster.average += point;
    cluster.box_min = cluster.box_min.cwiseMin(point);
    cluster.box_max = cluster.box_max.cwiseMax(point);
  }

  cluster.average /= static_cast<double>(cluster.filtered_cells.size());
}
