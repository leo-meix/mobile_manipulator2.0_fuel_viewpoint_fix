#include <plan_env/fuel_frontier_detector.h>
#include <plan_env/grid_map.h>
#include <plan_env/map_adapter.h>

#include <algorithm>
#include <memory>

class FuelFrontierDetectorNode
{
public:
  explicit FuelFrontierDetectorNode(ros::NodeHandle& nh) : nh_(nh), pnh_("~")
  {
    double update_rate = 1.0;
    pnh_.param("update_rate", update_rate, 1.0);
    update_rate = std::max(0.1, update_rate);

    grid_map_.reset(new GridMap);
    grid_map_->initMap(pnh_);

    map_adapter_.reset(new MapAdapter(grid_map_));
    detector_.reset(new FuelFrontierDetector(map_adapter_));
    detector_->setParams(pnh_);

    timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate),
                            &FuelFrontierDetectorNode::timerCallback, this);

    ROS_INFO("[FuelFrontierDetectorNode] started, update_rate=%.2f Hz", update_rate);
  }

private:
  void timerCallback(const ros::TimerEvent& /*event*/)
  {
    if (!grid_map_->hasDepthObservation()) {
      ROS_WARN_THROTTLE(2.0, "[FuelFrontierDetectorNode] waiting for map observation.");
      return;
    }

    ros::WallTime t_cb = ros::WallTime::now();
    detector_->searchFrontiers();
    double search_time = (ros::WallTime::now() - t_cb).toSec();

    ROS_INFO_THROTTLE(1.0,
        "[FuelFrontierDetectorNode] timer_cb: total=%.1f ms | raw_pts=%d",
        search_time * 1000.0, detector_->rawFrontierCount());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Timer timer_;
  GridMap::Ptr grid_map_;
  std::shared_ptr<MapAdapter> map_adapter_;
  std::shared_ptr<FuelFrontierDetector> detector_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fuel_frontier_detector_node");
  ros::NodeHandle nh;
  FuelFrontierDetectorNode node(nh);
  ros::spin();
  return 0;
}
