#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class CloudToWorldBridge {
 public:
  CloudToWorldBridge()
      : nh_(),
        pnh_("~"),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        startup_wall_time_(ros::WallTime::now()) {
    pnh_.param<std::string>("input_cloud_topic", input_cloud_topic_,
                            std::string("/robot1_camera/depth/color/points"));
    pnh_.param<std::string>("output_cloud_topic", output_cloud_topic_,
                            std::string("/gazebo_world/pointcloud"));
    pnh_.param<std::string>("target_frame", target_frame_, std::string("world"));
    pnh_.param<double>("tf_lookup_timeout", tf_lookup_timeout_sec_, 0.05);
    pnh_.param<int>("queue_size", queue_size_, 3);
    pnh_.param<bool>("warn_on_exact_fallback", warn_on_exact_fallback_, false);
    pnh_.param<double>("startup_quiet_sec", startup_quiet_sec_, 5.0);
    pnh_.param<double>("retry_period_sec", retry_period_sec_, 0.05);
    pnh_.param<double>("pending_cloud_max_age_sec", pending_cloud_max_age_sec_, 1.0);

    pub_cloud_world_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 5);
    sub_cloud_raw_ = nh_.subscribe(input_cloud_topic_, queue_size_,
                                   &CloudToWorldBridge::cloudCallback, this);
    retry_timer_ = nh_.createTimer(ros::Duration(retry_period_sec_),
                                   &CloudToWorldBridge::retryPendingCloud,
                                   this);

    ROS_INFO_STREAM("[cloud_to_world_bridge] input=" << input_cloud_topic_
                                                      << ", output=" << output_cloud_topic_
                                                      << ", target_frame=" << target_frame_
                                                      << ", startup_quiet_sec=" << startup_quiet_sec_
                                                      << ", retry_period_sec=" << retry_period_sec_);
  }

 private:
  bool inStartupQuietWindow() const {
    return (ros::WallTime::now() - startup_wall_time_).toSec() < startup_quiet_sec_;
  }

  bool lookupTransformWithFallback(const std::string& src_frame,
                                   const ros::Time& msg_stamp,
                                   geometry_msgs::TransformStamped& tf_msg,
                                   bool& used_latest) {
    used_latest = false;
    const ros::Duration timeout(tf_lookup_timeout_sec_);
    const ros::Time exact_stamp = msg_stamp.isZero() ? ros::Time(0) : msg_stamp;

    if (tf_buffer_.canTransform(target_frame_, src_frame, exact_stamp, timeout)) {
      tf_msg = tf_buffer_.lookupTransform(target_frame_, src_frame, exact_stamp, timeout);
      return true;
    }

    if (!exact_stamp.isZero() && tf_buffer_.canTransform(target_frame_, src_frame, ros::Time(0), timeout)) {
      tf_msg = tf_buffer_.lookupTransform(target_frame_, src_frame, ros::Time(0), timeout);
      used_latest = true;
      return true;
    }

    return false;
  }

  bool transformAndPublish(const sensor_msgs::PointCloud2ConstPtr& msg, bool from_retry) {
    geometry_msgs::TransformStamped tf_msg;
    bool used_latest = false;
    if (!lookupTransformWithFallback(msg->header.frame_id, msg->header.stamp, tf_msg,
                                     used_latest)) {
      pending_cloud_ = msg;
      pending_cloud_wall_time_ = ros::WallTime::now();

      if (!inStartupQuietWindow()) {
        ROS_WARN_THROTTLE(1.0,
                          "[cloud_to_world_bridge] TF not ready yet: %s -> %s%s",
                          msg->header.frame_id.c_str(), target_frame_.c_str(),
                          from_retry ? " (retrying cached cloud)" : "");
      } else {
        ROS_DEBUG_THROTTLE(1.0,
                           "[cloud_to_world_bridge] Waiting for TF chain: %s -> %s",
                           msg->header.frame_id.c_str(), target_frame_.c_str());
      }
      return false;
    }

    if (used_latest) {
      if (warn_on_exact_fallback_) {
        ROS_WARN_THROTTLE(5.0,
                          "[cloud_to_world_bridge] exact TF unavailable, fallback to latest TF: %s -> %s",
                          msg->header.frame_id.c_str(), target_frame_.c_str());
      } else {
        ROS_DEBUG_THROTTLE(5.0,
                           "[cloud_to_world_bridge] exact TF unavailable, fallback to latest TF: %s -> %s",
                           msg->header.frame_id.c_str(), target_frame_.c_str());
      }
    }

    sensor_msgs::PointCloud2 cloud_world;
    pcl_ros::transformPointCloud(target_frame_, tf_msg.transform, *msg, cloud_world);
    // Keep output stamp aligned with the transform actually used.
    if (used_latest && !tf_msg.header.stamp.isZero()) {
      cloud_world.header.stamp = tf_msg.header.stamp;
    } else if (!msg->header.stamp.isZero()) {
      cloud_world.header.stamp = msg->header.stamp;
    }
    pub_cloud_world_.publish(cloud_world);

    if (pending_cloud_ == msg) {
      pending_cloud_.reset();
    }
    return true;
  }

  void retryPendingCloud(const ros::TimerEvent&) {
    if (!pending_cloud_) {
      return;
    }

    if ((ros::WallTime::now() - pending_cloud_wall_time_).toSec() > pending_cloud_max_age_sec_) {
      ROS_DEBUG_THROTTLE(1.0,
                         "[cloud_to_world_bridge] Dropping stale cached cloud while waiting for TF.");
      pending_cloud_.reset();
      return;
    }

    transformAndPublish(pending_cloud_, true);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (msg->header.frame_id.empty()) {
      ROS_WARN_THROTTLE(2.0,
                        "[cloud_to_world_bridge] Received cloud without frame_id, skip.");
      return;
    }
    transformAndPublish(msg, false);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber sub_cloud_raw_;
  ros::Publisher pub_cloud_world_;
  ros::Timer retry_timer_;

  std::string input_cloud_topic_;
  std::string output_cloud_topic_;
  std::string target_frame_;
  double tf_lookup_timeout_sec_;
  double startup_quiet_sec_;
  double retry_period_sec_;
  double pending_cloud_max_age_sec_;
  int queue_size_;
  bool warn_on_exact_fallback_;
  ros::WallTime startup_wall_time_;
  ros::WallTime pending_cloud_wall_time_;
  sensor_msgs::PointCloud2ConstPtr pending_cloud_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_to_world_bridge");
  CloudToWorldBridge node;
  ros::spin();
  return 0;
}
