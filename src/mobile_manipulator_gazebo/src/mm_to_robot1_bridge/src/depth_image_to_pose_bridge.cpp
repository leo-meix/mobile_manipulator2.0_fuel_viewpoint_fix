#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class DepthImageToPoseBridge {
 public:
  DepthImageToPoseBridge()
      : nh_(),
        pnh_("~"),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        startup_wall_time_(ros::WallTime::now()) {
    pnh_.param<std::string>("depth_topic", depth_topic_,
                            std::string("/robot1_camera/depth/image_raw"));
    pnh_.param<std::string>("pose_topic", pose_topic_,
                            std::string("/robot1_camera/depth/pose"));
    pnh_.param<std::string>("target_frame", target_frame_, std::string("world"));
    pnh_.param<std::string>("source_frame", source_frame_, std::string(""));
    pnh_.param<double>("tf_lookup_timeout", tf_lookup_timeout_sec_, 0.05);
    pnh_.param<double>("startup_quiet_sec", startup_quiet_sec_, 5.0);
    pnh_.param<bool>("fallback_to_latest_tf", fallback_to_latest_tf_, true);
    pnh_.param<int>("queue_size", queue_size_, 10);

    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
    sub_depth_ = nh_.subscribe(depth_topic_, queue_size_,
                               &DepthImageToPoseBridge::depthCallback, this);

    ROS_INFO_STREAM("[depth_image_to_pose_bridge] depth=" << depth_topic_
                                                          << ", pose=" << pose_topic_
                                                          << ", target_frame=" << target_frame_
                                                          << ", source_frame="
                                                          << (source_frame_.empty()
                                                                  ? "<depth header frame>"
                                                                  : source_frame_));
  }

 private:
  bool inStartupQuietWindow() const {
    return (ros::WallTime::now() - startup_wall_time_).toSec() < startup_quiet_sec_;
  }

  bool lookupTransform(const std::string& src_frame, const ros::Time& stamp,
                       geometry_msgs::TransformStamped& tf_msg, bool& used_latest) {
    used_latest = false;
    const ros::Duration timeout(tf_lookup_timeout_sec_);
    const ros::Time exact_stamp = stamp.isZero() ? ros::Time(0) : stamp;

    if (tf_buffer_.canTransform(target_frame_, src_frame, exact_stamp, timeout)) {
      tf_msg = tf_buffer_.lookupTransform(target_frame_, src_frame, exact_stamp, timeout);
      return true;
    }

    if (fallback_to_latest_tf_ && !exact_stamp.isZero() &&
        tf_buffer_.canTransform(target_frame_, src_frame, ros::Time(0), timeout)) {
      tf_msg = tf_buffer_.lookupTransform(target_frame_, src_frame, ros::Time(0), timeout);
      used_latest = true;
      return true;
    }

    return false;
  }

  void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    const std::string src_frame = source_frame_.empty() ? msg->header.frame_id : source_frame_;
    if (src_frame.empty()) {
      ROS_WARN_THROTTLE(2.0,
                        "[depth_image_to_pose_bridge] Depth image has no frame_id and no source_frame param.");
      return;
    }

    geometry_msgs::TransformStamped tf_msg;
    bool used_latest = false;
    if (!lookupTransform(src_frame, msg->header.stamp, tf_msg, used_latest)) {
      if (!inStartupQuietWindow()) {
        ROS_WARN_THROTTLE(1.0,
                          "[depth_image_to_pose_bridge] TF not ready: %s -> %s",
                          src_frame.c_str(), target_frame_.c_str());
      }
      return;
    }

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = target_frame_;
    pose.pose.position.x = tf_msg.transform.translation.x;
    pose.pose.position.y = tf_msg.transform.translation.y;
    pose.pose.position.z = tf_msg.transform.translation.z;
    pose.pose.orientation = tf_msg.transform.rotation;
    pub_pose_.publish(pose);

    if (used_latest) {
      ROS_DEBUG_THROTTLE(5.0,
                         "[depth_image_to_pose_bridge] Exact TF unavailable, using latest TF for %s -> %s.",
                         src_frame.c_str(), target_frame_.c_str());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber sub_depth_;
  ros::Publisher pub_pose_;
  ros::WallTime startup_wall_time_;

  std::string depth_topic_;
  std::string pose_topic_;
  std::string target_frame_;
  std::string source_frame_;
  double tf_lookup_timeout_sec_;
  double startup_quiet_sec_;
  bool fallback_to_latest_tf_;
  int queue_size_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "depth_image_to_pose_bridge");
  DepthImageToPoseBridge node;
  ros::spin();
  return 0;
}
