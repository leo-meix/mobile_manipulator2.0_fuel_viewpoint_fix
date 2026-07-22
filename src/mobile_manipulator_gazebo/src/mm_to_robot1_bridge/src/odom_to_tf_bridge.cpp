#include <cmath>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class OdomToTfBridge {
 public:
  OdomToTfBridge() : nh_(), pnh_("~") {
    pnh_.param<std::string>("odom_topic", odom_topic_,
                            std::string("/robot1/robot1_agv_base_link_position"));
    pnh_.param<std::string>("world_frame", world_frame_, std::string("world"));
    pnh_.param<std::string>("base_footprint_frame", base_footprint_frame_,
                            std::string("robot1_agv_base_footprint"));
    pnh_.param<double>("footprint_z", footprint_z_, 0.0);
    pnh_.param<bool>("use_odom_stamp", use_odom_stamp_, true);
    pnh_.param<int>("queue_size", queue_size_, 20);

    odom_sub_ = nh_.subscribe(odom_topic_, queue_size_, &OdomToTfBridge::odomCallback, this);

    ROS_INFO_STREAM("[odom_to_tf_bridge] odom_topic=" << odom_topic_
                                                      << ", world_frame=" << world_frame_
                                                      << ", base_footprint_frame="
                                                      << base_footprint_frame_
                                                      << ", footprint_z=" << footprint_z_
                                                      << ", use_odom_stamp=" << use_odom_stamp_);
  }

 private:
  static double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static bool isFinite(double v) { return std::isfinite(v); }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    if (!msg) {
      return;
    }
    const auto& p = msg->pose.pose.position;
    const auto& o = msg->pose.pose.orientation;
    if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z) || !isFinite(o.w) ||
        !isFinite(o.x) || !isFinite(o.y) || !isFinite(o.z)) {
      ROS_WARN_THROTTLE(1.0, "[odom_to_tf_bridge] Received non-finite odom pose, skip.");
      return;
    }

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp =
        (use_odom_stamp_ && !msg->header.stamp.isZero()) ? msg->header.stamp : ros::Time::now();
    tf_msg.header.frame_id = world_frame_;
    tf_msg.child_frame_id = base_footprint_frame_;
    tf_msg.transform.translation.x = p.x;
    tf_msg.transform.translation.y = p.y;
    tf_msg.transform.translation.z = footprint_z_;

    tf2::Quaternion q_yaw;
    q_yaw.setRPY(0.0, 0.0, yawFromQuaternion(o));
    q_yaw.normalize();
    tf_msg.transform.rotation.x = q_yaw.x();
    tf_msg.transform.rotation.y = q_yaw.y();
    tf_msg.transform.rotation.z = q_yaw.z();
    tf_msg.transform.rotation.w = q_yaw.w();

    tf_broadcaster_.sendTransform(tf_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string odom_topic_;
  std::string world_frame_;
  std::string base_footprint_frame_;

  double footprint_z_;
  bool use_odom_stamp_;
  int queue_size_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_to_tf_bridge");
  OdomToTfBridge node;
  ros::spin();
  return 0;
}
