#include <ros/ros.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

int kfd = 0;
struct termios cooked, raw;

namespace
{
double odom_x = 0.0;
double odom_y = 0.0;
double odom_yaw = 0.0;
bool have_odom = false;

double target_x = 0.0;
double target_y = 0.0;
double target_yaw = 0.0;
bool target_initialized = false;

double normalizeAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_x = msg->pose.pose.position.x;
  odom_y = msg->pose.pose.position.y;
  odom_yaw = tf::getYaw(msg->pose.pose.orientation);
  have_odom = true;

  if (!target_initialized) {
    target_x = odom_x;
    target_y = odom_y;
    target_yaw = odom_yaw;
    target_initialized = true;
  }
}
}

void init_keyboard()
{
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VMIN] = 1;
  raw.c_cc[VTIME] = 0;
  tcsetattr(kfd, TCSANOW, &raw);
}

void restore_keyboard()
{
  tcsetattr(kfd, TCSANOW, &cooked);
  tcflush(kfd, TCIFLUSH);
}

char getch()
{
  char c;
  fd_set fds;
  struct timeval tv;

  FD_ZERO(&fds);
  FD_SET(kfd, &fds);

  tv.tv_sec = 0;
  tv.tv_usec = 0;

  int flags = fcntl(kfd, F_GETFL, 0);
  fcntl(kfd, F_SETFL, flags | O_NONBLOCK);

  if (select(kfd + 1, &fds, NULL, NULL, &tv) == 1)
  {
    if (read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
  }
  else
  {
    c = '\0';
  }

  fcntl(kfd, F_SETFL, flags);
  return c;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_control");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string cmd_topic;
  std::string odom_topic;
  double linear_speed;
  double yaw_speed;
  double publish_rate;

  pnh.param("cmd_topic", cmd_topic, std::string("/mm_controller_node/car_cmd"));
  pnh.param("odom_topic", odom_topic, std::string("/robot1/robot1_agv_base_link_position"));
  pnh.param("linear_speed", linear_speed, 0.25);
  pnh.param("yaw_speed", yaw_speed, 0.45);
  pnh.param("publish_rate", publish_rate, 30.0);

  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>(cmd_topic, 1);
  ros::Subscriber odom_sub = nh.subscribe(odom_topic, 1, odomCallback);

  init_keyboard();

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Manual mapping teleop publishes REMANI car_cmd to %s", cmd_topic.c_str());
  ROS_INFO("Keys: w/s forward/back, a/d strafe left/right, j/k yaw, space stop target drift, q quit.");

  ros::Rate rate(std::max(1.0, publish_rate));
  ros::Time last_time = ros::Time::now();

  while (ros::ok())
  {
    const ros::Time now = ros::Time::now();
    double dt = (now - last_time).toSec();
    last_time = now;
    if (dt <= 0.0 || dt > 0.2) dt = 1.0 / std::max(1.0, publish_rate);

    char c = getch();

    double vx_body = 0.0;
    double vy_body = 0.0;
    double wz = 0.0;

    switch (c)
    {
      case 'w':
        vx_body = linear_speed;
        break;
      case 's':
        vx_body = -linear_speed;
        break;
      case 'a':
        vy_body = linear_speed;
        break;
      case 'd':
        vy_body = -linear_speed;
        break;
      case 'j':
        wz = yaw_speed;
        break;
      case 'k':
        wz = -yaw_speed;
        break;
      case ' ':
        if (have_odom) {
          target_x = odom_x;
          target_y = odom_y;
          target_yaw = odom_yaw;
          target_initialized = true;
        }
        break;
      case 'q':
        restore_keyboard();
        return 0;
      default:
        break;
    }

    if (!target_initialized && have_odom) {
      target_x = odom_x;
      target_y = odom_y;
      target_yaw = odom_yaw;
      target_initialized = true;
    }

    if (target_initialized) {
      const double yaw = target_yaw;
      const double vx_world = std::cos(yaw) * vx_body - std::sin(yaw) * vy_body;
      const double vy_world = std::sin(yaw) * vx_body + std::cos(yaw) * vy_body;

      target_x += vx_world * dt;
      target_y += vy_world * dt;
      target_yaw = normalizeAngle(target_yaw + wz * dt);

      geometry_msgs::Twist cmd;
      cmd.linear.x = target_x;
      cmd.linear.y = target_y;
      cmd.linear.z = target_yaw;
      cmd.angular.x = vx_world;
      cmd.angular.y = vy_world;
      cmd.angular.z = wz;
      cmd_pub.publish(cmd);
    } else {
      ROS_WARN_THROTTLE(1.0, "Waiting for odom on %s before publishing teleop target.", odom_topic.c_str());
    }

    rate.sleep();
  }

  restore_keyboard();
  return 0;
}