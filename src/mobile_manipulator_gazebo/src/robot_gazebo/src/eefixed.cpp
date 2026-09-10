#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <vector>

#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>

#include <tf2_msgs/TFMessage.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <control_msgs/JointControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


#include <arm_controller_pub.h>
#include <gripper_controller_pub.h>
#include <wheel_controller_pub.h>
#include <KDL_IK.h>



const int robot_num = 1; // 机器人的数量

const double RTF = 0.07; // real time factor
const double PI = 3.14159265358979323846;
const double pi = 3.14159265358979323846;

double robot_pos[robot_num][3]; // for agvs' positions

tf::Transform world_agv;
tf::Transform agv_armbase;
tf::Transform world_armbase;
tf::Transform world_ee;
tf::Transform world_ee_fixed;
tf::Transform wrist3_ee;
tf::Transform world_wrist3;

tf::Transform armbase_ee;



std::vector<double> robot1_joint_angles = {-0.0, 1.0, -1.0, 0.0, -0.0, -0.0};




// 获取机械臂基座在世界坐标系下的位姿
void pos_world_armbase(const gazebo_msgs::ModelStates &msg)
{
    tf::Quaternion quat;
    std::vector<std::string> model_names = msg.name;
    std::string robot_name = "robot1";

      for(size_t j = 0; j < model_names.size(); j++)
      {
        if(model_names[j] == robot_name)
          {
            tf::quaternionMsgToTF(msg.pose[j].orientation, quat);
            world_agv.setRotation(quat);
            world_agv.setOrigin(tf::Vector3(msg.pose[j].position.x, msg.pose[j].position.y, msg.pose[j].position.z));
            break; 
          }
      }

    tf::Quaternion agv_armbase_quat(0, 0, 0, 1);
    agv_armbase.setOrigin(tf::Vector3(0.1395, 0, 0.1965));
    agv_armbase.setRotation(agv_armbase_quat);

    world_armbase = world_agv * agv_armbase;
}


// 获取机械臂末端在世界坐标系下的位姿
void pos_world_ee(const nav_msgs::Odometry::ConstPtr& msg)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation,quat);//将ROS消息中的四元数转换为tf库中的四元数
  world_wrist3.setRotation(quat);
  world_wrist3.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

  quat.setX(-0.70711);
  quat.setY(0.0);
  quat.setZ(0.0);
  quat.setW(0.70711);
  wrist3_ee.setRotation(quat);
  wrist3_ee.setOrigin(tf::Vector3(0, 0.0823, 0));

  world_ee = world_wrist3 * wrist3_ee;
}



void print_robot_position()
{
  for (int i = 0; i < robot_num; i++) 
    {
    std::cout << "Robot " << i+1 << " position: ";
    for (int j = 0; j < 3; j++) {
        std::cout << robot_pos[i][j] << " ";
    }
    std::cout << std::endl;
    }
}

void print_sim_time()
{
    ros::Time sim_time = ros::Time::now(); // 获取当前仿真时间
    ROS_INFO("Current simulation time: %f", sim_time.toSec()); // 输出仿真时间到控制台
}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "cmdveltest");
  ros::NodeHandle nh;

  ros::Subscriber sub1 = nh.subscribe("/gazebo/model_states", 100, pos_world_armbase);
  ros::Subscriber sub2 = nh.subscribe<nav_msgs::Odometry>("robot1/robot1_wrist_3_link_position", 100, pos_world_ee);

  //arm_controller
  ros::Publisher pub_robot1_arm = nh.advertise<trajectory_msgs::JointTrajectory>("/robot1/joint_controller/command", 100);

  //gripper_controller
  ros::Publisher pub_robot1_gripper = nh.advertise<trajectory_msgs::JointTrajectory>("/robot1/gripper_controller/command", 100);

  //wheel_controller
  ros::Publisher pub_robot1_v1= nh.advertise<std_msgs::Float64>("/robot1/front_left_controller/command", 30);
  ros::Publisher pub_robot1_v2= nh.advertise<std_msgs::Float64>("/robot1/front_right_controller/command", 30);
  ros::Publisher pub_robot1_v3= nh.advertise<std_msgs::Float64>("/robot1/rear_left_controller/command", 30);
  ros::Publisher pub_robot1_v4= nh.advertise<std_msgs::Float64>("/robot1/rear_right_controller/command", 30);

  ros::AsyncSpinner spinner(50);
  spinner.start();
  ros::Duration(2*RTF).sleep(); //Duration is the simulation time, this is 2s in real world

  send_joint_controller_command(pub_robot1_arm, {0,1,-1,0,0,0}, "robot1", 0.1);
  // send_joint_controller_command(pub_robot1_arm, {0.5, -0.5, 0.3, 0.0, 0.8, 0.0}, "robot1", 2.0);

  ros::Duration(1).sleep();

  world_ee_fixed = world_ee;
  std::vector<double> desired_Rotation(4);
  std::vector<double> desired_XYZ(3);
  std::vector<double> result = robot1_joint_angles;
  std::vector<double> result_next = robot1_joint_angles;
  tf::Quaternion rotation;
  tf::Vector3 translation;

  ros::Rate rate(100);

  while(ros::ok())//反馈控制系统
  { 
    armbase_ee = world_armbase.inverse() * world_ee_fixed;

    rotation = armbase_ee.getRotation();
    translation = armbase_ee.getOrigin();

    desired_Rotation[0] = rotation.getX();
    desired_Rotation[1] = rotation.getY();
    desired_Rotation[2] = rotation.getZ();
    desired_Rotation[3] = rotation.getW();

    desired_XYZ[0] = translation.getX();
    desired_XYZ[1] = translation.getY();
    desired_XYZ[2] = translation.getZ();



    result = arm_KDL_IK(result, desired_Rotation, desired_XYZ, "ur5");
    
    send_joint_controller_command(pub_robot1_arm, result, "robot1", 0.1);

    rate.sleep();
  }



  return 0;
}