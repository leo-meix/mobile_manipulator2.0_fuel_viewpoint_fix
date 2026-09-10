#ifndef UR5_KDL_IK_SOLVER
#define UR5_KDL_IK_SOLVER


#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <urdf_parser/urdf_parser.h>





std::vector<double> arm_KDL_IK(const std::vector<double>& initial_joints, const std::vector<double>& desired_RPY, const std::vector<double>& desired_XYZ, const std::string& robot_name)
{
  std::string urdf_file = ros::package::getPath("ur_robot") + "/urdf/ur5.urdf"; // Get URDF file path from parameter server
  if(robot_name == "ur5")
  {
    urdf_file = ros::package::getPath("ur_robot") + "/urdf/ur5.urdf"; // Get URDF file path from parameter server
  }

  // Load URDF model from file
  std::string urdf_xml;
  std::ifstream fin(urdf_file.c_str());
  if (fin.fail())
  {
    ROS_ERROR("Failed to open URDF file!");
  }
  else
  {
    std::stringstream buffer;
    buffer << fin.rdbuf();
    urdf_xml = buffer.str();
  }

  // Parse URDF model
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_xml);
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
  {
    ROS_ERROR("Failed to parse URDF model!");
  }

  //create KDL chain
  std::string root_link = "base_link";
  std::string tip_link = "tool0";

  KDL::Chain kdl_chain;
  if (!kdl_tree.getChain(root_link, tip_link, kdl_chain))
  {
    ROS_ERROR_STREAM("Failed to get KDL chain from URDF model between "<< root_link << " and " << tip_link);
  }

  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);

  KDL::ChainIkSolverVel_pinv vel_solver(kdl_chain);

  //joint limited
  KDL::JntArray q_min(initial_joints.size());
  KDL::JntArray q_max(initial_joints.size());

  for(int i=0; i<initial_joints.size(); i++) {
    q_min(i) = -M_PI; // lower limit
    q_max(i) = M_PI;  // upper limit
  }
  q_max(1) = M_PI/2;
  q_min(1) = -M_PI/2;

  q_max(5) = 2*M_PI;
  q_min(5) = -2*M_PI;
  
  KDL::ChainIkSolverPos_NR_JL ik_solver(kdl_chain, q_min, q_max, fk_solver, vel_solver, 100, 1e-4);



  KDL::JntArray q_initial(initial_joints.size());
  KDL::JntArray q_result(initial_joints.size());

  for(int i = 0; i < initial_joints.size(); i++)
  {
    q_initial(i) = initial_joints[i];
  }

  KDL::Frame desired_pose(KDL::Rotation::Quaternion(desired_RPY[0], desired_RPY[1], desired_RPY[2], desired_RPY[3]), KDL::Vector(desired_XYZ[0], desired_XYZ[1], desired_XYZ[2]));

  ik_solver.CartToJnt(q_initial, desired_pose, q_result);

  std::vector<double> goal_joints(initial_joints.size());

  for(int i = 0; i < initial_joints.size(); i++)
  {
    goal_joints[i] = q_result(i);
  }


  return goal_joints;
}






KDL::Frame arm_KDL_FK(const std::vector<double>& q_joints, const std::string& robot_name)
{
  std::string urdf_file = ros::package::getPath("ur_robot") + "/urdf/ur5.urdf"; // Get URDF file path from parameter server
  if(robot_name == "ur5")
  {
    urdf_file = ros::package::getPath("ur_robot") + "/urdf/ur5.urdf"; // Get URDF file path from parameter server
  }

  // Load URDF model from file
  std::string urdf_xml;
  std::ifstream fin(urdf_file.c_str());
  if (fin.fail())
  {
    ROS_ERROR("Failed to open URDF file!");
  }
  else
  {
    std::stringstream buffer;
    buffer << fin.rdbuf();
    urdf_xml = buffer.str();
  }

  // Parse URDF model
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_xml);
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
  {
    ROS_ERROR("Failed to parse URDF model!");
  }

  //create KDL chain
  std::string root_link = "base_link";
  std::string tip_link = "tool0";

  KDL::Chain kdl_chain;
  if (!kdl_tree.getChain(root_link, tip_link, kdl_chain))
  {
    ROS_ERROR_STREAM("Failed to get KDL chain from URDF model between "<< root_link << " and " << tip_link);
  }

  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);


  KDL::JntArray q(kdl_chain.getNrOfJoints());
  q.data << q_joints[0], q_joints[1], q_joints[2], q_joints[3], q_joints[4], q_joints[5];


  KDL::Frame pos_goal;
  fk_solver.JntToCart(q, pos_goal);

  std::cout << "Position: " << pos_goal.p.x() << " " << pos_goal.p.y() << " " << pos_goal.p.z() << std::endl;

//   // 打印旋转信息（欧拉角表示）
//   double roll, pitch, yaw;
//   pos_goal.M.GetRPY(roll, pitch, yaw);
//   std::cout << "Rotation (RPY): " << roll << ", " << pitch << ", " << yaw << std::endl;
// 
//   // 打印旋转信息（轴角表示）
//   KDL::Vector axis;
//   double angle;
//   pos_goal.M.GetRotAngle(axis, angle);
//   std::cout << "Rotation (Axis-Angle): " << axis.x() << ", " << axis.y() << ", " << axis.z() << ", " << angle << std::endl;
// 
  // 打印旋转信息（四元数表示）
  double qx, qy, qz, qw;
  pos_goal.M.GetQuaternion(qx, qy, qz, qw);
  std::cout << "Rotation (Quaternion): " << qx << ", " << qy << ", " << qz << ", " << qw << std::endl;

  // 打印旋转信息（旋转矩阵表示）
//   double qx, qy, qz, qw;
//   pos_goal.M.GetQuaternion(qx, qy, qz, qw);
//   tf::Quaternion quaternion(qx, qy, qz, qw);
//   tf::Matrix3x3 rotation_matrix(quaternion);
//   ROS_INFO("Rotation Matrix:");
//   for (int i = 0; i < 3; i++) 
//   {
//     tf::Vector3 row = rotation_matrix.getRow(i);
//     ROS_INFO("%f %f %f", row.x(), row.y(), row.z());
//   }
// 
//   std::cout<<rotation_matrix[0][0]<<" "<<rotation_matrix[0][1]<<" "<<rotation_matrix[0][2]<<" "<<pos_goal.p.x()<<" "<<rotation_matrix[1][0]<<" "<<rotation_matrix[1][1]<<" "<<rotation_matrix[1][2]<<" "<< pos_goal.p.y()<<" "<<rotation_matrix[2][0]<<" "<<rotation_matrix[2][1]<<" "<<rotation_matrix[2][2]<<" "<< pos_goal.p.z() << std::endl;
// 
//   return pos_goal;
}



#endif 