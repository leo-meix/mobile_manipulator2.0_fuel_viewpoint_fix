#ifndef GRIPPER_CONTROLLER_PUB
#define GRIPPER_CONTROLLER_PUB


// #include <trajectory_msgs/JointTrajectory.h>
// #include <trajectory_msgs/JointTrajectoryPoint.h>



void grippers_on(ros::Publisher& pub, const std::string& robot_name, const std::string& gripper_type, double time)
{

    trajectory_msgs::JointTrajectory joint_trajectory;
    trajectory_msgs::JointTrajectoryPoint point;

    if (gripper_type == "robotiq")
    {
        joint_trajectory.joint_names = {robot_name + "_gripper_finger1_joint"};
        point.positions.resize(1);
        point.positions[0] = 0.8;
    }

    if (gripper_type == "barrett")
    {
        joint_trajectory.joint_names = {robot_name + "_j11_joint",
                                        robot_name + "_j12_joint",
                                        robot_name + "_j13_joint",
                                        robot_name + "_j21_joint",
                                        robot_name + "_j22_joint",
                                        robot_name + "_j23_joint",
                                        robot_name + "_j32_joint",
                                        robot_name + "_j33_joint"};
        point.positions.resize(8);
        point.positions = {0,1.5,0,0,1.5,0,1.5,0};
    }

    if (gripper_type == "panda")
    {
        joint_trajectory.joint_names = {robot_name + "_finger_joint1",
                                        robot_name + "_finger_joint2"};
        point.positions.resize(2);
        point.positions = {0.04,0.04};
    }

    if (gripper_type == "kinova")
    {
        joint_trajectory.joint_names = {robot_name + "_joint_finger_1",
                                        robot_name + "_joint_finger_tip_1",
                                        robot_name + "_joint_finger_2",
                                        robot_name + "_joint_finger_tip_2"};
        point.positions.resize(4);
        point.positions = {0.9,0.8,0.9,0.8};
    }

    point.time_from_start = ros::Duration(time);
    joint_trajectory.points = {point};

    pub.publish(joint_trajectory);

}


void grippers_off(ros::Publisher& pub, const std::string& robot_name, const std::string& gripper_type, double time)
{

    trajectory_msgs::JointTrajectory joint_trajectory;
    trajectory_msgs::JointTrajectoryPoint point;

    if (gripper_type == "robotiq")
    {
        joint_trajectory.joint_names = {robot_name + "_gripper_finger1_joint"};
        point.positions.resize(1);
        point.positions[0] = 0;
    }

    if (gripper_type == "barrett")
    {
        joint_trajectory.joint_names = {robot_name + "_j11_joint",
                                        robot_name + "_j12_joint",
                                        robot_name + "_j13_joint",
                                        robot_name + "_j21_joint",
                                        robot_name + "_j22_joint",
                                        robot_name + "_j23_joint",
                                        robot_name + "_j32_joint",
                                        robot_name + "_j33_joint"};
        point.positions.resize(8);
        point.positions = {0,0,0,0,0,0,0,0};
    }

    if (gripper_type == "panda")
    {
        joint_trajectory.joint_names = {robot_name + "_finger_joint1",
                                        robot_name + "_finger_joint2"};
        point.positions.resize(2);
        point.positions = {0.0,0.0};
    }

    if (gripper_type == "kinova")
    {
        joint_trajectory.joint_names = {robot_name + "_joint_finger_1",
                                        robot_name + "_joint_finger_tip_1",
                                        robot_name + "_joint_finger_2",
                                        robot_name + "_joint_finger_tip_2"};
        point.positions.resize(4);
        point.positions = {0.0,0.0,0.0,0.0};
    }

    point.time_from_start = ros::Duration(time);
    joint_trajectory.points = {point};

    pub.publish(joint_trajectory);

}

#endif 