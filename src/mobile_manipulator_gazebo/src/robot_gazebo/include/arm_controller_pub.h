#ifndef ARM_CONTROLLER_PUB
#define ARM_CONTROLLER_PUB


// #include <trajectory_msgs/JointTrajectory.h>
// #include <trajectory_msgs/JointTrajectoryPoint.h>


void send_joint_controller_command(ros::Publisher& pub, const std::vector<double>& joint_angles, const std::string& robot_name, double time)
{

    trajectory_msgs::JointTrajectory joint_trajectory;
    trajectory_msgs::JointTrajectoryPoint point;
    std::vector<double> joint_velocities(joint_angles.size());
    std::vector<double> joint_accelerations(joint_angles.size());

    if (joint_angles.size() == 6)
    {
        joint_velocities = {0.0,0.0,0.0,0.0,0.0,0.0};
        joint_accelerations = {10,10,10,10,10,10};
        joint_trajectory.joint_names = {robot_name + "_shoulder_pan_joint",
                                        robot_name + "_shoulder_lift_joint",
                                        robot_name + "_elbow_joint",
                                        robot_name + "_wrist_1_joint",
                                        robot_name + "_wrist_2_joint",
                                        robot_name + "_wrist_3_joint"};
    }
    else if (joint_angles.size() == 7)
    {
        joint_velocities = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        joint_accelerations = {10,10,10,10,10,10,10};
        joint_trajectory.joint_names = {robot_name + "_joint1",
                                        robot_name + "_joint2",
                                        robot_name + "_joint3",
                                        robot_name + "_joint4",
                                        robot_name + "_joint5",
                                        robot_name + "_joint6",
                                        robot_name + "_joint7"};
    }
    point.positions = joint_angles;
    point.time_from_start = ros::Duration(time);
    joint_trajectory.points = {point};

    pub.publish(joint_trajectory);

}




#endif 