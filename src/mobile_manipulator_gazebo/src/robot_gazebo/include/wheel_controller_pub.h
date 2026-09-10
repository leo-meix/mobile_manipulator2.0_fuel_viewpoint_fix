#ifndef WHEEL_CONTROLLER_PUB
#define WHEEL_CONTROLLER_PUB

// #include <std_msgs/Float64.h>
// #include <cmath>



const double L_x = 0.24;   // 机器人长度
const double L_y = 0.275;  // 机器人宽度
const double R = 0.0895*1.05;    // 轮子半径


void send_wheel_controller_command(ros::Publisher& pub1, ros::Publisher& pub2, ros::Publisher& pub3, ros::Publisher& pub4, const std::vector<double>& robot_vel) 
{
    double v1 = (robot_vel[0] - robot_vel[1] - (L_x + L_y) * robot_vel[2]) / R;
    double v2 = (robot_vel[0] + robot_vel[1] + (L_x + L_y) * robot_vel[2]) / R;
    double v3 = (robot_vel[0] + robot_vel[1] - (L_x + L_y) * robot_vel[2]) / R;
    double v4 = (robot_vel[0] - robot_vel[1] + (L_x + L_y) * robot_vel[2]) / R;

    std_msgs::Float64 V;
    V.data = v1;
    pub1.publish(V);
    V.data = v2;
    pub2.publish(V);
    V.data = v3;
    pub3.publish(V);
    V.data = v4;
    pub4.publish(V);

}


#endif 