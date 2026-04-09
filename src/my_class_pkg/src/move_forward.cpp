#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// 封装一个函数：控制机器人以指定速度运动指定时间
void moveRobot(ros::Publisher& pub, ros::Rate& loop_rate, 
               double linear_x, double angular_z, double duration)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = linear_x;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = angular_z;

    // 计算需要发布消息的次数 = 频率 * 持续时间
    int publish_count = loop_rate.expectedCycleTime().toSec() > 0 ? 
                        (int)(duration / loop_rate.expectedCycleTime().toSec()) : 0;
    int count = 0;

    while (ros::ok() && count < publish_count)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    // 每次动作后停止机器人（防止惯性）
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    pub.publish(vel_msg);
    ros::Duration(0.5).sleep(); // 停顿0.5秒，确保停止指令生效
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_square_node");
    ros::NodeHandle nh;

    // 发布/cmd_vel话题，队列大小10
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate loop_rate(10); // 10Hz发布频率

    // 等待publisher完成注册（避免第一条指令丢失）
    ros::Duration(1.0).sleep();

    // 正方形参数配置（可根据实际需求调整）
    double linear_speed = 0.2;  // 线速度：0.2米/秒
    double angular_speed = 0.5; // 角速度：0.5弧度/秒（约28.6°/秒）
    double side_length = 1.0;   // 正方形边长：1米
    double turn_angle = M_PI_2; // 左转角度：90°（π/2弧度）

    // 计算前进1米需要的时间 = 距离 / 线速度
    double forward_duration = side_length / linear_speed;
    // 计算左转90°需要的时间 = 角度 / 角速度
    double turn_duration = turn_angle / angular_speed;

    ROS_INFO("let's begin...");

    // 循环4次：前进→左转（完成正方形）
    for (int i = 0; i < 4; i++)
    {
        ROS_INFO("the %d time：move forward 1 meter", i+1);
        moveRobot(pub, loop_rate, linear_speed, 0, forward_duration); // 前进1米

        ROS_INFO("the %d time,turn left 90°", i+1);
        moveRobot(pub, loop_rate, 0, angular_speed, turn_duration+0.65);   // 左转90°
    }

    ROS_INFO("正方形运动执行完成，机器人停止");

    // 最终停止机器人
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0;
    stop_msg.angular.z = 0;
    pub.publish(stop_msg);

    return 0;
}
