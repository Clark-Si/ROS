#include <ros/ros.h>

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "my_param_node");

    // 创建节点句柄
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string param_name = "my_param";
    std::string param_value = "hello";

    std::string my_param_first;
    //测试私有参数
private_nh.getParam(param_name, my_param_first);
    ROS_INFO("Private 1 The value of %s is %s", param_name.c_str(), my_param_first.c_str());

// 先通过私有句柄设置私有参数（放在节点命名空间下）    
private_nh.setParam(param_name, param_value); 
// 再通过私有句柄获取私有参数     
private_nh.getParam(param_name, my_param_first);  // 获取私有参数
ROS_INFO("Private 2 The value of %s is %s", param_name.c_str(), my_param_first.c_str());


    // 设置一个参数
    nh.setParam(param_name, param_value);

    // 获取一个参数
    std::string my_param;
    nh.getParam(param_name, my_param);
    ROS_INFO("The value of %s is %s", param_name.c_str(), my_param.c_str());

    ros::spin();

    return 0;
}
