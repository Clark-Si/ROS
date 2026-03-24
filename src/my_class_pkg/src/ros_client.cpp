#include "ros/ros.h" 
#include "my_class_pkg/MyServiceMsgRequest.h" 
#include "my_class_pkg/MyServiceMsgResponse.h" 
#include "my_class_pkg/MyServiceMsg.h"
int main(int argc, char **argv) {
ros::init(argc, argv, "my_service_client");
ros::NodeHandle nh;
// 创建一个 ROS 客户端，请求 my_service 服务
ros::ServiceClient client =
nh.serviceClient<my_class_pkg::MyServiceMsg>("my_service");
// 创建一个请求消息
my_class_pkg::MyServiceMsg srv;
srv.request.input = 42;
if (client.call(srv)) {
ROS_INFO("Service response: %ld", (long int) srv.response.output); // 输出响应结果
} else {
ROS_ERROR("Failed to call service my_service");
return 1;
}
return 0;
}
