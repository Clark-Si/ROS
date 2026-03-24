#include <ros/ros.h>

//包含必要的头文件，其中TutorialsConfig.h就是配置文件在编译过程中生成的头文件。
#include <dynamic_reconfigure/server.h>
#include <my_class_pkg/TutorialsConfig.h>

//定义回调函数，并将回调函数和服务端绑定。当客户端请求修改参数时，服务端即可跳转到回调函数进行处理
//level的作用：在复杂配置中，可以将参数分组（通过level位标记）。当一组参数同时改变时，回调中的level会包含这些组的位信息，从而只更新相关的组件，避免不必要的计算。
void callback(my_class_pkg::TutorialsConfig &config, uint32_t level)
{
   ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
           config.int_param, config.double_param, 
           config.str_param.c_str(), 
           config.bool_param?"True":"False", 
           config.size);
}

int main(int argc, char **argv) 
{
   ros::init(argc, argv, "dynamic_tutorials");
   //创建了一个参数动态配置的服务端实例
   //参数配置的类型与配置文件中描述的类型相同
   dynamic_reconfigure::Server<my_class_pkg::TutorialsConfig> server;
   dynamic_reconfigure::Server<my_class_pkg::TutorialsConfig>::CallbackType f;
   
   //绑定回调
   f = boost::bind(&callback, _1, _2);
   server.setCallback(f);//将绑定的回调函数注册到服务器。此后，每当有重新配置请求时，服务器就会调用callback。

   ROS_INFO("Spinning node");
   ros::spin();
   return 0;
}
