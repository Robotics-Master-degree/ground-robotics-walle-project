#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <my_dyn_rec/MyParamsConfig.h> //this file will be generated by ros

void callback(my_dyn_rec::MyParamsConfig &config,uint32_t level){
  ROS_INFO("New values: [%d] - [%s]",config.int_param,config.str_param.c_str());
}

int main(int argc,char **argv){
  ros::init(argc,argv,"my_node");
  dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig> Server;
  dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig>::CallbackType f;

  f = boost:: bind(&callback, _1, _2);
  Server.setCallback(f);
  ros::spin();
  return 0;
}
