#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <my_configure/my_configureConfig.h>



void callback(my_configure::my_configureConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %f",
             config.planner_frequency);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_reconfigure_node");
  dynamic_reconfigure::Server<my_configure::my_configureConfig> srv;
  dynamic_reconfigure::Server<my_configure::my_configureConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);
  ROS_INFO("Starting to spin...");
  ros::spin();
  return 0;
}
