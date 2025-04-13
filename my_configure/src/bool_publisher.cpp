#include <std_msgs/Bool.h>
#include <ros/ros.h>
using namespace std;

int main(int argc, char **argv) {

  ros::init(argc, argv, "bool_publisher");
  ros::NodeHandle nh;

  bool b;
  string a;
  cout<<"Activar exit_layer? yes(y) or no(n): "<<endl;
  cin>>a;
  if (a == "y") {
    b = true;
  }
  if (a == "n") {
    b = false;
  }


  nh.setParam("/move_base/global_costmap/exit/enabled", b);
  /*nh.setParam("/move_base/local_costmap/exit/enabled", b);*/





  /*ros::Publisher pub = nh.advertise<std_msgs::Bool>("power", 1000);
  ros::Rate loop_rate(2);
  std_msgs::Bool state;
  int value;
  cout<<"choose 0 or 1:"<<endl;
  cin>>value;
  //state.data = value;
  cout<<"your option was: "<<value<<endl;

  if (value == 1) {
    state.data = true;
  } else if (value == 0) {
    state.data == false;
  }




  while (ros::ok()) {
    pub.publish(state);
  //ros::spinOnce();
    loop_rate.sleep();
  }*/





  return 0;
}
