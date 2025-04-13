#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction>

using namespace std;

void myCallback(const geometry_msgs::Twist::ConstPtr &msg) {

}


int main1(int argc, char **argv) {

  ros::init(argc, argv, "topic_subscriber");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("status", 1000, myCallback);

  ros::spin();

  return 0;
}
  return 0;
}

int main2(int argc, char **argv) {

  ros::init(argc, argv, "topic_publisher");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1000);
  ros::Rate loop_rate(2);

  move_base_msgs::MoveBaseActionGoal goal;
  goal.position
  goal.orientation



  while (ros::ok()) {
    pub.publish(goal);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
