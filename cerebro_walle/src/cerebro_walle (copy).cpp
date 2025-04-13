#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


move_base_msgs::MoveBaseGoal goal;

ros::Publisher pub_goal;
ros::Publisher marker_pub;


geometry_msgs::PointStamped persones_points [6];
geometry_msgs::PointStamped perill_points [6];
geometry_msgs::PointStamped sortida_points [1];


void markers_rviz (geometry_msgs::PointStamped pointmark [], std::string color_def){//tf::StampedTransform transform){

  visualization_msgs::Marker marker;

  for (int i=1; i < 6; i++){
    if (pointmark[i].point.x != 0){
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time(0);
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = i;
  marker.type = visualization_msgs::Marker::CUBE;
//  marker.action = visualization_msgs::Marker::ADD;
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pointmark[i].point.x;
  marker.pose.position.y = pointmark[i].point.y;
  marker.pose.position.z = pointmark[i].point.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.5;
  // Set the color -- be sure to set alpha to something non-zero!
      if (color_def=="blue"){
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
      }
      if (color_def=="green"){
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
      }
      if (color_def=="red"){
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
      }
  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);
  }
}

}

void cerebro(){
  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

  if((persones_points[0].point.x != 0) and (persones_points[0].point.y != 0))
  {
    //Agafar persona
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = persones_points[0].point.x;
    goal.target_pose.pose.position.y = persones_points[0].point.y;

    ROS_INFO("Vaig a agafar persona............................\n");
    while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      {ac.sendGoal(goal);
      ROS_INFO("Sending goal");}
    ROS_INFO("Agafo persona ..................................\n");

      //Si sabem on és la sortida anem cap allà
      if ((sortida_points[0].point.x != 0) and (sortida_points[0].point.y != 0)){
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = sortida_points[0].point.x;
        goal.target_pose.pose.position.y = sortida_points[0].point.y;
        ac.sendGoal(goal);
        ROS_INFO("Vaig cap a la sortida............................\n");
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {ROS_INFO("Arribo a la sortida i deixo persona ............\n");}

      }

  }
}

void persones_subscriber_callback (geometry_msgs::PointStamped persones_sub)
{
  std::cout << "S'ha rebut Point stamp persona" << persones_sub << std::endl;

  for (int i=0; i < 6; i++){

    if (persones_points[0].point.x == 0 and persones_points[0].point.y == 0){
      persones_points[0].point.x = persones_sub.point.x;
      persones_points[0].point.y = persones_sub.point.y;
      persones_points[0].point.z = persones_sub.point.z;
      std::cout << "He afegit persona" << persones_points[0] << std::endl;

      }

  if ((persones_points[i].point.x != 0) and ((persones_sub.point.x-persones_points[i].point.x>abs(0.3)) or (persones_sub.point.y-persones_points[i].point.y > abs(0.3)))) //(punt_transformat.point.x-persones_points[i].point.x>0.3) or (punt_transformat.point.x-persones_points[i].point.x < -0.3) or (punt_transformat.point.y-persones_points[i].point.y < -0.3) or (punt_transformat.point.y-persones_points[i].point.y > 0.3)))
  {
      persones_points[i+1].point.x = persones_sub.point.x;
      persones_points[i+1].point.y = persones_sub.point.y;
      persones_points[i+1].point.z = persones_sub.point.z;
      std::cout << "He afegit persona" << persones_points[i] << std::endl;
      }
  }

  markers_rviz(persones_points,"blue");

  cerebro();

}


main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cerebro_walle");
  ros::NodeHandle nh;

  // Subscriber al cloud de la camera astra
    ros::Subscriber sub = nh.subscribe ("persones_point_output", 1, persones_subscriber_callback);
  //Publisher markers en RVIZ
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  //Publisher pub_goal
    pub_goal = nh.advertise<sensor_msgs::PointCloud2> ("output_astra_detection", 1);


    //tell the action client that we want to spin a thread by default

    //wait for the action server to come up
//    while(!ac.waitForServer(ros::Duration(5.0))){
//      ROS_INFO("Waiting for the move_base action server to come up");
//    }

    //we'll send a goal to the robot to move 1 meter forward
  //  goal.target_pose.header.frame_id = "map";
  //  goal.target_pose.header.stamp = ros::Time::now();

  //  goal.target_pose.pose.position.x = 1.0;
  //  goal.target_pose.pose.orientation.w = 1.0;




    ros::spin();

    return 0;


}
