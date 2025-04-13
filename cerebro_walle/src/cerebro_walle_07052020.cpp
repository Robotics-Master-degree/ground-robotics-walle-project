#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


ros::Publisher pub_goal;
ros::Publisher marker_pub;

bool personaagafada=false;

geometry_msgs::PointStamped persones_points [6];
geometry_msgs::PointStamped perill_points [6];
geometry_msgs::PointStamped sortida_points [1];


move_base_msgs::MoveBaseGoal goal;

void cerebro(){

  if (personaagafada == false){

      //Si no tenim persona agafada i hem vist persones anar a buscar
      if(persones_points[0].point.x != 0 and persones_points[0].point.y != 0){
        //Publish exploration false

        //Càlcul persona que està més aprop

        //Enviar goal a buscar persona

        MoveBaseClient ac("move_base", true);
        while(!ac.waitForServer(ros::Duration(5.0)));
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = persones_points[0].point.x - 0.2;
        goal.target_pose.pose.position.y = persones_points[0].point.y - 0.2;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Anem a buscar persona  .................................... ");
        ac.sendGoal(goal);

        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          ROS_INFO("Agafo persona  ...........................................");
          personaagafada = true;}
        else{ ROS_INFO("Hi ha algo que ha anat malament ........  :(  ");}



      }
      else{
        //Publish exploration true

      }

  }
  else{
      //Si tenim persona agafada i coneixem sortida
      if(sortida_points[0].point.x != 0 and sortida_points[0].point.y != 0){
          //Publish exploration false

          //Enviar goal sortida
      }
      else{
          //Publish exploration true

      }
  }

}

void persones_subscriber_callback (geometry_msgs::PointStamped persones_sub)
{
    persones_points[0].point.x = persones_sub.point.x;
    persones_points[0].point.y = persones_sub.point.y;
    persones_points[0].point.z = 0;
    persones_points[0].header.frame_id = "map";

    cerebro();

}


main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "cerebro_walle");
    ros::NodeHandle nh;

    // Subscriber al cloud de la camera astra
    ros::Subscriber sub = nh.subscribe ("persones_point_output", 1, persones_subscriber_callback);



    ros::spin();

    return 0;


}
