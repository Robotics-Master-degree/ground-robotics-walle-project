#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/common/common.h"
#include "pcl_ros/point_cloud.h"
#include <pcl/filters/conditional_removal.h>
#include <math.h>
#include <sstream>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>


typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;


ros::Publisher pub;
ros::Publisher pub_cloud_transformed;
PointCloudC::Ptr cloud(new PointCloudC());
tf::StampedTransform transform;
ros::Publisher pub_persones;
ros::Publisher pub_perill;
ros::Publisher pub_sortida;

int count = 0;

geometry_msgs::PointStamped persones_points [6];
geometry_msgs::PoseStamped persones_points_pose;
geometry_msgs::PointStamped perill_points [6];
geometry_msgs::PoseStamped perill_points_pose;
geometry_msgs::PointStamped sortida_points [1];
geometry_msgs::PoseStamped sortida_points_pose;


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr remove_outliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
	if (cloud->points.size() == 0)
		return cloud;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	// instance of the RadiusOutlierRemoval filter from pcl
    	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
	// set's the input cloud
    	outrem.setInputCloud (cloud);
	// filter it to the thresholded radius
    	outrem.setRadiusSearch(0.06);
	outrem.setMinNeighborsInRadius (15);

	// finally execute the filtering
    	outrem.filter (*cloud_filtered);

	// return the filtered cloud
	return cloud_filtered;

  }


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr check_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , int rMax , int gMax , int bMax, int rMin, int gMin, int bMin)
  {
  // Filter by red color
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_color (new pcl::PointCloud<pcl::PointXYZRGB>);
  // build the condition
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LE, rMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GE, rMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LE, gMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GE, gMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LE, bMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GE, bMin)));

   // build the filter
   pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
   condrem.setCondition(color_cond);
   condrem.setInputCloud (cloud);
   // apply filter
   condrem.filter (*cloud_filtered_color);


   return cloud_filtered_color;

  }



void function_cloud(PointCloudC::Ptr& cropped_cloud, std::string color_def)
{

    //conditional removal color blau
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);
  //   pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter_color;

  //   pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr color_condition_b(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GT, 90));
  //   pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr color_condition_g(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT, 70));
  //   pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr color_condition_r(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, 60));

     //aplicar filtre
  //   pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond_color (new pcl::ConditionAnd<pcl::PointXYZRGB> ());

      if (color_def == "blue"){
        cloud_color = check_color(cropped_cloud,0,0,255,0,0,60);}
  //     color_cond_color->addComparison (color_condition_b);}
      if (color_def == "red"){
        cloud_color = check_color(cropped_cloud,130,60,70,45,0,0);}

  //     color_cond_color->addComparison (color_condition_r);}
      if (color_def == "green"){
        cloud_color = check_color(cropped_cloud,50,150,50,0,40,0);}

  //     color_cond_color->addComparison (color_condition_g);}

  // color_filter_color.setInputCloud(cropped_cloud);
  //   color_filter_color.setCondition (color_cond_color);
  //   color_filter_color.filter(*cloud_color);

    // cloud_color = remove_outliers(cloud_color);
     // Create the filtering object
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_outliers (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> sor;
    // sor.setInputCloud(cloud_color);
    // sor.setRadiusSearch(0.5);
    // sor.setMinNeighborsInRadius(100);
    // sor.filter (*cloud_filtered_outliers);

     sensor_msgs::PointCloud2 msg_out;
     pcl::toROSMsg(*cloud_color, msg_out);
     pub.publish(msg_out);


     //Càlcul punt mig del pointcloud filtrat per colors.
     PointC min_pclb;
     PointC max_pclb;
     pcl::getMinMax3D<PointC>(*cloud_color, min_pclb, max_pclb);
     geometry_msgs::PointStamped centroidPointb;

     centroidPointb.point.x = (min_pclb.x + max_pclb.x)/2;
     centroidPointb.point.y = (min_pclb.y + max_pclb.y)/2;
     centroidPointb.point.z = (min_pclb.z + max_pclb.z)/2;
     centroidPointb.header.frame_id = "camera_depth_optical_frame";
     centroidPointb.header.stamp = ros::Time(0);

     tf::TransformListener listener;
     ros::Rate rate(10.0);



if (centroidPointb.point.x != 0 and centroidPointb.point.y != 0) {
  geometry_msgs::PointStamped punt_transformat;

  try{
       listener.waitForTransform("map", "camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0));
       listener.transformPoint("map", centroidPointb, punt_transformat);
//       std::cout << "Listener a base_link : " << punt_transformat << std::endl;
     }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

      //BLAAAAAUUUUU
      if ((punt_transformat.point.x != 0) and (color_def=="blue")){


        persones_points_pose.pose.position = punt_transformat.point;
        persones_points_pose.pose.orientation.x = 0;
        persones_points_pose.pose.orientation.y = 0;
        persones_points_pose.pose.orientation.z = 0;
        persones_points_pose.pose.orientation.w = 1;
        pub_persones.publish(persones_points_pose);

      }

      //VERMEEEELLL
      if ((punt_transformat.point.x != 0) and (color_def=="red")){


        perill_points_pose.pose.position = punt_transformat.point;
        perill_points_pose.pose.orientation.x = 0;
        perill_points_pose.pose.orientation.y = 0;
        perill_points_pose.pose.orientation.z = 0;
        perill_points_pose.pose.orientation.w = 1;
        pub_perill.publish(perill_points_pose);


      }



      if ((punt_transformat.point.x != 0) and (color_def == "green")){



        sortida_points_pose.pose.position = punt_transformat.point;
        sortida_points_pose.pose.orientation.x = 0;
        sortida_points_pose.pose.orientation.y = 0;
        sortida_points_pose.pose.orientation.z = 0;
        sortida_points_pose.pose.orientation.w = 1;
        pub_sortida.publish(sortida_points_pose);


      }
}


}

class Listener
{
  public:
    //const sensor_msgs::PointCloud2& msg;
    int a;
    tf::TransformListener listener_;
    void callback( const sensor_msgs::PointCloud2& msg){

      if (count < 10){
      //ROS_INFO(msg);
      //std::cout << "point cloud" << msg << std::endl;
      //  PointCloudC::Ptr cloud(new PointCloudC());
        pcl::fromROSMsg(msg, *cloud);
      //  ROS_INFO("Got point cloud with %ld points", cloud->size());
      //CROPPING --Per eliminar soroll. Elimina punts que llegim de la càmera per centrar l'atenció en una àrea en concret.
        PointCloudC::Ptr cropped_cloud(new PointCloudC());
        Eigen::Vector4f min_pt(-2.5, -1, 0.5, 1);
        Eigen::Vector4f max_pt(2.5, 1, 5, 1);
        pcl::CropBox<PointC> crop;
        crop.setInputCloud(cloud);
        crop.setMin(min_pt);
        crop.setMax(max_pt);
        crop.filter(*cropped_cloud);
        //ROS_INFO("Cropped to %ld points", cropped_cloud->size());
        PointC min_pcl;
        PointC max_pcl;
        pcl::getMinMax3D<PointC>(*cropped_cloud, min_pcl, max_pcl);
        geometry_msgs::Point centroidPoint;
        centroidPoint.x = (min_pcl.x + max_pcl.x)/2;
        centroidPoint.y = (min_pcl.y + max_pcl.y)/2;
        //std::cout << "Header point cloud camera" << cropped_cloud->header <<std::endl;

      ///
        function_cloud(cropped_cloud, "blue");
  //      function_cloud(cropped_cloud, "green");
  //      function_cloud(cropped_cloud, "red");

      count = 0;
}
else {
  count++;
}

      }

  Listener(){std::cout << "holis" << std::endl;}; //ros::NodeHandle* nh;);
};


main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "astra_detection");
  ros::NodeHandle nh;

  // Subscriber al cloud de la camera astra
  Listener listener;
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, &Listener::callback, &listener);

  // Publisher cloud filtres i tal
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output_astra_detection", 1);

  pub_persones = nh.advertise<geometry_msgs::PoseStamped>("vision/astra/blue_pose",1);
  pub_perill = nh.advertise<geometry_msgs::PoseStamped>("vision/astra/red_pose",1);
  pub_sortida = nh.advertise<geometry_msgs::PoseStamped>("vision/astra/green_pose",1);


  ros::spin();

    return 0;


}
