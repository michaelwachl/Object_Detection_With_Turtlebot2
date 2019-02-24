/*------------------------------------------------------------------------------
* ------------------------------------------------------------------------------
*Author: Michael Wachl
*Year: 2018
*Course: Leistungskurs C++
*Group: 8
*Univerity: Technische Universität München
* ------------------------------------------------------------------------------
------------------------------------------------------------------------------*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<sensor_msgs/image_encodings.h> 
#include<image_transport/image_transport.h> 
#include <sstream>
#include <string>
#include <iostream>
#include <time.h>
//#include <boost.h>
//pcl specific
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointT;

int
main (int argc, char** argv)
{
  std::string a="/home/miguel/pcd_files/PCD_go_pickup_leavecone/1.pcd", b="/home/miguel/pcd_files/PCD_go_pickup_leavecone/2.pcd",c="/home/miguel/pcd_files/PCD_go_pickup_leavecone/3.pcd",
d="/home/miguel/pcd_files/PCD_go_pickup_leavecone/4.pcd",e="/home/miguel/pcd_files/PCD_go_pickup_leavecone/5.pcd",f="/home/miguel/pcd_files/PCD_go_pickup_leavecone/6.pcd",
g="/home/miguel/pcd_files/PCD_go_pickup_leavecone/7.pcd",h="/home/miguel/pcd_files/PCD_go_pickup_leavecone/8.pcd",i="/home/miguel/pcd_files/PCD_go_pickup_leavecone/9.pcd",
j="/home/miguel/pcd_files/PCD_go_pickup_leavecone/10.pcd";
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_pub");
  ROS_INFO("inti");
  ros::NodeHandle nh;
  ros::Publisher pub;
  pub = nh.advertise<sensor_msgs::PointCloud2> ("test_cloud", 1);

  pcl::PCLPointCloud2 point_cloud2;  //PointCloud2's
  pcl::PCLPointCloud2ConstPtr cloud2Ptr(&point_cloud2);
  pcl::PointCloud<PointT>::Ptr ptr_cloud(new pcl::PointCloud<PointT>);

 pcl::PCDReader reader;
 if(reader.read(g,point_cloud2,0)==0)
         ROS_INFO("Loading success");

  ros::Duration(10.0).sleep();
  sensor_msgs::PointCloud2 output_msgs;
  pcl_conversions::fromPCL(point_cloud2, output_msgs);
  pub.publish(output_msgs);
  ROS_INFO("publ");
  ros::Duration(10.0).sleep();

   return 0;
}
