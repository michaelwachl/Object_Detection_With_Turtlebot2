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
#include "wachl_3d_object_detection/object.h"
#include "wachl_3d_object_detection/objectArray.h"

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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>


ros::Publisher pub,pub2;
typedef pcl::PointXYZRGB PointT;
using namespace std;
using namespace pcl;

class C3dDetection
{
public:
   explicit C3dDetection(ros::NodeHandle nh)
        : nh_(nh)
    {
         //sub_ = nh_.subscribe ("/camera/depth_registered/points", 1, &C3dDetection::cloud_cb, this);
         sub_ = nh_.subscribe ("/test_cloud", 1, &C3dDetection::cloud_cb, this);
         pub_ = nh_.advertise<wachl_3d_object_detection::objectArray> ("detected_objects", 1);
         pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2> ("output_cloud", 1);
         //Get parameter from param server
         nh.getParam("/object_detection/visiu_", visiu_);
         nh.getParam("/object_detection/console_output_", console_output_);
         nh.getParam("/object_detection/LEAF_SIZE_", LEAF_SIZE_);
         nh.getParam("/object_detection/Y_LIM_MIN_", Y_LIM_MIN_);
         nh.getParam("/object_detection/Y_LIM_MAX_", Y_LIM_MAX_);
         nh.getParam("/object_detection/Z_LIM_MIN_", Z_LIM_MIN_);
         nh.getParam("/object_detection/Z_LIM_MAX_", Z_LIM_MAX_);
         nh.getParam("/object_detection/CLUSTER_TOLERANCE_", CLUSTER_TOLERANCE_);
         nh.getParam("/object_detection/CLUSTER_MIN_", CLUSTER_MIN_);
         nh.getParam("/object_detection/CLUSTER_MAX_", CLUSTER_MAX_);
         nh.getParam("/object_detection/PLANE_RATIO_", PLANE_RATIO_);
         nh.getParam("/object_detection/ZYLINDER_RATIO_", ZYLINDER_RATIO_);
         nh.getParam("/object_detection/MAX_WIDTH_", MAX_WIDTH_);
         nh.getParam("/object_detection/MAX_DEPTH_", MAX_DEPTH_);
         nh.getParam("/object_detection/YELLOW_RANGE_H_MIN_", YELLOW_RANGE_H_MIN_);
         nh.getParam("/object_detection/YELLOW_RANGE_H_MAX_", YELLOW_RANGE_H_MAX_);
         nh.getParam("/object_detection/YELLOW_RANGE_V_MIN_", YELLOW_RANGE_V_MIN_);
         nh.getParam("/object_detection/YELLOW_RANGE_V_MAX_", YELLOW_RANGE_V_MAX_);
         nh.getParam("/object_detection/YELLOW_RANGE_S_MIN_", YELLOW_RANGE_S_MIN_);
         nh.getParam("/object_detection/GREEN_RANGE_H_MIN_", GREEN_RANGE_H_MIN_);
         nh.getParam("/object_detection/GREEN_RANGE_H_MAX_", GREEN_RANGE_H_MAX_);
         nh.getParam("/object_detection/GREEN_RANGE_V_MIN_", GREEN_RANGE_V_MIN_);
         nh.getParam("/object_detection/GREEN_RANGE_V_MAX_", GREEN_RANGE_V_MAX_);
         nh.getParam("/object_detection/GREEN_RANGE_S_MIN_", GREEN_RANGE_S_MIN_);
         nh.getParam("/object_detection/GREEN_RANGE_S_MAX_", GREEN_RANGE_S_MAX_);
         nh.getParam("/object_detection/BLUE_RANGE_H_MIN_", BLUE_RANGE_H_MIN_);
         nh.getParam("/object_detection/BLUE_RANGE_H_MAX_", BLUE_RANGE_H_MAX_);
         nh.getParam("/object_detection/BLUE_RANGE_V_MIN_", BLUE_RANGE_V_MIN_);
         nh.getParam("/object_detection/BLUE_RANGE_V_MAX_", BLUE_RANGE_V_MAX_);
         nh.getParam("/object_detection/BLUE_RANGE_S_MIN_", BLUE_RANGE_S_MIN_);
         nh.getParam("/object_detection/BLUE_RANGE_S_MAX_", BLUE_RANGE_S_MAX_);
         nh.getParam("/object_detection/DARK_RANGE_", DARK_RANGE_);
         nh.getParam("/object_detection/GREEN_RATIO_", GREEN_RATIO_);
         nh.getParam("/object_detection/BLUE_RATIO_", BLUE_RATIO_);
         nh.getParam("/object_detection/YELLOW_RATIO_", YELLOW_RATIO_);
         nh.getParam("/object_detection/DARK_RATIO_", DARK_RATIO_);
         nh.getParam("/object_detection/ROBO_HEIGHT_MIN_", ROBO_HEIGHT_MIN_);
         nh.getParam("/object_detection/ROBO_HEIGHT_MAX_", ROBO_HEIGHT_MAX_);
         nh.getParam("/object_detection/ROBO_WIDTH_MIN_", ROBO_WIDTH_MIN_);
         nh.getParam("/object_detection/ROBO_WIDTH_MAX_", ROBO_WIDTH_MAX_);

    }
private:
    //Visiualization param
    bool visiu_, console_output_;
    //filter param
    float LEAF_SIZE_,Y_LIM_MIN_,Y_LIM_MAX_,Z_LIM_MIN_,Z_LIM_MAX_,CLUSTER_TOLERANCE_,PLANE_RATIO_,ZYLINDER_RATIO_,MAX_WIDTH_,MAX_DEPTH_;
    int CLUSTER_MIN_,CLUSTER_MAX_;
    //color filter param
    float YELLOW_RANGE_H_MIN_,YELLOW_RANGE_H_MAX_,YELLOW_RANGE_V_MIN_,YELLOW_RANGE_V_MAX_,YELLOW_RANGE_S_MIN_,YELLOW_RANGE_S_MAX_;
    float GREEN_RANGE_H_MIN_,GREEN_RANGE_H_MAX_,GREEN_RANGE_V_MIN_,GREEN_RANGE_V_MAX_,GREEN_RANGE_S_MIN_,GREEN_RANGE_S_MAX_;
    float BLUE_RANGE_H_MIN_,BLUE_RANGE_H_MAX_,BLUE_RANGE_V_MIN_,BLUE_RANGE_V_MAX_,BLUE_RANGE_S_MIN_,BLUE_RANGE_S_MAX_,DARK_RANGE_;
    float GREEN_RATIO_,BLUE_RATIO_,YELLOW_RATIO_,DARK_RATIO_;
    float ROBO_HEIGHT_MIN_,ROBO_HEIGHT_MAX_,ROBO_WIDTH_MIN_,ROBO_WIDTH_MAX_;
    //variables
    ros::NodeHandle nh_;
    ros::Publisher pub_,pub_cloud_;
    ros::Subscriber sub_;

    //funcitons
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void RGBtoHSV(float& fR, float& fG, float& fB, float& fH, float& fS, float& fV);
}; //end of class defintion



void C3dDetection::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_INFO("Pointcloud received");
    //Stop time
    clock_t t;
    t = clock();

 //#####################################################################################
   //Container for original & filtered data
  //#####################################################################################
  wachl_3d_object_detection::object data;
  wachl_3d_object_detection::objectArray msg;

  pcl::PCLPointCloud2* point_cloud2 = new pcl::PCLPointCloud2; //PointCloud2's
  pcl::PCLPointCloud2ConstPtr cloud2Ptr(point_cloud2);
  pcl::PCLPointCloud2 cloud_out;

  pcl::PointCloud<PointT>::Ptr ptr_cloud(new pcl::PointCloud<PointT>); //PointCloud-XYZ's
  pcl::PointCloud<PointT>::Ptr ptr_cloud_filtered(new pcl::PointCloud<PointT>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);  //PointCloud-XYZ-RGB

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());


  //Filters and Segmentation container/variables
  pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
  pcl::PassThrough<PointT> pass;
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  std::vector<pcl::PointIndices>::const_iterator it;
  //std::vector<int>::const_iterator pit;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *point_cloud2);


  //#####################################################################################
 //## Do Filtering ############################################################
  //#####################################################################################
 //Do filtering with VoxelGrid
  vox.setInputCloud(cloud2Ptr);
  vox.setLeafSize (LEAF_SIZE_ , LEAF_SIZE_ , LEAF_SIZE_);
  vox.filter(cloud_out);

  //Convert Could2 to Cloud for additional filtering
  pcl::fromPCLPointCloud2(cloud_out, *ptr_cloud);

  //Do passthrough filtering to remove NaNs "z"
  pass.setInputCloud(ptr_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits(Z_LIM_MIN_, Z_LIM_MAX_);
  pass.filter(*ptr_cloud_filtered);

  // Build a passthrough filter to cutoff "y"
  // Create the filtering object
  pass.setInputCloud(ptr_cloud_filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(Y_LIM_MIN_, Y_LIM_MAX_); //check values in Matlab
  //pass.setFilterLimitsNegative (true);
  pass.filter(*ptr_cloud_filtered );

  //Statistical outlier removal
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(ptr_cloud_filtered) ;
  sor.setMeanK (50) ;
  sor.setStddevMulThresh(1.0);
  sor.filter(*ptr_cloud_filtered);
  if(console_output_)
        std::cout<<"PointCloud after filtering has: "<< ptr_cloud_filtered->points.size()<<" data points."<< std::endl;

  if(visiu_)
  {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (255, 255, 255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(ptr_cloud_filtered);
  viewer->addPointCloud<PointT> (ptr_cloud_filtered,rgb ,"sample cloud");
  viewer->spinOnce (100);
  ros::Duration(3.0).sleep();
  }


  //#####################################################################################
  //Do Clustering with Kd-Tree search to extract objects
  //#####################################################################################
  tree->setInputCloud(ptr_cloud_filtered);
  ec.setClusterTolerance(CLUSTER_TOLERANCE_); // 3cm
  ec.setMinClusterSize(CLUSTER_MIN_); //100
  ec.setMaxClusterSize(CLUSTER_MAX_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(ptr_cloud_filtered);
  ec.extract(cluster_indices);

  std::cout<<"PointCloud has: "<< cluster_indices.size() <<" clusters"<<std::endl;

  int current_size;
  float current_planeratio, current_zylinderratio;

  for (const auto it : cluster_indices)
      {
      //extract pointclouds
      pcl::ExtractIndices<PointT> extract2;
      extract2.setInputCloud (ptr_cloud_filtered);
      extract2.setIndices (boost::make_shared<const pcl::PointIndices> (it));
      pcl::PointCloud<PointT>::Ptr ptr_currentclustercloud (new pcl::PointCloud<PointT>);
      extract2.filter(*ptr_currentclustercloud);

      std::cout<<" "<<std::endl;

      if(console_output_)
            std::cout<<"Cluster has: "<< ptr_currentclustercloud->points.size() <<" Points"<<std::endl;
      current_size = ptr_currentclustercloud->points.size();

      //skip if object is too wide or too deep
      PointT minPt, maxPt;
      float cluster_width, cluster_height, cluster_depth;
      pcl::getMinMax3D (*ptr_currentclustercloud, minPt, maxPt);
      cluster_width = abs(maxPt.x -minPt.x);
      cluster_height = abs(maxPt.y -minPt.y);
      cluster_depth = abs(maxPt.z -minPt.z);
     if(console_output_)
     {
      std::cout<<"Cluster width: "<<cluster_width<<std::endl;
      std::cout<<"Cluster depth: "<<cluster_depth<<std::endl;
      std::cout<<"Cluster height: "<<cluster_height<<std::endl;
     }

      if(cluster_width>MAX_WIDTH_ || cluster_depth>MAX_DEPTH_)
      {   if(console_output_)
                std::cout<<"Cluster too deep or wide"<<std::endl;
          continue;
      }

      //#####################################################################################
      //search for plane or zylinder
      //#####################################################################################
      // Estimate point normals
      pcl::NormalEstimation<PointT, pcl::Normal> ne;
      pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
      ne.setSearchMethod(tree);
      ne.setInputCloud(ptr_currentclustercloud);
      ne.setKSearch(50);
      ne.compute(*cloud_normals);

      pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
      // Create the segmentation object for the planar model and set all the parameters
      pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);

      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setNormalDistanceWeight (0.1);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (70);
      seg.setDistanceThreshold (0.01);
      seg.setInputCloud (ptr_currentclustercloud);
      seg.setInputNormals (cloud_normals);
      pcl::ModelCoefficients::Ptr coefficients_plane_loose (new pcl::ModelCoefficients);
      seg.segment (*inliers_plane, *coefficients_plane_loose);
      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<PointT> extract;

      extract.setInputCloud (ptr_currentclustercloud);
      extract.setIndices (inliers_plane);
      extract.setNegative (false);

      extract.filter (*cloud_plane);
      if(console_output_)
            std::cerr << "Cluster has planar component: " << cloud_plane->points.size () << " data points." << std::endl;
      current_planeratio = (float)cloud_plane->points.size()/current_size;

      //zylinder
      pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT> ());
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_CYLINDER);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setNormalDistanceWeight(0.1);
      seg.setMaxIterations(500);
      seg.setDistanceThreshold(0.05);
      seg.setRadiusLimits(0.015, 0.1);
      seg.setInputCloud(ptr_currentclustercloud);
      seg.setInputNormals(cloud_normals);

      // Obtain the cylinder inliers and coefficients
      pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
      seg.segment(*inliers_cylinder, *coefficients_cylinder);

      extract.setInputCloud(ptr_currentclustercloud);
      extract.setIndices(inliers_cylinder);
      extract.setNegative(false);
      extract.filter(*cloud_cylinder);
      if(console_output_)
            std::cerr << "Cluster has zylinder component: " << cloud_cylinder->points.size () << " data points." << std::endl;
      current_zylinderratio = (float)cloud_cylinder->points.size()/current_size;

      if(console_output_)
      {
       std::cout << "Plane ratio: " << current_planeratio << std::endl;
       std::cout << "Zylinder ratio: " << current_zylinderratio << std::endl;
      }

      //#####################################################################################
      // Identify Objects by color
      //####################################################################################
       if(current_zylinderratio > ZYLINDER_RATIO_ || current_planeratio > PLANE_RATIO_)
       {

         if(visiu_)
           {
           boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
           viewer->setBackgroundColor (250, 250, 250);
           pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(ptr_currentclustercloud);
           viewer->addPointCloud<PointT> (ptr_currentclustercloud,rgb ,"sample cloud");
           viewer->spinOnce (100);
           ros::Duration(4.0).sleep();
           }

        int pixelcounter_y = 0,pixelcounter_b = 0,pixelcounter_g = 0, pixelcounter_d = 0;

        //loop and identify colors
        for(int i = 0; i<ptr_currentclustercloud->size(); ++i)
        {
         Eigen::Vector3i tricolor = ptr_currentclustercloud->at(i).getRGBVector3i();
        //Convert RGB to HSV
        float  fR = tricolor(0,0)/255.0;
        float  fG = tricolor(1,0)/255.0;
        float  fB = tricolor(2,0)/255.0;
        float fH,fS,fV;

         RGBtoHSV(fR,fG,fB,fH,fS,fV);
         if(console_output_)
             std::cout <<"H: "<<fH<<"  V: "<<fS<<"  S: "<<fV<<std::endl;

            //check if point in yellow range
         if (fH<=YELLOW_RANGE_H_MAX_ && fH>YELLOW_RANGE_H_MIN_ && fS>YELLOW_RANGE_S_MIN_ && fV>YELLOW_RANGE_V_MIN_)
         {
            pixelcounter_y++;
                    //std::cout<<"yellow"<<std::endl;
         }

                //check if point in green range
         if (fH<=GREEN_RANGE_H_MAX_ && fH>GREEN_RANGE_H_MIN_ && fS<=GREEN_RANGE_S_MAX_ && fS>GREEN_RANGE_S_MIN_ &&
                    fV<=GREEN_RANGE_V_MAX_ && fV>GREEN_RANGE_V_MIN_)
         {
            pixelcounter_g++;
                    // std::cout<<"green"<<std::endl;
         }

                //check if point in blue range
         if (fH<=BLUE_RANGE_H_MAX_ && fH>BLUE_RANGE_H_MIN_ && fS>BLUE_RANGE_S_MIN_ && fV>BLUE_RANGE_V_MIN_)
         {
            pixelcounter_b++;
                    //std::cout<<"blue"<<std::endl;
         }
         //check for low brightness level
         if (fV<=DARK_RANGE_)
             pixelcounter_d++;

       }
       if(console_output_)
       {
       std::cout << "Green count: " <<pixelcounter_g<<std::endl;
       std::cout << "Yellow count: " <<pixelcounter_y<<std::endl;
       std::cout << "Blue count: " <<pixelcounter_b<<std::endl;
       std::cout << "Dark count: " <<pixelcounter_d<<std::endl;

       std::cout << "Green-ratio: " <<((float)pixelcounter_g/(float)current_size)<<std::endl;
       std::cout << "Yellow-ratio: " <<((float)pixelcounter_y/(float)current_size)<<std::endl;
       std::cout << "Blue-ratio: " <<((float)pixelcounter_b/(float)current_size)<<std::endl;
       std::cout << "Dark-ratio: " <<((float)pixelcounter_d/(float)current_size)<<std::endl;
       }

       // Green Pole
       if(current_size!=0&&((float)pixelcounter_g/(float)current_size)>GREEN_RATIO_)
       {
        std::cout << "It's green!" << std::endl;
        //compute centroid
        Eigen::Vector4f centroid;
        const pcl::PointCloud<PointT> cloud = *ptr_currentclustercloud;
        if(pcl::compute3DCentroid(cloud, centroid) != 0)
        {
          //std::cout<< "Centroid:     " << centroid<<std::endl;

          data.type = "green_pole";
          data.dist = centroid(2,0);
          data.angle = asin(centroid(0,0)/centroid(2,0)); //in rad

          msg.objectArray.push_back(data);
        }
       }

       // Yellow puck
       else if(current_size!=0&&((float)pixelcounter_y/(float)current_size)>YELLOW_RATIO_)
       {
        std::cout << "It's yellow!" << std::endl;

        //compute centroid
        Eigen::Vector4f centroid;
        const pcl::PointCloud<PointT> cloud = *ptr_currentclustercloud;
        if(pcl::compute3DCentroid(cloud, centroid) != 0)
        {
          std::cout<< "Centroid:     " << centroid<<std::endl;

          data.type = "yellow_puck";
          data.dist = centroid(2,0);
          data.angle = asin(centroid(0,0)/centroid(2,0)); //in rad

           msg.objectArray.push_back(data);
        }
       }

       // Blue Pole
       else if(current_size!=0&&((float)pixelcounter_b/(float)current_size)>BLUE_RATIO_)
       {
        std::cout << "It's blue!" << std::endl;

        //compute centroid
        Eigen::Vector4f centroid;
        const pcl::PointCloud<PointT> cloud = *ptr_currentclustercloud;
        if(pcl::compute3DCentroid(cloud, centroid) != 0)
        {
          std::cout<< "Centroid:     " << centroid<<std::endl;

          data.type = "blue_puck";
          data.dist = centroid(2,0);
          data.angle = asin(centroid(0,0)/centroid(2,0)); //in rad

          msg.objectArray.push_back(data);
        }
       }
       // Other Turtlebot
       else if(current_size!=0&&((float)pixelcounter_d/(float)current_size)>DARK_RATIO_&&
               cluster_width>ROBO_WIDTH_MIN_ && cluster_width<ROBO_WIDTH_MAX_ &&
               cluster_height>ROBO_HEIGHT_MIN_ && cluster_height<ROBO_HEIGHT_MAX_)
       {
        std::cout << "It's a turtle!" << std::endl;

        //compute centroid
        Eigen::Vector4f centroid;
        const pcl::PointCloud<PointT> cloud = *ptr_currentclustercloud;
        if(pcl::compute3DCentroid(cloud, centroid) != 0)
        {
          std::cout<< "Centroid:     " << centroid<<std::endl;

          data.type = "turtlebot";
          data.dist = centroid(2,0);
          data.angle = asin(centroid(0,0)/centroid(2,0)); //in rad

          msg.objectArray.push_back(data);
        }
       }


      // save cloud
      // pcl::io::savePCDFileASCII("test_pcd.pcd", *ptr_currentclustercloud);
       }
       }

  //Time used
  std::cout<<"Used time: "<<(float)(clock()-t)/CLOCKS_PER_SEC<<" s"<<std::endl;

  // Publish the data.
  pub_.publish(msg);


}


void C3dDetection::RGBtoHSV(float& fR, float& fG, float& fB, float& fH, float& fS, float& fV)
{
  float fCMax = max(max(fR, fG), fB);
  float fCMin = min(min(fR, fG), fB);
  float fDelta = fCMax - fCMin;

  if(fDelta > 0) {
    if(fCMax == fR) {
      fH = 60 * (fmod(((fG - fB) / fDelta), 6));
    } else if(fCMax == fG) {
      fH = 60 * (((fB - fR) / fDelta) + 2);
    } else if(fCMax == fB) {
      fH = 60 * (((fR - fG) / fDelta) + 4);
    }

    if(fCMax > 0) {
      fS = fDelta / fCMax;
    } else {
      fS = 0;
    }

    fV = fCMax;
  } else {
    fH = 0;
    fS = 0;
    fV = fCMax;
  }

  if(fH < 0) {
    fH = 360 + fH;
  }
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "object_detection");
  ros::NodeHandle nh;
  ROS_INFO("NODE detect runs");
  C3dDetection detect(nh);

  while (ros::ok())
     ros::spin ();

   return 0;
}
