/*------------------------------------------------------------------------------
* ------------------------------------------------------------------------------
*Author: Michael Wachl
*Year: 2018
*Course: Leistungskurs C++
*Group: 8
*Univerity: Technische Universität München
* ------------------------------------------------------------------------------
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
* This class is used to detect several objects from a registered pointcloud
and publish them.
------------------------------------------------------------------------------*/

//include header
#include "wachl_3d_object_detection_node.h"




//********************************************************
// Call back to change parameter
//********************************************************
void C3dDetection::paramCallback(wachl_3d_object_detection::detectionConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure requested");

VISIU_ = config.VISIU_;
CONSOLE_OUTPUT_ = config.CONSOLE_OUTPUT_;
LEAF_SIZE_ = config.LEAF_SIZE_;
Z_LIM_MIN_ = config.Z_LIM_MIN_;
Z_LIM_MAX_ = config.Z_LIM_MAX_;
Y_LIM_MIN_ = config.Y_LIM_MIN_;
Y_LIM_MAX_ = config.Y_LIM_MAX_;
CLUSTER_MIN_ = config.CLUSTER_MIN_;
CLUSTER_MAX_ = config.CLUSTER_MAX_;
CLUSTER_TOLERANCE_ = config.CLUSTER_TOLERANCE_;
PLANE_RATIO_ = config.PLANE_RATIO_;
ZYLINDER_RATIO_ = config.ZYLINDER_RATIO_;
MAX_WIDTH_ = config.MAX_WIDTH_;
MAX_DEPTH_ = config.MAX_DEPTH_;
YELLOW_RANGE_H_MIN_ = config.YELLOW_RANGE_H_MIN_;
YELLOW_RANGE_H_MAX_ = config.YELLOW_RANGE_H_MAX_;
YELLOW_RANGE_V_MIN_ = config.YELLOW_RANGE_V_MIN_;
YELLOW_RANGE_V_MAX_ = config.YELLOW_RANGE_V_MAX_;
YELLOW_RANGE_S_MIN_ = config.YELLOW_RANGE_S_MIN_;
YELLOW_RANGE_S_MAX_ = config.YELLOW_RANGE_S_MAX_;
GREEN_RANGE_H_MIN_ = config.GREEN_RANGE_H_MIN_;
GREEN_RANGE_H_MAX_ = config.GREEN_RANGE_H_MAX_;
GREEN_RANGE_V_MIN_ = config.GREEN_RANGE_V_MIN_;
GREEN_RANGE_V_MAX_ = config.GREEN_RANGE_V_MAX_;
GREEN_RANGE_S_MIN_ = config.GREEN_RANGE_S_MIN_;
GREEN_RANGE_S_MAX_ = config.GREEN_RANGE_S_MAX_;
BLUE_RANGE_H_MIN_ = config.BLUE_RANGE_H_MIN_;
BLUE_RANGE_H_MAX_ = config.BLUE_RANGE_H_MAX_;
BLUE_RANGE_V_MIN_ = config.BLUE_RANGE_V_MIN_;
BLUE_RANGE_V_MAX_ = config.BLUE_RANGE_V_MAX_;
BLUE_RANGE_S_MIN_ = config.BLUE_RANGE_S_MIN_;
BLUE_RANGE_S_MAX_ = config.BLUE_RANGE_S_MAX_;
DARK_RANGE_ = config.DARK_RANGE_;
GREEN_RATIO_ = config.GREEN_RATIO_;
BLUE_RATIO_ = config.BLUE_RATIO_;
YELLOW_RATIO_ = config.YELLOW_RATIO_;
DARK_RATIO_ = config.DARK_RATIO_;
ROBO_HEIGHT_MIN_ = config.ROBO_HEIGHT_MIN_;
ROBO_HEIGHT_MAX_ = config.ROBO_HEIGHT_MAX_;
ROBO_WIDTH_MIN_ = config.ROBO_WIDTH_MIN_;
ROBO_WIDTH_MAX_ = config.ROBO_WIDTH_MAX_;
}


//********************************************************
//callback function to receive pointcloud
//********************************************************
void C3dDetection::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
 //#####################################################################################
   //Container for original & filtered data
  //#####################################################################################
  wachl_3d_object_detection::object data;
  wachl_3d_object_detection::objectArray msg;
  geometry_msgs::PointStamped msg_puck;
  data.point.header = cloud_msg->header;
  msg_puck.header = cloud_msg->header;

  pcl::PCLPointCloud2* point_cloud2 = new pcl::PCLPointCloud2; //PointCloud2's
  pcl::PCLPointCloud2ConstPtr cloud2Ptr(point_cloud2);
  pcl::PCLPointCloud2 cloud_out;

  pcl::PointCloud<PointT>::Ptr ptr_cloud(new pcl::PointCloud<PointT>); //PointCloud-XYZ's
  pcl::PointCloud<PointT>::Ptr ptr_cloud_filtered(new pcl::PointCloud<PointT>);

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

  //stop time
  ROS_INFO("Pointcloud received");
  //Stop time
  clock_t t;
  t = clock();

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

  //Statistical outlier removal , commented to improve performance
  /*
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(ptr_cloud_filtered) ;
  sor.setMeanK (50) ;
  sor.setStddevMulThresh(1.0);
  sor.filter(*ptr_cloud_filtered); */

  if(CONSOLE_OUTPUT_)
        std::cout<<"PointCloud after filtering has: "<< ptr_cloud_filtered->points.size()<<" data points."<< std::endl;

  if(VISIU_)
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

  if(CONSOLE_OUTPUT_)
        std::cout<<"PointCloud has: "<< cluster_indices.size() <<" clusters"<<std::endl;

  int current_size;
  float current_planeratio, current_zylinderratio;
  //loop throw clusters
  for (const auto it : cluster_indices)
      {
      //extract pointclouds
      pcl::ExtractIndices<PointT> extract2;
      extract2.setInputCloud (ptr_cloud_filtered);
      extract2.setIndices (boost::make_shared<const pcl::PointIndices> (it));
      pcl::PointCloud<PointT>::Ptr ptr_currentclustercloud (new pcl::PointCloud<PointT>);
      extract2.filter(*ptr_currentclustercloud);

      if(CONSOLE_OUTPUT_)
            std::cout<<"Cluster has: "<< ptr_currentclustercloud->points.size() <<" Points"<<std::endl;
      current_size = ptr_currentclustercloud->points.size();

      //skip if object is too wide or too deep
      PointT minPt, maxPt;
      float cluster_width, cluster_height, cluster_depth;
      pcl::getMinMax3D (*ptr_currentclustercloud, minPt, maxPt);
      cluster_width = abs(maxPt.x -minPt.x);
      cluster_height = abs(maxPt.y -minPt.y);
      cluster_depth = abs(maxPt.z -minPt.z);
     if(CONSOLE_OUTPUT_)
     {
      std::cout<<"Cluster width: "<<cluster_width<<std::endl;
      std::cout<<"Cluster depth: "<<cluster_depth<<std::endl;
      std::cout<<"Cluster height: "<<cluster_height<<std::endl;
     }

      if(cluster_width>MAX_WIDTH_ || cluster_depth>MAX_DEPTH_)
      {   if(CONSOLE_OUTPUT_)
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
      if(CONSOLE_OUTPUT_)
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
      if(CONSOLE_OUTPUT_)
            std::cerr << "Cluster has zylinder component: " << cloud_cylinder->points.size () << " data points." << std::endl;
      current_zylinderratio = (float)cloud_cylinder->points.size()/current_size;

      if(CONSOLE_OUTPUT_)
      {
       std::cout << "Plane ratio: " << current_planeratio << std::endl;
       std::cout << "Zylinder ratio: " << current_zylinderratio << std::endl;
      }

      //#####################################################################################
      // Identify Objects by color
      //####################################################################################
       if(current_zylinderratio > ZYLINDER_RATIO_ || current_planeratio > PLANE_RATIO_)
       {
         if(VISIU_)
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
         //if(CONSOLE_OUTPUT_)
         //    std::cout <<"H: "<<fH<<"  V: "<<fS<<"  S: "<<fV<<std::endl;

            //check if point in yellow range
         if (fH<=YELLOW_RANGE_H_MAX_ && fH>YELLOW_RANGE_H_MIN_ && fS>YELLOW_RANGE_S_MIN_ && fV>YELLOW_RANGE_V_MIN_)
         {
            pixelcounter_y++;
         }

                //check if point in green range
         if (fH<=GREEN_RANGE_H_MAX_ && fH>GREEN_RANGE_H_MIN_ && fS<=GREEN_RANGE_S_MAX_ && fS>GREEN_RANGE_S_MIN_ &&
                    fV<=GREEN_RANGE_V_MAX_ && fV>GREEN_RANGE_V_MIN_)
         {
            pixelcounter_g++;
         }

                //check if point in blue range
         if (fH<=BLUE_RANGE_H_MAX_ && fH>BLUE_RANGE_H_MIN_ && fS>BLUE_RANGE_S_MIN_ && fV>BLUE_RANGE_V_MIN_)
         {
            pixelcounter_b++;
         }
         //check for low brightness level
         if (fV<=DARK_RANGE_)
             pixelcounter_d++;

       }
       if(CONSOLE_OUTPUT_)
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
        //compute centroid
        Eigen::Vector4f centroid;
        const pcl::PointCloud<PointT> cloud = *ptr_currentclustercloud;
        if(pcl::compute3DCentroid(cloud, centroid) != 0)
        {
          std::cout << "It's green pole!" << std::endl;
          data.type = "green_pole";
          data.point.point.x = centroid(0,0);
          data.point.point.y = centroid(1,0);
          data.point.point.z = centroid(2,0);

          msg.objectArray.push_back(data);
        }
       }

       // Yellow puck
       else if(current_size!=0&&((float)pixelcounter_y/(float)current_size)>YELLOW_RATIO_)
       {
        //compute centroid
        Eigen::Vector4f centroid;
        const pcl::PointCloud<PointT> cloud = *ptr_currentclustercloud;
        if(pcl::compute3DCentroid(cloud, centroid) != 0)
        {
          std::cout << "It's a yellow puck!" << std::endl;

          data.type = "yellow_puck";
          data.point.point.x = centroid(0,0);
          data.point.point.y = centroid(1,0);
          data.point.point.z = centroid(2,0);

           msg.objectArray.push_back(data);
        }
       }

       // Blue Pole
       else if(current_size!=0&&((float)pixelcounter_b/(float)current_size)>BLUE_RATIO_)
       {
        //compute centroid
        Eigen::Vector4f centroid;
        const pcl::PointCloud<PointT> cloud = *ptr_currentclustercloud;
        if(pcl::compute3DCentroid(cloud, centroid) != 0)
        {
          std::cout << "It's a blue puck!" << std::endl;

          data.type = "blue_puck";
          data.point.point.x = centroid(0,0);
          data.point.point.y = centroid(1,0);
          data.point.point.z = centroid(2,0);          // asin(centroid(0,0)/centroid(2,0)); //in rad

          msg.objectArray.push_back(data);
        }
       }
       // Other Turtlebot
       else if(current_size!=0&&((float)pixelcounter_d/(float)current_size)>DARK_RATIO_&&
               cluster_width>ROBO_WIDTH_MIN_ && cluster_width<ROBO_WIDTH_MAX_)
       {
        //compute centroid
        Eigen::Vector4f centroid;
        const pcl::PointCloud<PointT> cloud = *ptr_currentclustercloud;
        if(pcl::compute3DCentroid(cloud, centroid) != 0)
        {
          std::cout << "It's a turtle!" << std::endl;

          data.type = "turtlebot";
          data.point.point.x = centroid(0,0);
          data.point.point.y = centroid(1,0);
          data.point.point.z = centroid(2,0);//in rad

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

  //loop through message and publish closest puck
  msg_puck.point.z = 4.0; //set heigh
  std::string color;
  bool puck_there =false;
  if(nh_.hasParam("/team_color_param"));
        nh_.getParam("/team_color_param",color);
  for(const auto msgit : msg.objectArray)
  {
    if(msgit.type == "blue_puck" && color == "blue" || msgit.type == "yellow_puck" && color == "blue" )
    {
       puck_there = true;
       if(msg_puck.point.z > msgit.point.point.z)
            msg_puck.point = msgit.point.point;
    }

  }
  //publish if a puck is there
  if(puck_there)
  {   try
      {
        geometry_msgs::PointStamped puck_baselink;
       tf_listener_.transformPoint("/base_link", ros::Time(0), msg_puck, "/camera_rgb_optical_frame", puck_baselink);
        pub_closest_puck_.publish(puck_baselink);
      }
      catch (tf::TransformException ex)
      {
              ROS_INFO("transform didn't work (in tfpublish_frames) (maybe cause there no timestamping in point_in?) \n");
              ROS_ERROR("%s", ex.what());
              ros::Duration(0.1).sleep();
      }
  }

}


//********************************************************
//Function to calculate hsv values
//********************************************************
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



//********************************************************
//********************************************************
//Main function
//********************************************************
//********************************************************
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "object_detection");
  ros::NodeHandle nh;
  ROS_INFO("NODE detection runs");
  C3dDetection detect(nh);

  //init parameter server
  dynamic_reconfigure::Server<wachl_3d_object_detection::detectionConfig> server;
  dynamic_reconfigure::Server<wachl_3d_object_detection::detectionConfig>::CallbackType f;

  f = boost::bind(&C3dDetection::paramCallback,&detect, _1, _2);
  server.setCallback(f);

  while (ros::ok())
     ros::spin ();

   return 0;
}
