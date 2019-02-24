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
 * This class is used to subscribe to the lidar scan publish the
 * nearest puck constantly.
------------------------------------------------------------------------------*/

//header
#include "wachl_puck_lidar_node.h"


//********************************************************
// callback function to receive laser scan
//********************************************************
void CPuckPickLaser::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PCLPointCloud2 point_cloud2;

  try
  {
      projector_.transformLaserScanToPointCloud("base_link",*scan_in, cloud_msg,listener_);

      //get current time
      geometry_msgs::PointStamped msg_puck;
      msg_puck.header = cloud_msg.header;

       // Define containers
       pcl::PointCloud<PointT>::Ptr ptr_cloud(new pcl::PointCloud<PointT>); //PointCloud-XYZ's
       pcl::PointCloud<PointT>::Ptr ptr_cloud_filtered(new pcl::PointCloud<PointT>);
       pcl::PCLPointCloud2 point_cloud2;
       pcl::PassThrough<PointT> pass;
       pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
       std::vector<pcl::PointIndices>::const_iterator it;
       std::vector<pcl::PointIndices> cluster_indices;
       pcl::EuclideanClusterExtraction<PointT> ec;

       // Convert msg to PCL data type
       pcl_conversions::toPCL(cloud_msg, point_cloud2);
       //Convert Could2 to Cloud for additional filtering
       pcl::fromPCLPointCloud2(point_cloud2, *ptr_cloud);

       //Convert Could2 to Cloud for additional filtering
       pcl::fromPCLPointCloud2(point_cloud2, *ptr_cloud);

       //Do passthrough cutoff "x" filtering
       pass.setInputCloud(ptr_cloud);
       pass.setFilterFieldName ("x");
       pass.setFilterLimits(0, SCAN_DEPTH_);
       pass.filter(*ptr_cloud_filtered);

       // Build a passthrough filter to cutoff "y"
       pass.setInputCloud(ptr_cloud_filtered);
       pass.setFilterFieldName("y");
       pass.setFilterLimits(-SCAN_WIDTH_, SCAN_WIDTH_); //check values in Matlab
       //pass.setFilterLimitsNegative (true);
       pass.filter(*ptr_cloud_filtered );

       //#####################################################################################
       //Do Clustering with Kd-Tree search to extract objects
       //#####################################################################################
       tree->setInputCloud(ptr_cloud_filtered);
       ec.setClusterTolerance(0.03); // 3cm
       ec.setMinClusterSize(3); //100
       ec.setMaxClusterSize(15);
       ec.setSearchMethod(tree);
       ec.setInputCloud(ptr_cloud_filtered);
       ec.extract(cluster_indices);

       std::cout<<"PointCloud has: "<< cluster_indices.size() <<" clusters"<<std::endl;

       std::vector<geometry_msgs::PointStamped> cluster_points;
       std::vector<float> dists;
       geometry_msgs::PointStamped cluster_point;
        for (const auto it : cluster_indices)
        {
                //extract pointclouds
                pcl::ExtractIndices<PointT> extract;
                extract.setInputCloud (ptr_cloud_filtered);
                extract.setIndices (boost::make_shared<const pcl::PointIndices> (it));
                pcl::PointCloud<PointT>::Ptr ptr_currentclustercloud (new pcl::PointCloud<PointT>);
                extract.filter(*ptr_currentclustercloud);

                //compute centroid
                Eigen::Vector4f centroid;
                const pcl::PointCloud<PointT> cone_cloud = *ptr_currentclustercloud;

                if(pcl::compute3DCentroid(cone_cloud, centroid) != 0)
                {
                  cluster_point.point.x = centroid(0,0);
                  cluster_point.point.y = centroid(1,0);
                  cluster_point.point.z = centroid(2,0);
                  cluster_points.push_back(cluster_point);
                  dists.push_back(pow(centroid(0,0),2.0)+pow(centroid(1,0),2.0));
                }    
       }

       if(dists.size()>0)
       {
         //find shortest dist
         int min_index = std::min_element(dists.begin(), dists.end()) - dists.begin();

         //publish shortest
         msg_puck.point = cluster_points.at(min_index).point;
         //publish
         pub_pose_.publish(msg_puck);
       }

   }
  catch (tf::TransformException& e)
  {
      std::cout << e.what();
      return;
  }
}


//********************************************************
// callback function to change parameter
//********************************************************
void CPuckPickLaser::paramCallback(wachl_puck_lidar::pickup_lidarConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure requested");

  //Update Parameter
  DETECT_ = config.DETECT_;
  SCAN_WIDTH_ = config.SCAN_WIDTH_;
  SCAN_DEPTH_ = config.SCAN_DEPTH_;
}



//********************************************************
//********************************************************
// main function
//********************************************************
//********************************************************
int main(int argc, char **argv)
{
  ros::init(argc, argv, "puck_pick_laser");
  ros::NodeHandle nh;
  CPuckPickLaser detect(nh);

  //init parameter server
  dynamic_reconfigure::Server<wachl_puck_lidar::pickup_lidarConfig> server;
  dynamic_reconfigure::Server<wachl_puck_lidar::pickup_lidarConfig>::CallbackType f;

  f = boost::bind(&CPuckPickLaser::paramCallback,&detect, _1, _2);
  server.setCallback(f);

  //Ros loop
  while (ros::ok())
     ros::spin ();

 return 0;
}
