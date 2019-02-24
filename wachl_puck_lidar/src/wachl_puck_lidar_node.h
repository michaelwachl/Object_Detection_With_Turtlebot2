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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/PointStamped.h"
#include <tf/transform_datatypes.h>

//to transform scan to pointcloud
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include <laser_geometry/laser_geometry.h>
#include "message_filters/subscriber.h"

//for pointclouds
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

//for dyn reconfig
#include <wachl_puck_lidar/pickup_lidarConfig.h>
#include <dynamic_reconfigure/server.h>

using namespace std;

typedef pcl::PointXYZ PointT;

class CPuckPickLaser
{

public:
   CPuckPickLaser(ros::NodeHandle nh):
        nh_(nh), 
        laser_sub_(nh_, "/scan_360", 10), 
        laser_notifier_(laser_sub_,listener_, "base_link", 10)
    {
      laser_notifier_.registerCallback(boost::bind(&CPuckPickLaser::scanCallback, this, _1));
      laser_notifier_.setTolerance(ros::Duration(0.01));

      pub_pose_ = nh_.advertise<geometry_msgs::PointStamped> ("closest_puck", 1);
    }

public:
  //callback function for dyn reconfigure
  void paramCallback(wachl_puck_lidar::pickup_lidarConfig &config, uint32_t level);

private:
  //Variables
  bool DETECT_;
  double  SCAN_WIDTH_,SCAN_DEPTH_;
  //Ros Tf variables
  ros::NodeHandle nh_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher pub_pose_;
  //callback function for laser scan
  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
};
//End Definition


