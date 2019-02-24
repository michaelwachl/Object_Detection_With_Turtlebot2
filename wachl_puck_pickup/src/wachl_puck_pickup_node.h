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
 * This class subscribes to the rgb camera stream and publishes if a puck is
 * picked up and the pixel distance to the camera center
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
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

//for dyn reconfig
#include <wachl_puck_pickup/pickupConfig.h>
#include <dynamic_reconfigure/server.h>

using namespace std;
using namespace cv;

class CPuckPickUp
{

public:
   explicit CPuckPickUp(ros::NodeHandle nh)
        : nh_(nh)
    {
        //initialize subscribers and publishers
         sub_ = nh_.subscribe("/camera/rgb/image_rect_color", 10, &CPuckPickUp::imageCallback,this);
         pub_pickedup_ = nh_.advertise<std_msgs::Bool>("puck_pickedup", 1);
	 pub_puckpos_ = nh_.advertise<std_msgs::Float32>("puck_pos_to_center", 1);
         pub_p_thres_ = nh_.advertise<sensor_msgs::Image>("image_pole_thres", 1);
         pub_b_thres_ = nh_.advertise<sensor_msgs::Image>("image_bottom_thres", 1);

         pub_puck_bin_ = nh_.advertise<sensor_msgs::Image>("image_binary", 1);
         pub_puck_detect_ = nh_.advertise<sensor_msgs::Image>("image_detect", 1);

    }
public:
    //variables
    Mat im_, disp_im_,hsv_im_ ,hsv_p_im_,hsv_b_im_,cropped_pole_im_,cropped_bott_im_,thres_im_b_,thres_im_y_;
    //ros variables
    ros::NodeHandle nh_;
    ros::Publisher pub_pickedup_, pub_puckpos_, pub_p_thres_, pub_b_thres_,pub_puck_bin_,pub_puck_detect_;
    ros::Subscriber sub_;

    // dyn params
    bool CALIBRATION_MODE_, DETECT_;
    int H_MIN_, H_MAX_, S_MIN_, S_MAX_,V_MIN_ ,V_MAX_,
        Y_H_MIN_, Y_H_MAX_, Y_S_MIN_, Y_S_MAX_,Y_V_MIN_ ,Y_V_MAX_,
        B_H_MIN_, B_H_MAX_, B_S_MIN_, B_S_MAX_,B_V_MIN_ ,B_V_MAX_;

    int POLE_TOP_X_, POLE_TOP_Y_, POLE_WIDTH_, POLE_HEIGHT_, BOTTOM_TOP_X_, BOTTOM_TOP_Y_, BOTTOM_WIDTH_, BOTTOM_HEIGHT_;
    int TEAM_COLOR_;
    float THRES_AREA_;
    double CAMERA_CENTER_OFFSET_;
    //max detected objects, parameters
    int MAX_NUM_OBJECTS_ = 30;//
    //minimum and maximum object area
    int MIN_OBJECT_AREA_ = 20*30;
    int MAX_OBJECT_AREA_ = 640*480*0.8;

  //for video stream
  void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg);
  void paramCallback(wachl_puck_pickup::pickupConfig &config, uint32_t level);
  void trackFilteredObject(Mat &thresh, int i);
  void drawObject(Rect &boundrect,int i, double x, double y);
  void filterAndTrack();
  void publishFrames();

private:
  //function
  void morphOps(Mat &thresh);


};//End Definition
