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
#include "std_msgs/Bool.h"


using namespace std;
using namespace cv;

class CPuckPickedUp
{

public:
   explicit CPuckPickedUp(ros::NodeHandle nh)
        : nh_(nh)
    {
         sub_ = nh_.subscribe("/camera/rgb/image_rect_color", 10, &CPuckPickedUp::imageCallback,this);
         pub_ = nh_.advertise<std_msgs::Bool>("puck_pickedup", 1);
         pub_y_thres_ = nh_.advertise<sensor_msgs::Image>("image_yellow_thres", 1);
         pub_b_thres_ = nh_.advertise<sensor_msgs::Image>("image_blue_thres", 1);


          //Get parameter from param server
         nh.getParam("/puck_pickedup/H_MIN_", H_MIN_);
         nh.getParam("/puck_pickedup/H_MAX_", H_MAX_);
         nh.getParam("/puck_pickedup/S_MIN_", S_MIN_);
         nh.getParam("/puck_pickedup/S_MAX_", S_MAX_);
         nh.getParam("/puck_pickedup/V_MIN_", V_MIN_);
         nh.getParam("/puck_pickedup/V_MAX_", V_MAX_);
         nh.getParam("/puck_pickedup/Y_H_MIN_", Y_H_MIN_);
         nh.getParam("/puck_pickedup/Y_H_MAX_", Y_H_MAX_);
         nh.getParam("/puck_pickedup/Y_S_MIN_", Y_S_MIN_);
         nh.getParam("/puck_pickedup/Y_S_MAX_", Y_S_MAX_);
         nh.getParam("/puck_pickedup/Y_V_MIN_", Y_V_MIN_);
         nh.getParam("/puck_pickedup/Y_V_MAX_", Y_V_MAX_);
         nh.getParam("/puck_pickedup/B_H_MIN_", B_H_MIN_);
         nh.getParam("/puck_pickedup/B_H_MAX_", B_H_MAX_);
         nh.getParam("/puck_pickedup/B_S_MIN_", B_S_MIN_);
         nh.getParam("/puck_pickedup/B_S_MAX_", B_S_MAX_);
         nh.getParam("/puck_pickedup/B_V_MIN_", B_V_MIN_);
         nh.getParam("/puck_pickedup/B_V_MAX_", B_V_MAX_);
         nh.getParam("/puck_pickedup/THRES_AREA_", THRES_AREA_);
         nh.getParam("/puck_pickedup/CALIBRATION_MODE_", CALIBRATION_MODE_);


    }
public:
    Mat im_, disp_im_,hsv_p_im_,hsv_b_im_,cropped_pole_im_,cropped_bott_im_,thres_im_b_p_,thres_im_b_b_,thres_im_y_p_,thres_im_y_b_;

    //variables
    ros::NodeHandle nh_;
    ros::Publisher pub_, pub_y_thres_, pub_b_thres_;

    ros::Subscriber sub_;

    const string btrackbar_window_name_ = "Blue Trackbars";
    const string ytrackbar_window_name_ = "Yellow Trackbars";
    bool CALIBRATION_MODE_;
    int H_MIN_, H_MAX_, S_MIN_, S_MAX_,V_MIN_ ,V_MAX_,
        Y_H_MIN_, Y_H_MAX_, Y_S_MIN_, Y_S_MAX_,Y_V_MIN_ ,Y_V_MAX_,
        B_H_MIN_, B_H_MAX_, B_S_MIN_, B_S_MAX_,B_V_MIN_ ,B_V_MAX_;
    float THRES_AREA_;

  //for video stream
  void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg);
  void createTrackbars(string color);






};//End Definition


void onTrackbar(int,void*)
{

}



void CPuckPickedUp::imageCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{
  // Convert image.
  cv_bridge::CvImageConstPtr image_ptr;
  try {
    image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
    im_ = image_ptr->image; 
    im_.copyTo(disp_im_);
  }
  catch (const cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge::Exception when converting image-msg: "
                     << e.what());
    return;
  }
}


void CPuckPickedUp::createTrackbars(string trackbar_window_name)
{
    namedWindow(trackbar_window_name,0);
    //memory to store trackbarname
    char trackbar_name[50];
    sprintf(trackbar_name, "h_min");
    sprintf(trackbar_name, "h_max");
    sprintf(trackbar_name, "s_min");
    sprintf(trackbar_name, "s_max");
    sprintf(trackbar_name, "v_min");
    sprintf(trackbar_name, "v_max");

 //Yellow
  if(trackbar_window_name == ytrackbar_window_name_)
  {
      //bars to change values
      createTrackbar("h_min", trackbar_window_name, &Y_H_MIN_, H_MAX_, onTrackbar);
      createTrackbar("h_max", trackbar_window_name, &Y_H_MAX_, H_MAX_, onTrackbar);
      createTrackbar("s_min", trackbar_window_name, &Y_S_MIN_, S_MAX_, onTrackbar);
      createTrackbar("s_max", trackbar_window_name, &Y_S_MAX_, S_MAX_, onTrackbar);
      createTrackbar("v_min", trackbar_window_name, &Y_V_MIN_, V_MAX_, onTrackbar);
      createTrackbar("v_max", trackbar_window_name, &Y_V_MAX_, V_MAX_, onTrackbar);
  }

  //Blue
   if(trackbar_window_name == btrackbar_window_name_)
   {
       //bars to change values
       createTrackbar("h_min", trackbar_window_name, &B_H_MIN_, H_MAX_, onTrackbar);
       createTrackbar("h_max", trackbar_window_name, &B_H_MAX_, H_MAX_, onTrackbar);
       createTrackbar("s_min", trackbar_window_name, &B_S_MIN_, S_MAX_, onTrackbar);
       createTrackbar("s_max", trackbar_window_name, &B_S_MAX_, S_MAX_, onTrackbar);
       createTrackbar("v_min", trackbar_window_name, &B_V_MIN_, V_MAX_, onTrackbar);
       createTrackbar("v_max", trackbar_window_name, &B_V_MAX_, V_MAX_, onTrackbar);
   }
}



//
int main(int argc, char **argv)
{
ros::init(argc, argv, "puck_pickedup");
  ros::NodeHandle nh;
CPuckPickedUp detect(nh);


//create trackbars
if(detect.CALIBRATION_MODE_)
{
detect.createTrackbars(detect.btrackbar_window_name_);
detect.createTrackbars(detect.ytrackbar_window_name_);
}

//get image in color and check it
ros::Rate loop_rate(1);
//Ros loop
while (ros::ok())
{

if(!detect.im_.empty())
{
   //crop image
    //cropped_pole_im_,cropped_bott_im_
   detect.cropped_pole_im_ = detect.im_(Rect(300, 0, 200, 390));
   detect.cropped_bott_im_ = detect.im_(Rect(130, 330, 500, 150));
   //convert frame from BGR to HSV colorspace
   //cropped_pole_im_,cropped_bott_im_
   cvtColor(detect.cropped_pole_im_,detect.hsv_p_im_,COLOR_BGR2HSV);
   cvtColor(detect.cropped_bott_im_,detect.hsv_b_im_,COLOR_BGR2HSV);

   // Detect the object based on HSV Range Values
   //first find blue objects
   inRange(detect.hsv_b_im_,Scalar(detect.B_H_MIN_,detect.B_S_MIN_,detect.B_V_MIN_),Scalar(detect.B_H_MAX_,detect.B_S_MAX_,detect.B_V_MAX_),detect.thres_im_b_b_);
   inRange(detect.hsv_p_im_,Scalar(detect.B_H_MIN_,detect.B_S_MIN_,detect.B_V_MIN_),Scalar(detect.B_H_MAX_,detect.B_S_MAX_,detect.B_V_MAX_),detect.thres_im_b_p_);
   //then yellow
   inRange(detect.hsv_b_im_,Scalar(detect.Y_H_MIN_,detect.Y_S_MIN_,detect.Y_V_MIN_),Scalar(detect.Y_H_MAX_,detect.Y_S_MAX_,detect.Y_V_MAX_),detect.thres_im_y_b_);
   inRange(detect.hsv_p_im_,Scalar(detect.Y_H_MIN_,detect.Y_S_MIN_,detect.Y_V_MIN_),Scalar(detect.Y_H_MAX_,detect.Y_S_MAX_,detect.Y_V_MAX_),detect.thres_im_y_p_);

   //show Images
   if(detect.CALIBRATION_MODE_)
   {
   //imshow(detect.window_orig_,detect.im_);

   //Publish the frames
   cv_bridge::CvImage cvim_b, cvim_y;
   cvim_b.image = detect.thres_im_b_b_;
   cvim_y.image = detect.thres_im_y_b_;
   cvim_b.encoding = "mono8";
   cvim_y.encoding = "mono8";
   sensor_msgs::Image ros_image_b, ros_image_y;
   cvim_b.toImageMsg(ros_image_b);
   cvim_y.toImageMsg(ros_image_y);
   imshow("detect blue" ,detect.thres_im_b_b_);
   imshow("detect yellow"  ,detect.thres_im_y_b_);
   detect.pub_y_thres_.publish(ros_image_y);
   detect.pub_b_thres_.publish(ros_image_b);
   }

   //check binary images
   int size_bott = 500*150;
   int size_pole = 200*390;
   int blue_bott = cv::countNonZero(detect.thres_im_b_b_);
   int blue_pole = cv::countNonZero(detect.thres_im_b_p_);
   int yellow_bott = cv::countNonZero(detect.thres_im_y_b_);
   int yellow_pole = cv::countNonZero(detect.thres_im_y_p_);

   //cout<<"Count blue bott ratio: "<<(float)blue_bott/size_bott<<"Count blue pole ratio: "<<(float)blue_pole/size_pole<<std::endl;
   //cout<<"Count yellow bott ratio: "<<(float)yellow_bott/size_bott<<"Count yellow pole ratio: "<<(float)yellow_pole/size_pole<<std::endl;


   std_msgs::Bool msg;
   //publish
   if(((float)blue_bott/size_bott>=detect.THRES_AREA_ && (float)blue_pole/size_pole>detect.THRES_AREA_) ||((float)yellow_bott/size_bott>=detect.THRES_AREA_ && (float)yellow_pole/size_pole>=detect.THRES_AREA_))
   {
       msg.data= true;
       ROS_INFO("Puck picked up");
   }
   else
   {
       msg.data= false;
       ROS_INFO("Puck NOT picked up");
   }

   detect.pub_.publish(msg);
   //cout<<"Puck pickedup: "<<msg.data<<std::endl;

   //wait to refresh
   cvWaitKey(30);
}
else
 ROS_INFO("Image empty");

 ros::spinOnce();
 loop_rate.sleep();
}
 return 0;
}
