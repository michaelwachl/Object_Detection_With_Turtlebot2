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

//header
#include "wachl_puck_pickup_node.h"





//*************************************************************************
// comparison function object
//*************************************************************************
bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i < j );
}

//*************************************************************************
// callback function for image stream
//*************************************************************************
void CPuckPickUp::imageCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{
  // Convert image
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


//*************************************************************************
// callback function to change parameter
//*************************************************************************
void CPuckPickUp::paramCallback(wachl_puck_pickup::pickupConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure requested");

  //Update Parameter
  Y_H_MIN_ = config.Y_H_MIN_;
  Y_H_MAX_ = config.Y_H_MAX_;
  Y_S_MIN_ = config.Y_S_MIN_;
  Y_S_MAX_ = config.Y_S_MAX_;
  Y_V_MIN_ = config.Y_V_MIN_;
  Y_V_MAX_ = config.Y_V_MAX_;
  B_H_MIN_ = config.B_H_MIN_;
  B_H_MAX_ = config.B_H_MAX_;
  B_S_MIN_ = config.B_S_MIN_;
  B_S_MAX_ = config.B_S_MAX_;
  B_V_MIN_ = config.B_V_MIN_;
  B_V_MAX_ = config.B_V_MAX_;
  POLE_TOP_X_ = config.POLE_TOP_X_;
  POLE_TOP_Y_ = config.POLE_TOP_Y_;
  POLE_WIDTH_= config.POLE_WIDTH_;
  POLE_HEIGHT_ = config.POLE_HEIGHT_;
  BOTTOM_TOP_X_ = config.BOTTOM_TOP_X_;
  BOTTOM_TOP_Y_ = config.BOTTOM_TOP_Y_;
  BOTTOM_WIDTH_ = config.BOTTOM_WIDTH_;
  BOTTOM_HEIGHT_ = config.BOTTOM_HEIGHT_;
  THRES_AREA_ = config.THRES_AREA_;
  CALIBRATION_MODE_ = config.CALIBRATION_MODE_;
  //puck tracking
  DETECT_ = config.DETECT_;
  TEAM_COLOR_ = config.TEAM_COLOR_;
  THRES_AREA_= config.THRES_AREA_;
  CAMERA_CENTER_OFFSET_= config.CAMERA_CENTER_OFFSET_;
  MAX_NUM_OBJECTS_= config.MAX_NUM_OBJECTS_;
  MIN_OBJECT_AREA_= config.MIN_OBJECT_AREA_;
  MAX_OBJECT_AREA_= config.MAX_OBJECT_AREA_;

}

//*************************************************************************
// function to track filterd objects
//*************************************************************************
void CPuckPickUp::trackFilteredObject(Mat &thresh, int i)
{
 int x,y;
 Mat temp;
 thresh.copyTo(temp);
 //these two vectors needed for output of findContours
 vector< vector<Point> > contours;
 vector<Vec4i> hierarchy;
 //find contours of filtered image using openCV findContours function
 findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);

 if (hierarchy.size() > 0)
 {
     int numObjects = hierarchy.size();

     //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
     if(numObjects<MAX_NUM_OBJECTS_)
     {
       // sort contours
       std::sort(contours.begin(), contours.end(), compareContourAreas);

       //loop through contours, big to small
       for (int index = contours.size()-1; index >= 0; index--)
       {
          //calculate moments
          Moments moment = moments((cv::Mat)contours[index]);
          double area = moment.m00;

         if(area>MIN_OBJECT_AREA_ && area<MAX_OBJECT_AREA_)
         {
            //calculate coordinates
            x = moment.m10/area;
            y = moment.m01/area;
            //calculate bounding rect
            Rect boundrect = boundingRect(contours[index]);
            if (boundrect.height > boundrect.width)
            {
                drawObject(boundrect,i, x ,y);
                //calculate puck position to frame center
                float puck_pose = x - 320 + CAMERA_CENTER_OFFSET_;
                cout<<"Center dist in px: "<<puck_pose<<endl;
                //publish pose
                std_msgs::Float32 msg;
                msg.data= puck_pose;
                pub_puckpos_.publish(msg);

                break;
            }
         }
       }
      }
     else
         putText(im_,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
  }
}


//*************************************************************************
// function to draw bounding rectange
//*************************************************************************
void CPuckPickUp::drawObject(Rect &boundrect,int i, double x, double y)
{
 if (i==1)
    rectangle(disp_im_,Point(boundrect.x,boundrect.y),Point(boundrect.x+boundrect.width,boundrect.y+boundrect.height), Scalar(0,200,200), 2); //yellow
 else if (i==0)
    rectangle(disp_im_,Point(boundrect.x,boundrect.y),Point(boundrect.x+boundrect.width,boundrect.y+boundrect.height), Scalar(200,0,0), 2); //blue

line(disp_im_, Point(x,y-30), Point(x,y+30), Scalar(100,100,100), 2, 8, 0);
}

void CPuckPickUp::morphOps(Mat &thresh)
{
        //create structuring element that will be used to "dilate" and "erode" image.
        //the element chosen here is a 3px by 3px rectangle
        Mat erodeElement = getStructuringElement( MORPH_RECT,Size(5,2));
        //dilate with larger element so make sure object is nicely visible
        Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

        //erode(thresh,thresh,erodeElement);
        erode(thresh,thresh,erodeElement);

        //dilate(thresh,thresh,dilateElement);
        dilate(thresh,thresh,dilateElement);
}


//*************************************************************************
// function to filter objects
//*************************************************************************
void CPuckPickUp::filterAndTrack()
{
    //convert frame from BGR to HSV colorspace
    cvtColor(im_,hsv_im_,COLOR_BGR2HSV);

   // Detect the object based on HSV Range Values
    //yellow
   if(TEAM_COLOR_ == 1)
   {
       inRange(hsv_im_,Scalar(Y_H_MIN_,Y_S_MIN_,Y_V_MIN_),Scalar(Y_H_MAX_,Y_S_MAX_,Y_V_MAX_),thres_im_y_);
       //Morph operation
       morphOps(thres_im_y_);
       //cut top and bottom frames
       cropped_pole_im_ = thres_im_y_(Rect(POLE_TOP_X_, POLE_TOP_Y_, POLE_WIDTH_, POLE_HEIGHT_));
       cropped_bott_im_ = thres_im_y_(Rect(BOTTOM_TOP_X_, BOTTOM_TOP_Y_, BOTTOM_WIDTH_, BOTTOM_HEIGHT_));
       //Track puck
       trackFilteredObject(thres_im_y_,1);
   }
   //blue
   else if(TEAM_COLOR_ == 0)
   {
       inRange(hsv_im_,Scalar(B_H_MIN_,B_S_MIN_,B_V_MIN_),Scalar(B_H_MAX_,B_S_MAX_,B_V_MAX_),thres_im_b_);
       //Morph operation
       morphOps(thres_im_b_);
       //cut top and bottom frames
       cropped_pole_im_ = thres_im_b_(Rect(POLE_TOP_X_, POLE_TOP_Y_, POLE_WIDTH_, POLE_HEIGHT_));
       cropped_bott_im_ = thres_im_b_(Rect(BOTTOM_TOP_X_, BOTTOM_TOP_Y_, BOTTOM_WIDTH_, BOTTOM_HEIGHT_));
       //Track puck
       trackFilteredObject(thres_im_b_,0);
   }

   // cout<<"Teamcolor: "<<TEAM_COLOR_<<endl;
}


//*************************************************************************
// function to publish several images
//*************************************************************************
void CPuckPickUp::publishFrames()
{
  cv_bridge::CvImage cvim_b, cvim_p, cvim_bin, cvim_detect;
  cvim_b.image = cropped_bott_im_;
  cvim_p.image = cropped_bott_im_;
  if (TEAM_COLOR_ == 1)
       cvim_bin.image = thres_im_y_;
  else if(TEAM_COLOR_ == 0)
       cvim_bin.image = thres_im_b_;
  cvim_detect.image = disp_im_;
  cvim_b.encoding = "mono8";
  cvim_p.encoding = "mono8";
  cvim_bin.encoding = "mono8";
  cvim_detect.encoding = "bgr8";
  sensor_msgs::Image ros_image_b, ros_image_p, ros_image_bin, ros_image_detect;
  cvim_b.toImageMsg(ros_image_b);
  cvim_p.toImageMsg(ros_image_p);
  cvim_bin.toImageMsg(ros_image_bin);
  cvim_detect.toImageMsg(ros_image_detect);

  if(TEAM_COLOR_ == 1)
  {
    imshow("Detect yellow bottom"  ,cropped_bott_im_);
    imshow("Detect yellow pole"  ,cropped_pole_im_);
      imshow("Binary image", thres_im_y_);
  }
  else if(TEAM_COLOR_ == 0)
  {
    imshow("Detect blue bottom" ,cropped_bott_im_);
    imshow("Detect blue pole" ,cropped_pole_im_);
    imshow("Binary image", thres_im_b_);
  }

  imshow("Tracked objects"  ,disp_im_);
  pub_b_thres_.publish(ros_image_b);
  pub_p_thres_.publish(ros_image_p);
  pub_puck_bin_.publish(ros_image_bin);
  pub_puck_detect_.publish(ros_image_detect);
}




//*************************************************************************
/***************************************************************************
 * Main function
 ***************************************************************************/
//*************************************************************************
int main(int argc, char **argv)
{
ros::init(argc, argv, "puck_pickup");
  ros::NodeHandle nh;
CPuckPickUp detect(nh);

//init parameter server
dynamic_reconfigure::Server<wachl_puck_pickup::pickupConfig> server;
dynamic_reconfigure::Server<wachl_puck_pickup::pickupConfig>::CallbackType f;

f = boost::bind(&CPuckPickUp::paramCallback,&detect, _1, _2);
server.setCallback(f);

//get image in color and check it
ros::Rate loop_rate(4);
//Ros loop
while (ros::ok())
{
    if(!detect.im_.empty() && detect.DETECT_)
    {
     //calculate HSV, morph und track
     detect.filterAndTrack();             

     //Publish the frames
     if(detect.CALIBRATION_MODE_)
           detect.publishFrames();

     //check binary images
     int size_bott = detect.BOTTOM_WIDTH_*detect.BOTTOM_HEIGHT_;
     int size_pole = detect.POLE_WIDTH_*detect.POLE_HEIGHT_;

     int count_bott = cv::countNonZero(detect.cropped_bott_im_);
     int count_pole = cv::countNonZero(detect.cropped_pole_im_);

     //cout<<"Count blue bott ratio: "<<(float)blue_bott/size_bott<<"Count blue pole ratio: "<<(float)blue_pole/size_pole<<std::endl;
     //cout<<"Count yellow bott ratio: "<<(float)yellow_bott/size_bott<<"Count yellow pole ratio: "<<(float)yellow_pole/size_pole<<std::endl;

     std_msgs::Bool msg;
     //publish
     if(((float)count_bott/size_bott>=detect.THRES_AREA_ && (float)count_pole/size_pole>detect.THRES_AREA_))
     {
       msg.data= true;
       ROS_INFO("Puck picked up");
     }
     else
     {
       msg.data= false;
       ROS_INFO("Puck NOT picked up");
     }

     detect.pub_pickedup_.publish(msg);

     //wait to refresh
     cvWaitKey(15); //30
    }
    else
        ROS_INFO("Image empty or waiting for pickup request");

 ros::spinOnce();
 loop_rate.sleep();
}
 return 0;
}
