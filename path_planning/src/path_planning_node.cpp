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
 * This class is used to provide services for the path planning
------------------------------------------------------------------------------*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include "path_planning/service_path.h"

#include "SetPath/set_path.h"

using namespace std;
using namespace cv;

class CPathPlanning
{
public:
  CPathPlanning(ros::NodeHandle);

private:
 //service
 bool service_path(path_planning::service_pathRequest &req, path_planning::service_pathResponse &res);
 //variables
 geometry_msgs::PoseArray vectorToPose(vector<Point>);
 ros::NodeHandle nh_;
 ros::ServiceServer service_path_;
};
//end class declaration


//constructor
CPathPlanning::CPathPlanning(ros::NodeHandle nh):
        nh_(nh)
{
    service_path_ = nh_.advertiseService("calculate_path", &CPathPlanning::service_path, this); //boost::bind(&CPathPlanning::service_path, &path, _1, _2)
}



//********************************************************
// service fuction to provide several options for goal
// and calculate path
//********************************************************
bool CPathPlanning::service_path(path_planning::service_pathRequest &req, path_planning::service_pathResponse &res)
 {
   ROS_INFO("Path service in use");

   int PUCK_RADIUS = req.PUCK_RADIUS;
   int ROBOT_RADIUS = req.ROBOT_RADIUS;
   int POST_RADIUS = req.POST_RADIUS;
   float a = req.a;
   float b = req.b;
   std::string goal = req.goal;
   geometry_msgs::PoseArray array;


   if(PUCK_RADIUS>0 && ROBOT_RADIUS>0 && POST_RADIUS>0 && a>0 && b>0)
   {
        if(goal=="to_goal")
        {
            cout<<"to_goal"<<endl;
            vector<Point> pathVectorPoint;
            SetPath path(PUCK_RADIUS,ROBOT_RADIUS,POST_RADIUS,a,b);
            path.get_all_objects();
            pathVectorPoint = path.set_start_goal_ourpucks_enemypucks_enemyrobot_post(path.current_position_,
                                                                                      path.current_goal_to_attack_, path.current_our_pucks_,
                                                                                      path.current_oppo_pucks_, path.current_turtles_,
                                                                                      path.current_posts_);
            array = vectorToPose(pathVectorPoint);
            res.path = array;
            res.success = true;
            return true;


        }
        if(goal=="in_goal_left")
        {
            cout<<"in_goal_left"<<endl;
            vector<Point> pathVectorPoint;
            SetPath path(PUCK_RADIUS,ROBOT_RADIUS,POST_RADIUS,a,b);
            path.get_all_objects();
            cv::Point left_in_goal;
            left_in_goal.x = path.current_goal_to_attack_.x;
            left_in_goal.y = path.current_goal_to_attack_.y + 16;
            pathVectorPoint = path.set_start_goal_ourpucks_enemypucks_enemyrobot_post(path.current_position_,
                                                                                      left_in_goal, path.current_our_pucks_,
                                                                                      path.current_oppo_pucks_, path.current_turtles_,
                                                                                      path.current_posts_);
            array = vectorToPose(pathVectorPoint);
            res.path = array;
            res.success = true;
            return true;
        }
        if(goal=="in_goal_right")
        {
            cout<<"in_goal_right"<<endl;
            vector<Point> pathVectorPoint;
            SetPath path(PUCK_RADIUS,ROBOT_RADIUS,POST_RADIUS,a,b);
            path.get_all_objects();
            cv::Point right_in_goal;
            right_in_goal.x = path.current_goal_to_attack_.x;
            right_in_goal.y = path.current_goal_to_attack_.y - 16;
            pathVectorPoint = path.set_start_goal_ourpucks_enemypucks_enemyrobot_post(path.current_position_,
                                                                                      right_in_goal, path.current_our_pucks_,
                                                                                      path.current_oppo_pucks_, path.current_turtles_,
                                                                                      path.current_posts_);
            array = vectorToPose(pathVectorPoint);
            res.path = array;
            res.success = true;
            return true;
        }
        if(goal=="to_goal_left")
        {
            cout<<"to_goal_left"<<endl;
            vector<Point> pathVectorPoint;
            SetPath path(PUCK_RADIUS,ROBOT_RADIUS,POST_RADIUS,a,b);
            path.get_all_objects();
            cv::Point left_goal;
            left_goal.x = path.current_goal_to_attack_.x + 80;
            left_goal.y = path.current_goal_to_attack_.y - 50;
            pathVectorPoint = path.set_start_goal_ourpucks_enemypucks_enemyrobot_post(path.current_position_,
                                                                                      left_goal, path.current_our_pucks_,
                                                                                      path.current_oppo_pucks_, path.current_turtles_,
                                                                                      path.current_posts_);
            array = vectorToPose(pathVectorPoint);
            res.path = array;
            res.success = true;
            return true;
        }
        if(goal=="to_goal_right")
        {
            cout<<"to_goal_right"<<endl;
            vector<Point> pathVectorPoint;
            SetPath path(PUCK_RADIUS,ROBOT_RADIUS,POST_RADIUS,a,b);
            path.get_all_objects();
            cv::Point right_goal;
            right_goal.x = path.current_goal_to_attack_.x + 80;
            right_goal.y = path.current_goal_to_attack_.y - 50;
            pathVectorPoint = path.set_start_goal_ourpucks_enemypucks_enemyrobot_post(path.current_position_,
                                                                                      right_goal, path.current_our_pucks_,
                                                                                      path.current_oppo_pucks_, path.current_turtles_,
                                                                                      path.current_posts_);
            array = vectorToPose(pathVectorPoint);
            res.path = array;
            res.success = true;
            return true;
        }
        else if(goal=="to_puck_0")
        {
            cout<<"to_puck_0"<<endl;
            vector<Point> pathVectorPoint;
            SetPath path(PUCK_RADIUS,ROBOT_RADIUS,POST_RADIUS,a,b);
            path.get_all_objects();
            if(path.current_our_pucks_.size()>0)
            {
                cv::Point puck_0;
                puck_0.x = path.current_our_pucks_.at(0).x;
                puck_0.y = path.current_our_pucks_.at(0).y;
                path.current_our_pucks_.erase(path.current_our_pucks_.begin());
                pathVectorPoint = path.set_start_goal_ourpucks_enemypucks_enemyrobot_post(path.current_position_,
                                                                                      puck_0, path.current_our_pucks_,
                                                                                      path.current_oppo_pucks_, path.current_turtles_,
                                                                                      path.current_posts_);
                array = vectorToPose(pathVectorPoint);
                res.path = array;
                res.success = true;
                return true;
            }
            else
            {
                res.path = array;
                res.success = false;
                return false;
            }
        }
        else if(goal=="to_puck_1")
        {
            cout<<"to_puck_1"<<endl;
            vector<Point> pathVectorPoint;
            SetPath path(PUCK_RADIUS,ROBOT_RADIUS,POST_RADIUS,a,b);
            path.get_all_objects();
            if(path.current_our_pucks_.size()>1)
            {
                cv::Point puck_1;
                puck_1.x = path.current_our_pucks_.at(1).x;
                puck_1.y = path.current_our_pucks_.at(1).y;
                path.current_our_pucks_.erase(path.current_our_pucks_.begin()+1);
                pathVectorPoint = path.set_start_goal_ourpucks_enemypucks_enemyrobot_post(path.current_position_,
                                                                                      puck_1, path.current_our_pucks_,
                                                                                      path.current_oppo_pucks_, path.current_turtles_,
                                                                                      path.current_posts_);
                array = vectorToPose(pathVectorPoint);
                res.path = array;
                res.success = true;
                return true;
            }
            else
            {
                res.path = array;
                res.success = false;
                return false;
            }
        }
        else if(goal=="to_puck_2")
        {
            cout<<"to_puck_2"<<endl;
            vector<Point> pathVectorPoint;
            SetPath path(PUCK_RADIUS,ROBOT_RADIUS,POST_RADIUS,a,b);
            path.get_all_objects();
            if(path.current_our_pucks_.size()>2)
            {
                cv::Point puck_2;
                puck_2.x = path.current_our_pucks_.at(2).x;
                puck_2.y = path.current_our_pucks_.at(2).y;
                path.current_our_pucks_.erase(path.current_our_pucks_.begin()+2);
                pathVectorPoint = path.set_start_goal_ourpucks_enemypucks_enemyrobot_post(path.current_position_,
                                                                                      puck_2, path.current_our_pucks_,
                                                                                      path.current_oppo_pucks_, path.current_turtles_,
                                                                                      path.current_posts_);
                array = vectorToPose(pathVectorPoint);
                res.path = array;
                res.success = true;
                return true;
            }
            else
            {
                res.path = array;
                res.success = false;
                return false;
            }
        }

   }
   else
   {
     res.path = array;
     res.success = false;
     return false;
   }

 }


//********************************************************
// function to transform vector to pose array
//********************************************************
geometry_msgs::PoseArray CPathPlanning::vectorToPose(vector<Point> vec)
{
  geometry_msgs::PoseArray pa;
  geometry_msgs::Pose p;

  for(auto it : vec)
  {
    p.position.x = it.x;
    p.position.y = it.y;
    pa.poses.push_back(p);

  }

  return  pa;
}


//********************************************************
//********************************************************
// main function
//********************************************************
//********************************************************
int main(int argc, char **argv)
{

ros::init(argc, argv, "path_planning");

ros::NodeHandle nh;

CPathPlanning path(nh);

ROS_INFO("Services ready.");
ros::spin();

 return 0;
}
