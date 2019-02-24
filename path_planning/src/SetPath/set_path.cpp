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
 * This class is used to set a path
------------------------------------------------------------------------------*/


#include "set_path.h"





//**********************************************
// function to clear the map and set the new coordinates.
// Then sets the output path to the global variable
//**********************************************
vector<Point> SetPath::set_start_goal_ourpucks_enemypucks_enemyrobot_post(Point start, Point goal, vector<Point> obstacles_our_pucks,
  vector<Point> obstacles_enemy_puck_, vector<Point> obstacles_enemy_robot_, vector<Point> post) {
    // Erase the previous cost map
    path.eraseObject();
    // Set the start and goal coordinates
    path.setStartandGoal(start.x, start.y, goal.x, goal.y);
    cout << "start x: " << start.x << "start y: " << start.y <<"  goal x: " << goal.x << " goal y: " << goal.y << endl;
    // Then set the coordinates of the obstacles
    // (x_coordi, y_coordi, radius of the obstacle)
    for (auto our_puck_i : obstacles_our_pucks) {
      path.setObstacle(our_puck_i.x, our_puck_i.y, puck_radius_);
      cout << "obstacle our puck x: " << our_puck_i.x << " obstacle our puck y: " <<  our_puck_i.y << endl;
    }
    for (auto enemy_puck_i : obstacles_enemy_puck_) {
      path.setObstacle(enemy_puck_i.x, enemy_puck_i.y, puck_radius_);
      cout << "obstacle enemy puck x: " << enemy_puck_i.x << " obstacle our puck y: " <<  enemy_puck_i.y << endl;
    }
    for (auto enemy_robot_i : obstacles_enemy_robot_) {
      path.setObstacle(enemy_robot_i.x, enemy_robot_i.y, robot_radius_);
      cout << "obstacle enemy robot x: " << enemy_robot_i.x << " obstacle our robot y: " <<  enemy_robot_i.y << endl;
    }
    for (auto post_i : post) {
      path.setObstacle(post_i.x, post_i.y, post_radius_);
      cout << "obstacle post x: " << post_i.x << " obstacle post y: " <<  post_i.y << endl;
    }

    try {
       if(path.replanPath()) {  // Plan the path
         // Save the vector with the path points inside the GLOBAL variable pathVectorPoint
         // Clear the last path (maybe unnecesary, but just to be sure that it works)
         pathVectorPoint_.clear();
         pathVectorPoint_ = path.getVectorPoint(); // Print the states
         for(auto state1 : pathVectorPoint_) {
           cout << "x: " << state1.x<< "     " << "y: " << state1.y << endl;
         }
         return pathVectorPoint_;
       }
       else
         throw "No path found!";
     }
     catch (tf::TransformException ex){
       ROS_ERROR("%s",ex.what());
       pathVectorPoint_.clear();
       pathVectorPoint_.push_back(Point(10,10)); // In case that the replanpath cannot be run, return to the center of the odometry
       return pathVectorPoint_;
     }
     catch (const char* e){
       ROS_ERROR("Other execption: %s",e);
       pathVectorPoint_.clear();
       pathVectorPoint_.push_back(Point(10,10)); // In case that the replanpath cannot be run, return to the center of the odometry
       return pathVectorPoint_;
     }
 }

//**********************************************
// function to transforms frames into map coordinates for path planning
//**********************************************
void SetPath::get_all_objects() {
  //clear all vectors
    current_our_pucks_.clear();
    current_oppo_pucks_.clear();
    current_posts_.clear();
    current_turtles_.clear();


   //define frames
  std::vector<std::string> object_frames{"/goal_to_attack",
                                         "/our_puck0", "/our_puck1", "/our_puck2", "/our_puck3", "/our_puck4","/our_puck5",
                                         "/oppo_puck0", "/oppo_puck1", "/oppo_puck2", "/oppo_puck3","/oppo_puck4","/oppo_puck5",
                                         "/post0","/post1","/post2","/post3","/post4","/post5","/post6","/post7","/post8","/post9","/post10",
                                         "/post11","/post12","/post13","/post14","/turtle0","/turtle1", "/turtle2", "/turtle3", "/base_link"};
  tf::StampedTransform transform;
  for(const auto it : object_frames)
   {
    try{
          cout<<"Iteration tf: "<<it<<endl;
          tf_listener_.waitForTransform("/map", "/base_link",
                                    ros::Time(0), ros::Duration(2.0));
          tf_listener_.lookupTransform("/map", it,
                              ros::Time(0), transform); // Mapped w.r.t the map frame

           if(it.find("goal_to_attack") != std::string::npos)  {
              cout << "goal_to_attack x " << transform.getOrigin().x() << " and y " << transform.getOrigin().y() << endl;
              cv::Point frame((int)(transform.getOrigin().x()*100),(int)(transform.getOrigin().y()*100));
              cout<<"gooal attack "<<frame.x<<"  "<<frame.y<<endl;
              if(frame.y>0 && frame.y<(int)b_*100 && frame.x >0 && frame.x<(int)300*a_)
              {
                cout<<"Goal to put puck in transformed: "<<frame<<endl;
                current_goal_to_attack_ = frame;
              }
          }

           else if (it.find("our_puck") != std::string::npos) {
               cout<<"our puck found!"<<endl;
               cv::Point frame((int)(transform.getOrigin().x()*100),(int)(transform.getOrigin().y()*100));
               if(frame.y>0 && frame.y<(int)b_*100 && frame.x >0 && frame.x<(int)300*a_) {
                  // if(sqrt(pow(current_goal_to_attack_.y - frame.y,2) + pow(current_goal_to_attack_.x - frame.x,2)) > 30 ) {  // If the puck is already near the enemy goal, don't add it
                     current_our_pucks_.push_back(frame); // We should also set current_our_pucks_  (WITHOUT current_nearest_puck_) AS OBSTACLES!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                     cout<<"Our puck transformed: "<<frame<<endl;
               }
           }

           else if(it.find("oppo_puck") != std::string::npos) {
               cv::Point frame((int)(transform.getOrigin().x()*100),(int)(transform.getOrigin().y()*100));
               if(frame.y>0 && frame.y<(int)b_*100 && frame.x >0 && frame.x<(int)300*a_)
               {
                 cout<<"Opp puck transformed: "<<frame<<endl;
                 current_oppo_pucks_.push_back(frame);
               }
           }

           else if(it.find("post") != std::string::npos)  {
               cv::Point frame((int)(transform.getOrigin().x()*100),(int)(transform.getOrigin().y()*100));
               cout<<"Post transformed: "<<frame<<endl;
               current_posts_.push_back(frame);

           }

           else if(it.find("turtle") != std::string::npos)  {
               cv::Point frame((int)(transform.getOrigin().x()*100),(int)(transform.getOrigin().y()*100));
               if(frame.y>0 && frame.y<(int)b_*100 && frame.x >0 && frame.x<(int)300*a_)
               {
                 cout<<"Turtle transformed: "<<frame<<endl;
                 current_turtles_.push_back(frame);
               }
           }

           else if(it.find("base_link") != std::string::npos)  {
               cout << "base link x " << transform.getOrigin().x() << " and y " << transform.getOrigin().y() << endl;
               cout<<"base_link found!"<<endl;
               cv::Point frame((int)(transform.getOrigin().x()*100),(int)(transform.getOrigin().y()*100));
               // if(frame.y>0 && frame.y<(int)b_*100 && frame.x >0 && frame.x<(int)300*a_)
               // {
                 cout<<"Current pose transformed: "<<frame<<endl;
                 current_position_ = frame;
               // }
           }
        }

    catch (tf::TransformException ex) {
         ROS_ERROR("%s",ex.what());
         ros::Duration(0.1).sleep();
        }
    }
}

//**********************************************
// function to update a and b
//**********************************************
void SetPath::set_a_b(float a, float b) {
    a_ = a;
    b_ = b;
    cout<<"a and b updated in path"<<a_<<"  "<<b_<<endl;
}
