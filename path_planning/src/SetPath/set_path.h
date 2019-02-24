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


#ifndef SET_PATH
#define SET_PATH

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include <string.h>
#include <algorithm>
//dstar
#include "../Dstar/Dstar_planner.h"
//tf listener
#include <tf/transform_listener.h>

using namespace std;
using namespace cv;

class SetPath {
public:

  // Constructor: intilize all variables
  SetPath(int PUCK_RADIUS,int ROBOT_RADIUS, int POST_RADIUS, float a, float b): 
   puck_radius_(PUCK_RADIUS), robot_radius_(ROBOT_RADIUS), post_radius_(POST_RADIUS), a_(a), b_(b)
 {
    // Necessary for visualization (Path algorithm)
    path.map_dimensions_ = vector<int> {600,600}; // Set the height and width of the display window (can be ignored)
  }
  ~SetPath() {}

  //Variables
  vector<Point> current_our_pucks_, current_oppo_pucks_, current_posts_, current_turtles_;
  Point current_position_, current_goal_to_attack_, current_nearest_puck_;

  // This function clears the map and set the new coordinates. Then sets the output path to the global variable
  vector<Point> set_start_goal_ourpucks_enemypucks_enemyrobot_post(Point start, Point goal, vector<Point> obstacles_our_pucks,
        vector<Point> obstacles_enemy_puck_, vector<Point> obstacles_enemy_robot_, vector<Point> post);

// This function transforms frames into map coordinates for path planning
  void get_all_objects();
  void set_a_b(float a, float b);


private:
  // Create the Path object which calculates the path given a goal, obstacles and start
  Path path;
  //Variables
  int puck_radius_, robot_radius_, post_radius_;
  float a_, b_;
  vector<Point> pathVectorPoint_;
  tf::TransformListener tf_listener_;
};

#endif
