#ifndef DSTAR_PLANNER
#define DSTAR_PLANNER

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include <string.h>
#include "Dstar.h"
#include <algorithm>    // std::min

using namespace std;
using namespace cv;
#define EPSILON 15
#define GRAYAREA 2; // Use for the radius of the Safe Zone (extension of the circle around each object's center point)


// Create the Path class (all states are saved and displayed here)
class Path {
public:
  const void drawPath(string windowName);
  const vector<Point> getVectorPoint();
  void newGoal(int new_goal_x, int new_goal_y);
  void newStart(int new_start_x, int new_start_y);
  bool replanPath();
  void setStartandGoal(int x_start, int y_start, int x_goal, int y_goal);
  void setObstacle(int x_obs, int y_obs, int radius);
  bool isBetween(Point a, Point b, Point c);
  void eraseObject();
  Path();
  ~Path();
  vector<int> map_dimensions_;

private:
  vector<state> vectorPath_; // Temporal variable for the vector of states
  vector<Point> vectorPoint_;
  Dstar *dstar;
  list<state> mypath_;
  vector<Vec3i> obstacles_;
  Mat obstacle_map_;
  Mat world_map_;
};

#endif
