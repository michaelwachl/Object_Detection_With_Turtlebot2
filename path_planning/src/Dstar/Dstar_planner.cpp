#include "Dstar_planner.h"

Path::Path() {
  dstar = new Dstar(); // Create the algorithm object on the stack
  map_dimensions_ = vector<int> {600,600};
  obstacle_map_ = Mat(Size(map_dimensions_.at(0), map_dimensions_.at(1)), CV_8UC3, CV_RGB(255,255,255)); // Initialize the obstacle map image
  world_map_ = Mat(Size(map_dimensions_.at(0), map_dimensions_.at(1)), CV_8UC3, CV_RGB(255,255,255)); // Initialize the world map image
}

Path::~Path() {
  delete dstar;
}

// Using a zero Matrix, draw on top the path using circles
const void Path::drawPath(string windowName) {
   // Clean the world map by intilizing it to the obstacle_map_ (possible problem if the resources are limited)
   world_map_ = obstacle_map_.clone();
   int myradius = 3; // Radius of the circles

   // Draw the circles for the start and goal
   circle(world_map_, Point(vectorPoint_[0].x, vectorPoint_[0].y), myradius, CV_RGB(0,100,0),-1,1,0);
   circle(world_map_, Point(vectorPoint_.back().x, vectorPoint_.back().y), myradius, CV_RGB(0,0,100),-1,1,0);
   // Draw the rest of the circles
   for (int i=1;i<vectorPoint_.size()-1;i++)
     circle(world_map_, Point(vectorPoint_[i].x, vectorPoint_[i].y), myradius, CV_RGB(100,0,0),-1,8,0);
   // Display the window
   namedWindow(windowName, WINDOW_AUTOSIZE);
   imshow(windowName, world_map_);
}

// getter function for the path
const vector<Point> Path::getVectorPoint() {
  return vectorPoint_;
}

void Path::newGoal(int new_goal_x, int new_goal_y) {
  dstar->updateGoal(new_goal_x,new_goal_y);         // Move goal
}

void Path::newStart(int new_start_x, int new_start_y) {
  dstar->updateStart(new_start_x, new_start_y); // Move start
}

bool Path::replanPath() {
   if(dstar->replan()) {             // Plan a path
     mypath_ = dstar->getPath();     // Retrieve path

     // Transform the state list to a vector state
     vector<state> v{ begin(mypath_), end(mypath_) };
     vectorPath_ = v;

     if(!vectorPoint_.empty())
       vectorPoint_.clear(); // Clear the remains of the last calculation
     // Transform the vector state to a vector point (path)
     for(int i = 0; i < v.size(); i++) {
       // Save every 7 value in the path (make it smoother and less noisy)
       if(i%8 == 0) {
       vectorPoint_.push_back(Point(vectorPath_[i].x, vectorPath_[i].y));
       }
     }
     vectorPoint_.pop_back();    // Pop the back element, so that the goal point is not too near to the last point of the vector
     vectorPoint_.push_back(Point(vectorPath_[v.size()-1].x, vectorPath_[v.size()-1].y)); // Push the goal to the vector

     for(int j = 0; j < vectorPoint_.size()-2; j++) {
       if( isBetween(vectorPoint_.at(j), vectorPoint_.at(j+1), vectorPoint_.at(j+2)) )
        vectorPoint_.erase(vectorPoint_.begin()+j+1);
     }
     return true;
  }
  else
    return false;
}

void Path::eraseObject() {
  // Clear all the points of the obstacles inside the dstar map
  for (auto obstacle_i : obstacles_) {
    int outer_radius = obstacle_i[2] + GRAYAREA;
    for (int y = obstacle_i[1]-outer_radius; y <= obstacle_i[1]+outer_radius; y++) {
      for (int x = obstacle_i[0]-outer_radius; x <= obstacle_i[0]+outer_radius; x++) {
          dstar->updateCell(x, y, 10); // Set cell weight and coordinates (10 means transversable, but costs more)
      }
    }
  }
  // Clear the obstacles saved up until now
  obstacles_.clear();
  // Clear the visual maps
  obstacle_map_ = Mat(Size(map_dimensions_.at(0), map_dimensions_.at(1)), CV_8UC3, CV_RGB(255,255,255)); // Initialize the obstacle map image
  world_map_ = Mat(Size(map_dimensions_.at(0), map_dimensions_.at(1)), CV_8UC3, CV_RGB(255,255,255)); // Initialize the world map image
}

bool Path::isBetween(Point a, Point b, Point c) {
  // int crossproduct = (c.y - a.y) * (b.x - a.x) - (c.x - a.x) * (b.y - a.y);
  // if (abs(crossproduct) > EPSILON)
  //   return false;
  // return true;

  Point line_ab = a - b;
  Point line_cb = c - b;
  int area = line_ab.x * line_cb.y - line_ab.y * line_cb.x;
  if (abs(area) > EPSILON)
    return false;
  return true;

  // int dotproduct = (c.x - a.x) * (b.x - a.x) + (c.y - a.y)*(b.y - a.y);
  // if (dotproduct < 0)
  //   return false;
  //
  // int squaredlengthba = (b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y);
  // if (dotproduct > squaredlengthba)
  //   return false;
  // if((-EPSILON < crossproduct < EPSILON) && (min(a.x, b.x) <= c.x) && (c.x <= max(a.x, b.x)) )//&& (min(a.y, b.y) <= c.y) && (c.y <= max(a.y, b.y)))
  //   return true;
  return true;
}

void Path::setObstacle(int x_obs, int y_obs, int radius) {
  obstacles_.push_back(Vec3i(x_obs, y_obs, radius));
  obstacle_map_ = Mat(Size(map_dimensions_.at(0), map_dimensions_.at(1)), CV_8UC3, CV_RGB(255,255,255));

  for (auto obstacle_i : obstacles_) {
    // Draw first the outer circle
    int outer_radius = obstacle_i[2] + GRAYAREA;
    circle(obstacle_map_, Point(obstacle_i[0], obstacle_i[1]), outer_radius, CV_RGB(220,220,220),-1,1,0);
    // Check in each coordinate if the pixel value is equal gray
    for (int y = obstacle_i[1]-outer_radius; y <= obstacle_i[1]+outer_radius; y++) {
      for (int x = obstacle_i[0]-outer_radius; x <= obstacle_i[0]+outer_radius; x++) {
        if(obstacle_map_.at<Vec3b>(y,x)[0] == 220)
          dstar->updateCell(x, y, 10); // Set cell weight and coordinates (10 means transversable, but costs more)
      }
    }
  }

  for (auto obstacle_i : obstacles_) {
    // Draw the inner circle
    int inner_radius = obstacle_i[2];
    circle(obstacle_map_, Point(obstacle_i[0], obstacle_i[1]), inner_radius, CV_RGB(0,0,0),-1,1,0);
    // Check in each coordinate if the pixel value is equal black
    for (int y = obstacle_i[1]-inner_radius; y <= obstacle_i[1]+inner_radius; y++) {
      for (int x = obstacle_i[0]-inner_radius; x <= obstacle_i[0]+inner_radius; x++) {
        if(obstacle_map_.at<Vec3b>(y,x)[0] == 0)
          dstar->updateCell(x, y, -1); // Set cell weight and coordinates (-1 := non traversable)
      }
    }
  }
}

void Path::setStartandGoal(int x_start, int y_start, int x_goal, int y_goal) {
   dstar->init(x_start,y_start,x_goal,y_goal);         // Set start to (x_start,y_start) and goal to (x_goal,y_goal)
}
