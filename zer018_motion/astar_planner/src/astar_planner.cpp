#include <iostream>
#include <cstring>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <core_util/zdebug.h>
#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"
#include "visualize.h"
#include "lookup.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "core_msgs/VehicleState.h"
#include "opencv2/opencv.hpp"
#include <boost/thread.hpp>
#include <boost/bind.hpp>

float vel =1.0;//m/s
float delta = 0.64;//delta in radian
float map_resol;
float wheelbase = 1.6;
bool isparkmission = false;
int map_width, map_height;
int headings;
float delay = 0.4;
int flag_obstacle = 0;
int target_x = 0;
int target_y = 0;

// image_transport::Publisher publishMonintorMap;
// sensor_msgs::ImagePtr msgMonitorMap;


using namespace HybridAStar;

class Astar
{
public:
  Astar();
  void plan(geometry_msgs::PoseWithCovarianceStamped start, geometry_msgs::PoseStamped goal);
  void initializeLookups();
  cv::Mat gridmap;
  /// The path produced by the hybrid A* algorithm

  /// The smoother used for optimizing the path
  Smoother smoother;
  /// The path smoothed and ready for the controller
  Path smoothedPath = Path(true);
  /// The visualization used for search visualization
  // Visualize visualization;

  /// The collission detection for testing specific configurations
  CollisionDetection configurationSpace;
  /// The voronoi diagram
  DynamicVoronoi voronoiDiagram;
  Path path;
  Constants::config collisionLookup[Constants::headings * Constants::positions];
  float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
};


Astar::Astar() {
  z_print("Astar Created");
}

void Astar::plan(geometry_msgs::PoseWithCovarianceStamped start, geometry_msgs::PoseStamped goal) {
      // ___________________________

      //TODO Getting Rid of grid in this whole library, replace it with pacakge!!!!!!
      //!!!!!!!

      int depth = headings;
      int length = map_width * map_height * depth;
      // define list pointers and initialize lists
      // std::cout<<"length is "<<length<<std::endl;
      Node3D* nodes3D = new Node3D[length](); //200*200*12 = 480000
      Node2D* nodes2D = new Node2D[map_width * map_height]();

      // ________________________
      // retrieving goal position
      //this is in image frame
      float x = goal.pose.position.x;
      float y = goal.pose.position.y;
      float t = tf::getYaw(goal.pose.orientation);
      // set theta to a value (0,2PI]
      t = Helper::normalizeHeadingRad(t);
      // std::cout<<"goal heading in rad is "<<t<<std::endl;
      const Node3D nGoal(x, y, t, 0, 0, nullptr);
      // retrieving start position
      x = start.pose.pose.position.x;
      y = start.pose.pose.position.y;
      t = tf::getYaw(start.pose.pose.orientation);
      // set theta to a value (0,2PI]
      t = Helper::normalizeHeadingRad(t);
      // std::cout<<"start heading in rad is "<<t<<std::endl;

      Node3D nStart(x, y, t, 0, 0, nullptr);
      // ___________
      // DEBUG START
      //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);

      // std::cout<<"nStart and nGoal set up properly" <<std::endl;

      // ___________________________
      // START AND TIME THE PLANNING
      ros::Time plan_t0 = ros::Time::now();

      // CLEAR THE VISUALIZATION
      //visualization.clear();

      // CLEAR THE PATH
      path.clear();
      smoothedPath.clear();
      // FIND THE PATH
      Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, map_width, map_height, configurationSpace, dubinsLookup);
      // std::cout<<"nSolution is solved" <<std::endl;

      //DEBUG
      ros::Time plan_t1 = ros::Time::now();
      ros::Duration d1(plan_t1 - plan_t0);
      // std::cout << "Planning Time in ms: " << d1 * 1000 << std::endl;


      if(nSolution==nullptr) std::cout<<"nSolution is Null Pointer" <<std::endl;
      // TRACE THE PATH (update the path in the smoother object)
      else {
        smoother.tracePath(nSolution);
        // CREATE THE UPDATED PATH
        path.updatePath(smoother.getPath());
        // SMOOTH THE PATH
        smoother.smoothPath(voronoiDiagram);
        // CREATE THE UPDATED PATH
        smoothedPath.updatePath(smoother.getPath());
        ros::Time plan_t2 = ros::Time::now();
        ros::Duration d2(plan_t2 - plan_t1);
        // std::cout << "Smoothing Time in ms: " << d2 * 1000 << std::endl;

        // _________________________________
        // PUBLISH THE RESULTS OF THE SEARCH

        path.publishPath();
        smoothedPath.publishPath();
      }
      delete [] nodes3D;
      delete [] nodes2D;
}

void Astar::initializeLookups() {
  Lookup::collisionLookup(collisionLookup);
}

/// A pointer to the grid the planner runs on
//nav_msgs::OccupancyGrid::Ptr grid;
/// The start pose set through RViz

// void drawMonitorMap(Astar& astar) {
//   for(int i = 0; i< astar.smoothedPath.getPath().pathpoints.size(); i++) {
//     int px = (int)astar.smoothedPath.getPath().pathpoints.at(i).x;
//     int py = (int)astar.smoothedPath.getPath().pathpoints.at(i).y;
//     astar.gridmap.at<cv::Vec3b>(cv::Point(px,py)) = cv::Vec3b(0,255,0);
//   }
//   msgMonitorMap = cv_bridge::CvImage(std_msgs::Header(),"rgb8", astar.gridmap).toImageMsg();
//   publishMonintorMap.publish(msgMonitorMap);
//
// }

void callbackState(const core_msgs::VehicleStateConstPtr& msg_state) {
  delta = -(msg_state->steer)*M_PI/180.0;
  if(msg_state->speed<20.0 && msg_state->gear ==0)  vel = msg_state->speed;
  else if(msg_state->speed<20.0 && msg_state->gear ==2)  vel = -msg_state->speed;
}


void callbackTerminate(const std_msgs::Int32Ptr& record) {
  ros::shutdown();
  return;
}

void callbackMain(const sensor_msgs::ImageConstPtr& msg_map, Astar& astar)
{
  if(Z_DEBUG && flag_obstacle!=0)  std::cout << "------------------------------------------------------------------" << std::endl;
  if(flag_obstacle==0) {
    std::cout<<flag_obstacle<<std::endl;
    return;
  }
  if(target_x <= 0 && target_y <= 0) {
    return;
  }
  // if()
  ros::Time map_time = msg_map->header.stamp; //the time when the map is recorded
  ros::Time t0 = ros::Time::now();

  astar.initializeLookups();
  // z_print("here!");
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg_map, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  float yaw_delta = (float)(vel*delta*delay)/wheelbase;

  float x_delay_shift, y_delay_shift;
  if(yaw_delta != 0) {
    x_delay_shift = (float)(wheelbase*(1-cos(yaw_delta))/delta)/map_resol;
    y_delay_shift = (float)(wheelbase*sin(yaw_delta)/delta)/map_resol;
  }
  else {
    x_delay_shift = 0;
    y_delay_shift = vel*delay/map_resol;
  }
  astar.gridmap = cv_ptr->image.clone();
  astar.configurationSpace.updateGrid(astar.gridmap);
  //create array for Voronoi diagram
  int height = msg_map->height;
  int width = msg_map->width;
  //std::cout<<"height and width of msg_map is "<<height<<", "<<width<<std::endl;
  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      cv::Vec3b px_val = astar.gridmap.at<cv::Vec3b>(x,y);
      //if(x == width/2 && y == height/2) std::cout<<"the red pix value of map center is "<<(int)px_val.val[0]<<std::endl;
      binMap[x][y] = (int)px_val.val[0]==255 ? true : false;//val[0]: red
    }
  }
  //if(binMap[width/2][height/2]) std::cout<<"the center is red!!"<<std::endl;

  astar.voronoiDiagram.initializeMap(width, height, binMap);
  astar.voronoiDiagram.update();
  std::string vorono_path = ros::package::getPath("astar_planner")+"/config/result.ppm";
  const char* vorono_path_chr = vorono_path.c_str();
  astar.voronoiDiagram.visualize(vorono_path_chr);
  //free won't work, is it neccessary?
  //free((char*)vorono_path_chr);
  ros::Time t1 = ros::Time::now();
  ros::Duration d(t1 - t0);
  //std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;
  z_print("created Voronoi Diagram");
  // assign the values to start from base_link
  geometry_msgs::PoseWithCovarianceStamped start;
  //setting start point
  //TODO: check how the plan() function use this start point
  //TODO: change this later according to it
  // std::cout<<"yaw_delta in rad is"<<yaw_delta<<std::endl;
  start.pose.pose.position.y = map_width/2 - x_delay_shift;
  start.pose.pose.position.x = map_height -1 - y_delay_shift;
  if (start.pose.pose.position.y < 0) {
    start.pose.pose.position.y = 0;
  }
  else if (start.pose.pose.position.y > map_width-1) start.pose.pose.position.y = map_width-1;
  if (start.pose.pose.position.x < 0) {
    start.pose.pose.position.x = 0;
  }
  else if (start.pose.pose.position.y > map_height-1) start.pose.pose.position.y = map_height-1;
   // x and y flips for the input of path planning
  // std::cout<<"start x and y is ("<<start.pose.pose.position.x<<", "<<start.pose.pose.position.y<<")"<<std::endl;
  if(binMap[(int)(start.pose.pose.position.x)][(int)(start.pose.pose.position.y)]) {
    z_print("start x and y is in the occupied region");
    return;
  }


  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw_delta+M_PI);
  tf::quaternionTFToMsg(q,start.pose.pose.orientation);


  //TODO: have to define the goal as the center of the lane of the farrest side
  geometry_msgs::PoseStamped goal;
  goal.pose.position.y = target_y;
  goal.pose.position.x = target_x;
  if (goal.pose.position.y < 0) {
    goal.pose.position.y = 0;
  }
  else if (goal.pose.position.y > map_width-1) goal.pose.position.y = map_width-1;
  if (goal.pose.position.x < 0) {
    goal.pose.position.x = 0;
  }
  else if (goal.pose.position.y > map_height-1) goal.pose.position.y = map_height-1;

  tf::Quaternion q_goal = tf::createQuaternionFromRPY(0, 0, M_PI);
  tf::quaternionTFToMsg(q_goal,goal.pose.orientation);
  // std::cout<<"goal x and y is ("<<goal.pose.position.x<<", "<<goal.pose.position.y<<")"<<std::endl;
  if(binMap[(int)(goal.pose.position.x)][(int)(goal.pose.position.y)]) {
    z_print("goal x and y is in the occupied region");
    return;
  }


  astar.plan(start, goal);
  // drawMonitorMap(astar);
  ros::Time t2 = ros::Time::now();
  ros::Duration d_final(t2 - t0);
  // cout<<"the delay ground truth is: " <<d_final.toSec()<<" sec" <<endl;

  delay = 0.4 * 0.4 + delay * 0.36 + d_final.toSec() * 0.24;
  z_print("final position for the callback");
}

void callbackFlagObstacle(const std_msgs::Int32::ConstPtr & msg_flag_obstacle) {
  flag_obstacle = msg_flag_obstacle->data;
}

void callbackTarget(const geometry_msgs::Vector3::ConstPtr & msg_target) {
  target_x = msg_target->x;
  target_y = msg_target->y;
}


int main(int argc, char** argv) {
  std::string config_path = ros::package::getPath("core_util");
	cv::FileStorage params_config(config_path+"/config/system_config.yaml", cv::FileStorage::READ);
  float car_width = params_config["Vehicle.width"];
  float car_length = params_config["Vehicle.length"];
  map_width = params_config["Map.width"];
  map_height = params_config["Map.height"];
  std::cout<<"map_width and map_height are "<<map_width<<", "<<map_height<<std::endl;
  map_resol = params_config["Map.resolution"];
  headings = params_config["Path.headings"];
  wheelbase = params_config["Vehicle.wheelbase"];
  ros::init(argc, argv, "astar_planner");
  ros::start();
  Astar astar;
  // /map publish를 위한 설정 (publishMap & msgMap)
  ros::NodeHandle nh;
  // this is for monitoring
  image_transport::ImageTransport it(nh);
  // publishMonintorMap = it.advertise("/monitor_map",1);
  // msgMonitorMap.reset(new sensor_msgs::Image);

  ros::Subscriber stateSub = nh.subscribe("/vehicle_state",1,callbackState);
  ros::Subscriber flagobstacleSub = nh.subscribe("/flag_obstacle",1,callbackFlagObstacle);
  //TODO:: for park mission, the message also tell us the target point

  image_transport::Subscriber mapSub = it.subscribe("/occupancy_map",1,boost::bind(callbackMain, _1, boost::ref(astar)));
  ros::Subscriber endSub = nh.subscribe("/end_system",1,callbackTerminate);
  ros::Subscriber targetSub = nh.subscribe("/planning_target", 1, callbackTarget);

  //TODO: add v & delta service client
  //ros::Rate loop_rate(20);
  ros::spin();
  return 0;
}
