#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string>
#include <signal.h>
#include <math.h>
#include <ctime>
#include <sstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include "core_msgs/PathArray.h"
#include "core_msgs/VehicleState.h"
#include "core_msgs/Control.h"
#include "core_msgs/CenPoint.h"
#include "core_msgs/Estop.h"
#include "core_util/zdebug.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include <boost/thread.hpp>

bool MONITOR_DEBUG=false;
bool USE_ROSBAG = false;

struct vehicle_state {
  bool is_auto;
  bool estop;
  int flag_obstacle;
  int gear;
  int brake;
  float speed;
  float steer;
  int control_mode;
};

std::string config_path;

cv::VideoWriter outputMonitorVideo;

vehicle_state vState;
vehicle_state vControl;

image_transport::Publisher publishMonitorImg;
sensor_msgs::ImagePtr msgMonitorImg;
int map_width, map_height;
float map_res;
int x_waypoint, y_waypoint;
cv::Mat occupancy_map;
double occupancy_map_timestamp;
cv::Mat lane_topview;
double lane_topview_timestamp;

cv::Mat state_monitor;
cv::Mat monitor_img;
std::vector<geometry_msgs::Vector3> path_points;
std::vector<geometry_msgs::Vector3> path_points_update;

double pathtracking_timestamp;

boost::mutex map_mutex_;
int e_stop = 0;
int spath_n = 0;
float uturn_angle =0.0;

void callbackLaneCam(const sensor_msgs::ImageConstPtr& msg_lane_cam)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg_lane_cam, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  cv::Mat temp = cv_ptr->image.clone();
  cv::Mat temp_resized;
  cv::resize(temp, temp_resized, cv::Size(map_height*2,map_width*2),0,0);

  cv::transpose(temp_resized, temp_resized);
  cv::Mat rot_matrix = getRotationMatrix2D(cv::Point(map_height, map_width), 180, 1.0);
  cv::warpAffine(temp_resized, lane_topview, rot_matrix, temp_resized.size());

  // lane_topview = cv_ptr->image.clone();
  lane_topview_timestamp = msg_lane_cam->header.stamp.toSec();
}
void callbackOccupancyRaw(const sensor_msgs::ImageConstPtr& msg_occupancy_raw)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg_occupancy_raw, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  state_monitor = cv_ptr->image.clone();
}

void callbackOccupancyMap(const sensor_msgs::ImageConstPtr& msg_occupancy_map)
{
  //if(Z_DEBUG) std::cout<<"occupanc map callback!"<<std::endl;
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg_occupancy_map, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  occupancy_map = cv_ptr->image.clone();
  occupancy_map_timestamp = msg_occupancy_map->header.stamp.toSec();
}

void callbackWaypoint(const core_msgs::CenPointConstPtr& msg_waypoint) {
  x_waypoint = (float)(msg_waypoint->x_waypoint)/map_res;
  y_waypoint = (float)(msg_waypoint->y_waypoint)/map_res;
}

void callbackFlagObstacle(const std_msgs::Int32::ConstPtr & msg_flag_obstacle) {
  vState.flag_obstacle = msg_flag_obstacle->data;
}

void callbackEstop(const core_msgs::Estop::ConstPtr & msg_estop) {
  e_stop = msg_estop->estop;
}

void callbackSpathn(const std_msgs::Int32::ConstPtr & msg_lookahead_n) {
  spath_n = msg_lookahead_n->data;
}

void callbackUturn(const std_msgs::Float32::ConstPtr & msg_uturn_angle) {
  uturn_angle = msg_uturn_angle->data;
}

void callbackPath(const core_msgs::PathArrayConstPtr& msg_path_tracking)
{
  path_points = msg_path_tracking->pathpoints;
  pathtracking_timestamp = msg_path_tracking->header.stamp.toSec();
  // if(Z_DEBUG) std::cout<<"path callback timestamp:"<<pathtracking_timestamp<<std::endl;
}
void callbackPathUpdate(const core_msgs::PathArrayConstPtr& msg_path_tracking)
{
  path_points_update = msg_path_tracking->pathpoints;
}

void callbackState(const core_msgs::VehicleStateConstPtr& msg_state) {
  vState.is_auto = msg_state->is_auto;
  vState.estop = msg_state->estop;
  vState.gear = msg_state->gear;
  vState.brake = msg_state->brake;
  vState.steer = -msg_state->steer;
  vState.speed = msg_state->speed;
}

void callbackControl(const core_msgs::ControlConstPtr& msg_control) {
  vControl.is_auto = msg_control->is_auto;
  vControl.estop = msg_control->estop;
  vControl.gear = msg_control->gear;
  vControl.brake = msg_control->brake;
  vControl.steer = -msg_control->steer;
  vControl.speed = msg_control->speed;
  vControl.control_mode = msg_control->control_mode;
}
void callbackTerminate(const std_msgs::Int32Ptr& record){
  outputMonitorVideo.release();
  ROS_INFO("Monitor Video recording safely terminated");
  ros::shutdown();
  return;
}

void drawPath(cv::Mat &canvas) {
  if(vControl.control_mode == 2) {
    std::cout<<"drawing path"<<std::endl;
    for(int i= path_points.size()-1; i>=1;i--){
      int start_x = (int)(2*path_points.at(i).x);
      int start_y = (int)(2*path_points.at(i).y);
      int end_x = (int)(2*path_points.at(i-1).x);
      int end_y = (int)(2*path_points.at(i-1).y);
      if(i == path_points.size()-spath_n) {
          cv::line(canvas,cv::Point(start_x, start_y), cv::Point(end_x, end_y), cv::Scalar(255,255,100),2);
      }
      else cv::line(canvas,cv::Point(start_x, start_y), cv::Point(end_x, end_y), cv::Scalar(100,255,100),2);
    }
  }
  if(MONITOR_DEBUG && vControl.control_mode ==2) {
    for(int i= path_points_update.size()-1; i>=1;i--){
      int start_x = (int)(2*path_points_update.at(i).x);
      int start_y = (int)(2*path_points_update.at(i).y);
      int end_x = (int)(2*path_points_update.at(i-1).x);
      int end_y = (int)(2*path_points_update.at(i-1).y);
      cv::line(canvas,cv::Point(start_x, start_y), cv::Point(end_x, end_y), cv::Scalar(100,255,255),2);
    }
  }
  else if(vControl.control_mode == 3) {
    cv::line(canvas,cv::Point(map_width, map_height*2), cv::Point(map_width - 2*y_waypoint, map_height*2-2*x_waypoint), cv::Scalar(255,255,100),2);
  }
  else if(vControl.control_mode ==4) {
    cv::line(canvas,cv::Point(map_width, map_height*2), cv::Point(map_width - 2*y_waypoint, map_height*2-2*x_waypoint), cv::Scalar(170,255,100),2);
  }
}

int main(int argc, char** argv)
{
  //argument setting initialization
  if(argc < 3)  {
      std::cout << "usage: rosrun zero_monitor z_monitor debug_mode for_rosbag" << std::endl;
      std::cout << "debug_mode is true for debug" << std::endl;
      std::cout << "for_rosbag is true for monitoring rosbag files" << std::endl;

      return -1;
  }

  if (!strcmp(argv[1], "true")) MONITOR_DEBUG = true;
  if (!strcmp(argv[2], "true")) USE_ROSBAG = true;

  std::string record_path = ros::package::getPath("core_util");
  //TODO: add date&time to the file name
  record_path += "/data/";

  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer,sizeof(buffer),"%m%d-%Y_%I:%M:%S",timeinfo);
  std::string date_str(buffer);
  record_path += date_str;
  record_path += "_monitor.avi";

  ROS_INFO_STREAM(record_path);

  x_waypoint = 180;
  y_waypoint = 0;

  //TODO: change root path later
  config_path = ros::package::getPath("core_util");
  config_path += "/config/system_config.yaml";
  cv::FileStorage params_config(config_path, cv::FileStorage::READ);
  map_width = params_config["Map.width"];
  map_height = params_config["Map.height"];
  map_res = params_config["Map.resolution"];
  int vehicle_width = params_config["Vehicle.width"];
  int vehicle_length_front = params_config["Vehicle.length_front"];

  bool isVideoOpened =  outputMonitorVideo.open(record_path, CV_FOURCC('X', 'V', 'I', 'D'), 10, cv::Size(map_width*6, map_height*2+100), true);

  if(isVideoOpened)
    ROS_INFO("monitor video starts recorded!");

  occupancy_map = cv::Mat::zeros(map_height,map_width,CV_8UC3);
  lane_topview = cv::Mat::zeros(map_height,map_width,CV_8UC3);
  state_monitor = cv::Mat::zeros(map_height,map_width,CV_8UC3);

  ros::init(argc, argv, "map_generator");
  ros::start();

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  if(!USE_ROSBAG) publishMonitorImg = it.advertise("/monitor",1);
  else publishMonitorImg = it.advertise("/monitor_rosbag",1);
  msgMonitorImg.reset(new sensor_msgs::Image);

  ros::Subscriber laneSub, occupancySub, occupancyRawSub;
  if(!USE_ROSBAG) {
    laneSub = nh.subscribe("/warped_image",1,callbackLaneCam);
    occupancySub = nh.subscribe("/occupancy_map",1,callbackOccupancyMap);
    occupancyRawSub = nh.subscribe("/occupancy_map_raw",1,callbackOccupancyRaw);
  }
  else {
    laneSub = nh.subscribe("/warped_image",1,callbackLaneCam);
    occupancySub = nh.subscribe("/occupancy_map/rosbag",1,callbackOccupancyMap);
    occupancyRawSub = nh.subscribe("/occupancy_map_raw",1,callbackOccupancyRaw);
  }


  ros::Subscriber pathSub, pathupdateSub;
  if(MONITOR_DEBUG){
    pathSub = nh.subscribe("/sPath",1,callbackPath);
    pathupdateSub = nh.subscribe("/path_tracking",1,callbackPathUpdate);
  }
  else pathSub = nh.subscribe("/path_tracking",1,callbackPath);

  ros::Subscriber waypointSub, flagobstacleSub, stateSub, controlSub, endSub, estopSub, spathnSub, uturnSub;
  waypointSub = nh.subscribe("/waypoints",1,callbackWaypoint);
  flagobstacleSub = nh.subscribe("/flag_obstacle",1,callbackFlagObstacle);
  stateSub = nh.subscribe("/vehicle_state",1,callbackState);
  controlSub = nh.subscribe("/control",1,callbackControl);
  endSub = nh.subscribe("/end_system",1,callbackTerminate);
  estopSub = nh.subscribe("/emergency_stop",1,callbackEstop);
  spathnSub = nh.subscribe("/lookahead_n",1, callbackSpathn);
  uturnSub = nh.subscribe("/uturn_angle",1,callbackUturn);
  vState.speed = 0;
  vState.steer = 0;
  if(MONITOR_DEBUG) vState.flag_obstacle = 1;
  else vState.flag_obstacle = 0;
  ros::Rate loop_rate(10);
  ros::Time init_time = ros::Time::now();
  while(ros::ok()){
    //if(Z_DEBUG) std::cout<<"while loop starts!!"<<std::endl;

    //monitor initialization
    monitor_img = cv::Mat::zeros(map_height*2+100, map_width*6,CV_8UC3);

    cv::Mat occupancy_map_resized = cv::Mat::zeros(map_height*2,map_width*2,CV_8UC3);
    cv::resize(occupancy_map, occupancy_map_resized, cv::Size(map_height*2,map_width*2),0,0);

    cv::Mat lane_topview_resized = cv::Mat::zeros(map_height*2,map_width*2,CV_8UC3);
    cv::resize(lane_topview, lane_topview_resized, cv::Size(map_height*2,map_width*2),0,0);

    //drawing waypoint
    cv::circle(state_monitor, cv::Point(map_width/2 - y_waypoint, map_height-x_waypoint),3,cv::Scalar(255,255,100), -1);
    cv::Mat state_monitor_resized = cv::Mat::zeros(map_height*2,map_width*2,CV_8UC3);
    cv::resize(state_monitor, state_monitor_resized, cv::Size(map_height*2,map_width*2),0,0);
    //if(Z_DEBUG) std::cout<<"state monitor resized!!"<<std::endl;
    drawPath(state_monitor_resized);
    drawPath(lane_topview_resized);
    z_print("MONITOR CAR BOUNDARY");
    cv::rectangle(state_monitor_resized, cv::Point(map_width - vehicle_width/map_res, 2*map_height-1), cv::Point(map_width+vehicle_width/map_res, 2*map_height-1 - vehicle_length_front/map_res), cv::Scalar(150,150,200), -1);

    //TODO: save path image to other directory (in image files)
    ///add above three images to the monitor_img
    lane_topview_resized.copyTo(monitor_img(cv::Rect(0,0,lane_topview_resized.cols,lane_topview_resized.rows)));
    //if(Z_DEBUG) std::cout<<"lane topview copied!!"<<std::endl;
    state_monitor_resized.copyTo(monitor_img(cv::Rect(lane_topview_resized.cols,0,state_monitor_resized.cols,state_monitor_resized.rows)));
    //if(Z_DEBUG) std::cout<<"state monitor copied!!"<<std::endl;
    occupancy_map_resized.copyTo(monitor_img(cv::Rect(lane_topview_resized.cols+state_monitor_resized.cols,0,occupancy_map_resized.cols,occupancy_map_resized.rows)));
    //if(Z_DEBUG) std::cout<<"occupancy map copied!!"<<std::endl;
    //TODO: add text info to the image
    int state_count;
    nh.getParam("/state_count", state_count);
    std::stringstream stream_state;
    stream_state <<"vehicle state    count: " <<state_count;
    std::string state_text =  stream_state.str();

    cv::putText(monitor_img, state_text, cv::Point(20, map_height*2+10), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(250,250,250));
    std::string basic_state = "";
    if(vState.is_auto) basic_state +="AUTO   ";
    else basic_state +="MANUAL ";

    if(vState.gear ==0) basic_state +="DRIVE   ";
    else if(vState.gear == 1) basic_state +="NEUTRAL ";
    else if(vState.gear == 2) basic_state +="REAR    ";

    if(vState.estop) basic_state+="!!ESTOP!!";
    if(!vState.estop) cv::putText(monitor_img, basic_state, cv::Point(20, map_height*2+30), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(250,250,250));
    else cv::putText(monitor_img, basic_state, cv::Point(20, map_height*2+30), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,100,100));

    std::stringstream stream;
    double v_kmph = 3.6*vState.speed;
    stream << "v(km/h): " << std::fixed << std::setprecision(2) << v_kmph << "  steer(deg): "<<vState.steer;
    std::stringstream stream_brake;
    stream_brake << "Brake: " << vState.brake;
    std::string v_state = stream.str();
    std::string b_state = stream_brake.str();
    cv::putText(monitor_img, v_state, cv::Point(20, map_height*2+50), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(250,250,250));
    cv::putText(monitor_img, b_state, cv::Point(20, map_height*2+70), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(250,250,250));


    std::string basic_control = "";
    if(vControl.is_auto) basic_control +="AUTO   ";
    else basic_control +="MANUAL ";

    if(vControl.gear ==0) basic_control +="DRIVE   ";
    else if(vControl.gear == 1) basic_control +="NEUTRAL ";
    else if(vControl.gear == 2) basic_control +="REAR    ";

    if(vControl.estop) basic_control+="!!ESTOP!!";
    if(!vControl.estop) cv::putText(monitor_img, basic_control, cv::Point(20+map_width*4, map_height*2+30), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,255,150));
    else cv::putText(monitor_img, basic_control, cv::Point(20+map_width*4, map_height*2+30), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,100,100));



    int control_count;
    nh.getParam("/control_count", control_count);
    std::stringstream stream_control;
    stream_control <<"control output    count: " <<control_count;
    std::string control_text =  stream_control.str();

    cv::putText(monitor_img, control_text, cv::Point(map_width*4+20, map_height*2+10), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(250,250,250));
    std::stringstream control;
    control << "v(km/h): " << std::fixed << std::setprecision(2) << 3.6*vControl.speed << "  steer(deg): "<<vControl.steer;
    std::stringstream control_brake;
    control_brake << "Brake: " << vControl.brake;
    std::string v_control = control.str();
    std::string b_control = control_brake.str();
    cv::putText(monitor_img, v_control, cv::Point(map_width*4+20, map_height*2+50), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(255,255,150));
    cv::putText(monitor_img, b_control, cv::Point(map_width*4+20, map_height*2+70), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(255,255,150));

    std::string control_mode_text = "";
    if(vControl.control_mode == 0) control_mode_text += "NO CONTROL APPLIED";//No Control
    else if(vControl.control_mode == 1) control_mode_text += "Autonomous Emergency Braking";//No Control
    else if(vControl.control_mode == 2) control_mode_text += "OBSTACLE AVOID (NORMAL_S)";//No Control
    else if(vControl.control_mode == 3) control_mode_text += "LANE TRACE (NORMAL_C)";//No Control
    else if(vControl.control_mode == 4) control_mode_text += "OBSTACLE AVOID: NO PATH(SLOW_C)";//No Control
    else if(vControl.control_mode == 5) control_mode_text += "MAKING U TURN";//No Control
    else if(vControl.control_mode == 6) control_mode_text += "GETTING OUT OF PARKING LOT";//No Control
    cv::Scalar control_mode_text_color = cv::Scalar(220,220,220);
    if(vControl.control_mode == 1) control_mode_text_color = cv::Scalar(255,200,200);//No Control
    else if(vControl.control_mode == 2) control_mode_text_color = cv::Scalar(100, 255, 100);
    else if(vControl.control_mode == 3) control_mode_text_color = cv::Scalar(255, 255, 100);
    else if(vControl.control_mode == 4) control_mode_text_color = cv::Scalar(170, 255, 100);
    else if(vControl.control_mode == 5 || vControl.control_mode ==6) control_mode_text_color = cv::Scalar(150, 150, 255);
    std::stringstream add_stream;
    if(vControl.control_mode == 5) add_stream <<"Uturn angle(deg): "<<std::fixed<< std::setprecision(2)<<uturn_angle;
    else add_stream << "Obstacle Flag: " <<vState.flag_obstacle;
    std::string add_text = add_stream.str();
    cv::putText(monitor_img, add_text, cv::Point(map_width*2+20, map_height*2+60), cv::FONT_HERSHEY_PLAIN, 1.0, control_mode_text_color);
    cv::putText(monitor_img, "Control Mode", cv::Point(map_width*2+20, map_height*2+20), cv::FONT_HERSHEY_PLAIN, 1.0, control_mode_text_color);
    cv::putText(monitor_img, control_mode_text, cv::Point(map_width*2+20, map_height*2+40), cv::FONT_HERSHEY_PLAIN, 1.0, control_mode_text_color);


    ros::Time current_time = ros::Time::now();
    ros::Duration to_now(current_time - init_time);
    std::stringstream time_stream;
    time_stream << "execution time(sec): " << std::fixed << std::setprecision(2) << to_now.toSec();
    std::string time_text = time_stream.str();
    cv::putText(monitor_img, time_text, cv::Point(map_width*2+20, map_height*2+80), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(100,100,255));


    //publishing monitor image
    cv::cvtColor(monitor_img, monitor_img, cv::COLOR_BGR2RGB);
    msgMonitorImg = cv_bridge::CvImage(std_msgs::Header(),"rgb8", monitor_img).toImageMsg();

    publishMonitorImg.publish(msgMonitorImg);

    //adding it to the video record
    outputMonitorVideo << monitor_img;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
