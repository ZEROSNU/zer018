//reference: ttbot - map_gen.cpp and rl_map.cpp
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
//#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include "deque"

#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ctime>
//#include <thread>
//TODO: Define and set core_msgs and their msg files listed here
#include "core_msgs/ROIPointArray.h"
#include "core_msgs/Estop.h"
#include "core_msgs/ParkPoints.h"
#include "core_util/zdebug.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#define min(a,b) ((a)<(b)?(a):(b))
#define RAD2DEG(x) ((x)*180./M_PI)
bool MAP_DEBUG=false;
bool IS_RECORD = false;

std::string config_path;


bool flag_imshow = true;
bool flag_record = true;
bool is_set_parkmode = false;
cv::VideoWriter outputVideo;

int is_occupied_x;
int is_occupied_y;
float lane_width;
int map_width, map_height, paddingx, paddingy, safex, safey;
float map_resol, map_offset, min_range, max_range, min_theta, max_theta; //this is equal to length from vehicle_cm to lidar
float vehicle_width, vehicle_length;
image_transport::Publisher publishMapImg;
sensor_msgs::ImagePtr msgMapImg;

image_transport::Publisher publishMapRawImg;
sensor_msgs::ImagePtr msgMapRawImg;

ros::Publisher flag_obstacle_publisher;
ros::Publisher target_publisher;
std_msgs::Int32 msg_flag_obstacle;

ros::Publisher emergency_publisher;
core_msgs::Estop msg_emergency_stop;
int estop_count = 0;
const int estop_count_threshold = 10;
int estop_angle_min, estop_angle_max; //deg
float estop_range_threshold;

geometry_msgs::Vector3 msgTarget;

cv::Mat occupancy_map;
cv::Mat occupancy_map_raw;
cv::Mat lane_map_mono;
boost::mutex map_mutex_;
std::vector<geometry_msgs::Vector3> obstacle_points;
// int lidar_count=0;
// float lidar_angle_min = 0;
using namespace std;

bool check_out(int cx, int cy)
{
    return (cx<map_width-1)&&(cx>0)&&(cy<map_height-1)&&(cy>0);
}


//TODO: usage of mapInit()
void mapInit(cv::FileStorage& params_config) {
  if(MAP_DEBUG) cout<<"map_generator map initialization!"<<endl;

  map_width = params_config["Map.width"];
  map_height = params_config["Map.height"];
  map_resol = params_config["Map.resolution"];
  paddingx = params_config["Map.obstacle.paddingx"];
  paddingy = params_config["Map.obstacle.paddingy"];
  safex = params_config["Map.obstacle.safex"];
  safey = params_config["Map.obstacle.safey"];
  map_offset = params_config["Vehicle.cm_lidar_dist"];
  min_range = params_config["Map.obstacle.min_range"];
  max_range = params_config["Map.obstacle.max_range"];
  min_theta = params_config["Map.obstacle.min_theta"];//in degree
  max_theta = params_config["Map.obstacle.max_theta"];//in degree
  estop_angle_max = params_config["Map.obstacle.estop_max_theta"];
  estop_angle_min = params_config["Map.obstacle.estop_min_theta"];
  estop_range_threshold = params_config["Map.obstacle.estop_range_threshold"];

  vehicle_width = params_config["Vehicle.width"];
  vehicle_length = params_config["Vehicle.length"];
}

bool isOccupied(float x, float y) {
  std::cout <<" is Occupied Called!" <<std::endl;
  //x, y is in meter
  double cx = -y;
  double cy = x-map_offset;
  double r = sqrt(cx*cx + cy*cy);
  std::cout<<"r is "<< r<<std::endl;
  double theta = atan2(cy, cx);
  for(int i= obstacle_points.size()-1; i>=0;i--){
    //cout<<"HERE"<<endl;
    float range_i = obstacle_points.at(i).x;
    float theta_i = obstacle_points.at(i).y;//in radian
    if(abs(theta_i - theta) <0.36 && range_i < r + 0.4 && range_i >min_range && range_i<5.0)
    {
      std::cout << "occupied! " <<std::endl;
      int obstacle_x = range_i*cos(theta_i);
      int obstacle_y = range_i*sin(theta_i);
      int cx = map_width/2 + (int)(obstacle_x/map_resol);
      int cy = map_height - (int)((obstacle_y+map_offset)/map_resol);
      is_occupied_x = cx;
      is_occupied_y = cy;
      return true;
    }
  }
  return false;
}

int drawObstaclePoints(std::vector<geometry_msgs::Vector3>& _obstacle_points) {
  float obstacle_x, obstacle_y;
  int cx, cy;
  int cx1, cx2, cy1, cy2;
  int obstacle_count = 0;
  estop_count = 0;

  for(int i= _obstacle_points.size()-1; i>=0;i--){
    //cout<<"HERE"<<endl;
    float range_i = _obstacle_points.at(i).x;
    float theta_i = _obstacle_points.at(i).y;//in radian
    if(range_i > 0.05 && range_i < estop_range_threshold && RAD2DEG(theta_i) >= estop_angle_min && RAD2DEG(theta_i) <= estop_angle_max) {
      estop_count++;
      // std::cout<<RAD2DEG(theta_i);
      // std::cout<<", "<<range_i<<std::endl;
    }
    if(range_i>min_range && range_i<max_range && RAD2DEG(theta_i)>min_theta && RAD2DEG(theta_i)<max_theta){
      obstacle_y = range_i*cos(theta_i);
      obstacle_x = range_i*sin(theta_i);
      cx = map_width/2 + (int)(obstacle_y/map_resol);
      cy = map_height - (int)((obstacle_x+map_offset)/map_resol);
      if(check_out(cx, cy)){
        if (lane_map_mono.at<uchar>(cy,cx)!=255) obstacle_count++;//only add number of obstacle when it is not outside the lane. this is very important.
        if(check_out(cx-paddingx, cy-paddingy)){
          if(check_out(cx+paddingx,cy+paddingy)){
            cx1 = cx-paddingx;
            cy1 = cy-paddingy;
            cx2 = cx+paddingx;
            cy2 = cy+paddingy;
          }else {
            cx1 = cx-paddingx;
            cy1 = cy-paddingy;
            cx2 = cx; cy2 = cy;
          }
        } else if(check_out(cx+paddingx,cy+paddingy)) {
          cx1 = cx; cy1 = cy;
          cx2 = cx+paddingx;
          cy2 = cy+paddingy;
        } else {
          cx1 = cx; cy1 = cy;
          cx2 = cx; cy2 = cy;
        }

        if(!MAP_DEBUG) {
          cv::Point padding_points[4];
          padding_points[0]= cv::Point(cx+safex,cy);
          padding_points[1]= cv::Point(cx,cy+safey);
          padding_points[2]= cv::Point(cx-safex,cy);
          padding_points[3]= cv::Point(cx,cy-safey);
          // cv::Point padding_points[6];
          // int tune = 10;
          // padding_points[0]= cv::Point(cx+safex,cy-tune);
          // padding_points[1]= cv::Point(cx+safex,cy+tune);
          //
          // padding_points[2]= cv::Point(cx, cy+safey);
          //
          // padding_points[3]= cv::Point(cx-safex,cy+tune);
          // padding_points[4]= cv::Point(cx-safex,cy-tune);
          //
          // padding_points[5]= cv::Point(cx,cy-safey);

          // std::cout<<"padding_points added"<<std::endl;
          cv::fillConvexPoly(occupancy_map, padding_points,4, cv::Scalar(255,0,0));
          cv::fillConvexPoly(occupancy_map_raw, padding_points,4, cv::Scalar(100,0,0));
          // cv::fillConvexPoly(occupancy_map, padding_points,6, cv::Scalar(255,0,0));
          // cv::fillConvexPoly(occupancy_map_raw, padding_points,6, cv::Scalar(100,0,0));

          // cv::ellipse(occupancy_map,cv::Point(cx,cy), cv::Size(safex, safey), 0.0, 0.0, 360.0, cv::Scalar(255,0,0), -1);
          // //TODO: consider the area of obstacle padding && outside lane
          // cv::ellipse(occupancy_map_raw,cv::Point(cx,cy), cv::Size(safex, safey), 0.0, 0.0, 360.0, cv::Scalar(100,0,0), -1);
          cv::rectangle(occupancy_map_raw,cv::Point(cx1, cy1), cv::Point(cx2, cy2), cv::Scalar(255,0,0), -1);
        }
      }
    }
  }
  //Z_DEBUG
  if(MAP_DEBUG) {
    cv::rectangle(occupancy_map,cv::Point(0,0), cv::Point(map_width/2-20,5), cv::Scalar(255,0,0), -1);
    cv::rectangle(occupancy_map,cv::Point(map_width/2+10,0), cv::Point(map_width/2+30,5), cv::Scalar(255,0,0), -1);

    cv::circle(occupancy_map, cv::Point(map_width/2,map_height/2-20),15,cv::Scalar(255,0,0), -1);
    cv::circle(occupancy_map, cv::Point(map_width/2-10,map_height-20),5,cv::Scalar(255,0,0), -1);

    //cv::rectangle(occupancy_map,cv::Point(map_width/2-60, map_height/2-20), cv::Point(map_width/2-48, map_height/2+20), cv::Scalar(255,0,0), -1);
    //cv::rectangle(occupancy_map,cv::Point(map_width/2+48, map_height/2+10), cv::Point(map_width/2+60, map_height/2+30), cv::Scalar(255,0,0), -1);
    obstacle_count = 10;
  }
  cv::rectangle(occupancy_map,cv::Point(map_width/2 - (int)(0.5*vehicle_width/map_resol), map_height-1 - (int)(0.5*vehicle_length/map_resol)), cv::Point(map_width/2 + (int)(0.5*vehicle_width/map_resol), map_height-1), cv::Scalar(0,0,0), -1);
  if (is_occupied_x != 0 && is_occupied_y != 0) cv::line(occupancy_map_raw,cv::Point(map_width/2, map_height - (int)(map_offset/map_resol)), cv::Point(is_occupied_x, is_occupied_y), cv::Scalar(255,150,150), 1);

  return obstacle_count;
}

void drawLidarPosition() {
  int cx = map_width/2;
  int cy = map_height - (int)(map_offset/map_resol);
  cv::rectangle(occupancy_map_raw,cv::Point(cx-1, cy), cv::Point(cx, cy), cv::Scalar(200,200,200), -1);
}

void findTargetInOneRow(int & target_x, int & target_y, int row_index) {
  //TODO
  // int target_threshold = 20;
  int index_lane_left[] = {0, 0, 0, 0, 0};
  int index_lane_right[] = {map_width -3, map_width -3, map_width -3, map_width -3, map_width -3};
  int free_length[] = {0,0,0,0,0};
  // cout<<"first row: "<<endl;
  int left_i = 0;
  int right_i = 0;
  for(int i = 2 ; i<map_width-3 ; i++) {
    cv::Vec3b px_val1 = occupancy_map.at<cv::Vec3b>(row_index,i);
    cv::Vec3b px_val2 = occupancy_map.at<cv::Vec3b>(row_index,i+1);
    //if(x == width/2 && y == height/2) std::cout<<"the red pix value of map center is "<<(int)px_val.val[0]<<std::endl;
    bool isred1 = (int)px_val1.val[0]==255 ? true : false;//val[0]: red
    bool isred2 = (int)px_val2.val[0]==255 ? true : false;//val[0]: red

    // if(isred1) cout<<1;
    // else cout<<0;
    if(isred1 && !isred2 && left_i < 5) {
      index_lane_left[left_i]=i;
      left_i++;
    }
    else if(!isred1 && isred2 & right_i < 5) {
      index_lane_right[right_i] = i;
      right_i++;
    }
  }
  if (left_i == 0 && right_i == 0) {
    // std::cout<<"going to next row -first cond."<<std::endl;
    findTargetInOneRow(target_x, target_y, row_index+1);
    return;
  }
  if (left_i != right_i) {
    // std::cout<<"left and right index different, target may not be valid"<<std::endl;
    // std::cout<<"left_i: "<<left_i<<", righ_i: "<<right_i<<std::endl;
  }
  if (left_i <= right_i){
    if(left_i == 0) {//this means that right_i = 0
      free_length[0] = index_lane_right[0] - index_lane_left[0];
    }
    else if(index_lane_right[0] < index_lane_left[0]) {
      for (int j = 0; j< left_i-1; j++) {
        if(index_lane_right[j+1] != map_width - 3)
          free_length[j] = index_lane_right[j+1] - index_lane_left[j];
      }
    }
    else {
      for (int j = 0; j< left_i; j++) {
        if(index_lane_right[j] != map_width - 3)
          free_length[j] = index_lane_right[j] - index_lane_left[j];
      }
    }
  }
  else {//when right_i < left_i
    if(right_i == 0) {
      free_length[0] = index_lane_right[0] - index_lane_left[0];
    }
    else if(index_lane_right[0] < index_lane_left[0]) {
      for (int j = 0; j< right_i-1; j++) {
        if(index_lane_left[j] != 0)
          free_length[j] = index_lane_right[j+1] - index_lane_left[j];
      }
    }
    else {
      for (int j = 0; j< right_i; j++) {
        if(index_lane_left[j] != 0)
          free_length[j] = index_lane_right[j] - index_lane_left[j];
      }
    }
  }
  int final_free = *std::max_element(free_length, free_length+5);
  int index_final_free = std::distance(free_length, std::max_element(free_length, free_length+5));
  int final_free_left = index_lane_left[index_final_free];
  int final_free_right;
  if(index_lane_right[index_final_free] > final_free_left) final_free_right = index_lane_right[index_final_free];
  else final_free_right = index_lane_right[index_final_free + 1];
  if(final_free == 0) {
    findTargetInOneRow(target_x, target_y, row_index+1);
    std::cout<<"going to next row -second cond."<<std::endl;
  }
  else {
    target_x = (final_free_left + final_free_right)/ 2;
    target_y = row_index;
    return;
  }
  if(final_free == 0 && row_index >=50) {
    std::cout<< "invalid target" << std::endl;
    target_x = -1;//target invalid
    target_y = -1;
    return;
  }
}

void drawTargetPoint(int flag_obstacle) {
  int target_x=0;
  int target_y=0;
  findTargetInOneRow(target_x, target_y, 5);
  if(target_x < 0) return;//target invalid

  cv::Vec3b px_val = occupancy_map_raw.at<cv::Vec3b>(cv::Point(target_x,target_y));
  if(px_val.val[0]!=255){
    cv::circle(occupancy_map_raw, cv::Point(target_x,target_y),3,cv::Scalar(0,0,255), -1);
    //occupancy_map_raw.at<cv::Vec3b>(cv::Point(target_x,target_y)) = cv::Vec3b(0,0,255);
  }
  else {
    //std::cout<<"the target point is not in the free region!"<<std::endl;
    //TODO: Change the below target point settings later
    cv::circle(occupancy_map_raw, cv::Point(target_x,target_y),3,cv::Scalar(255,0,255), -1);
  }
  msgTarget.x = target_y;
  msgTarget.y = target_x;//flipped for the planner's coordinate
  msgTarget.z = 0;
  target_publisher.publish(msgTarget);
}

int drawObjects() {
    //if(Z_DEBUG) std::cout<<"draw Objects started"<<std::endl;

    int flag_obstacle = 0;
    //flag_obstacle is more than 1 if there are any obstacle within the lane
    flag_obstacle = drawObstaclePoints(obstacle_points);
    //if(Z_DEBUG) std::cout<<"draw Obstacles finished"<<std::endl;
    drawLidarPosition();
    //if(Z_DEBUG) std::cout<<"draw Lidar Position finished"<<std::endl;
    //decide the location of target and draw on the map
    drawTargetPoint(flag_obstacle);
    //if(Z_DEBUG) std::cout<<"draw Target Position finished"<<std::endl;

    return flag_obstacle;
}

void publishMessages(int flag_obstacle, ros::NodeHandle& nh) {

  //cv_ptr = cv_bridge::toCvShare(map, sensor_msgs::image_encodings::BGR8);
  msgMapImg->header.stamp = ros::Time::now();
  msgMapRawImg->header.stamp = ros::Time::now();

  msgMapImg = cv_bridge::CvImage(std_msgs::Header(),"rgb8", occupancy_map).toImageMsg();
  publishMapImg.publish(msgMapImg);
  msgMapRawImg = cv_bridge::CvImage(std_msgs::Header(),"rgb8", occupancy_map_raw).toImageMsg();
  publishMapRawImg.publish(msgMapRawImg);

  if(IS_RECORD)  outputVideo << occupancy_map;

  msg_flag_obstacle.data = flag_obstacle;
  flag_obstacle_publisher.publish(msg_flag_obstacle);
  // cout<<"estop_count : "<<estop_count <<endl;

  //TODO: for contest, activate this
  int uturn_mode;
  nh.getParam("/uturn_mode", uturn_mode);
  // std::cout<<"estop count is "<<estop_count<<std::endl;

  //DEBUG
  if(estop_count > estop_count_threshold && uturn_mode != 2) {
    msg_emergency_stop.header.stamp = ros::Time::now();
    msg_emergency_stop.estop = estop_count;
    // std::cout<<"estop message occured and count is "<<estop_count<<std::endl;
    emergency_publisher.publish(msg_emergency_stop);
  }

}
void callbackLane(const sensor_msgs::ImageConstPtr& msg_lane_map)
{
  //if(Z_DEBUG) std::cout<<"callbackLane of Map Generator called!"<<std::endl;
  //Saving msg_lane_map(which is grayscale image) to map(which is CV_8UC3 cv::Mat object)
  ros::Time lane_receive_t0;
  // if(Z_DEBUG) lane_receive_t0 = ros::Time::now();
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg_lane_map, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  lane_map_mono = cv_ptr->image.clone();
  std::vector<cv::Mat> images(3);
  cv::Mat black = cv::Mat::zeros(lane_map_mono.rows, lane_map_mono.cols, lane_map_mono.type());
  images.at(0) = lane_map_mono; //for blue channel
  images.at(1) = black;   //for green channel
  images.at(2) = black;  //for red channel
  cv::merge(images, occupancy_map);
  cv::merge(images, occupancy_map_raw);
  // if(Z_DEBUG)
  // {
  //   ros::Time lane_receive_t1 = ros::Time::now();
  //   ros::Duration d(lane_receive_t1-lane_receive_t0);
  //   //std::cout << "Lane Receiving Time in ms: " << d * 1000 << std::endl;
  // }
}

void callbackPark(ros::NodeHandle& nh, const core_msgs::ParkPointsConstPtr& msg_park){
  float goal_x = msg_park->goal_point.x;
  float goal_y = msg_park->goal_point.y;
 if(!isOccupied(goal_x, goal_y) && !is_set_parkmode){
   nh.setParam("/park_mode",2);
   is_set_parkmode = true;
 }
}

void callbackObstacle(const core_msgs::ROIPointArrayConstPtr& msg_obstacle, ros::NodeHandle& nh)
{
  //if(Z_DEBUG) std::cout<<"callbackObstacle of Map Generator called!"<<std::endl;
  map_mutex_.lock();
  obstacle_points = msg_obstacle->Vector3DArray;
  //cout<<"HERE"<<endl;

  //lidar_count = msg_obstacle->id[0];

  //lidar_angle_min = msg_obstacle->extra[0];

  map_mutex_.unlock();

  int flag_obstacle = drawObjects();
  publishMessages(flag_obstacle,nh);
}


void callbackTerminate(const std_msgs::Int32Ptr& record){

  if(IS_RECORD) {
    outputVideo.release();

    ROS_INFO("Video recording safely terminated");
  }
  ros::shutdown();
  return;
}


//argv: 1:nodeName 2:flag_imshow 3:flag_record(true if you are to record video)
//CHOI? flag_imshow의 usage?
int main(int argc, char** argv)
{
  //argument setting initialization
  if(argc < 3)  {
      std::cout << "usage: rosrun map_generator map_generator_node debug_mode is_record" << std::endl;
      std::cout << "debug_mode is true for debug" << std::endl;
      std::cout << "is_record is true for video recording of the map"<<std::endl;
      return -1;
  }

  if (!strcmp(argv[1], "true")) MAP_DEBUG = true;
  if (!strcmp(argv[2],"true")) IS_RECORD = true;
  std::string record_path = ros::package::getPath("core_util");
  //TODO: add date&time to the file name
  record_path += "/data/";
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer,sizeof(buffer),"%y%m%d-%H:%M:%S",timeinfo);
  std::string date_str(buffer);
  record_path += date_str;

  ROS_INFO_STREAM(record_path);

  config_path = ros::package::getPath("core_util");
  config_path += "/config/system_config.yaml";
  cv::FileStorage params_config(config_path, cv::FileStorage::READ);
  lane_width = params_config["Road.lanewidth"];

  mapInit(params_config);
  bool isVideoOpened = false;
  if(IS_RECORD) {
    record_path += "_map.mp4";
    isVideoOpened = outputVideo.open(record_path, CV_FOURCC('X', '2', '6', '4'), 20, cv::Size(map_width,map_height), true);
    if(isVideoOpened)
     ROS_INFO("video starts recorded!");
  }

  occupancy_map = cv::Mat::zeros(map_height,map_width,CV_8UC3);
  occupancy_map_raw = cv::Mat::zeros(map_height,map_width,CV_8UC3);
  lane_map_mono = cv::Mat::zeros(map_height,map_width,CV_8UC1);

  ros::init(argc, argv, "map_generator");
  ros::start();
  ros::NodeHandle nh;

  flag_obstacle_publisher = nh.advertise<std_msgs::Int32>("/flag_obstacle",1);
  target_publisher = nh.advertise<geometry_msgs::Vector3>("/planning_target",1);
  emergency_publisher = nh.advertise<core_msgs::Estop>("/emergency_stop",1);

  image_transport::ImageTransport it(nh);
  publishMapImg = it.advertise("/occupancy_map",1);
  msgMapImg.reset(new sensor_msgs::Image);
  publishMapRawImg = it.advertise("/occupancy_map_raw",1);
  msgMapRawImg.reset(new sensor_msgs::Image);

  ros::Subscriber laneSub = nh.subscribe("/lane_map",1,callbackLane);
  ros::Subscriber obstacleSub = nh.subscribe<core_msgs::ROIPointArray>("/obstacle_points",1,boost::bind(&callbackObstacle, _1, boost::ref(nh)));
  ros::Subscriber parkSub = nh.subscribe<core_msgs::ParkPoints>("/initial_points_for_park",1,boost::bind(&callbackPark, boost::ref(nh), _1));
  ros::Subscriber endSub = nh.subscribe("/end_system",1,callbackTerminate);
  ros::spin();
  return 0;
}
