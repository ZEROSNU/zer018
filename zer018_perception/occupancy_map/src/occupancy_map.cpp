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
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include <boost/thread.hpp>

#define min(a,b) ((a)<(b)?(a):(b))
#define RAD2DEG(x) ((x)*180./M_PI)
bool MAP_DEBUG=false;
std::string config_path;


bool flag_imshow = true;
bool flag_record = true;

cv::VideoWriter outputVideo;

float lane_width;
int map_width, map_height, paddingx, paddingy, safex, safey;
float map_resol, map_offset, min_range, max_range, min_theta, max_theta; //this is equal to length from vehicle_cm to lidar
image_transport::Publisher publishMapImg;
sensor_msgs::ImagePtr msgMapImg;

image_transport::Publisher publishMapRawImg;
sensor_msgs::ImagePtr msgMapRawImg;

ros::Publisher flag_obstacle_publisher;
ros::Publisher target_publisher;
std_msgs::Int32 msg_flag_obstacle;

ros::Publisher emergency_publisher;
std_msgs::Int32 msg_emergency_stop;
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
    if(range_i > 0 && range_i < estop_range_threshold && RAD2DEG(theta_i) >= estop_angle_min && RAD2DEG(theta_i) <= estop_angle_max) estop_count++;
    if(range_i>min_range && range_i<max_range && RAD2DEG(theta_i)>min_theta && RAD2DEG(theta_i)<max_theta){
      obstacle_x = range_i*cos(theta_i);
      obstacle_y = range_i*sin(theta_i);
      cx = map_width/2 + (int)(obstacle_x/map_resol);
      cy = map_height - (int)((obstacle_y+map_offset)/map_resol);
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
          //TODO: activate this!!!!!!
          //cout<<"draw points"<<endl;
          cv::ellipse(occupancy_map,cv::Point(cx,cy), cv::Size(safex, safey), 0.0, 0.0, 360.0, cv::Scalar(255,0,0), -1);
          cv::rectangle(occupancy_map_raw,cv::Point(cx1, cy1), cv::Point(cx2, cy2), cv::Scalar(255,0,0), -1);
        }
      }
    }
  }
  //Z_DEBUG
  if(MAP_DEBUG) {
    cv::rectangle(occupancy_map,cv::Point(0,0), cv::Point(map_width/2,5), cv::Scalar(255,0,0), -1);
    cv::circle(occupancy_map, cv::Point(map_width/2,map_height/2-20),15,cv::Scalar(255,0,0), -1);
    //cv::rectangle(occupancy_map,cv::Point(map_width/2-60, map_height/2-20), cv::Point(map_width/2-48, map_height/2+20), cv::Scalar(255,0,0), -1);
    //cv::rectangle(occupancy_map,cv::Point(map_width/2+48, map_height/2+10), cv::Point(map_width/2+60, map_height/2+30), cv::Scalar(255,0,0), -1);
    obstacle_count = 10;
  }
  return obstacle_count;
}

void drawLidarPosition() {
  int cx = map_width/2;
  int cy = map_height - (int)(map_offset/map_resol);
  cv::rectangle(occupancy_map_raw,cv::Point(cx-1, cy), cv::Point(cx, cy), cv::Scalar(200,200,200), -1);
}

void findTargetInOneRow(int & target_x, int & target_y, int row_index) {
  //TODO
  int target_threshold = 20;
  int index_lane_left = 0;
  int index_lane_right = 199;
  int index_free= 0;
  // cout<<"first row: "<<endl;

  for(int i = 0 ; i<map_width-1 ; i++) {
    cv::Vec3b px_val1 = occupancy_map.at<cv::Vec3b>(row_index,i);
    cv::Vec3b px_val2 = occupancy_map.at<cv::Vec3b>(row_index,i+1);
    //if(x == width/2 && y == height/2) std::cout<<"the red pix value of map center is "<<(int)px_val.val[0]<<std::endl;
    bool isred1 = (int)px_val1.val[0]==255 ? true : false;//val[0]: red
    bool isred2 = (int)px_val2.val[0]==255 ? true : false;//val[0]: red

    // if(isred1) cout<<1;
    // else cout<<0;
    if(isred1 && !isred2) index_lane_left=i;
    else if(!isred1 && isred2 && i < index_lane_right) index_lane_right = i;
    if(!isred1) index_free++;
  }
  // cout<<endl;
  //cout<<"index_lane_left: "<<index_lane_left<<" and right: "<<index_lane_right<<", free length is: "<<index_free<<endl;
  if(index_free == 0) findTargetInOneRow(target_x, target_y, row_index+1);
  else {
    target_x = index_lane_left + index_free/ 2;
    target_y = row_index;
    return;
  }
  if(index_free == 0 && row_index >=20) {
    target_x = -1;//target invalid
    target_y = -1;
    return;
  }
}

void drawTargetPoint(int flag_obstacle) {
  int target_x=0;
  int target_y=0;
  findTargetInOneRow(target_x, target_y, 1);
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

void publishMessages(int flag_obstacle) {

  //cv_ptr = cv_bridge::toCvShare(map, sensor_msgs::image_encodings::BGR8);
  msgMapImg->header.stamp = ros::Time::now();
  msgMapRawImg->header.stamp = ros::Time::now();

  msgMapImg = cv_bridge::CvImage(std_msgs::Header(),"rgb8", occupancy_map).toImageMsg();
  publishMapImg.publish(msgMapImg);
  msgMapRawImg = cv_bridge::CvImage(std_msgs::Header(),"rgb8", occupancy_map_raw).toImageMsg();
  publishMapRawImg.publish(msgMapRawImg);

  outputVideo << occupancy_map;

  msg_flag_obstacle.data = flag_obstacle;
  flag_obstacle_publisher.publish(msg_flag_obstacle);
  cout<<"estop_count : "<<estop_count <<endl;
  if(estop_count > estop_count_threshold) {
    msg_emergency_stop.data = 1;
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

void callbackObstacle(const core_msgs::ROIPointArrayConstPtr& msg_obstacle)
{
  //if(Z_DEBUG) std::cout<<"callbackObstacle of Map Generator called!"<<std::endl;
  map_mutex_.lock();
  obstacle_points = msg_obstacle->Vector3DArray;
  //cout<<"HERE"<<endl;

  //lidar_count = msg_obstacle->id[0];

  //lidar_angle_min = msg_obstacle->extra[0];

  map_mutex_.unlock();

  int flag_obstacle = drawObjects();
  publishMessages(flag_obstacle);
}


void callbackTerminate(const std_msgs::Int32Ptr& record){
  //if(flag_record){
    outputVideo.release();
  //}

  // std::string path = ros::package::getPath("map_generator");
  // path += "/../../../data/test_data/";
  // cv::Mat map_resized = cv::Mat::zeros(800,800,CV_8UC3);
  // cv::resize(occupancy_map, map_resized, cv::Size(800,800), 0,0, CV_8UC3);
  // cv::imwrite(path+"map.png",map_resized);

  ROS_INFO("Video recording safely terminated");
  ros::shutdown();
  return;
}


//argv: 1:nodeName 2:flag_imshow 3:flag_record(true if you are to record video)
//CHOI? flag_imshow의 usage?
int main(int argc, char** argv)
{
  //argument setting initialization
  if(argc < 2)  {
      std::cout << "usage: rosrun map_generator map_generator_node debug_mode" << std::endl;
      std::cout << "debug_mode is true for debug" << std::endl;
      return -1;
  }

  if (!strcmp(argv[1], "true")) MAP_DEBUG = true;
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
  record_path += "_map.mp4";

  ROS_INFO_STREAM(record_path);
  //if(flag_record) {

  //TODO: change root path later
  config_path = ros::package::getPath("core_util");
  config_path += "/config/system_config.yaml";
  cv::FileStorage params_config(config_path, cv::FileStorage::READ);
  lane_width = params_config["Road.lanewidth"];

  mapInit(params_config);
  bool isVideoOpened =  outputVideo.open(record_path, CV_FOURCC('X', '2', '6', '4'), 20, cv::Size(map_width,map_height), true);
  //}
  if(isVideoOpened)
    ROS_INFO("video starts recorded!");

  occupancy_map = cv::Mat::zeros(map_height,map_width,CV_8UC3);
  occupancy_map_raw = cv::Mat::zeros(map_height,map_width,CV_8UC3);
  lane_map_mono = cv::Mat::zeros(map_height,map_width,CV_8UC1);

  ros::init(argc, argv, "map_generator");
  ros::start();
  ros::NodeHandle nh;

  flag_obstacle_publisher = nh.advertise<std_msgs::Int32>("/flag_obstacle",1);
  target_publisher = nh.advertise<geometry_msgs::Vector3>("/planning_target",1);
  emergency_publisher = nh.advertise<std_msgs::Int32>("/emergency_stop",1);

  image_transport::ImageTransport it(nh);
  publishMapImg = it.advertise("/occupancy_map",1);
  msgMapImg.reset(new sensor_msgs::Image);
  publishMapRawImg = it.advertise("/occupancy_map_raw",1);
  msgMapRawImg.reset(new sensor_msgs::Image);

  ros::Subscriber laneSub = nh.subscribe("/lane_map",1,callbackLane);
  ros::Subscriber obstacleSub = nh.subscribe("/obstacle_points",1,callbackObstacle);

  ros::Subscriber endSub = nh.subscribe("/end_system",1,callbackTerminate);
  ros::spin();
  return 0;
}
