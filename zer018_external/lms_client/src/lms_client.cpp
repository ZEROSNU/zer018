#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "core_msgs/ROIPointArray.h"
#include "core_util/zdebug.h"
bool LMS_CLIENT_DEBUG = false;
#define RAD2DEG(x) ((x)*180./M_PI)

ros::Publisher scan_publisher;
core_msgs::ROIPointArrayPtr obstacle_points;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  std::cout<<"scan callback"<<std::endl;
  int count = scan->scan_time / scan->time_increment;
  ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  ROS_INFO("angle_increment, %f", RAD2DEG(scan->angle_increment));

  obstacle_points->Vector3DArray.clear();
  obstacle_points->id.clear();
  obstacle_points->extra.clear();

  geometry_msgs::Vector3 point_;

  for(int i = 0; i< count; i++) {
    float radian = scan->angle_min + scan->angle_increment *i;
    point_.x = scan->ranges[i];
    point_.y = radian + M_PI/2;
    point_.z = 1;
    obstacle_points->Vector3DArray.push_back(point_);
  }
  obstacle_points->id.push_back(count);
  obstacle_points->extra.push_back(scan->angle_min+M_PI);
  obstacle_points->extra.push_back(scan->angle_increment);

  obstacle_points->header.stamp = ros::Time::now();
}

//the obstacle_points data is published only after the lane_map data is published
//this is for the synchronization for the lane_map and obstacle_points, also needed for map_generator
//TODO: Need more sophisticated synchronization
void callbackLane(const sensor_msgs::ImageConstPtr& msg_lane_map) {
  if(LMS_CLIENT_DEBUG)
  {
    obstacle_points->Vector3DArray.clear();
    geometry_msgs::Vector3 point_;

    for(int i = 0; i< 150; i++) {
      point_.x = 0.0;
      if(i == 60) point_.x = 2.5;
      if(i == 80) point_.x = 1.5;
      if(i == 100) point_.x = 2.5;
      if(i == 110) point_.x = 2.0;

      point_.y = i * 0.031415;
      point_.z = 1;
      obstacle_points->Vector3DArray.push_back(point_);
    }
    obstacle_points->id.push_back(100);
    obstacle_points->extra.push_back(M_PI);
    obstacle_points->extra.push_back(0.05);

    obstacle_points->header.stamp = ros::Time::now();
  }
  scan_publisher.publish(obstacle_points);
}

int main(int argc, char **argv) {
  //argument setting initialization
  if(argc < 2)  {
      std::cout << "usage: rosrun lms_client lms_client debug_mode" << std::endl;
      std::cout << "debug_mode is true for debug" << std::endl;
      return -1;
  }

  if (!strcmp(argv[1], "true")) LMS_CLIENT_DEBUG = true;

  ros::init(argc, argv, "lms_client");
  ros::NodeHandle nh;

  scan_publisher = nh.advertise<core_msgs::ROIPointArray>("/obstacle_points", 1);
  obstacle_points.reset(new core_msgs::ROIPointArray);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);
  ros::Subscriber laneSub = nh.subscribe("/lane_map",1,callbackLane);

  ros::spin();
  return 0;
}
