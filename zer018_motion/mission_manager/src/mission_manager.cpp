#include <iostream>
#include <cstring>
#include <ros/ros.h>
#include <ros/package.h>
#include <core_msgs/ToggleMission.h>
#include <core_util/zdebug.h>
#include "std_msgs/Int32.h"

//0: default   1:park sign is detected, search for goal parking lot    2: found free parking lot, move to the goal point
//3: reverse mode, getting out of the parking lot
//TODO: save park_mission_mode to the yaml file during the shutdown
int park_mode = 0; //IMPORTANT: this is the ros parameter
//These toggles are for parking mission
void callbackTerminate(const std_msgs::Int32Ptr& end_system){
  ros::shutdown();
  return;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_manager");
  ros::start();
  ros::NodeHandle nh;

  nh.setParam("park_mode", 0);
  ros::ServiceClient client_of_lane_detector = nh.serviceClient<core_msgs::ToggleMission>("/toggle_lanedetector_forpark_srv");
  ros::ServiceClient client_of_path_tracker = nh.serviceClient<core_msgs::ToggleMission>("/toggle_pathtracker_forpark_srv");

  ros::Subscriber endSub = nh.subscribe("/end_system",1,callbackTerminate);
  ros::Rate loop_rate(10);
  while(ros::ok()) {
    nh.getParam("park_mode", park_mode);
    core_msgs::ToggleMission srv;
    if(park_mode == 0) continue;
    else if(park_mode == 1) {
      srv.request.toggle = true;
      if(client_of_lane_detector.call(srv) && srv.response.is_confirmed) {
        z_print("successfully asked Lane Detector to look out for park_initpoints and goalpoints");
      }
    }
    else if(park_mode == 2) {
      srv.request.toggle = true;
      if(client_of_path_tracker.call(srv) && srv.response.is_confirmed) {
        z_print("successfully asked Path Tracker to pursue to the parking lot goalpoints");
      }
      if(client_of_lane_detector.call(srv) && srv.response.is_confirmed) {
        z_print("successfully asked Lane Detector to stop looking for park_initpoints and goalpoints");
      }
    }
    // else if(park_mode == 3) {
    //   srv.request.toggle = true;
    // }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
