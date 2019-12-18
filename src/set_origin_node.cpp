/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  usv_odom.h
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/18
  * - Brief:     Node to invoke service to set origin point of USV
  ******************************************************************************
  * History:
  * 2019/12/18   
  * Define and implement interface of class, test pass
  ******************************************************************************
*/

#include <ros/ros.h>
#include <usv_odom/SetOrigin.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "set_origin_node");
  ros::NodeHandle nh;

  ros::ServiceClient set_origin_client = nh.serviceClient<usv_odom::SetOrigin>("set_origin");

  usv_odom::SetOrigin srv_msgs;

  if(set_origin_client.call(srv_msgs)){
    ROS_INFO("usv_odom: set origin point successfully!");
  }
  else{
    ROS_ERROR("usv_odom: set origin point failed!");
    return -1;
  }

  return 0;
}
