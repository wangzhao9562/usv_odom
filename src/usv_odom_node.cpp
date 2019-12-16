/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  usv_odom_node.cpp
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/4
  * - Brief:     Ros node to run UsvOdom
  ******************************************************************************
  * History:
  * 2019/12/4   
  * Complete node, complie successfully
  ******************************************************************************
*/

#include <usv_odom/usv_odom.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "usv_odom_node");
  ros::NodeHandle nh; 
 
  UsvOdom usv_odom;

  ros::Rate loop_rate(100);

  while(ros::ok()){
    usv_odom.publishOdom();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
