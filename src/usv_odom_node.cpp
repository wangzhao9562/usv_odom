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
  * History:
  * 2019/12/17 
  * Add ros parameter pub_time
  ******************************************************************************
*/

#include <usv_odom/usv_odom.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "usv_odom_node");
  ros::NodeHandle nh;
 
  int pub_time;
  nh.param("pub_time", pub_time, 100);
 
  UsvOdom usv_odom;

  ros::Rate loop_rate(pub_time);

  while(ros::ok()){
    usv_odom.publishOdom();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
