/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  odom_filter.h
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/19
  * - Brief:     Odom base filter 
  ******************************************************************************
  * History:
  * 2019/12/19
  * Complete common base interface for odom filter
  ******************************************************************************
*/

#ifndef ODOM_FILTER_H_
#define ODOM_FILTER_H_

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

/**
 * @class OdomFilter
 * @brief Provides base interface to implement odom filtering for different filters
 */
class OdomFilter{
public:
  /**
   * @brief Constructor
   * @param work_time Filter start to output result after count of work_time 
   */
  OdomFilter(size_t work_time) : wt_(work_time){} 

  /**
   * @brief Base interface, receive observed pose parameters and complete data filtering
   * @param obs_pose Input current observed pose of robot
   * @param time_c Count of time of data input 
   * @return Return filtered result
   */
  virtual nav_msgs::Odometry odomFilter(nav_msgs::Odometry obs_pose, size_t time_c, double dt) = 0;

  size_t wt_;
};

#endif
