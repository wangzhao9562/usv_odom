/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  slide_avr_filter.h
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/19
  * - Brief:     Odom filter implemented through slide average filtering algorithm
  ******************************************************************************
  * History:
  * 2019/12/19
  * Complete implementation of odom filter with slide average filtering algorithm
  ******************************************************************************
*/

#ifndef SLIDE_AVR_FILTER_H_
#define SLIDE_AVR_FILTER_H_

#include <usv_odom/odom_filter.h>
#include <vector>

/**
 * @class SlideAvrFilter
 * @brief Provides implementation of base odom filter with slide average filtering algorithm
 */
class SlideAvrFilter : public OdomFilter{
public:
  /**
   * @brief Constructor
   * @param work_time Filter start to output result after count of work_time, in slide filter algorithm, it is equal to the size of sliding window
   */
  SlideAvrFilter(size_t work_time);

  /**
   * @brief De;ructor
   */
  ~SlideAvrFilter();

  /**
   * @brief Base interface, receive observed pose parameters and complete data filtering
   * @param obs_pose Input current observed pose of robot
   * @param time_c Count of time of data input 
   * @return Return filtered result
   */
  nav_msgs::Odometry odomFilter(nav_msgs::Odometry obs_pose, size_t time_c, double dt)override;

private:
  /**
   * @brief Implementation of slide average filtering
   * @param obs_pose Input current observed pose of robot
   * @param time_c Count of time of data input
   * @return Return filtered result
   */
  nav_msgs::Odometry slideAvrFiltering(nav_msgs::Odometry obs_pose, size_t time_c);

  /**
   * @brief Find maximum and minimum param in slide window
   * @param slide_window Slide window of slide average filter
   * @return position of minimum and maximum parameter in slide window
   */
  std::pair<size_t, size_t> findMinAndMaxParam(std::vector<double>& param_slide_window);

  /**
   * @brief Get filtering result of parameters
   * @param slide_window Slide window of slide average filter
   * @return Filtering result
   */
  double getFilteredParam(std::vector<double>& param_slide_window, std::pair<size_t, size_t>& filtered_pos);

private:
  std::vector<nav_msgs::Odometry>* slide_window_; // sliding window
};
 

#endif

