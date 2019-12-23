/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  slide_avr_filter.cpp
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/19
  * - Brief:     Odom filter implemented through slide average filtering algorithm
  *****************************************************************************
  * History:
  * 2019/12/19
  * Complete implementation of odom filter with slide average filtering algorithm
  ****************************************************************************
*/

#include <usv_odom/slide_avr_filter.h>

SlideAvrFilter::SlideAvrFilter(size_t work_time) : OdomFilter(work_time){
  slide_window_ = new std::vector<nav_msgs::Odometry>(work_time); // create slide window
}

SlideAvrFilter::~SlideAvrFilter(){
  if(slide_window_ != nullptr){
    slide_window_->clear();
    delete slide_window_;
    slide_window_ = nullptr;
  }
}

nav_msgs::Odometry SlideAvrFilter::odomFilter(nav_msgs::Odometry obs_pose, size_t time_c, double dt){
  return SlideAvrFilter::slideAvrFiltering(obs_pose, time_c);
}

nav_msgs::Odometry SlideAvrFilter::slideAvrFiltering(nav_msgs::Odometry obs_odom, size_t time_c){
  if(time_c <= this->wt_){
    slide_window_->at(time_c - 1) = obs_odom;
  }
  else{
   std::vector<nav_msgs::Odometry> temp_window;
   temp_window.insert(temp_window.end(), slide_window_->begin() + 1, slide_window_->end());
   temp_window.push_back(obs_odom);
   slide_window_->clear();
   slide_window_->insert(slide_window_->begin(), temp_window.begin(), temp_window.end()); 
  }

  if(time_c >= this->wt_){
    std::vector<double> pose_x_vec, pose_y_vec, pose_z_vec, yaw_vec;
    std::vector<double> linear_vel_x_vec, linear_vel_y_vec, linear_vel_z_vec;
    std::vector<double> angular_vel_x_vec, angular_vel_y_vec, angular_vel_z_vec;
   
    // get slide window of each parameters
    for(int index = 0; index < slide_window_->size(); ++index){
      nav_msgs::Odometry odom = slide_window_->at(index);
      pose_x_vec.push_back(odom.pose.pose.position.x);
      pose_y_vec.push_back(odom.pose.pose.position.y);
      pose_z_vec.push_back(odom.pose.pose.position.z);
      yaw_vec.push_back(tf::getYaw(odom.pose.pose.orientation));
      linear_vel_x_vec.push_back(odom.twist.twist.linear.x);
      linear_vel_y_vec.push_back(odom.twist.twist.linear.y);
      linear_vel_z_vec.push_back(odom.twist.twist.linear.z);
      angular_vel_x_vec.push_back(odom.twist.twist.angular.x);
      angular_vel_y_vec.push_back(odom.twist.twist.angular.y);
      angular_vel_z_vec.push_back(odom.twist.twist.angular.z);
    }

    // get positions of maximum and minimum value in slide windows
    std::pair<size_t, size_t> pos_of_filtered_x = findMinAndMaxParam(pose_x_vec);
    std::pair<size_t, size_t> pos_of_filtered_y = findMinAndMaxParam(pose_y_vec);
    std::pair<size_t, size_t> pos_of_filtered_z = findMinAndMaxParam(pose_z_vec);
    std::pair<size_t, size_t> pos_of_filtered_yaw = findMinAndMaxParam(yaw_vec);
    std::pair<size_t, size_t> pos_of_filtered_lvx = findMinAndMaxParam(linear_vel_x_vec);
    std::pair<size_t, size_t> pos_of_filtered_lvy = findMinAndMaxParam(linear_vel_y_vec);
    std::pair<size_t, size_t> pos_of_filtered_lvz = findMinAndMaxParam(linear_vel_z_vec);
    std::pair<size_t, size_t> pos_of_filtered_avx = findMinAndMaxParam(angular_vel_x_vec);
    std::pair<size_t, size_t> pos_of_filtered_avy = findMinAndMaxParam(angular_vel_y_vec);
    std::pair<size_t, size_t> pos_of_filtered_avz = findMinAndMaxParam(angular_vel_z_vec);
  
    // get filtered value
    double filtered_x = getFilteredParam(pose_x_vec, pos_of_filtered_x);
    double filtered_y = getFilteredParam(pose_y_vec, pos_of_filtered_y);
    double filtered_z = getFilteredParam(pose_z_vec, pos_of_filtered_z);
    double filtered_yaw = getFilteredParam(yaw_vec, pos_of_filtered_yaw);
    double filtered_lvx = getFilteredParam(linear_vel_x_vec, pos_of_filtered_lvx);
    double filtered_lvy = getFilteredParam(linear_vel_y_vec, pos_of_filtered_lvy);
    double filtered_lvz = getFilteredParam(linear_vel_z_vec, pos_of_filtered_lvz);
    double filtered_avx = getFilteredParam(angular_vel_x_vec, pos_of_filtered_avx);
    double filtered_avy = getFilteredParam(angular_vel_y_vec, pos_of_filtered_avy);
    double filtered_avz = getFilteredParam(angular_vel_z_vec, pos_of_filtered_avz);
    nav_msgs::Odometry filtered_odom;
    filtered_odom.pose.pose.position.x = filtered_x;
    filtered_odom.pose.pose.position.y = filtered_y;
    filtered_odom.pose.pose.position.z = filtered_z;
    filtered_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(filtered_yaw); 
    filtered_odom.twist.twist.linear.x = filtered_lvx;
    filtered_odom.twist.twist.linear.y = filtered_lvy;
    filtered_odom.twist.twist.linear.z = filtered_lvz;
    filtered_odom.twist.twist.angular.x = filtered_avx;
    filtered_odom.twist.twist.angular.y = filtered_avy;
    filtered_odom.twist.twist.angular.z = filtered_avz;

    return filtered_odom;    
  }

  return slide_window_->at(time_c - 1);
}

std::pair<size_t, size_t> SlideAvrFilter::findMinAndMaxParam(std::vector<double>& slide_window){
  size_t max_pos = 0;
  size_t min_pos = 0;
  double max_value = slide_window[0];
  double min_value = max_value;

  std::vector<double>::iterator iter = slide_window.begin();

  for(; iter != slide_window.end(); ++iter){
    double temp_value = *iter;
    if(temp_value > max_value){
      max_pos = iter - slide_window.begin();
    }
    if(temp_value < min_value){
      min_pos = iter - slide_window.begin();
    }
  }

  return std::pair<size_t, size_t>(min_pos, max_pos);
}

double SlideAvrFilter::getFilteredParam(std::vector<double>& slide_window, std::pair<size_t, size_t>& filtered_pos){
  double sum = 0;
  for(int ind = 0; ind < slide_window.size(); ++ind){
    if(ind != filtered_pos.first && ind != filtered_pos.second){
      sum += slide_window[ind];
    }
  }
  return sum / (slide_window.size() - 2);
}

