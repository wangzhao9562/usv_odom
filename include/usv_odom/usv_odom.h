/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  usv_odom.h
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/2
  * - Brief:     Implementation for rasperiberry to obtain data from base controller through serial port, and publish odometry parameters of usv
  ******************************************************************************
  * History:
  * 2019/12/4   
  * Complete interface of UsvOdom class, complie successfully
  ******************************************************************************
  * History:
  * 2019/12/17 
  * Overload UsvOdom::sendCommand
  ******************************************************************************
  * History:
  * 2019/12/18 
  * Add mutex for serial port writing
  * Add ros service to set origin point of USV
  ******************************************************************************
*/

#ifndef USV_ODOM_H_
#define USV_ODOM_H_

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <usv_odom/serial_port.h>
#include <usv_odom/unpack_protocol.h>
#include <usv_odom/pack_protocol.h>

#include <geometry_msgs/Quaternion.h>
#include <geographic_msgs/GeoPoint.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <usv_odom/SetOrigin.h>
#include <usv_odom/slide_avr_filter.h>

#include <cmath>
#include <string>
#include <algorithm>

class UsvOdom{

public:
  /**
   * @brief Constructor of UsvOdom
   */
  UsvOdom();

  /**
   * @brief Constructor of UsvOdom
   * @param pub_time Publish interval of odom publisher 
   */
  UsvOdom(int pub_time);

  /**
   * @brief Deconstructor of UsvOdom
   */
  ~UsvOdom();

  /**
   * @brief Publish odometry of usv
   */
  bool publishOdom(); 

private:
  /**
   * @brief Initialization
   */
  void initialize();

  /** 
   * @brief Open serial port
   * @return True if open port successfully, otherwise return false
   */
  bool openSerialPort();

  /**
   * @brief Unpack the data stack
   */
  std::vector<uint8_t> dataProcess(std::vector<uint8_t> read_buf);
  // std::string dataProcess(std::string read_buf);

  /**
   * @brief Unpack the data stack
   */
  int dataProcess(std::vector<uint8_t> read_buf, int buf_len);
  // int dataProcess(const char* buf, int buf_len);

  /**
   * @brief Callback function of next goal message subscriber
   * @param next_goal Next goal point to follow with
   */
  void nextGoalCb(const geometry_msgs::PoseStamped::ConstPtr& next_goal);

  /*
   * @brief Send command to base controller
   */
  bool sendCommand(std::string command);

  /*
   * @brief Send command to base controller
   * @param command Command to be sent
   * @param command_len Length of command
   * @param If command is written into port successfully, return true. Otherwise return false
   */
  bool sendCommand(uint8_t* command, size_t command_len);
  
  /**
   * @biref Test interface, print received string to console
   */
  void testPrint(std::vector<uint8_t> read_buf); 
 
  /**
   * @biref Test interface, print received string to console
   * @param data_buf Data to print on control console
   * @param data_len Length of data stack
   */
  void testPrint(uint8_t* data_buf, size_t data_len); 

  /**
   * @brief Set origin point of USV
   * @param req Request from client which invoke the service
   * @param res Response to client which invoke the service
   * @return If command is written into port successfully, return true. Ohterwise return false.
   */
  bool setOrigin(usv_odom::SetOrigin::Request& req, usv_odom::SetOrigin::Response& res);

private:
  SerialPort* serial_port_;

  std::string odom_frame_;
  std::string robot_base_frame_;

  // read length
  size_t read_len_;

  // ship num
  int ship_num_;

  // odom message
  long double ori_lat_;
  long double ori_lng_;

  // old position parameters
  double pre_north_;
  double pre_east_;
  double pre_yaw_;

  // ros components
  ros::Publisher odom_pub_;
  ros::Publisher geographic_pos_pub_;
  ros::Publisher ned_pos_pub_;

  ros::Subscriber goal_sub_;

  ros::ServiceServer set_origin_srv_;
 
  int pub_interval_;

  int rud_; // record rud cmd
  int speed_; // record speed cmd

  boost::mutex 	write_mutex_; // mutex for parameters write and read

  size_t count_;

  OdomFilter* odom_filter_;
  int filter_st_;

  boost::posix_time::ptime last_pub_time_;
};

#endif

