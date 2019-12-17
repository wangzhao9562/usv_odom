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
*/

#ifndef USV_ODOM_H_
#define USV_ODOM_H_

#include <usv_odom/serial_port.h>
#include <usv_odom/unpack_protocol.h>
#include <usv_odom/pack_protocol.h>

#include <geometry_msgs/Quaternion.h>
#include <geographic_msgs/GeoPoint.h>
#include <tf/transform_datatypes.h>

#include <algorithm>

class UsvOdom{

public:
  /**
   * @brief Constructor of UsvOdom
   */
  UsvOdom();

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
  // void testPrint(std::string str);
 
private:
  SerialPort* serial_port_;

  // read length
  size_t read_len_;

  // ship num
  int ship_num_;

  // odom message
  double ori_lat_;
  double ori_lng_;

  // ros components
  ros::Publisher odom_pub_;
  ros::Publisher geographic_pos_pub_;
  ros::Publisher ned_pos_pub_;

  ros::Subscriber goal_sub_;

  int rud_; // record rud cmd
  int speed_; // record speed cmd
};

#endif

