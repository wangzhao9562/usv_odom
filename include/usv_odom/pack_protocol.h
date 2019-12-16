/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  pack_protocol.h
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/3
  * - Brief:     Provide interface for packing protocol
  ******************************************************************************
  * History:
  * 2019/12/4  
  * Complete interface of PackProtocol struct
  ******************************************************************************
*/

#ifndef PACK_PROTOCOL_H_
#define PACK_PROTOCOL_H_

#include <string>
#include <utility>

enum class MissionType{
  PID_SETTING, // set PID parameter
  FIXED_VEL_NAV, // navigating with fixed velocity
  FIXED_YAW_NAV, // navigating with fixed yaw
  POINT_FOLLOW,  // follow point
  LINE_FOLLOW, // follow line
  ORI_SETTING  // set position of origin point 
};

struct PackProtocol{
  struct FixedVelNav{};
  struct FixedYawNav{};

  // data bit information
  static const char pack_head_ = 0xA1;
  static const char pack_sec_bit_ = 0X1A;
  static const char pack_tail_ = 0xAA;
  
  static const char pack_len_pid_ = 0x0C;
  static const char pack_len_fixed_nav_ = 0x07;
  static const char pack_len_point_ = 0x0E;
  static const char pack_len_line_ = 0x16;
  static const char pack_len_ori_ = 0x06;
  static const int pack_len_pid_int_ = 12;

  static const char fbit_pid_ = 0xF2;
  static const char fbit_fixed_vel_ = 0xB1;
  static const char fbit_fixed_yaw_ = 0xB2;
  static const char fbit_point_ = 0xB3;
  static const char fbit_line_ = 0xB6;
  static const char fbit_ori_ = 0xF3;

  static const char ship_num_ = 0x00;

  // packing protocol
  static std::string getDataStack(int ship_num, double kp, double ki, double kd, double kp1, double ki1, double kd1); 
  static std::string getDataStack(int ship_num, double vel, FixedVelNav);
  static std::string getDataStack(int ship_num, double yaw, FixedYawNav);
  static std::string getDataStack(int ship_num, double lat, double lng);
  static std::string getDataStack(int ship_num, double lat1, double lng1, double lat2, double lng2); // Invalid
  static std::string getDataStack(int ship_num);
};

#endif
