/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  pack_protocol.cpp
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/4
  * - Brief:     Implementation of interfaces of PackProtocol
  ******************************************************************************
  * History:
  * 2019/12/4  
  * Complete interface of PackProtocol struct
  ******************************************************************************
*/

#include <usv_odom/pack_protocol.h>

// PID setting 
std::string PackProtocol::getDataStack(int ship_num, double kp, double ki, double kd, double kp1, double ki1, double kd1){
  char data_stack[PackProtocol::pack_len_pid_int_] = {PackProtocol::pack_head_, PackProtocol::pack_sec_bit_, pack_len_pid_, PackProtocol::ship_num_, PackProtocol::fbit_pid_, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, PackProtocol::pack_tail_}; // default data stack

  data_stack[3] = static_cast<char>(ship_num);
  data_stack[5] = static_cast<char>(kp * 10);
  data_stack[6] = static_cast<char>(ki * 100);
  data_stack[7] = static_cast<char>(kd * 10);
  data_stack[8] = static_cast<char>(kp1 * 10);
  data_stack[9] = static_cast<char>(ki1 * 100);
  data_stack[10] = static_cast<char>(kd1 * 10);

  return std::string(data_stack);
}

// Fixed velocity navigating
std::string PackProtocol::getDataStack(int ship_num, double vel, FixedVelNav){
  char data_stack[PackProtocol::pack_len_fixed_nav_] = {PackProtocol::pack_head_, PackProtocol::pack_sec_bit_, pack_len_fixed_nav_, PackProtocol::ship_num_, PackProtocol::fbit_fixed_vel_, 0x00, PackProtocol::pack_tail_};
  data_stack[3] = static_cast<char>(ship_num);
  data_stack[5] = static_cast<char>(vel * 10);

  return std::string(data_stack);
}

// Fixed orientation navigating
std::string PackProtocol::getDataStack(int ship_num, double yaw, FixedYawNav){
  char data_stack[PackProtocol::pack_len_fixed_nav_] = {PackProtocol::pack_head_, PackProtocol::pack_sec_bit_, pack_len_fixed_nav_, PackProtocol::ship_num_, PackProtocol::fbit_fixed_vel_, 0x00, PackProtocol::pack_tail_};
  data_stack[3] = static_cast<char>(ship_num);
  data_stack[5] = static_cast<char>(yaw);

  return std::string(data_stack);
}

// Point follow
std::string PackProtocol::getDataStack(int ship_num, double lat, double lng){
  char data_stack[PackProtocol::pack_len_point_] = {PackProtocol::pack_head_, PackProtocol::pack_sec_bit_, PackProtocol::pack_len_point_, PackProtocol::ship_num_, PackProtocol::fbit_point_, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, PackProtocol::pack_tail_};
  
  data_stack[3] = static_cast<char>(ship_num);

  int lat_int = static_cast<int>(lat - 30) * 100000000;
  int lng_int = static_cast<int>(lng - 114) * 100000000;

  data_stack[5] = static_cast<char>(lat_int >> 24);
  data_stack[6] = static_cast<char>(lat_int >> 16);
  data_stack[7] = static_cast<char>(lat_int >> 8);
  data_stack[8] = static_cast<char>(lat_int);

  data_stack[9] = static_cast<char>(lng_int >> 24);
  data_stack[10] = static_cast<char>(lng_int >> 16);
  data_stack[11] = static_cast<char>(lng_int >> 8);
  data_stack[12] = static_cast<char>(lng_int);
 
  return std::string(data_stack); 
}

// Common line follow
std::string PackProtocol::getDataStack(int ship_num, double lat1, double lng1, double lat2, double lng2){
  /* Invalid */
  return std::string("");
}

// Origin point setting
std::string PackProtocol::getDataStack(int ship_num){
  char data_stack[PackProtocol::pack_len_ori_] = {PackProtocol::pack_head_, PackProtocol::pack_sec_bit_, PackProtocol::pack_len_ori_, PackProtocol::ship_num_, PackProtocol::fbit_ori_, PackProtocol::pack_tail_};

  data_stack[3] = static_cast<char>(ship_num);

  return std::string(data_stack);
}
