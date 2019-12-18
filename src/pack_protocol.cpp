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
  * History:
  * 2019/12/17  
  * Add open control data stack interface 
  ******************************************************************************
  * History:
  * 2019/12/18 
  * Modify data stack package interface
  ******************************************************************************
*/

#include <usv_odom/pack_protocol.h>

// Open control
std::vector<uint8_t> PackProtocol::getDataStack(int ship_num, int rud_det, int speed_det, int& rud, int& speed){
  uint8_t data_stack[PackProtocol::pack_len_open_] = {PackProtocol::pack_head_, PackProtocol::pack_sec_bit_, pack_len_open_, PackProtocol::ship_num_, PackProtocol::fbit_open_, 0x00, 0x00, PackProtocol::pack_tail_};
 
  data_stack[3] = static_cast<uint8_t>(ship_num); 
  if(rud == PackProtocol::max_left_rud_ || rud == PackProtocol::max_right_rud_){
    data_stack[5] = static_cast<uint8_t>(rud);
  }
  else{
    rud = rud + rud_det;
    data_stack[5] = static_cast<uint8_t>(rud);
  }
  if(speed == PackProtocol::max_velocity_ || speed == 0){
    data_stack[6] = static_cast<uint8_t>(speed);
  }
  else{
    speed = speed + speed_det;
    data_stack[6] = static_cast<uint8_t>(speed);
  }

  std::vector<uint8_t> data_queue;
  data_queue.insert(data_queue.end(), data_stack, data_stack + PackProtocol::pack_len_open_);
  
  return data_queue;
}

// PID setting 
std::vector<uint8_t> PackProtocol::getDataStack(int ship_num, double kp, double ki, double kd, double kp1, double ki1, double kd1){
  uint8_t data_stack[PackProtocol::pack_len_pid_int_] = {PackProtocol::pack_head_, PackProtocol::pack_sec_bit_, pack_len_pid_, PackProtocol::ship_num_, PackProtocol::fbit_pid_, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, PackProtocol::pack_tail_}; // default data stack

  data_stack[3] = static_cast<uint8_t>(ship_num);
  data_stack[5] = static_cast<uint8_t>(kp * 10);
  data_stack[6] = static_cast<uint8_t>(ki * 100);
  data_stack[7] = static_cast<uint8_t>(kd * 10);
  data_stack[8] = static_cast<uint8_t>(kp1 * 10);
  data_stack[9] = static_cast<uint8_t>(ki1 * 100);
  data_stack[10] = static_cast<uint8_t>(kd1 * 10);

  std::vector<uint8_t> data_queue;
  data_queue.insert(data_queue.end(), data_stack, data_stack + PackProtocol::pack_len_pid_int_);
  
  return data_queue;
}

// Fixed velocity navigating
std::vector<uint8_t> PackProtocol::getDataStack(int ship_num, double vel, FixedVelNav){
  uint8_t data_stack[PackProtocol::pack_len_fixed_nav_] = {PackProtocol::pack_head_, PackProtocol::pack_sec_bit_, pack_len_fixed_nav_, PackProtocol::ship_num_, PackProtocol::fbit_fixed_vel_, 0x00, PackProtocol::pack_tail_};
  data_stack[3] = static_cast<uint8_t>(ship_num);
  data_stack[5] = static_cast<uint8_t>(vel * 10);

  std::vector<uint8_t> data_queue;
  data_queue.insert(data_queue.end(), data_stack, data_stack + PackProtocol::pack_len_fixed_nav_);
 
  return data_queue;
}

// Fixed orientation navigating
std::vector<uint8_t> PackProtocol::getDataStack(int ship_num, double yaw, FixedYawNav){
  uint8_t data_stack[PackProtocol::pack_len_fixed_nav_] = {PackProtocol::pack_head_, PackProtocol::pack_sec_bit_, pack_len_fixed_nav_, PackProtocol::ship_num_, PackProtocol::fbit_fixed_vel_, 0x00, PackProtocol::pack_tail_};
  data_stack[3] = static_cast<uint8_t>(ship_num);
  data_stack[5] = static_cast<uint8_t>(yaw);

  std::vector<uint8_t> data_queue;
  data_queue.insert(data_queue.end(), data_stack, data_stack + PackProtocol::pack_len_fixed_nav_);
  
  return data_queue;
}

// Point follow
std::vector<uint8_t> PackProtocol::getDataStack(int ship_num, double lat, double lng){
  uint8_t data_stack[PackProtocol::pack_len_point_] = {PackProtocol::pack_head_, PackProtocol::pack_sec_bit_, PackProtocol::pack_len_point_, PackProtocol::ship_num_, PackProtocol::fbit_point_, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, PackProtocol::pack_tail_};
  
  data_stack[3] = static_cast<uint8_t>(ship_num);

  int lat_int = static_cast<int>(lat - 30) * 100000000;
  int lng_int = static_cast<int>(lng - 114) * 100000000;

  data_stack[5] = static_cast<uint8_t>(lat_int >> 24);
  data_stack[6] = static_cast<uint8_t>(lat_int >> 16);
  data_stack[7] = static_cast<uint8_t>(lat_int >> 8);
  data_stack[8] = static_cast<uint8_t>(lat_int);

  data_stack[9] = static_cast<uint8_t>(lng_int >> 24);
  data_stack[10] = static_cast<uint8_t>(lng_int >> 16);
  data_stack[11] = static_cast<uint8_t>(lng_int >> 8);
  data_stack[12] = static_cast<uint8_t>(lng_int);

  std::vector<uint8_t> data_queue;
  data_queue.insert(data_queue.end(), data_stack, data_stack + PackProtocol::pack_len_point_);
  
  return data_queue; 
}

// Common line follow
std::vector<uint8_t> PackProtocol::getDataStack(int ship_num, double lat1, double lng1, double lat2, double lng2){
  /* Invalid */ 
  return std::vector<uint8_t>();
}

// Origin point setting
std::vector<uint8_t> PackProtocol::getDataStack(int ship_num){
  uint8_t data_stack[PackProtocol::pack_len_ori_] = {PackProtocol::pack_head_, PackProtocol::pack_sec_bit_, PackProtocol::pack_len_ori_, PackProtocol::ship_num_, PackProtocol::fbit_ori_, PackProtocol::pack_tail_};

  data_stack[3] = static_cast<uint8_t>(ship_num);

  std::vector<uint8_t> data_queue;
  data_queue.insert(data_queue.end(), data_stack, data_stack + PackProtocol::pack_len_ori_);
  
  return data_queue;
}
