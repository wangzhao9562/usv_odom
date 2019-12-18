/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  unpack_protocol.h
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/3
  * - Brief:     Provide interface for unpack protocol
  ******************************************************************************
  * History:
  * 2019/12/3  
  * Complete interface of UnpackProtocol struct
  ******************************************************************************
*/

#ifndef UNPACK_PROTOCOL_H_
#define UNPACK_PROTOCOL_H_

#include <cmath>

const double PI = 3.1415926535;

/**
 * @brief Provide transfer parameter for gps data to lat and lng
 */
struct GpsParam{
  static constexpr double earth_a_ = 6378137.0;
  static constexpr double earth_e_ = 0.003352810664;
};

/**
 * @brief Provide data bits of parameters according to communiction protocol 
 */
struct UnpackProtocol{
  // data bit information
  static const uint8_t pack_head_ = 0xA5; 
  static const uint8_t pack_tail_ = 0xAA; 
  static const uint8_t pack_sec_bit_ = 0x5A;

  static const size_t pack_len_ = 30;

  static const size_t pid_bit_ = 13; // PID ouput
  static const size_t yaw_bit_ = 14; // orientation
  static const size_t time_bit_ = 15; // time in second
  static const size_t rud_ang_bit_ = 18; // rudder angle 
  static const size_t gear_bit_ = 19; // gear

  // data bit of real-time position
  static const size_t lat_bit_1_ = 5;
  static const size_t lat_bit_2_ = 6;
  static const size_t lat_bit_3_ = 7;
  static const size_t lat_bit_4_ = 8;
  
  static const size_t lng_bit_1_ = 9;
  static const size_t lng_bit_2_ = 10;
  static const size_t lng_bit_3_ = 11;
  static const size_t lng_bit_4_ = 12;
 
  // data bit of velocity
  static const size_t speed_bit_1_ = 16;
  static const size_t speed_bit_2_ = 17;

  // data bit of origin position
  static const size_t ori_lat_bit_1_ = 20;
  static const size_t ori_lat_bit_2_ = 21;
  static const size_t ori_lat_bit_3_ = 22;
  static const size_t ori_lat_bit_4_ = 23;
   
  static const size_t ori_lng_bit_1_ = 24;
  static const size_t ori_lng_bit_2_ = 25;
  static const size_t ori_lng_bit_3_ = 26;
  static const size_t ori_lng_bit_4_ = 27;

  /**
   * @brief Calculate rudder angle
   * @param rud_byte Data bit of rudder angle
   * @return calculate result
   */
  static double getRudAng(char rud_byte){
    return (rud_byte - 32) * 0.9;
  }

  /**
   * @brief Calculate yaw
   * @param yaw_byte Data bit of yaw
   * @return calculate result
   */
  static double getYaw(char raw_byte){
    return (raw_byte - 180);
  }

  /**
   * @brief Calculate pid output
   * @param pid_byte Data bit of pid
   * @return calculate result
   */
  static double getPidOutput(char pid_byte){
    return (pid_byte - 200) / 10.0;
  }

  /**
   * @brief Calculate speed
   * @param speed_byte1 First data bit of speed
   * @param speed_byte2 Second data bit of speed
   * @return calculate result
   */
  static double getSpeed(char speed_byte1, char speed_byte2){
    return ((speed_byte1 << 8) + (speed_byte2)) * 0.514 / 1000.0;
  }

  /**
   * @brief Calculate latitude
   * @param lat_byte1 First data bit of latitude
   * @param lat_byte2 Second data bit of latitude
   * @param lat_byte3 Third data bit of latitude
   * @param lat_byte4 Forth data bit of latitude
   * @return calculate result
   */
  static double getLat(char lat_byte1, char lat_byte2, char lat_byte3, char lat_byte4){
    return static_cast<double>(static_cast<int>(lat_byte1) << 24) + (static_cast<int>(lat_byte2 << 16) + static_cast<int>(lat_byte3 << 8) + static_cast<int>(lat_byte4)) / 100000000 + 30;
  }
 
  /**
   * @brief Calculate longitude
   * @param lng_byte1 First data bit of longitude
   * @param lng_byte2 Second data bit of longitude
   * @param lng_byte3 Third data bit of longitude
   * @param lng_byte4 Forth data bit of longitude
   * @return calculate result
   */
  static double getLng(char lng_byte1, char lng_byte2, char lng_byte3, char lng_byte4){
    return static_cast<double>(static_cast<int>(lng_byte1) << 24) + (static_cast<int>(lng_byte2 << 16) + static_cast<int>(lng_byte3 << 8) + static_cast<int>(lng_byte4)) / 100000000 + 114;
  }

  /**
   * @brief Calculate north coordination in NED
   * @param lat latitude
   * @param ori_lat latitude of origin point
   * @param Caculate result
   */
  static double getNorth(double lat, double ori_lat){
    return (lat - ori_lat) * GpsParam::earth_a_ * (1 - std::pow(GpsParam::earth_e_, 2)) * PI / (180 * std::sqrt(std::pow((1 - std::pow(GpsParam::earth_e_ * std::sin(lat * PI / 180), 2)), 3))); 
  }

  /**
   * @brief Calculate east coordination in NED
   * @param lat latitude
   * @param lng longitude
   * @param ori_lng longitude of origin point
   * @param Caculate result
   */
  static double getEast(double lng, double lat, double ori_lng){
    return (lng - ori_lng) * GpsParam::earth_a_ * std::cos(lat * PI / 180) * PI / (180 * std::sqrt(1 - std::pow(GpsParam::earth_e_ * std::sin(lat * PI / 180), 2)));
  }

  /**
   * @brief Calculate latitude according to NED
   * @param north X coordinate in NED
   * @param ori_lat Latitiude of origin point 
   * @return Result of calculation
   */
  static double getLat(double north,  double ori_lat){
    return (ori_lat + north * std::sqrt(std::pow(1 - std::pow(GpsParam::earth_e_ * std::sin(ori_lat * PI / 180), 2), 3)) * 180 / (GpsParam::earth_a_ * (1 - std::pow(GpsParam::earth_e_ , 2)) * PI));
  }
  
  /**
   * @brief Calculate longitude according to NED
   * @param east Y coordinate in NED
   * @param ori_lat Latitiude of origin point 
   * @param ori_lng Longitude of origin point
   * @return Result of calculation
   */
  static double getLng(double east, double ori_lat, double ori_lng){
    return (ori_lng + east * std::sqrt(1 - std::pow(GpsParam::earth_e_ * std::sin(ori_lat * PI / 180), 2)) * 180 / (GpsParam::earth_a_ * std::cos(ori_lat * PI / 180) * PI));
  }
};

#endif

