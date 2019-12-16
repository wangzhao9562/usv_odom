/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  serial_port.h
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/4
  * - Brief:     Implementation for rasperiberry to obtain data from base controller through serial port
  ******************************************************************************
  * History:
  * 2019/12/4   
  * Complete interface of SerialPort class, complie successfully
  ******************************************************************************
*/

#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_	

#include <serial/serial.h>
#include <string>
#include <vector>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <usv_odom/unpack_protocol.h>

#define MAX_BUF_SIZE 1024

class SerialPort{

public:
  /**
   * @brief Constructor of SerialPort
   * @param port_num Port number to open
   * @param baud_rate Baud rate of opened serial port
   */
  SerialPort(std::string port_num, int baud_rate);

  /**
   * @brief Constructor of SerialPort
   * @param port_num Port number to open
   * @param baud_rate Baud rate of opened serial port
   * @param time_out Time out for Serial class
   */
  SerialPort(std::string port_num, int baud_rate, int time_out);

  /** 
   * @brief Deconstructor of SerialPort
   */
  virtual ~SerialPort();

  /** 
   * @brief Initialize parameters
   * @param port_num Port number to open
   * @param baud_rate Baud rate of serial port 
   */
  virtual void initialize(std::string port_num, int baud_rate);

  /** 
   * @brief Initialize parameters
   * @param port_num Port number to open
   * @param baud_rate Baud rate of serial port 
   * @param time_out Time out setting for Class Serial
   */
  virtual void initialize(std::string port_num, int baud_rate, int time_out);

  /**
   * @brief Open serial port
   * @return If open port successfully, return true, otherwise, false;
   */
  virtual bool openPort();

  /**
   * @brief Return status of port
   * @return True if port is open, otherwise false
   */
  virtual bool isPortOpen(){ return ros_serial_port_->isOpen(); };

  /**
   * @brief Read data from serial port
   */
  virtual bool readFromPort();

  /**
   * @brief Read data from serial port
   * @param read_len Read length
   */
  virtual bool readFromPort(size_t read_len);
  
  /**
   * @brief Update read buffer
   */
  virtual void updateBuffer(std::vector<uint8_t> new_buf);
  // virtual void updateBuffer(std::string new_buf);
  
  /* @brief Wrtie data into serial port
   */
  virtual void writeInToPort(std::string data_str);

  /* @brief Flush serial port
   */
  virtual void flushPort();

  /* @brief Close serial port
   */
  virtual void closePort(){ ros_serial_port_->close(); };
  
  /* @brief Output read queue
   * @return Return read queue
   */
  virtual std::vector<uint8_t> getReadQueue(){ return read_queue_; };
  // virtual std::string getReadBuffer(){ return read_buf_; };

public:
  serial::Serial* ros_serial_port_;  
  std::string read_str_;
  std::vector<uint8_t> read_queue_;
 
  bool is_port_open_;
};

#endif 
