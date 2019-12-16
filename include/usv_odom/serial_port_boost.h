/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  serial_porti_boost.h
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/5
  * - Brief:     Implementation for rasperiberry to obtain data from base controller through serial port based on boost library
  ******************************************************************************
  * History:
  * 2019/12/5   
  * Complete interface of SerialPortBoost class, complie successfully
  ******************************************************************************
*/

#ifndef SERIAL_PORT_BOOST_H_
#define SERIAL_PORT_BOOST_H_

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <iostream>

typedef boost::function<void(char*, size_t)> RecvCb;	

class SerialPortBoost{

public:
  SerialPortBoost(std::string port_name, int baud_rate);

  ~SerialPortBoost();

  void setRecvCb(RecvCb pfunc){ recv_cb_ = pfunc; };

  void setReadLen(int read_len){ read_len_ = read_len; };

  void readFromSerialPort();

  void writeInToSerialPort(std::string data);

private:
  void call(){ io_service_.run(); };

  bool initialize(std::string port_name, int baud_rate);
  
  void handleRead(char buf[], boost::system::error_code ec, size_t bytes_transferred);

  void readThread();

private:
  boost::asio::io_service io_service_;
  boost::asio::serial_port* serial_port_;
  boost::system::error_code ec_;

  boost::thread* read_thread_;
  int read_len_; 

  bool is_port_open_;

  RecvCb recv_cb_;
  char read_buffer_[1024];
};

#endif
