/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  serial_porti_boost.cpp
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/5
  * - Brief:     Implementation for rasperiberry to obtain data from base controller through serial port based on boost library
  ******************************************************************************
  * History:
  * 2019/12/5   
  * Complete implementation of interfaces of SerialPortBoost class, complie successfully
  ******************************************************************************
*/

#include <usv_odom/serial_port_boost.h>
#include <iostream>
#include <algorithm>

SerialPortBoost::SerialPortBoost(std::string port_name, int baud_rate) : is_port_open_(false){
  serial_port_ = new boost::asio::serial_port(io_service_);
  if(serial_port_){
    initialize(port_name, baud_rate);
  }
}

SerialPortBoost::~SerialPortBoost(){
  if(read_thread_){
    read_thread_->join();
    delete read_thread_;
  }

  if(serial_port_){
    if(is_port_open_){
      serial_port_->close();
      io_service_.stop();
      io_service_.reset();
    }
    delete serial_port_;
  }
}

bool SerialPortBoost::initialize(std::string port_name, int baud_rate){
  if(!serial_port_){
    return false;
  }
  serial_port_->open(port_name, ec_);
  serial_port_->set_option(boost::asio::serial_port::baud_rate(baud_rate), ec_);
  serial_port_->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none), ec_);
  serial_port_->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none), ec_);
  serial_port_->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one),ec_);
  serial_port_->set_option(boost::asio::serial_port::character_size(8),ec_);

  if(!ec_){
    is_port_open_ = true;
    std::cout << "serial_port: port is open!" << std::endl;
  }
  else{
    std::cout << "serial port: open port failed!" << std::endl;
  }
  
  return true;	
}

void SerialPortBoost::handleRead(char buf[], boost::system::error_code ec, size_t bytes_transferred){
  std::cout << "handle read" << std::endl; 
  std::cout << "serial_port: check recv " << bytes_transferred << " ";
  if(recv_cb_ && !ec){
    // std::vector<char> buf_copy;
    // buf_copy.insert(buf_copy.end(), buf, buf + bytes_transferred); 
    // for(int i = 0; i < buf_copy.size(); ++i){
    //   std::cout << buf_copy[i];
    // }
    // std::cout << std::endl;
    recv_cb_(read_buffer_, bytes_transferred);
  }
}

void SerialPortBoost::readFromSerialPort(){
  read_thread_ = new boost::thread(boost::bind(&SerialPortBoost::readThread, this));
}

void SerialPortBoost::writeInToSerialPort(std::string data){
  if(!serial_port_ || !is_port_open_){
    return;
  }

  size_t data_len = boost::asio::write(*serial_port_, boost::asio::buffer(data),ec_);
}

void SerialPortBoost::readThread(){
  while(true){
    if(!serial_port_ || !is_port_open_){
      return;
    }
    std::cout << "read" << std::endl;
    // boost::asio::async_read(*serial_port_, boost::asio::buffer(read_buffer_, read_len_), boost::bind(&SerialPortBoost::handleRead, this, read_buffer_, _1, _2));
    serial_port_->async_read_some(boost::asio::buffer(read_buffer_), boost::bind(&SerialPortBoost::handleRead, this, read_buffer_, _1, _2));
    std::cout << "wait" << std::endl; 
    boost::this_thread::sleep(boost::posix_time::millisec(200));
  }

  call();
}

