/**
  i*****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  serial_port.h
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/2
  * - Brief:     Implementation for rasperiberry to obtain data from base controller through serial port
  ******************************************************************************
  * History:
  * 2019/12/2   
  * Complete interface of SerialPort class, complie successfully
  ******************************************************************************
*/

#include <usv_odom/serial_port.h>

SerialPort::SerialPort(std::string port_num, int baud_rate) : is_port_open_(false), read_str_(""){
  ros_serial_port_ =  new serial::Serial();
  initialize(port_num, baud_rate); // initialize
}

SerialPort::SerialPort(std::string port_num, int baud_rate, int time_out) : is_port_open_(false), read_str_(""){
  ros_serial_port_ = new serial::Serial();
  initialize(port_num, baud_rate, time_out); // initialize
}

SerialPort::~SerialPort(){
  if(ros_serial_port_){
    if(is_port_open_){
      ros_serial_port_->close();
    }
    delete ros_serial_port_;
  }
}

void SerialPort::initialize(std::string port_num, int baud_rate){
  ros_serial_port_->setPort(port_num); // set port 
  ros_serial_port_->setBaudrate(baud_rate); // set baud rate
}

void SerialPort::initialize(std::string port_num, int baud_rate, int time_out){
  try{
    ros_serial_port_->setPort(port_num); // set port 
    ros_serial_port_->setBaudrate(baud_rate); // set baud rate
    serial::Timeout to = serial::Timeout::simpleTimeout(time_out);
    ros_serial_port_->setTimeout(to); // set timeout
  }catch(serial::IOException& e){
    ROS_ERROR_STREAM("serial_port: Set serial port failed!");
  }
}

bool SerialPort::openPort(){
  try{
    ros_serial_port_->open();
  }
  catch(serial::IOException& e){
    ROS_ERROR_STREAM("serial_port: Open port failed!");
    return false;
  }
  
  is_port_open_ = true;

  return true;
}

/*
bool SerialPort::readFromPort(){
  if(ros_serial_port_->available()){
   std_msgs::String serial_data;
   serial_data.data = ros_serial_port_->read(ros_serial_port_->available());
   std::cout << serial_data.data.c_str() << std::endl;
   read_str_ += serial_data.data; // stich data stream
   return true;
  }
  return false;
}
*/

bool SerialPort::readFromPort(){
  if(ros_serial_port_->available()){
    size_t read_len = ros_serial_port_->available();

    uint8_t read_buf[MAX_BUF_SIZE]; // create temp read buffer
    read_len = ros_serial_port_->read(read_buf, read_len);

    read_queue_.insert(read_queue_.end(), read_buf, read_buf + read_len); // push data into read queue  
    return true;
  }
  return false;
}

bool SerialPort::readFromPort(size_t read_len){
  if(ros_serial_port_->available()){

    uint8_t read_buf[MAX_BUF_SIZE]; // create temp read buffer
    read_len = ros_serial_port_->read(read_buf, read_len);

    std::cout << "read length: " << read_len;

    read_queue_.insert(read_queue_.end(), read_buf, read_buf + read_len); // push data into read queue  
    return true;
  }
  return false;
}

void SerialPort::writeInToPort(std::string data_str){
  ros_serial_port_->write(data_str);
}

void SerialPort::writeInToPort(uint8_t* data_str, size_t data_len){
  ros_serial_port_->write(data_str, data_len);
}

void SerialPort::updateBuffer(std::vector<uint8_t> new_buf){
  read_queue_ = new_buf;
}
// void SerialPort::updateBuffer(std::string new_buf){
//   read_str_ = new_buf;
// }

void SerialPort::flushPort(){
  ros_serial_port_->flush();
} 
