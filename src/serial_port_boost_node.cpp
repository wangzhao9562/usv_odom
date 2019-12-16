/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  serial_porti_boost_node.cpp
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/5
  * - Brief:     Test node for SerialPortBoost
  ******************************************************************************
  * History:
  * 2019/12/5   
  * Complete node for SerailPortBoost, complie successfully
  ******************************************************************************
*/

#include <ros/ros.h>
#include <usv_odom/serial_port_boost.h>
#include <iostream>

void printTest(char* buf, size_t bytes_transferred){
  std::cout << "serial_port_node: print read buffer " << bytes_transferred << " ";
  std::cout.write(buf, bytes_transferred);
  std::cout << std::endl;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "serial_port_boost_node");
  ros::NodeHandle nh;
  
  std::string port_name = "/dev/ttyUSB2";
  int baud_rate = 9600;

  SerialPortBoost serial_port(port_name, baud_rate);  
  serial_port.setRecvCb(boost::bind(&printTest, _1, _2));
  serial_port.setReadLen(1);
  // ros::Rate loop_rate(100);

  serial_port.readFromSerialPort();
    
  ros::spin();

  return 0;
}
