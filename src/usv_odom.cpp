/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  usv_odom.cpp
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/4
  * - Brief:     Interface implementation of UsvOdom Class
  ******************************************************************************
  * History:
  * 2019/12/4   
  * Complete interface of UsvOdom class, complie successfully
  ******************************************************************************
*/

#include <usv_odom/usv_odom.h>

UsvOdom::UsvOdom(){
  int baud_rate, time_out;
  std::string port_num;

  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  private_nh.param("port_num", port_num, std::string("/dev/ttyUSB0")); // for windows
  private_nh.param("baud_rate", baud_rate, 9600);
  private_nh.param("time_out", time_out, 200);
  private_nh.param("ship_num", ship_num_, 1);

  odom_pub_ = nh.advertise<geometry_msgs::PoseStamped>("odom", 1);
  geographic_pos_pub_ = nh.advertise<geographic_msgs::GeoPoint>("geo_position", 1);
  goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("next_goal", 1, boost::bind(&UsvOdom::nextGoalCb, this, _1));
  
  serial_port_ = new SerialPort(port_num, baud_rate); 

  read_len_ = 1024;

  if(!serial_port_->isPortOpen()){
    if(!UsvOdom::openSerialPort()){
      ROS_INFO("usv_odom: open port failed!");
    }
  }
}

UsvOdom::~UsvOdom(){
  if(serial_port_){
    if(serial_port_->isPortOpen()){
     serial_port_->closePort();
    }
  }
  delete serial_port_;
}

bool UsvOdom::openSerialPort(){
  if(serial_port_->openPort()){
    ROS_INFO("usv_odom: open port!");
    return true;
  }
  return false;
}


bool UsvOdom::publishOdom(){
  if(serial_port_->isPortOpen()){
    if(serial_port_->readFromPort(read_len_)){
      // std::string read_data = serial_port_->getReadBuffer(); // get read buffer
      std::vector<uint8_t> read_data = serial_port_->getReadQueue(); // get read buffer
      // std::string read_data = serial_port_->getReadBuffer(); // get read buffer
      // testPrint(read_data);
      
      std::vector<uint8_t> new_buf = dataProcess(read_data); // unpack
      serial_port_->updateBuffer(new_buf); // update buffer
      serial_port_->flushPort();
    }
    return true;
  }
  return false;
}

bool UsvOdom::sendCommand(std::string command){
  if(serial_port_->isPortOpen()){
    serial_port_->writeInToPort(command);
    return true;
  }
  return false;
}

// std::string UsvOdom::dataProcess(std::string read_buf){
std::vector<uint8_t> UsvOdom::dataProcess(std::vector<uint8_t> read_buf){
  size_t cut_pos = dataProcess(read_buf, read_buf.size()); // get cut position
  if(cut_pos != 0){
    read_buf.erase(read_buf.begin(), read_buf.begin() + cut_pos);
    return read_buf;
  }
  return read_buf;
}

// int UsvOdom::dataProcess(const char* buf, int buf_len){
int UsvOdom::dataProcess(std::vector<uint8_t> read_buf, int buf_len){
  // Find pack head
  uint8_t pack_head = UnpackProtocol::pack_head_;
  uint8_t pack_tail = UnpackProtocol::pack_tail_;
 
  int cut_pos = 0;

  std::vector<uint8_t>::iterator head_iter = std::find(read_buf.begin(), read_buf.end(), pack_head); // allert! cannot input UnpackProtocol::pack_head_ directly, it will cause 'undefine error' while compile
  // testPrint(read_buf);
  std::cout << "first char in read buffer: " << std::hex << (read_buf[0] & 0xff) << std::endl;
  std::cout << "result of comparison" << (pack_head == read_buf[0]) << std::endl;
 
  if(head_iter != read_buf.end()){
    // If pack head is found, find pack tail
    std::vector<uint8_t>::iterator tail_iter = std::find(read_buf.begin(), read_buf.end(), pack_tail); // allert! cannot input UnpackProtocol::pack_tail_ directly, it will cause 'undefine error' while compile
    if(tail_iter != read_buf.end() && tail_iter - head_iter == UnpackProtocol::pack_len_){
      // Get cut position of str 
      cut_pos = tail_iter - read_buf.begin();
 
      // If pack tail is found and length between head and tail is equal to pack length, check the second bit
      if(*(head_iter + 1) == UnpackProtocol::pack_sec_bit_){
        // unpack
        double rud_ang = UnpackProtocol::getRudAng(*(head_iter + UnpackProtocol::rud_ang_bit_)); // get rudder angle
        double gear = *(head_iter + UnpackProtocol::gear_bit_); // get gear
        double yaw = UnpackProtocol::getYaw(*(head_iter + UnpackProtocol::yaw_bit_)) * PI / 180; // get yaw in rad
        double speed = UnpackProtocol::getSpeed(*(head_iter + UnpackProtocol::speed_bit_1_), *(head_iter + UnpackProtocol::speed_bit_2_)); // get speed
        double time_in_sec = *(head_iter + UnpackProtocol::time_bit_); // get time stamp
        double pid_output = UnpackProtocol::getPidOutput(*(head_iter + UnpackProtocol::pid_bit_)); // get output of pid
        double ori_lat = UnpackProtocol::getLat(*(head_iter + UnpackProtocol::ori_lat_bit_1_), *(head_iter + UnpackProtocol::ori_lat_bit_2_), *(head_iter + UnpackProtocol::ori_lat_bit_3_), *(head_iter + UnpackProtocol::ori_lat_bit_4_)); // get latitude of origin point
        double ori_lng = UnpackProtocol::getLng(*(head_iter + UnpackProtocol::ori_lng_bit_1_), *(head_iter + UnpackProtocol::ori_lng_bit_2_), *(head_iter + UnpackProtocol::ori_lng_bit_3_), *(head_iter + UnpackProtocol::ori_lng_bit_4_)); // get longitude of origin point
        double lat = UnpackProtocol::getLat(*(head_iter + UnpackProtocol::lat_bit_1_), *(head_iter + UnpackProtocol::lat_bit_2_), *(head_iter + UnpackProtocol::lat_bit_3_), *(head_iter + UnpackProtocol::lat_bit_4_)); // get latitude of origin point
        double lng = UnpackProtocol::getLng(*(head_iter + UnpackProtocol::lng_bit_1_), *(head_iter + UnpackProtocol::lng_bit_2_), *(head_iter + UnpackProtocol::lng_bit_3_), *(head_iter + UnpackProtocol::lng_bit_4_));
        double north = UnpackProtocol::getNorth(lat, ori_lat); // get coordination of north in NED
        double east = UnpackProtocol::getEast(lat, lng, ori_lng); // get coordination of east in NED

        // store position of origin point
        ori_lat_ = ori_lat;
        ori_lng_ = ori_lng;

        // create odom message 
        geometry_msgs::PoseStamped odom_msgs;
        odom_msgs.header.stamp = ros::Time::now();
        odom_msgs.pose.position.x = north;
        odom_msgs.pose.position.y = east;
        geometry_msgs::Quaternion quad = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
        odom_msgs.pose.orientation = quad;

        // create geographic position message
        geographic_msgs::GeoPoint geo_pos_msgs;
        geo_pos_msgs.latitude = lat;
        geo_pos_msgs.longitude = lng;

        // publish message
        ROS_INFO("usv_odom: publish odom");
        odom_pub_.publish(odom_msgs);
        geographic_pos_pub_.publish(geo_pos_msgs);

        return cut_pos;
      }
    }
    else{
      int head_pos = head_iter - read_buf.begin();
      if((read_buf.end() - head_iter) < UnpackProtocol::pack_len_){
        cut_pos = head_pos;
      }
      else{
        cut_pos = buf_len;
      }
    }
  }
  
  return cut_pos;
}

void UsvOdom::nextGoalCb(const geometry_msgs::PoseStamped::ConstPtr& next_goal){
  // get coordinate in NED
  double north = next_goal->pose.position.x;
  double east = next_goal->pose.position.y;

  // transfer coordinate from NED to LatLng
  double next_lat = UnpackProtocol::getLat(north, ori_lat_);
  double next_lng = UnpackProtocol::getLng(east, ori_lat_, ori_lng_);

  std::string data_stack = PackProtocol::getDataStack(ship_num_, next_lat, next_lng);
  if(sendCommand(data_stack)){
    ROS_INFO("usv_odom: write into port!");
  }
  else{
    ROS_ERROR("usv_odom: write failed!");
  }
  /* Pack up lat and lng in form of data stack, then call writeToPort */
}

void UsvOdom::testPrint(std::vector<uint8_t> read_buf){
  std::for_each(read_buf.begin(), read_buf.end(), [](uint8_t data){
     std::cout << data << std::hex << (data & 0xff) << " "; 
    }
  );
  std::cout << std::endl;
}

/*
void UsvOdom::testPrint(std::string str){
  std_msgs::String str_S;
  str_S.data = str;
  ROS_INFO_STREAM("usv_odom: test print " << str_S.data);
}
*/
