/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  usv_odom.cpp
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/4
  * - Brief:     Interface implementation of UsvOdom Class
  *****************************************************************************
  * History:
  * 2019/12/4   
  * Complete interface of UsvOdom class, complie successfully
  *****************************************************************************
  * History:
  * 2019/12/17  
  * Overload UsvOdom::sendCommand
  * Modify UsvOdom::NextGoalCb, add stop open control command sending
  * Modify parameter type to publish on topic "odom"
  *****************************************************************************
  * History:
  * 2019/12/18 
  * Modify callback function of topic "next_goal"
  * Add ros service to set origin point of USV
  *****************************************************************************
  * History:
  * 2019/12/19
  * Add data filterint for odom message
  *****************************************************************************
*/

#include <usv_odom/usv_odom.h>
#include <exception>

UsvOdom::UsvOdom() : rud_(PackProtocol::init_rud_), speed_(0), 
                     pub_interval_(100), count_(0), odom_filter_(nullptr)
{
  initialize();
}

UsvOdom::UsvOdom(int pub_time) : rud_(PackProtocol::init_rud_), speed_(0), 
                                 pub_interval_(pub_time), count_(0), 
                                 odom_filter_(nullptr)
{
  initialize();
}

void UsvOdom::initialize(){
  int baud_rate, time_out;
  std::string port_num;
  bool use_slide_avr_filter;

  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  private_nh.param("port_num", port_num, std::string("/dev/ttyUSB0")); // for windows
  private_nh.param("baud_rate", baud_rate, 9600);
  private_nh.param("time_out", time_out, 200);
  private_nh.param("ship_num", ship_num_, 1);
  private_nh.param("odom_frame", odom_frame_, std::string("odom"));
  private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
  private_nh.param("origin_latitude", ori_lat_, 30.0);
  private_nh.param("origin_longitude", ori_lng_, 114.0);
  private_nh.param("use_slide_avr_filter", use_slide_avr_filter, true);
  private_nh.param("filter_start_time", filter_st_, 5);  

  // odom_pub_ = nh.advertise<nav_msgs::Odometry>(odom_frame_, 1); // temp to be anotated
  geographic_pos_pub_ = nh.advertise<geographic_msgs::GeoPoint>("geo_position", 1);
  goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("next_goal", 1, boost::bind(&UsvOdom::nextGoalCb, this, _1));
  // set_origin_srv_ = nh.advertiseService("set_origin", boost::bind(&UsvOdom::setOrigin, this, _1, _2));
  set_origin_srv_ = nh.advertiseService("set_origin", &UsvOdom::setOrigin, this);

  serial_port_ = new SerialPort(port_num, baud_rate); 

  read_len_ = 1024;

  if(use_slide_avr_filter){
    odom_filter_ = new SlideAvrFilter(static_cast<size_t>(filter_st_));
  }

  last_pub_time_ = boost::posix_time::microsec_clock::universal_time();

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
    delete serial_port_;
    serial_port_ = nullptr;
  }

  if(odom_filter_){
    delete odom_filter_;
    odom_filter_ = nullptr; 
  }
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
    {
      boost::lock_guard<boost::mutex> lock(write_mutex_);
      serial_port_->writeInToPort(command);
    }
    return true;
  }
  return false;
}

bool UsvOdom::sendCommand(uint8_t* command, size_t command_len){
  if(serial_port_->isPortOpen()){
    {
      boost::lock_guard<boost::mutex> lock(write_mutex_);
      serial_port_->writeInToPort(command, command_len);
    }
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

        // update count
        ++count_;

        // update cmd 
        rud_ = rud_ang;
        speed_ = speed;

        // store position of origin point
        ori_lat_ = ori_lat;
        ori_lng_ = ori_lng;

        double det_x = north - pre_north_;
        double det_y = east - pre_east_; 
     
        // create odom message
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = odom_frame_;
        odom.pose.pose.position.x = north;
        odom.pose.pose.position.y = east;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);

        odom.child_frame_id = robot_base_frame_;
        odom.twist.twist.linear.x = (det_x + det_y) * std::sin(yaw) * std::cos(yaw) * 1000 / pub_interval_;
        odom.twist.twist.linear.y = (det_y - det_x) * std::sin(yaw) * std::cos(yaw) * 1000 / (std::cos(yaw) * std::cos(yaw) - std::sin(yaw) * std::sin(yaw));
        odom.twist.twist.angular.z = (yaw - pre_yaw_) / pub_interval_;

        // create geographic position message
        geographic_msgs::GeoPoint geo_pos_msgs;
        geo_pos_msgs.latitude = lat;
        geo_pos_msgs.longitude = lng;

        nav_msgs::Odometry filtered_odom = odom;
        
        if(odom_filter_){
          boost::posix_time::ptime cur_time = boost::posix_time::microsec_clock::universal_time();
          double dt = static_cast<double>((cur_time - last_pub_time_).ticks()) / 1000000;
          filtered_odom = odom_filter_->odomFilter(odom, count_, dt);
        }

        // publish message
        ROS_INFO("usv_odom: publish odom");
        // odom_pub_.publish(odom); //temp to be anotated
        odom_pub_.publish(filtered_odom);
        geographic_pos_pub_.publish(geo_pos_msgs);

        last_pub_time_ = boost::posix_time::microsec_clock::universal_time(); // upate publish time

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
  double high = next_goal->pose.position.z;
 
  if(high != -1){
    // transfer coordinate from NED to LatLng
    double next_lat = UnpackProtocol::getLat(north, ori_lat_);
    double next_lng = UnpackProtocol::getLng(east, ori_lat_, ori_lng_);

    ROS_INFO_STREAM("usv_odom: latitude and longitude of next goal: " << std::setprecision(6) << north << "," << east << "," << ori_lat_ << "," << ori_lng_ << "," << next_lat << "," << next_lng);

    std::vector<uint8_t> data_queue = PackProtocol::getDataStack(ship_num_, next_lat, next_lng);
    uint8_t data_stack[data_queue.size()];

    for(int i = 0; i < data_queue.size(); ++i){
      data_stack[i] = data_queue[i];
    }

    if(sendCommand(data_stack, data_queue.size())){
      ROS_INFO("usv_odom: write point follow command!");
      testPrint(data_queue);
    }
    else{
      ROS_ERROR("usv_odom: write point follow command failed!");
      testPrint(data_queue);
    }
  }
  else{
    size_t data_len;
    rud_ = PackProtocol::init_rud_;
    speed_ = 0;
    std::vector<uint8_t> data_queue = PackProtocol::getDataStack(ship_num_, 0, 0, rud_, speed_);
    uint8_t data_stack[data_queue.size()];
    for(int i = 0; i < data_queue.size(); ++i){
      data_stack[i] = data_queue[i];
    }
    if(sendCommand(data_stack, data_queue.size())){
      ROS_INFO("usv_odom: write stop command!");
      testPrint(data_queue);
    }
    else{
      ROS_ERROR("usv_odom: write stop command failed!");
      testPrint(data_queue);
    }
  }
}

void UsvOdom::testPrint(uint8_t* data_buf, size_t len){
  std::vector<uint8_t> data_queue;
  try{
    data_queue.insert(data_queue.end(), data_buf, data_buf+len);
  }
  catch(std::exception& e){
    ROS_ERROR("usv_odom: transfer buffer to queue failed");
  }
  try{
    testPrint(data_queue);
  }
  catch(std::exception& e){
    ROS_ERROR("usv_odom: print data queue failed");
  }
}

void UsvOdom::testPrint(std::vector<uint8_t> read_buf){
  std::for_each(read_buf.begin(), read_buf.end(), [](uint8_t data){
     std::cout << std::hex << (data & 0xff) << " "; 
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

bool UsvOdom::setOrigin(usv_odom::SetOrigin::Request& req, usv_odom::SetOrigin::Response& res){
  std::vector<uint8_t> data_queue = PackProtocol::getDataStack(ship_num_);
  uint8_t data_stack[data_queue.size()];
  for(int i = 0; i < data_queue.size(); ++i){
    data_stack[i] = data_queue[i];
  }
 
  if(sendCommand(data_stack, data_queue.size())){
    res.result = true; 
    ROS_INFO("usv_odom: write set origin point command!");
    testPrint(data_queue);
  }
  else{
    res.result = false;
    ROS_ERROR("usv_odom: write set origin command failed!");
  }
  return res.result;
}
