cmake_minimum_required(VERSION 2.8.3)
project(usv_odom)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamic_reconfigure
  geographic_msgs
  geometry_msgs
  roscpp
  rospy
  serial
  std_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES usv_odom
 CATKIN_DEPENDS 
   cmake_modules 
   dynamic_reconfigure 
   geographic_msgs 
   geometry_msgs 
   roscpp 
   rospy 
   serial 
   std_msgs 
   tf
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(usv_odom
  src/serial_port.cpp
  src/usv_odom.cpp
  src/pack_protocol.cpp
  src/serial_port_boost.cpp
)
target_link_libraries(usv_odom ${catkin_LIBRARIES})
add_dependencies(usv_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(usv_odom_node
  src/usv_odom_node.cpp
)
add_dependencies(usv_odom_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(usv_odom_node usv_odom ${catkin_LIBRARIES})

add_executable(serial_port_boost_node
  src/serial_port_boost_node
)
add_dependencies(serial_port_boost_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(serial_port_boost_node usv_odom ${catkin_LIBRARIES})

