cmake_minimum_required(VERSION 2.8.3)
project(services)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   SchedulerServerData.srv
 )

 generate_messages(
   DEPENDENCIES
   roscpp
 )

catkin_package(

CATKIN_DEPENDS message_runtime std_msgs roscpp

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



