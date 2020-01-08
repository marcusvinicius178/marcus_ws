cmake_minimum_required(VERSION 2.8.3)
project(abstract_sensor)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)



## Generate messages in the 'msg' folder
add_message_files(
    FILES
    Abstractsensor.msg
)



## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)



catkin_package( CATKIN_DEPENDS rospy message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


