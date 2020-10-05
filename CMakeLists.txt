cmake_minimum_required(VERSION 3.0.2)
project(f1t)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
)

# set(CUDNN_INCLUDE_PATH /usr/local/cuda/include)
set(Torch_DIR /home/user/Downloads/libtorch/share/cmake/Torch)

find_package(Torch REQUIRED)

# download binaries from https://github.com/pytorch/pytorch/issues/17492#issuecomment-524692441

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

find_package(OpenCV REQUIRED)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS roscpp 
               rospy 
               std_msgs 
               sensor_msgs
               cv_bridge
               image_transport
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Torch_INCLUDE_DIRS}
)

add_executable(image_converter src/image_converter.cpp)
target_link_libraries(image_converter ${OpenCV_LIBS} ${catkin_LIBRARIES})


add_executable(model_node src/model_node.cpp)
target_link_libraries(model_node ${OpenCV_LIBS} ${TORCH_LIBRARIES} ${catkin_LIBRARIES})


# catkin_install_python(PROGRAMS
#   scripts/model.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
