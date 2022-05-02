cmake_minimum_required(VERSION 2.8.3)
project(apollo_simulator)

add_compile_options(-std=c++11)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2
  std_msgs
  geometry_msgs
  apollo_msgs
  apollo_common
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/apollo_simulator.cpp
  src/vehicle_sim_model/sim_model_constant_acceleration.cpp
  src/vehicle_sim_model/sim_model_ideal.cpp
  src/vehicle_sim_model/sim_model_interface.cpp
  src/vehicle_sim_model/sim_model_time_delay.cpp
  src/vehicle_sim_model/sim_model_util.cpp
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  protobuf
  glog
  gflags
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)