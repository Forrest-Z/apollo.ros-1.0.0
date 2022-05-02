cmake_minimum_required(VERSION 3.0.2)
project(apollo_control)

add_compile_options(-std=c++17)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  apollo_msgs
  apollo_common
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES apollo_control
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME} 
  src/main.cc
  src/control.cc
  # common
  src/common/control_gflags.cc
  src/common/hysteresis_filter.cc
  src/common/interpolation_2d.cc
  src/common/pid_controller.cc
  src/common/trajectory_analyzer.cc
  # controller
  src/controller/controller_agent.cc
  src/controller/lat_controller.cc
  src/controller/lon_controller.cc
  # filters
  src/filters/digital_filter_coefficients.cc
  src/filters/digital_filter.cc
  src/filters/mean_filter.cc
)
add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  protobuf
  glog
  gflags
)

# install
install(TARGETS ${PROJECT_NAME} 
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)