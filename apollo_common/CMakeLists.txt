cmake_minimum_required(VERSION 3.0.2)
project(apollo_common)

add_compile_options(-std=c++17)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_msgs
  tf2_ros
  apollo_msgs
)

catkin_package(
	INCLUDE_DIRS   include
	LIBRARIES      ${PROTO_LIB_NAME} ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp tf2_msgs tf2_ros apollo_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  /usr/include
)

file(GLOB MATH_SRC_FILES "src/math/*.cc")

add_library(${PROJECT_NAME}
  src/apollo_app.cc
  # adapters
  src/adapters/adapter_gflags.cc
  src/adapters/adapter_manager.cc
  # configs
  src/configs/config_gflags.cc
  src/configs/vehicle_config_helper.cc
  # math
  ${MATH_SRC_FILES}
  # monitor
  src/monitor/monitor_buffer.cc
  src/monitor/monitor.cc
  # util
  src/util/file.cc
  src/util/string_tokenizer.cc
  src/util/util.cc
  # vehicle_state
  src/vehicle_state/vehicle_state.cc
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
install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)