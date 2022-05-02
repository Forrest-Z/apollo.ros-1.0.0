cmake_minimum_required(VERSION 3.0.2)
project(apollo_msgs)

add_compile_options(-std=c++17)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")

# for proto message.
set(PROTO_LIB_NAME apollo_proto)
macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()
set(PROTOBUF_PROTOC_EXECUTABLE protoc)
set(PROTO_HRDS)
set(PROTO_SRCS)
SUBDIRLIST(SUBDIRS ${PROJECT_SOURCE_DIR}/proto/${PROJECT_NAME}/proto)
foreach(subdir ${SUBDIRS})
  file(GLOB PROTO_FILES "${PROJECT_SOURCE_DIR}/proto/${PROJECT_NAME}/proto/${subdir}/*.proto")
  message(STATUS "proto files: ${PROTO_FILES}")
  set(proto_out_path ${PROJECT_SOURCE_DIR}/include)
  file(REMOVE ${proto_out_path})
  file(MAKE_DIRECTORY ${proto_out_path})
  foreach(proto_file ${PROTO_FILES})
    get_filename_component(msg_path ${proto_file} ABSOLUTE)
    get_filename_component(msg_name ${proto_file} NAME_WE)
    list(APPEND PROTO_HRDS ${proto_out_path}/${PROJECT_NAME}/proto/${subdir}/${msg_name}.pb.h)
    list(APPEND PROTO_SRCS ${proto_out_path}/${PROJECT_NAME}/proto/${subdir}/${msg_name}.pb.cc)
    execute_process(
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} 
      # -I=${PROJECT_SOURCE_DIR}/proto/${subdir}
      -I=${PROJECT_SOURCE_DIR}/proto
      --cpp_out=${proto_out_path} ${msg_path}
      # WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
  endforeach()
  message(STATUS "proto srcs: ${PROTO_SRCS}")
endforeach()

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
	INCLUDE_DIRS   include
	LIBRARIES      ${PROTO_LIB_NAME}
	CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include
)

# proto message lib.
add_library(${PROTO_LIB_NAME}
	${PROTO_HRDS}
	${PROTO_SRCS}
)
target_link_libraries(${PROTO_LIB_NAME}
	protobuf
)

# install
install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROTO_LIB_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
