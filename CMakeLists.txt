cmake_minimum_required(VERSION 2.8.3)

project(ueye_cam)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -Wall") # For G++ <4.7; also appears to be supported in G++ 4.7+ as alias to c++11

set(UEYE_INCLUDE_PATH /usr/include)
set(UEYE_LIBRARY_PATH /usr/lib)
set(UEYE_LIBRARY ueye_api)
set(UEYECAM_NODELET_NAME ueye_cam_nodelet)
set(UEYECAM_LIB_NAME ueye_wrapper)

set(UEYECAM_NODE_SOURCES
  src/ueye_cam_node.cpp
)

set(UEYECAM_LIB_SOURCES
  src/ueye_cam_driver.cpp
)

set(UEYECAM_LIB_HEADERS
  include/ueye_cam/logging_macros.hpp
  include/ueye_cam/ueye_cam_driver.hpp
)

set(UEYECAM_NODELET_SOURCES
  src/ueye_cam_nodelet.cpp
)

set(UEYECAM_NODELET_HEADERS
  include/ueye_cam/ueye_cam_nodelet.hpp
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  dynamic_reconfigure
  image_transport
  sensor_msgs
  camera_calibration_parsers
  camera_info_manager
  )

generate_dynamic_reconfigure_options(
  cfg/UEyeCam.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${UEYECAM_LIB_NAME} ${UEYECAM_NODELET_NAME}
  CATKIN_DEPENDS roscpp nodelet dynamic_reconfigure image_transport sensor_msgs camera_calibration_parsers camera_info_manager
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${UEYE_INCLUDE_PATH}
  )


add_library(${UEYECAM_LIB_NAME} ${UEYECAM_LIB_SOURCES} ${UEYECAM_LIB_HEADERS})
target_link_libraries(${UEYECAM_LIB_NAME} ${UEYE_LIBRARY})

add_library(${UEYECAM_NODELET_NAME} ${UEYECAM_NODELET_SOURCES} ${UEYECAM_NODELET_HEADERS})
target_link_libraries(${UEYECAM_NODELET_NAME} ${catkin_LIBRARIES} ${UEYECAM_LIB_NAME})
add_dependencies(${UEYECAM_NODELET_NAME} ${PROJECT_NAME}_gencfg)

install(TARGETS ${UEYECAM_LIB_NAME} ${UEYECAM_NODELET_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

