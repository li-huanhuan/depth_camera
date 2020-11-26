cmake_minimum_required(VERSION 2.8.3)
project(depth_camera_layer)
#set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  pluginlib
  roscpp
  sensor_msgs
  tf2_geometry_msgs
  message_filters
  tf2
  tf2_ros
  robot_msg
)

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(
    include
    ./include/depth_camera_layer
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )

generate_dynamic_reconfigure_options(cfg/DepthCameraLayer.cfg)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES costmap_2d layers
    CATKIN_DEPENDS
      angles
      costmap_2d
      dynamic_reconfigure
      geometry_msgs
      pluginlib
      roscpp
      sensor_msgs
      tf2_geometry_msgs
      message_filters
      tf2
      tf2_ros
      robot_msg
    LIBRARIES
      depth_camera_layer
    DEPENDS
      Boost
)

add_library(${PROJECT_NAME} src/depth_camera_layer.cpp)
add_dependencies(${PROJECT_NAME} ${depth_camera_layer_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS depth_camera_layer
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES costmap_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
#catkin_install_python(PROGRAMS scripts/send_message.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
